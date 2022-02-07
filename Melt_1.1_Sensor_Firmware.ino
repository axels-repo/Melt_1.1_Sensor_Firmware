/*
Forked by Axel Baylon axelbaylon@hotmail.com based off
Kai James kaicjames@outlook.com code for Melt Sensors
*/
#include <SDI12.h>
#include "RTClib.h"
#include "Wire.h"
#include <SD.h>
#include "ADS1X15.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include "RTCZero.h"
#include "RH_RF95.h"
// #include <FlashAsEEPROM.h> Not using this right now

#define TEST_LOOP             (false)     //Run test loop instead of actual loop
#define SLEEP_ENABLED         (true)    //Disable to keep serial coms alive for testing
#define FIRMWARE_VERSION      (104)
#define BV_OFFSET             (0.01)
#define OTT_OFFSET            (0.15)
#define ALS_POWER_WAIT        (1000) //Depends on probe. 1000 is safe
#define RTC_OFFSET_S          (12)
#define WRAP_AROUND_S_LOWER   (11)
#define WRAP_AROUND_S_UPPER   (49)
#define MAX_LOG_SIZE_BYTES    (1800000000)  //1.8GB. Max for some versions of FAT16
#define WRAP_AROUND_BUFF      (1)
#define ALS_AVE_COUNT         (11)  //Number of ADC reads for ALS averaging
#define ALS_AVE_DELAY         (0) //Period (mS) between ADC reads for ALS averaging. ALS takes around 10mS to poll regardless. 
#define TEMP_INIT_TIME        (100)
#define MAX_ALS_CAL_POINTS    (12)
#define MAX_REPEATED_NODES    (10)
#define RAIN_CUMULATIVE       (true)  //true is much better for telemetry monitoring
//Hardware pins
#define ONE_WIRE_BUS          (5)        //One wire temp sensors
#define RAIN_GAUGE_INT        (9)
#define DATA_PIN              (12)          // The pin of the SDI-12 data bus
#define SD_SPI_CS             (A4)
#define USS_ECHO              (A2)
#define USS_TRIG              (A3)
#define USS_INIT_TIME         (50)
#define LED                   (13)
#define FET_POWER             (4)
#define BATT                  (A5)
//I2C Addressing
#define ADC_ADDR              (0x48)
#define RTC_ADDR              (0x68)  //Set automatically, here as a reminder
//Other
#define WATCHDOG_TIMER_MS     (11)   //Valid values: 0-11. 11 gives 16s timeout. 10 gives 8s timeout and so on  //resetWDT(); as necessary
#define LORA_FREQUENCY        (919.9)
#define LORA_BANDWIDTH        (125000)  //Increase to 250000 or 500000 for increased speed, less range. Linear change.

//CONFIG.txt variables
String NodeID;
double ALSslope = 1;
double ALSoffset = 0;
int LoRaPollOffset = 23;
int pollPerLoRa = 6;
int LoRaRepeater = 0;
String Nodes_to_repeat[MAX_REPEATED_NODES] = {"NULL"};
String project = "TST";
String siteID = "AA";
float raingaugeTip_mm = 0.2;
bool LoRaEnabled = true;
bool SDEnabled = true;
int logFileSizeLast = 10;
// int lastSDSize = 1;

const char *monthName[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

typedef struct {
  uint8_t sensorCount = 0;
  double measure[10];
  double measure_2[10];
  uint8_t pollPeriod = 240;
  uint8_t cyclesPerPoll;
  uint8_t cycleCount;
  DeviceAddress addr[4];
} value_t;
value_t value;

typedef struct {
  float temp[MAX_ALS_CAL_POINTS] = {15};
  float slope[MAX_ALS_CAL_POINTS] = {1.28783};
  float offset[MAX_ALS_CAL_POINTS] = { -1290.16};
} ALSCal_t;
ALSCal_t ALSCal;

typedef struct {
  value_t rainGauge;
  value_t temp;
  value_t RTCTemp;
  value_t ALS;
  value_t OTT;
  value_t USS;
} sensor_t;
sensor_t sensor;

typedef struct {
  String packet;
  int RSSI;
} LoRaReceive_t;
LoRaReceive_t LoRaReceive;

//Initialise libraries
RH_RF95 rf95(12, 6);
uint8_t rfbuf[RH_RF95_MAX_MESSAGE_LEN];//
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensors(&oneWire);
DeviceAddress Thermometer;
DeviceAddress hello;

RTC_DS3231 rtc;
ADS1115 ads(ADC_ADDR);
SDI12 mySDI12(DATA_PIN);
RTCZero internalrtc;

int pollPeriod;
File logFile;
File configFile;
String UNIXtimestamp;
String normTimestamp;
String fileNameStr;
String dataString;
int sleep_now_time;
int sleep_remaining_s = 0;
uint8_t tx_count = 0;
char addr[5];
char hex_chars[] = "0123456789ABCDEF";
int Year;
bool setupLoop = true;
int logFileSize = 0;
// float nodeDesyncRatio;
int tarSec;
int loraPollCount = pollPerLoRa;
String CSVHeader;
int logIncrement = 97;  //ASCII lowercase a
int lastUpTime = 0;
int WakeTime = 0;
int SensWakeTime = 0;
bool dailyReset = false;
float rainfall_mm = 0;
volatile uint16_t tip_count = 0;
volatile bool rain_interrupt = false;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////    SETUP           ///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup () {
  sensor.RTCTemp.sensorCount = 1;
  hexstr(getChipId(), addr, sizeof(addr));  //Establish default node ID. Overwrite if specified in config
  NodeID = String(addr);
  pinMode(FET_POWER, OUTPUT);

  if (rf95.init() == false) {
    while (1) {
      delay(50);
      if (rf95.init()) {
        break;
      }
    }  //Watchdog will reset
  } else {
    rf95.setTxPower(23, false);
    rf95.setFrequency(LORA_FREQUENCY);
    rf95.setSignalBandwidth(LORA_BANDWIDTH); //500kHz
  }
  sendLoRaIgnore(String(NodeID) + ": sLoRa started");

  //SD card
  if (!SD.begin(SD_SPI_CS)) {
    //SerialUSB.println("SD initialization failed!");
    crashNflash(1);  //10*10ms high period
  }
  OneWireTempSetup(); //Must auto detect sensors BEFORE configRead()
  configRead();
  sendLoRaIgnore("Config updated");


  while (LoRaRepeater) {
    LoRaListen(999);   //Enter 999 for no timeout
    String tmp = LoRaReceive.packet;
    int i = 0;
    char * pch = strtok((char*)tmp.c_str(), ",");
    bool repeatPacket = false;
    while (pch != NULL) {
      for (int j = 0 ; j < MAX_REPEATED_NODES ; j++) {
        if (String(pch) == Nodes_to_repeat[j]) {
          repeatPacket = true;
        }
      }
      pch = strtok (NULL, ",");
      i++;
    }
    if (repeatPacket == true) {
      delay(500); //Wait for existing signal to clear
      // sendLoRaRaw("Repeat packet:");
      sendLoRaRaw(LoRaReceive.packet);
      // LoRaReceive.RSSI;
    }
  }

  Wire.begin(); //Might not need this
  setupWDT( WATCHDOG_TIMER_MS );

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  //Ultra sonic
  pinMode(USS_TRIG, OUTPUT);
  pinMode(USS_ECHO, INPUT);
  digitalWrite(USS_TRIG, LOW);

  //OTT probe
  if (sensor.OTT.sensorCount > 0) {
    mySDI12.begin();
  }
  sendLoRaIgnore("About to start ADC");
  ads.begin();
  ads.setGain(0);  // 6.144 volt
  sendLoRaIgnore("ADC started");
  //ALS Sensor
  CSVHeader = "TIMESTAMP,SITE_ID,NODE_ID,FW_VER,COUNT,UPTIME,FILE_SIZE,BATT_V,";
  String CSVUnits = "TS,SI,NI,FW,INT,mS,BYTES,V,";
  if (sensor.rainGauge.sensorCount > 0) {
    CSVHeader += "RAIN,";
    CSVUnits += "mm,";
  }
  for (int j = 0; j < sensor.temp.sensorCount ; j++) {
    CSVHeader +=  "Temp_" + String(j) + ",";
    CSVUnits += "Deg C,";
  }
  if (sensor.RTCTemp.sensorCount > 0) {
    CSVHeader += "RTCTemp,";
    CSVUnits += "Deg C,";
  }
  for (int j = 0; j < sensor.ALS.sensorCount ; j++) {
    CSVHeader += + "RawALS_" + String(j) + ",";
    CSVUnits += "mm,";
  }
  for (int j = 0; j < sensor.ALS.sensorCount ; j++) {
    CSVHeader += + "EstLevel_" + String(j) + ",";
    CSVUnits += "mm,";
  }
  for (int j = 0; j < sensor.OTT.sensorCount ; j++) {
    CSVHeader += + "OTT_" + String(j) + ",";
    CSVUnits += "mm,";
  }
  if (sensor.USS.sensorCount > 0) {
    CSVHeader += "USS,";
    CSVUnits += "uS,";
  }

  fileNameStr = fileNameGen(logIncrement);
  while (SD.exists((char*)fileNameStr.c_str()) && logIncrement < 122) {
    logIncrement++;
    fileNameStr = fileNameGen(logIncrement);
  }
  logIncrement--;
  fileNameStr = fileNameGen(logIncrement);
  logFile = SD.open((char*)fileNameStr.c_str());
  String currHeader = logFile.readStringUntil('\n');
  logFileSize = logFile.size();
  logFile.close();
  if (logFileSize == 0 && logFileSizeLast == 0) { //Quick check for SD driver crash
    systemReset();
  }
  logFileSizeLast = logFileSize;
  currHeader.trim(); //Remove whitespace. THIS IS NEEDED
  if ((currHeader != CSVHeader) || (logFileSize > MAX_LOG_SIZE_BYTES)) { //Reprint header if something has changed.
    logIncrement++;
    fileNameStr = fileNameGen(logIncrement);
    logFile = SD.open((char*)fileNameStr.c_str(), FILE_WRITE);
    logFile.println(CSVHeader);
    logFile.close();
  }

  sendLoRaIgnore("About to start RTC");
  //RTC
  if (! rtc.begin()) {
    crashNflash(2);
  }
  sendLoRaIgnore("RTC started");

  DateTime now = rtc.now();
  int Hour;
  int Min;
  int Sec;
  int Day;
  char Month[12];
  uint8_t monthIndex;
  bool updateTime;
  sscanf(__TIME__, "%d:%d:%d", &Hour, &Min, &Sec);
  sscanf(__DATE__, "%s %d %d", Month, &Day, &Year);
  for (monthIndex = 0; monthIndex < 12; monthIndex++) {
    if (strcmp(Month, monthName[monthIndex]) == 0) break;
  }
  int uploadTime = Min + Hour * 60 + Day * 24 * 60 + (monthIndex + 1) * 24 * 60 * 31;
  int RTCTime = now.minute() + now.hour() * 60 + now.day() * 24 * 60 + now.month() * 24 * 60 * 31;
  if (RTCTime < uploadTime || now.year() < Year) {
    if (Sec >= 60 - RTC_OFFSET_S) {
      Min = Min + 1;
      Sec = Sec + RTC_OFFSET_S - 60;
    }
    else {
      Sec = Sec + RTC_OFFSET_S;
    }
    rtc.adjust(DateTime(Year, monthIndex + 1, Day, Hour, Min, Sec));
  }

  updatePollFreq(); //Calculate sensor poll frequencies
  tarSec = LoRaPollOffset % (pollPeriod * 60); //Gives us the target start second for polling

  internalrtc.begin(false);
  internalrtc.attachInterrupt(wake_from_sleep);

  analogReference(AR_INTERNAL2V23); //For internal battery level calculation


  sendLoRaIgnore("Going to sleep to sync time");
  if (!TEST_LOOP) {
    sleepTillSynced();
  }
  String tmpStr = String("Node ") + NodeID + " ready for work. Time offset " + String(LoRaPollOffset, DEC) + " seconds";
  sendLoRaIgnore(tmpStr); //Quick message to say we've woken up


  setupWDT( WATCHDOG_TIMER_MS ); // initialize and activate WDT with maximum period
  //Rain gauge. Add last to ensure the interrupt doesn't interfere with initial time sync
  if (sensor.rainGauge.sensorCount > 0) {
    pinMode(RAIN_GAUGE_INT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(RAIN_GAUGE_INT), rain_isr, FALLING);
    // Configure EIC to use GCLK1 which uses XOSC32K. Allows rise/fall interupt modes to work.
    SYSCTRL->XOSC32K.bit.RUNSTDBY = 1;
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) |
                        GCLK_CLKCTRL_GEN_GCLK1 |
                        GCLK_CLKCTRL_CLKEN;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////      LOOP           ///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop () {
  while (TEST_LOOP) {
    disableWatchdog();
    digitalWrite(LED, HIGH);
  }

  resetWDT();
  wake_system();


  DateTime now;
  if (sensor.rainGauge.sensorCount > 0) {
    rainfall_mm = tip_count * raingaugeTip_mm;
    if (RAIN_CUMULATIVE) {
      now = rtc.now();
      if (now.hour() == 0) {
        if (dailyReset) {
          tip_count = 0;
          dailyReset = false;
        }
      }
      else if (!dailyReset) {
        dailyReset = true;
      }
    }
    else {
      tip_count = 0;
    }
  }

  if (sensor.temp.sensorCount > 0) {
    if (sensor.temp.cycleCount == 1) { //Only test if we have woken the sensor and reset counter
      if (!tempUpdate()) {  //This and subsequent blank if statements are used in problem solving. Can defs remove them
      }
    }
  }
  if (sensor.RTCTemp.sensorCount > 0) {
    if (sensor.RTCTemp.cycleCount == 1) { //Only test if we have woken the sensor and reset counter
      if (!RTCTempUpdate()) {  //This and subsequent blank if statements are used in problem solving. Can defs remove them
      }
    }
  }

  if (sensor.ALS.sensorCount > 0) {
    if (sensor.ALS.cycleCount == 1) { //Only test if we have woken the sensor and reset counter
      if (!ALSUpdate()) {  //This and subsequent blank if statements are used in problem solving. Can defs remove them
      }
    }
  }

  if (sensor.OTT.sensorCount > 0) {
    if (sensor.OTT.cycleCount == 1) { //Only test if we have woken the sensor and reset counter
      if (!OTTUpdate()) {  //This and subsequent blank if statements are used in problem solving. Can defs remove them
      }
    }
  }
  if (sensor.USS.sensorCount > 0) {
    if (sensor.USS.cycleCount == 1) { //Only test if we have woken the sensor and reset counter
      if (!USSUpdate()) {  //This and subsequent blank if statements are used in problem solving. Can defs remove them
      }
    }
  }

  sleep_system();
  buildTimestamps();
  buildCSVDataString();

  if (SDEnabled) {
    logDataToSD();
  }

  if (LoRaEnabled) {
    if (loraPollCount >= pollPerLoRa) {
      loraPollCount = 0;
      LoRaUpdate();
    }
    loraPollCount++;
  }

  rf95.sleep();
  delay(20);    //Delay 20ms to ensure the chips have gone to sleep before powering off the board
  disableWatchdog(); // disable watchdog
  rain_interrupt = false;
  bool first_loop = true;
  now = rtc.now();
  lastUpTime = millis() - WakeTime;
  while ((first_loop || rain_interrupt)) {
    if (first_loop) { //REMEMBER first loop is relative to this wake cycle considering rain interupt. NOT first_loop for the whole system.
      first_loop = false;
      if ((tarSec < WRAP_AROUND_S_LOWER) && (now.second() >= 30))  { //Wrap down case
        sleep_remaining_s = pollPeriod * 60 + tarSec - (-60 + now.second()) + 1 + WRAP_AROUND_BUFF;
      }
      else if ((tarSec > WRAP_AROUND_S_UPPER) && (now.second() <= 30)) { //Wrap up case
        sleep_remaining_s = pollPeriod * 60 + tarSec - (60 + now.second()) - 1 + WRAP_AROUND_BUFF;
      }
      else {  //Standard time correction
        sleep_remaining_s = pollPeriod * 60 + tarSec - now.second() + WRAP_AROUND_BUFF;
      }
    }
    rain_interrupt = false;
    sleep();
  }
  WakeTime = millis();  //For wake period calculation
  setupWDT( WATCHDOG_TIMER_MS ); // initialize and activate WDT with maximum period
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////    FUNCTIONS         ///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

String fileNameGen(int increment) {
  String fileName = String(siteID) + String(NodeID) + String(char(increment)) + ".csv";
  return fileName;
}

void updatePollFreq() {
  pollPeriod = min(min(min(min(min(sensor.temp.pollPeriod, sensor.RTCTemp.pollPeriod), sensor.ALS.pollPeriod), sensor.OTT.pollPeriod), sensor.USS.pollPeriod), sensor.rainGauge.pollPeriod);
  //Calculate cycles per poll
  sensor.rainGauge.cyclesPerPoll = sensor.rainGauge.pollPeriod / pollPeriod;
  sensor.temp.cyclesPerPoll = sensor.temp.pollPeriod / pollPeriod;
  sensor.RTCTemp.cyclesPerPoll = sensor.RTCTemp.pollPeriod / pollPeriod;
  sensor.ALS.cyclesPerPoll = sensor.ALS.pollPeriod / pollPeriod;
  sensor.OTT.cyclesPerPoll = sensor.OTT.pollPeriod / pollPeriod;
  sensor.USS.cyclesPerPoll = sensor.USS.pollPeriod / pollPeriod;
  //Init counter
  sensor.rainGauge.cycleCount = sensor.rainGauge.cyclesPerPoll;
  sensor.temp.cycleCount = sensor.temp.cyclesPerPoll;
  sensor.RTCTemp.cycleCount = sensor.RTCTemp.cyclesPerPoll;
  sensor.ALS.cycleCount = sensor.ALS.cyclesPerPoll;
  sensor.USS.cycleCount = sensor.USS.cyclesPerPoll;
  sensor.OTT.cycleCount = sensor.OTT.cyclesPerPoll;
}

void sleep() {
  sleep_now_time = internalrtc.getEpoch();
  if (sleep_remaining_s > 0) {
    if (SLEEP_ENABLED) {
      internalrtc.setAlarmEpoch(sleep_now_time + (sleep_remaining_s - 1));
      internalrtc.enableAlarm(internalrtc.MATCH_HHMMSS);
      SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
      SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
      __DSB();
      __WFI();
      SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
    }
    else if (!SLEEP_ENABLED) {
      delay(sleep_remaining_s * 1000);
    }
    sleep_remaining_s = sleep_remaining_s - (internalrtc.getEpoch() - sleep_now_time); // Restarts clock in case of wake due to rain interupt
  }
}

uint16_t getChipId() {
  volatile uint32_t *ptr = (volatile uint32_t *)0x0080A048;
  return *ptr;
}

void hexstr(uint16_t v, char *buf, size_t Size) {
  uint8_t i;
  if (Size > 4) {
    for (i = 0; i < 4; i++) {
      buf[3 - i] = hex_chars[v >> (i * 4) & 0x0f];
    }
    buf[4] = '\0';
  }
}

static void   WDTsync() {
  while (WDT->STATUS.bit.SYNCBUSY == 1); //Just wait till WDT is free
}

void setupWDT( uint8_t period) {
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(5) | GCLK_GENDIV_DIV(4);
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(5) |
                      GCLK_GENCTRL_GENEN |
                      GCLK_GENCTRL_SRC_OSCULP32K |
                      GCLK_GENCTRL_DIVSEL;
  while (GCLK->STATUS.bit.SYNCBUSY);  // Syncronize write to GENCTRL reg.
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_WDT |
                      GCLK_CLKCTRL_CLKEN |
                      GCLK_CLKCTRL_GEN_GCLK5;
  WDT->CTRL.reg = 0; // disable watchdog
  WDTsync(); // sync is required
  WDT->CONFIG.reg = min(period, 11); // see Table 17-5 Timeout Period (valid values 0-11)
  WDT->CTRL.reg = WDT_CTRL_ENABLE; //enable watchdog
  WDTsync();
}

void systemReset() {  // use the WDT watchdog timer to force a system reset.
  WDT->CLEAR.reg = 0x00; // system reset via WDT
  WDTsync();
}

void resetWDT() {
  WDT->CLEAR.reg = 0xA5; // reset the WDT
  WDTsync();
}

void wake_from_sleep() {  //ISR runs whenever system wakes up from RTC
}

void rain_isr() {
  tip_count++;
  rain_interrupt = true;

  // static unsigned long last_interrupt_time = 0;
  // unsigned long interrupt_time = millis();
  // // If interrupts come faster than 200ms, assume it's a bounce and ignore
  // if (interrupt_time - last_interrupt_time > 200)
  // {
  //   ... do your thing
  // }
  // last_interrupt_time = interrupt_time;

}

void LoRaUpdate() {
  char *pmsg;
  String LoRaString = "PKT:" + normTimestamp + dataString;
  pmsg = (char*)LoRaString.c_str();
  rf95.send((uint8_t *)pmsg, strlen(pmsg) + 1);
  rf95.waitPacketSent();    //This takes 189ms
}

void sendLoRaRaw(String msg) {
  char *ppmsg;
  ppmsg = (char*)msg.c_str();
  rf95.send((uint8_t *)ppmsg, strlen(ppmsg) + 1);
  rf95.waitPacketSent();
}

void debug(String msg) {
  char *ppmsg;
  msg = "DBG:" + String(NodeID) + ":" + msg;
  ppmsg = (char*)msg.c_str();
  rf95.send((uint8_t *)ppmsg, strlen(ppmsg) + 1);
  rf95.waitPacketSent();
}

void sendLoRaIgnore(String msg) { //Thrown out at the gateway
  char *ppmsg;
  msg = "IGN:" + String(NodeID) + ":" + msg;
  ppmsg = (char*)msg.c_str();
  rf95.send((uint8_t *)ppmsg, strlen(ppmsg) + 1);
  rf95.waitPacketSent();
}

float battVoltUpdate() {
  float BATT_LVL = analogRead(BATT);
  BATT_LVL = BATT_LVL / 1024 * 2.23 * 2 + BV_OFFSET;
  return BATT_LVL;
}

bool tempUpdate() { //Make sure battery power is connected.
  tempSensors.requestTemperatures();
  for (int i = 0;  i < sensor.temp.sensorCount;  i++) {
    sensor.temp.measure[i] = tempSensors.getTempC(sensor.temp.addr[i]);
  }
  return true;
}

bool ALSUpdate() {
  double f = ads.toVoltage(1);
  int wait = ALS_POWER_WAIT - (millis() - SensWakeTime);
  delay(wait);

  for (int i = 0; i < sensor.ALS.sensorCount ; i++) {
    sensor.ALS.measure[i] = 0;
    for (int k = 0; k < ALS_AVE_COUNT; k++) {

      sensor.ALS.measure[i] += ads.readADC(i) * f * 1000;

      delay(ALS_AVE_DELAY);
    }
    sensor.ALS.measure[i] = sensor.ALS.measure[i] / ALS_AVE_COUNT; //true makes a call, only do it once per poll session
  }

  return true;
}

bool OTTUpdate() {
  String myComAdress = "?!";
  String address;
  String myComId = "0I!";
  String myComSend = "0D0!";
  String myComMeasure = "0M!";

  mySDI12.sendCommand(myComMeasure);
  while (mySDI12.available()) {
    mySDI12.read();
  }
  delay(1000);
  mySDI12.clearBuffer();
  delay(500);
  mySDI12.sendCommand(myComSend);
  delay(50);
  String rawdata = readSDI12();
  mySDI12.clearBuffer();
  delay(50);
  //Decoding string sent from probe
  int p = 0;
  int pos[] = {0, 0};
  for (int z = 0 ; z < rawdata.length() ; z++)  {
    char u = rawdata.charAt(z);
    if (u == '+' || u == '-') {
      pos[p] = z ;
      p++;
    }
    delay (50);
  }
  String level = rawdata.substring(pos[0], pos[1]);
  String temp = rawdata.substring(pos[1], rawdata.length());
  sensor.OTT.measure[0] = level.toDouble() - OTT_OFFSET;
  sensor.OTT.measure[1] = temp.toDouble();
  return true;
}

String readSDI12() {
  String sdiResponse = "";
  delay(30);
  while (mySDI12.available()) {  // write the response to the screen
    char c = mySDI12.read();
    if ((c != '\n') && (c != '\r')) {
      sdiResponse += c;
      delay(5);
    }
  }
  return sdiResponse;
}

bool RTCTempUpdate() {
  sensor.RTCTemp.measure[0] = rtc.getTemperature();
  return true;
}

bool USSUpdate() {
  digitalWrite(FET_POWER, LOW);
  delay(USS_INIT_TIME);
  digitalWrite(USS_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(USS_TRIG, HIGH);
  delayMicroseconds(20);
  digitalWrite(USS_TRIG, LOW);
  double duration = pulseIn(USS_ECHO, HIGH);
  float dist = duration / 2 * 0.000343;
  delay(50);
  sensor.USS.measure[0] = dist;   //Should return duration and have temp compensation
  digitalWrite(FET_POWER, LOW);
  return true;
}

void logDataToSD() {
  logFile = SD.open((char*)fileNameStr.c_str(), FILE_WRITE);
  // SerialUSB.println(SD.exists((char*)fileNameStr.c_str()));
  logFile.print(normTimestamp);
  logFile.println(dataString);
  // logFile.print(UNIXtimestamp);
  logFileSize = logFile.size();
  logFile.close();
  if (logFileSize == 0) {
    systemReset();
    sendLoRaIgnore("Failed to write to log, restarting system");
  }
  if (logFileSize > MAX_LOG_SIZE_BYTES) {
    logIncrement++;
    fileNameStr = fileNameGen(logIncrement);
    logFile = SD.open((char*)fileNameStr.c_str(), FILE_WRITE);
    logFile.println(CSVHeader);
    logFile.close();
  }
  // lastSDSize = logFileSize;
}

void crashNflash(int identifier) {
  while (1) {
    for (int k = 0; k < identifier; k++) {
      digitalWrite(LED, HIGH);
      delay(50);
      digitalWrite(LED, LOW);
      delay(300);
    }
    delay(2000);
  }
}

void buildTimestamps() {
  String day;
  String month;
  String hour;
  String minute;
  String second;

  DateTime now = rtc.now();
  if ((tarSec < WRAP_AROUND_S_LOWER) && (now.second() >= 30)) { //We will favor being forward in time, so this won't happen often
    delay((60 - int(now.second())) * 1000); //Wait until next second
    now = rtc.now();
    debug("Early wake, wasting power while waiting");//Wrap around occurring. Add 1 sec buffer to sleep time
    // logFile = SD.open((char*)fileNameStr.c_str(), FILE_WRITE);
    // logFile.println("Early wake, wasting power while waiting");
    // logFile.close();
  }
  if ((tarSec >= WRAP_AROUND_S_UPPER) && (now.second() <= 30)) { //We will favor being forward in time, so this won't happen often
    now = rtc.now();
    debug("Late wake, RTC time in next minute");
    // logFile = SD.open((char*)fileNameStr.c_str(), FILE_WRITE);
    // logFile.println("Late wake, RTC time in next minute");
    // logFile.close();
  }
  // if (tarSec < 10) {
  //   second = "0" + String(tarSec);
  // } else {
  //   second = String(tarSec);
  // }
  // if (now.second() < 10) {
  //   second = "0" + String(now.second(),DEC);
  // } else {
  //   second = String(now.second(),DEC);
  // }
  if (tarSec < 10) {
    second = "0" + String(tarSec, DEC);
  } else {
    second = String(tarSec, DEC);
  }
  if (now.day() < 10) {
    day = "0" + String(now.day(), DEC);
  } else {
    day = String(now.day(), DEC);
  }
  if (now.month() < 10) {
    month = "0" + String(now.month(), DEC);
  } else {
    month = String(now.month(), DEC);
  }
  if (now.hour() < 10) {
    hour = "0" + String(now.hour(), DEC);
  } else {
    hour = String(now.hour(), DEC);
  }
  if (now.minute() < 10) {
    minute = "0" + String(now.minute(), DEC);
  } else {
    minute = String(now.minute(), DEC);
  }
  normTimestamp = day + "/" + month + "/" + String(now.year(), DEC) + " " + hour + ":" + minute + ":" + second + ",";
  UNIXtimestamp = String(now.unixtime());
}

void buildCSVDataString() {
  dataString = siteID + ",";   //First "SITE_ID" identifies packet to relevant gateway
  dataString += NodeID + ",";
  dataString += String(FIRMWARE_VERSION) + ",";
  dataString += String(tx_count++, DEC) + ",";
  dataString += String(lastUpTime) + ",";
  dataString += String(logFileSize) + ",";
  dataString += String(battVoltUpdate()) + ",";

  if (sensor.rainGauge.sensorCount > 0) {
    dataString +=  String(rainfall_mm, 2) + ",";
  }
  if (sensor.temp.sensorCount > 0) {
    for (int i = 0; i < sensor.temp.sensorCount; i++) {
      // dataString += String(234.342, 2) + ",";
      dataString += String(sensor.temp.measure[i], 2) + ",";
    }
  }
  if (sensor.RTCTemp.sensorCount > 0) {
    for (int i = 0; i < sensor.RTCTemp.sensorCount; i++) {
      dataString += String(sensor.RTCTemp.measure[i], 2) + ",";
    }
  }
  if (sensor.ALS.sensorCount > 0) {
    convertALS(); //Could do this in the ALS update loop
    for (int i = 0; i < sensor.ALS.sensorCount; i++) {
      dataString += String(sensor.ALS.measure[i], 2) + ",";
      dataString += String(sensor.ALS.measure_2[i], 2) + ",";
    }
  }
  if (sensor.OTT.sensorCount > 0) {
    for (int i = 0; i < sensor.OTT.sensorCount; i++) {
      dataString += String(sensor.OTT.measure[i], 2) + ",";
    }
  }
  if (sensor.USS.sensorCount > 0) {
    for (int i = 0; i < sensor.USS.sensorCount; i++) {
      dataString += String(sensor.USS.measure[i], 2) + ",";
    }
  }
}

void configRead() {
  String configFileName = "CFG_" + String(FIRMWARE_VERSION) + ".txt";
  String key;
  String value;
  String line;
  char * pch;
  if (!SD.exists(configFileName)) {
    crashNflash(3);
  }
  configFile = SD.open((char*)configFileName.c_str());
  while (configFile.available()) {
    line = configFile.readStringUntil('\n');
    key = strtok((char*)line.c_str(), "=");
    value = strtok(NULL, ";");
    if (value != "DEFAULT" && value != NULL) {
      key.trim();
      value.trim();
      if (key == "Node_ID") {
        NodeID = value;
      } else if (key == "ALS_Slope") {
        ALSslope = value.toDouble();
      } else if (key == "ALS_Offset") {
        ALSoffset = value.toDouble();
      } else if (key == "LoRaPollOffset") {
        LoRaPollOffset = value.toInt();
      } else if (key == "PollPerLoRa") {
        pollPerLoRa = value.toInt();
      } else if (key == "Project_ID") {
        project = value;
      } else if (key == "Site_ID") {
        siteID = value;
      } else if (key == "Raingauge_mm") {
        raingaugeTip_mm = value.toFloat();
      } else if (key == "LoRa_EN") {
        LoRaEnabled = value.toInt();
      } else if (key == "SD_EN") {
        SDEnabled = value.toInt();
      } else if (key == "LoRaRepeater") {
        LoRaRepeater = value.toInt();
      } else if (key == "Nodes_to_repeat") {
        int i = 0;
        pch = strtok((char*)value.c_str(), "{},");
        while (pch != NULL) {
          Nodes_to_repeat[i] = pch;
          sendLoRaIgnore("Repeating: " + Nodes_to_repeat[i]);
          pch = strtok (NULL, ",{}");
          i++;
        }
      } else if (key == "ALS_Count") {
        sensor.ALS.sensorCount = value.toInt();
      } else if (key == "ALS_Period") {
        if (sensor.ALS.sensorCount != 0) {
          sensor.ALS.pollPeriod = value.toInt();
        }
      } else if (key == "Temp_Count") {
        // sensor.temp.sensorCount = value.toInt(); Auto detected
      } else if (key == "Temp_Period") {
        if (sensor.temp.sensorCount != 0) {
          sensor.temp.pollPeriod = value.toInt();
        }
      } else if (key == "USS_Count") {
        sensor.USS.sensorCount = value.toInt();
      } else if (key == "USS_Period") {
        if (sensor.USS.sensorCount != 0) {
          sensor.USS.pollPeriod = value.toInt();
        }
      } else if (key == "OTT_Count") {
        sensor.OTT.sensorCount = value.toInt();
      } else if (key == "OTT_Period") {
        if (sensor.OTT.sensorCount != 0) {
          sensor.OTT.pollPeriod = value.toInt();
        }
      } else if (key == "Raingauge_Count") {
        sensor.rainGauge.sensorCount = value.toInt();
      } else if (key == "Raingauge_Period") {
        if (sensor.rainGauge.sensorCount != 0) {
          sensor.rainGauge.pollPeriod = value.toInt();
        }
      } else if (key == "RTC_Temp_Count") {
        sensor.RTCTemp.sensorCount = value.toInt();
      } else if (key == "RTC_Temp_Period") {
        if (sensor.RTCTemp.sensorCount != 0) {
          sensor.RTCTemp.pollPeriod = value.toInt();
        }
      }
      else if (key == "ALS_cal_temps") {
        int i = 0;
        pch = strtok((char*)value.c_str(), "{},");
        while (pch != NULL) {
          ALSCal.temp[i] = atof(pch);
          sendLoRaIgnore("Temp " + String(i) + " = " + String(ALSCal.temp[i]));
          pch = strtok (NULL, ",{}");
          i++;
        }
      }
      else if (key == "ALS_cal_slopes") {
        int i = 0;
        pch = strtok((char*)value.c_str(), "{},");
        while (pch != NULL) {
          ALSCal.slope[i] = atof(pch);
          sendLoRaIgnore("slope " + String(i) + " = " + String(ALSCal.slope[i]));
          pch = strtok (NULL, ",{}");
          i++;
        }
      }
      else if (key == "ALS_cal_offsets") {
        int i = 0;
        pch = strtok((char*)value.c_str(), "{},");
        while (pch != NULL) {
          ALSCal.offset[i] = atof(pch);
          sendLoRaIgnore("Offset " + String(i) + " = " + String(ALSCal.offset[i]));
          pch = strtok (NULL, ",{}");
          i++;
        }
      }
    }

  }
  configFile.close();
}

void wake_system() {
  digitalWrite(FET_POWER, LOW);
  SensWakeTime = millis();
}

void sleep_system() {
  digitalWrite(FET_POWER, HIGH);
}

void disableWatchdog() {
  WDT->CTRL.reg = 0;
}

void sleepTillSynced() { //Initial clock sync
  rf95.sleep();
  delay(20);    //Delay 20ms to ensure the chips have gone to sleep before powering off the board
  WDT->CTRL.reg = 0; // disable watchdog
  DateTime now = rtc.now();
  // sleep_remaining_s = pollPeriod*60 - (now.minute()%pollPeriod)*60 - now.second();
  int relativeTimeS = (now.minute() * 60 + now.second()) % (pollPeriod * pollPerLoRa * 60);
  if (relativeTimeS > LoRaPollOffset) {
    sleep_remaining_s = pollPeriod * pollPerLoRa * 60 - relativeTimeS + LoRaPollOffset;
  }
  else if (relativeTimeS < LoRaPollOffset) {
    sleep_remaining_s = LoRaPollOffset - relativeTimeS;
  }
  sleep();
}

void convertALS() {
  double Slope;
  double Offset;
  float linearInterpRatio;

  for (int i = 0; i < sensor.ALS.sensorCount; i++) {    //Per each ALS sensor
    if (sensor.temp.measure[i] < ALSCal.temp[0]) { //Temperature below minimum
      Slope = ALSCal.slope[0];
      Offset = ALSCal.offset[0];
      // debug("Below min calibrated ALS temp");
    }
    else {
      for (int j = 1; j < MAX_ALS_CAL_POINTS - 1 ; j++) {
        if (ALSCal.temp[j] == 0) { //Temperature above maximum
          Slope = ALSCal.slope[j - 1];
          Offset = ALSCal.offset[j - 1];
          // debug("Above max calibrated ALS temp");
          break;
        }
        else if (sensor.temp.measure[i] < ALSCal.temp[j]) {
          linearInterpRatio = (sensor.temp.measure[i] - ALSCal.temp[j - 1]) / (ALSCal.temp[j] - ALSCal.temp[j - 1]);
          Slope = linearInterpRatio * (ALSCal.slope[j] - ALSCal.slope[j - 1]) + ALSCal.slope[j - 1];
          Offset = linearInterpRatio * (ALSCal.offset[j] - ALSCal.offset[j - 1]) + ALSCal.offset[j - 1];
          break;
        }
      }
    }
    sensor.ALS.measure_2[i] = sensor.ALS.measure[i] * Slope + Offset;
  }
}

bool LoRaListen(int timeout_S) {
  // Example use
  // if(LoRaListen(10)){;   //Enter 999 for no timeout
  // SerialUSB.println(LoRaReceive.RSSI);
  // SerialUSB.println(LoRaReceive.packet);}

  uint8_t len;
  int startTime = millis();
  while ((millis() - startTime) < (timeout_S * 1000) || timeout_S == 999) {
    while (rf95.available()) {
      len = sizeof(rfbuf);
      if (rf95.recv(rfbuf, &len)) {
        LoRaReceive.packet = (char*)rfbuf;
        LoRaReceive.RSSI = rf95.lastRssi();
      }
      return true;
    }
  }
  return false;
}

void bubbleSort(int a[], int arrayIndex[], int size) {
  for (int i = 0; i < (size - 1); i++) {
    for (int o = 0; o < (size - (i + 1)); o++) {
      if (a[o] > a[o + 1]) {
        int t = a[o];
        a[o] = a[o + 1];
        a[o + 1] = t;

        int tmp = arrayIndex[o];
        arrayIndex[o] = arrayIndex[o + 1];
        arrayIndex[o + 1] = tmp;
      }
    }
  }
}

void OneWireTempSetup() {
  int humanVal[4];
  digitalWrite(FET_POWER, LOW); //Turn on sensor
  delay(TEMP_INIT_TIME);  //Wait till we are warmed up
  tempSensors.begin();  // Start up the library
  sensor.temp.sensorCount = tempSensors.getDeviceCount(); //Check how many devices are attached
  sendLoRaIgnore("Temp sens count: " + String(sensor.temp.sensorCount));
  tempSensors.requestTemperatures();
  for (int i = 0;  i < sensor.temp.sensorCount;  i++) { //For each sensor, grab the value we will label with (Nibble 1 and 7 of full device address)
    tempSensors.getAddress(Thermometer, i);
    humanVal[i] = 256 * Thermometer[1] + Thermometer[2];
    sendLoRaIgnore("Temp_Address_" + String(i) + " = " + String(humanVal[i]) + ". Temp = " + String(tempSensors.getTempCByIndex(i)));
  }
  int arrayIndex[4] = {0, 1, 2, 3}; //This is how we will keep track of the order of the devices after sorting
  bubbleSort(humanVal, arrayIndex, sensor.temp.sensorCount);  //Sorts from smallest to largest. arrayIndex tells us where each value moved.
  for (int i = 0;  i < sensor.temp.sensorCount;  i++) { //For each sensor
    // tempSensors.getAddress(Thermometer, i);   //Read address
    tempSensors.getAddress(sensor.temp.addr[i], arrayIndex[i]);   //Assign addresses so that temp_1 will be the smallest address. Associate with relevant ALS probe.
    sendLoRaIgnore("HumanValOrdered_" + String(i) + " = " + String(humanVal[i]));

  }
  digitalWrite(FET_POWER, HIGH);  //Switch off temp sensor
}
