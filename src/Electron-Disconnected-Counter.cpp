/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "/Users/chipmc/Documents/Maker/Particle/Projects/Electron-Disconnected-Counter/src/Electron-Disconnected-Counter.ino"
/*
* Project Electron Disconnected Counter - purpose build for no-connectivity use cases
* Description: Cellular Connected Data Logger for Utility and Solar powered installations
* Author: Chip McClelland
* Date:September 1st 2020
*/

/* This is an accomodation for situations where there is no collular connectivity.  Connection will only 
be by pressing the user button while powering on.  This gies the possiblility of a remote update or minor 
configuration update.
*/

//v1.00 - Adapted from the Electron Connected Counter Baseline


// Particle Product definitions
void setup();
void loop();
void recordCount();
void recordHourlyData();
void writeToDataLog();
void initializeDataLog();
void takeMeasurements();
void getSignalStrength();
void getBatteryContext();
int getTemperature();
void sensorISR();
void watchdogISR();
void petWatchdog();
int setPowerConfig();
bool enableCharging(bool enableCharge);
void loadSystemDefaults();
void checkSystemValues();
bool connectToParticle();
bool disconnectFromParticle();
bool notConnected();
int resetFRAM(String command);
int resetCounts(String command);
int hardResetNow(String command);
int setDisconnectedMode(String command);
int setSolarMode(String command);
int setSensorType(String command);
int setverboseMode(String command);
int setTimeZone(String command);
int setOpenTime(String command);
int setCloseTime(String command);
int setLowPowerMode(String command);
void publishStateTransition(void);
void fullModemReset();
int setDSTOffset(String command);
bool isDSTusa();
bool isDSTnz();
#line 17 "/Users/chipmc/Documents/Maker/Particle/Projects/Electron-Disconnected-Counter/src/Electron-Disconnected-Counter.ino"
PRODUCT_ID(11878);                                  // Boron Connected Counter Header
PRODUCT_VERSION(1);
#define DSTRULES isDSTusa
char currentPointRelease[6] ="1.00";

namespace FRAM {                                    // Moved to namespace instead of #define to limit scope
  enum Addresses {
    versionAddr           = 0x00,                   // Version of the FRAM memory map
    systemStatusAddr      = 0x01,                   // Where we store the system status data structure
    currentCountsAddr     = 0x50,                   // Where we store the current counts data structure
    hourlyCountsAddr      = 0x80                    // Where we store the hourly totals       
  };
};

const int FRAMversionNumber = 1;                    // Increment this number each time the memory map is changed

struct systemStatus_structure {                     // currently 14 bytes long
  uint8_t structuresVersion;                        // Version of the data structures (system and data)
  uint8_t disconnectedLogger;                       // Allows us to toggle between connected and disconnected modes 
  uint8_t metricUnits;                              // Status of key system states
  uint8_t connectedStatus;
  uint8_t verboseMode;
  uint8_t solarPowerMode;
  uint8_t lowPowerMode;
  uint8_t lowBatteryMode;
  int stateOfCharge;                                // Battery charge level
  uint8_t powerState;                               // Stores the current power state
  int resetCount;                                   // reset count of device (0-256)
  float timezone;                                   // Time zone value -12 to +12
  float dstOffset;                                  // How much does the DST value change?
  int openTime;                                     // Hour the park opens (0-23)
  int closeTime;                                    // Hour the park closes (0-23)
  unsigned long lastHookResponse;                   // Last time we got a valid Webhook response
  uint8_t sensorType;                                // What is the sensor type - 0-Pressure Sensor, 1-PIR Sensor, 2-Legacy Pressure Senssor
} sysStatus;

struct currentCounts_structure {                    // currently 10 bytes long
  int hourlyCount;                                  // In period hourly count
  int hourlyCountInFlight;                          // In flight and waiting for Ubidots to confirm
  int dailyCount;                                   // In period daily count
  unsigned long lastCountTime;                      // When did we record our last count
  int temperature;                                  // Current Temperature
  int alertCount;                                   // What is the current alert count
  int maxMinValue;                                  // Highest count in one minute in the current period
} current;

struct hourlyCounts_structure {
  unsigned long startingTimeStamp;                  // When did we start counting each day
  int dailyCount;                                   // Total count for the day
  int hourlyCount[24];                              // Array to hold counts for each hour in the day
  int minStateOfCharge;                             // Minimum battery value that day
  int maxStateOfCharge;                             // Maximum battery value that day
} hourlies;

// Atomic vairables - values are set and get atomically for use with ISR
std::atomic<uint32_t> hourlyAtomic;
std::atomic<uint32_t> dailyAtomic;

// Included Libraries
#include "electrondoc.h"                            // Pinout Documentation File
#include "MB85RC256V-FRAM-RK.h"                     // Rickkas Particle based FRAM Library
#include <atomic>

// Prototypes and System Mode calls
SYSTEM_MODE(SEMI_AUTOMATIC);                        // This will enable user code to start executing automatically.
SYSTEM_THREAD(ENABLED);                             // Means my code will not be held up by Particle processes.
STARTUP(System.enableFeature(FEATURE_RESET_INFO));
SystemSleepConfiguration config;                    // Initialize new Sleep 2.0 Api
MB85RC64 fram(Wire, 0);                             // Rickkas' FRAM library

// State Maching Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, SLEEPING_STATE, NAPPING_STATE, REPORTING_STATE};
char stateNames[8][14] = {"Initialize", "Error", "Idle", "Sleeping", "Napping", "Reporting", "Response Wait" };
State state = INITIALIZATION_STATE;
State oldState = INITIALIZATION_STATE;

// Pin Constants - Boron Carrier Board v1.2a
const int tmp36Pin =      A0;                       // Simple Analog temperature sensor
const int dataLogResetPin = A2;
const int wakeUpPin =     A7;                       // This is the Particle Electron WKP pin
const int tmp36Shutdwn =  B5;                       // Can turn off the TMP-36 to save energy
const int hardResetPin =  D4;                       // Power Cycles the Electron and the Carrier Board
const int donePin =       D6;                       // Pin the Electron uses to "pet" the watchdog
const int blueLED =       D7;                       // This LED is on the Electron itself
const int userSwitch =    D5;                       // User switch with a pull-up resistor
// Pin Constants - Sensor

const int intPin =        B1;                       // Pressure Sensor inerrupt pin
const int disableModule = B3;                       // For Production Devices - Bringining this low turns on the sensor (pull-up on sensor board)
// const int disableModule = B2;                       // For Electron-Dev - Bringining this low turns on the sensor (pull-up on sensor board)
const int ledPower =      B4;                       // Allows us to control the indicator LED on the sensor board

// Timing Variables
const int wakeBoundary = 1*3600 + 0*60 + 0;         // 1 hour 0 minutes 0 seconds
unsigned long stayAwakeTimeStamp = 0;
const unsigned long resetWait = 30000;              // How long will we wait in ERROR_STATE until reset
unsigned long resetTimeStamp = 0;                   // Resets - this keeps you from falling into a reset loop
char currentOffsetStr[10];                          // What is our offset from UTC
int currentHourlyPeriod = 0;                        // Need to keep this separate from time so we know when to report

// Program Variables
volatile bool watchdogFlag;                         // Flag to let us know we need to pet the dog
bool dataInFlight = false;                          // Tracks if we have sent data but not yet cleared it from counts until we get confirmation
char SignalString[64];                              // Used to communicate Wireless RSSI and Description
char batteryContextStr[16];                         // Tracks the battery context
char lowPowerModeStr[6];                            // In low power mode?
bool systemStatusWriteNeeded = false;               // Keep track of when we need to write
bool currentCountsWriteNeeded = false;
bool hourliesCountsWriteNeeded = false;
char sensorTypeConfigStr[16];

// This section is where we will initialize sensor specific variables, libraries and function prototypes
// Pressure Sensor Variables
volatile bool sensorDetect = false;                 // This is the flag that an interrupt is triggered

void setup()                                        // Note: Disconnected Setup()
{

  Serial1.begin(115200);
  /* Setup is run for three reasons once we deploy a sensor:
       1) When you deploy the sensor
       2) Each hour while the device is sleeping
       3) After a reset event
    All three of these have some common code - this will go first then we will set a conditional
    to determine which of the three we are in and finish the code
  */
  pinMode(wakeUpPin,INPUT);                         // This pin is active HIGH
  pinMode(userSwitch,INPUT);                        // Momentary contact button on board for direct user input
  pinMode(blueLED, OUTPUT);                         // declare the Blue LED Pin as an output
  pinMode(donePin,OUTPUT);                          // Allows us to pet the watchdog
  pinResetFast(hardResetPin);
  pinMode(hardResetPin,OUTPUT);                     // For a hard reset active HIGH
  pinMode(tmp36Shutdwn,OUTPUT);                     // Turn on the temp sensor
  pinSetFast(tmp36Shutdwn);                         // The sensor draws only 50uA so will just leave it on.
  //pinSetFast(dataLogResetPin);
  pinMode(dataLogResetPin,OUTPUT);
  digitalWrite(dataLogResetPin,HIGH);

  // Pressure / PIR Module Pin Setup
  pinMode(intPin,INPUT_PULLDOWN);                   // pressure sensor interrupt
  pinMode(disableModule,OUTPUT);                    // Turns on the module when pulled low
  pinResetFast(disableModule);                      // Turn on the module - send high to switch off board
  pinMode(ledPower,OUTPUT);                         // Turn on the lights
  pinSetFast(ledPower);                             // Turns on the LED on the pressure sensor board

  digitalWrite(blueLED,HIGH);

  petWatchdog();                                    // Pet the watchdog - This will reset the watchdog time period
  attachInterrupt(wakeUpPin, watchdogISR, RISING);  // The watchdog timer will signal us and we have to respond

  Particle.variable("HourlyCount", current.hourlyCount);                // Define my Particle variables
  Particle.variable("DailyCount", current.dailyCount);                  // Note: Don't have to be connected for any of this!!!
  Particle.variable("Signal", SignalString);
  Particle.variable("Release",currentPointRelease);
  Particle.variable("stateOfChg", sysStatus.stateOfCharge);
  Particle.variable("OpenTime",sysStatus.openTime);
  Particle.variable("CloseTime",sysStatus.closeTime);
  Particle.variable("Alerts",current.alertCount);
  Particle.variable("TimeOffset",currentOffsetStr);
  Particle.variable("BatteryContext",batteryContextStr);
  Particle.variable("SensorStatus",sensorTypeConfigStr);

  Particle.function("resetFRAM", resetFRAM);                          // These are the functions exposed to the mobile app and console
  Particle.function("resetCounts",resetCounts);
  Particle.function("HardReset",hardResetNow);
  Particle.function("Set-Timezone",setTimeZone);
  Particle.function("Set-DSTOffset",setDSTOffset);
  Particle.function("Set-OpenTime",setOpenTime);
  Particle.function("Set-Close",setCloseTime);
  Particle.function("Set-SensorType",setSensorType);
  Particle.function("Set-Disconnected", setDisconnectedMode);

  // Load FRAM and reset variables to their correct values
  fram.begin();                                                       // Initialize the FRAM module

  byte tempVersion;
  fram.get(FRAM::versionAddr, tempVersion);
  if (tempVersion != FRAMversionNumber) {                             // Check to see if the memory map in the sketch matches the data on the chip
    fram.erase();                                                     // Reset the FRAM to correct the issue
    fram.put(FRAM::versionAddr, FRAMversionNumber);                   // Put the right value in
    loadSystemDefaults();                                             // Out of the box, we need the device to be awake and connected
  }
  else fram.get(FRAM::systemStatusAddr,sysStatus);                    // Loads the System Status array from FRAM

  checkSystemValues();                                                // Make sure System values are all in valid range

  if (System.resetReason() == RESET_REASON_PIN_RESET || System.resetReason() == RESET_REASON_USER) { // Check to see if we are starting from a pin reset or a reset in the sketch
    sysStatus.resetCount++;
    systemStatusWriteNeeded = true;                                    // If so, store incremented number - watchdog must have done This
  }

  (sysStatus.lowPowerMode) ? strcpy(lowPowerModeStr,"True") : strcpy(lowPowerModeStr,"False");

  if (sysStatus.sensorType == 0) strcpy(sensorTypeConfigStr,"Pressure Sensor");
  else if (sysStatus.sensorType == 1) strcpy(sensorTypeConfigStr,"PIR Sensor");
  else strcpy(sensorTypeConfigStr,"Legacy Sensor");

  Time.setDSTOffset(sysStatus.dstOffset);                              // Set the value from FRAM if in limits
  DSTRULES() ? Time.beginDST() : Time.endDST();    // Perform the DST calculation here
  Time.zone(sysStatus.timezone);                                       // Set the Time Zone for our device

  snprintf(currentOffsetStr,sizeof(currentOffsetStr),"%2.1f UTC",(Time.local() - Time.now()) / 3600.0);   // Load the offset string

  // Done with the System Stuff - now load the current counts
  fram.get(FRAM::currentCountsAddr, current);
  if (current.hourlyCount) currentHourlyPeriod = Time.hour(current.lastCountTime);
  else currentHourlyPeriod = Time.hour();                              // The local time hourly period for reporting purposes

  setPowerConfig();                                                    // Executes commands that set up the Power configuration between Solar and DC-Powered

  if (!digitalRead(userSwitch)) {
    if (sysStatus.disconnectedLogger) setDisconnectedMode("0");        // If we want to make the device connect for updates / configuration - or switch it back to disconnected
    else setDisconnectedMode("1");
  }

  // Here is where the code diverges based on why we are running Setup()
  // Deterimine when the last counts were taken check when starting test to determine if we reload values or start counts over
  
  if (Time.day() != Time.day(current.lastCountTime)) resetCounts("1"); // Zero the counts for the new day

  if ((Time.hour() >= sysStatus.closeTime || Time.hour() < sysStatus.openTime)) {} // The park is closed - don't connect
  else {                                                              // Park is open let's get ready for the day
    hourlyAtomic.store(0,std::memory_order_relaxed);
    dailyAtomic.store(0,std::memory_order_relaxed);
    attachInterrupt(intPin, sensorISR, RISING);                       // Pressure Sensor interrupt from low to high
    if (sysStatus.connectedStatus && !Particle.connected()) connectToParticle(); // Only going to connect if we are in connectionMode
    takeMeasurements();                                               // Populates values so you can read them before the hour
  }

  pinResetFast(ledPower);                                             // Turns off the LED on the sensor board

  if (state == INITIALIZATION_STATE) state = IDLE_STATE;              // IDLE unless otherwise from above code

  digitalWrite(blueLED,LOW);
}

void loop()
{
  switch(state) {
  case IDLE_STATE:                                                    // Where we spend most time - note, the order of these conditionals is important
    if (watchdogFlag) petWatchdog();                                  // Watchdog flag is raised - time to pet the watchdog
    if (sensorDetect) recordCount();                                  // The ISR had raised the sensor flag
    if (sysStatus.disconnectedLogger && millis() - stayAwakeTimeStamp > 1000) state = NAPPING_STATE;          // We will always nap between counts
    if (Time.hour() != currentHourlyPeriod) state = REPORTING_STATE;  // We want to report on the hour but not after bedtime
    if ((Time.hour() >= sysStatus.closeTime || Time.hour() < sysStatus.openTime)) state = SLEEPING_STATE;   // The park is closed - sleep
    break;

  case SLEEPING_STATE: {                                              // This state is triggered once the park closes and runs until it opens
    detachInterrupt(intPin);                                          // Done sensing for the day
    pinSetFast(disableModule);                                        // Turn off the pressure module for the hour
    if (current.hourlyCount) {                                        // If this number is not zero then we need to send this last count
      state = REPORTING_STATE;
      break;
    }
    if (sysStatus.connectedStatus) disconnectFromParticle();          // Disconnect cleanly from Particle
    digitalWrite(blueLED,LOW);                                        // Turn off the LED
    petWatchdog();
    int wakeInSeconds = constrain(wakeBoundary - Time.now() % wakeBoundary, 1, wakeBoundary);
    System.sleep(SLEEP_MODE_DEEP,wakeInSeconds);                      // Very deep sleep until the next hour
    } break;

  case NAPPING_STATE: {  // This state puts the device in low power mode quickly
    if (sensorDetect) break;                                          // Don't nap until we are done with event
    if (sysStatus.connectedStatus) disconnectFromParticle();          // If we are in connected mode we need to Disconnect from Particle
    int wakeInSeconds = constrain(wakeBoundary - Time.now() % wakeBoundary, 1, wakeBoundary);
    petWatchdog();                                                    // Reset the watchdog timer interval
    config.mode(SystemSleepMode::STOP).gpio(userSwitch,CHANGE).gpio(intPin,RISING).duration(wakeInSeconds * 1000).flag(SystemSleepFlag::WAIT_CLOUD);
    SystemSleepResult result = System.sleep(config);                  // Put the device to sleep
    if (result.wakeupPin() == userSwitch) initializeDataLog();        // Simply puts the headers on the new file after a memory card swap
    else if (result.wakeupPin() == intPin) stayAwakeTimeStamp = millis(); // Keeps us from napping too fase interferes with charging
    state = IDLE_STATE;                                               // Back to the IDLE_STATE after a nap - not enabling updates here as napping is typicallly disconnected
    } break;

  case REPORTING_STATE:
      takeMeasurements();                                             // Update Temp, Battery and Signal Strength values
      recordHourlyData();
      writeToDataLog();
      state = IDLE_STATE;                                             // Wait for Response
    break;

  case ERROR_STATE:                                                   // To be enhanced - where we deal with errors
    if (sysStatus.verboseMode && state != oldState) publishStateTransition();
    if (millis() > resetTimeStamp + resetWait) {
      if (sysStatus.resetCount <= 3) {                                          // First try simple reset
        if (Particle.connected()) Particle.publish("State","Error State - Reset", PRIVATE);    // Brodcast Reset Action
        delay(2000);
        System.reset();
      }
      else {                                                          // If we have had 3 resets - time to do something more
        if (Particle.connected()) Particle.publish("State","Error State - Full Modem Reset", PRIVATE);            // Brodcase Reset Action
        delay(2000);
        sysStatus.resetCount = 0;                                     // Zero the ResetCount
        systemStatusWriteNeeded=true;
        fullModemReset();                                             // Full Modem reset and reboots
      }
    }
    break;
  }
  if (systemStatusWriteNeeded) {
    fram.put(FRAM::systemStatusAddr,sysStatus);
    systemStatusWriteNeeded = false;
  }
  if (currentCountsWriteNeeded) {
    fram.put(FRAM::currentCountsAddr,current);
    currentCountsWriteNeeded = false;
  }
  if (hourliesCountsWriteNeeded) {
    fram.put(FRAM::hourlyCountsAddr,hourlies);
    hourliesCountsWriteNeeded = false;
  }
}

void recordCount() // This is where we check to see if an interrupt is set when not asleep or act on a tap that woke the Arduino
{
  static byte currentMinutePeriod;                                    // Current minute

  pinSetFast(blueLED);                                                // Turn on the blue LED

  if (currentMinutePeriod != Time.minute()) {                       // Done counting for the last minute
    currentMinutePeriod = Time.minute();                            // Reset period
    current.maxMinValue = 1;                                        // Reset for the new minute
  }
  current.maxMinValue++;

  current.lastCountTime = Time.now();
  current.hourlyCount += hourlyAtomic.fetch_and(0,std::memory_order_relaxed);   // Increment the hourlyCount from the atomic variable
  current.dailyCount += dailyAtomic.fetch_and(0,std::memory_order_relaxed);    // Increment the dailyCount from the atomic vairable

  currentCountsWriteNeeded = true;                                    // Write updated values to FRAM
  pinResetFast(blueLED);                                              // Turn off the blue LED
  sensorDetect = false;                                               // Reset the flag

  // This is diagnostic code
  /*
  if (sysStatus.stateOfCharge > hourlies.maxStateOfCharge) hourlies.maxStateOfCharge = sysStatus.stateOfCharge;
  if (sysStatus.stateOfCharge < hourlies.minStateOfCharge) hourlies.minStateOfCharge = sysStatus.stateOfCharge;
  hourlies.hourlyCount[Time.hour()] = current.hourlyCount;
  hourlies.dailyCount = current.dailyCount;
  hourliesCountsWriteNeeded = true;
  writeToDataLog();
  */
  // End diagnostic code
}

void recordHourlyData() {
  if (sysStatus.stateOfCharge > hourlies.maxStateOfCharge) hourlies.maxStateOfCharge = sysStatus.stateOfCharge;
  if (sysStatus.stateOfCharge < hourlies.minStateOfCharge) hourlies.minStateOfCharge = sysStatus.stateOfCharge;
  hourlies.hourlyCount[Time.hour()] = current.hourlyCount;
  current.hourlyCount = 0;                                            // reset each hour
  hourlies.dailyCount = current.dailyCount;
  hourliesCountsWriteNeeded = true;
  currentCountsWriteNeeded = true;
  currentHourlyPeriod = Time.hour();
}

void writeToDataLog() {
  char data[256];
  snprintf(data, sizeof(data), "%i/%i, %i, %i, %i", Time.month(hourlies.startingTimeStamp), Time.day(hourlies.startingTimeStamp), hourlies.dailyCount, hourlies.maxStateOfCharge, hourlies.minStateOfCharge);
  for (int i=0; i <24; i++) {
    strcat(data, ", ");
    strcat(data, String(hourlies.hourlyCount[i]));
  }
  Serial1.println(data);
}

void initializeDataLog() {
  char data[256];
  snprintf(data, sizeof(data), "Time, Daily, Max, Min, 12AM, 1AM, 2AM, 3AM, 4AM, 5AM, 6AM, 7AM, 8AM, 9AM, 10AM, 11AM, 12PM, 1PM, 2PM, 3PM, 4PM, 5PM, 6PM, 7PM, 8PM, 9PM, 10PM, 11PM");
  Serial1.println(data);
  delay(50);
}


// These are the functions that are part of the takeMeasurements call
void takeMeasurements()
{
  if (Cellular.ready()) getSignalStrength();                          // Test signal strength if the cellular modem is on and ready
  getTemperature();                                                   // Get Temperature at startup as well
  getBatteryContext();                                                // What is the battery up to?
  sysStatus.stateOfCharge = int(System.batteryCharge());             // Percentage of full charge
  systemStatusWriteNeeded=true;
}


void getSignalStrength() {
  const char* radioTech[10] = {"Unknown","None","WiFi","GSM","UMTS","CDMA","LTE","IEEE802154","LTE_CAT_M1","LTE_CAT_NB1"};
  // New Signal Strength capability - https://community.particle.io/t/boron-lte-and-cellular-rssi-funny-values/45299/8
  CellularSignal sig = Cellular.RSSI();

  auto rat = sig.getAccessTechnology();

  //float strengthVal = sig.getStrengthValue();
  float strengthPercentage = sig.getStrength();

  //float qualityVal = sig.getQualityValue();
  float qualityPercentage = sig.getQuality();

  snprintf(SignalString,sizeof(SignalString), "%s S:%2.0f%%, Q:%2.0f%% ", radioTech[rat], strengthPercentage, qualityPercentage);
}

void getBatteryContext() {
  const char* batteryContext[7] ={"Unknown","Not Charging","Charging","Charged","Discharging","Fault","Diconnected"};
  // Battery conect information - https://docs.particle.io/reference/device-os/firmware/boron/#batterystate-

  snprintf(batteryContextStr, sizeof(batteryContextStr),"%s", batteryContext[System.batteryState()]);

}

int getTemperature()
{
  int reading = analogRead(tmp36Pin);                                 //getting the voltage reading from the temperature sensor
  float voltage = reading * 3.3;                                      // converting that reading to voltage, for 3.3v arduino use 3.3
  voltage /= 4096.0;                                                  // Electron is different than the Arduino where there are only 1024 steps
  int temperatureC = int(((voltage - 0.5) * 100));                    // converting from 10 mv per degree with 500 mV offset to degrees ((voltage - 500mV) times 100) - 5 degree calibration
  current.temperature = int((temperatureC * 9.0 / 5.0) + 32.0);       // now convert to Fahrenheit
  currentCountsWriteNeeded=true;
  return current.temperature;
}

// Here are the various hardware and timer interrupt service routines
void sensorISR()
{
  static bool frontTireFlag = false;
  if (frontTireFlag || sysStatus.sensorType == 1) {                   // Counts the rear tire for pressure sensors and once for PIR
    sensorDetect = true;                                              // sets the sensor flag for the main loop
    hourlyAtomic.fetch_add(1, std::memory_order_relaxed);
    dailyAtomic.fetch_add(1, std::memory_order_relaxed);
    frontTireFlag = false;
  }
  else if (sysStatus.sensorType == 2) {
    static long unsigned lastCountMillis = 0;
    if (millis() - lastCountMillis >= 700) {                          // Set a standard 700mSec debounce
      sensorDetect = true;                                            // sets the sensor flag for the main loop
      hourlyAtomic.fetch_add(1, std::memory_order_relaxed);
      dailyAtomic.fetch_add(1, std::memory_order_relaxed);
      lastCountMillis = millis(); 
    }
  }
  else frontTireFlag = true;
}

void watchdogISR()
{
  watchdogFlag = true;
}

void petWatchdog()
{
  digitalWriteFast(donePin, HIGH);                                        // Pet the watchdog
  digitalWriteFast(donePin, LOW);
  watchdogFlag = false;
}


// Power Management function
int setPowerConfig() {
  SystemPowerConfiguration conf;
  System.setPowerConfiguration(SystemPowerConfiguration());  // To restore the default configuration

  if (sysStatus.solarPowerMode) {
    conf.powerSourceMaxCurrent(550) // Set maximum current the power source can provide (applies only when powered through VIN)
        .powerSourceMinVoltage(5080) // ** was 5080 *** Set minimum voltage the power source can provide (applies only when powered through VIN)
        .batteryChargeCurrent(512) // Set battery charge current
        .batteryChargeVoltage(4208) // Set battery termination voltage
        .feature(SystemPowerFeature::USE_VIN_SETTINGS_WITH_USB_HOST); // For the cases where the device is powered through VIN
                                                                     // but the USB cable is connected to a USB host, this feature flag
                                                                     // enforces the voltage/current limits specified in the configuration
                                                                     // (where by default the device would be thinking that it's powered by the USB Host)
    int res = System.setPowerConfiguration(conf); // returns SYSTEM_ERROR_NONE (0) in case of success
    enableCharging(true);
    return res;
  }
  else  {
    conf.powerSourceMaxCurrent(900)                                   // default is 900mA 
        .powerSourceMinVoltage(4208)                                  // This is the default value for the Boron
        .batteryChargeCurrent(900)                                    // higher charge current from DC-IN when not solar powered
        .batteryChargeVoltage(4112)                                   // default is 4.112V termination voltage
        .feature(SystemPowerFeature::USE_VIN_SETTINGS_WITH_USB_HOST) ;
    int res = System.setPowerConfiguration(conf); // returns SYSTEM_ERROR_NONE (0) in case of success
    enableCharging(true);
    return res;
  }
}

bool enableCharging(bool enableCharge)
{
  PMIC pmic(true);
  if(enableCharge) {
    pmic.enableCharging();
    return TRUE;
  }
  else {
    pmic.disableCharging();
    return FALSE;
  }
}

void loadSystemDefaults() {                                         // Default settings for the device - connected, not-low power and always on
  connectToParticle();                                              // Get connected to Particle - sets sysStatus.connectedStatus to true
  takeMeasurements();                                               // Need information to set value here - sets sysStatus.stateOfCharge
  if (Particle.connected()) Particle.publish("Mode","Loading System Defaults", PRIVATE);
  sysStatus.structuresVersion = 2;
  sysStatus.metricUnits = false;
  sysStatus.verboseMode = false;
  if (sysStatus.stateOfCharge < 30) sysStatus.lowBatteryMode = true;
  else sysStatus.lowBatteryMode = false;
  setLowPowerMode("0");
  sysStatus.timezone = -5;                                          // Default is East Coast Time
  sysStatus.dstOffset = 1;
  sysStatus.openTime = 6;
  sysStatus.closeTime = 21;
  sysStatus.sensorType = 0;
  strcpy(sensorTypeConfigStr,"Pressure Sensor");
  fram.put(FRAM::systemStatusAddr,sysStatus);                       // Write it now since this is a big deal and I don't want values over written
}

void checkSystemValues() {                                          // Checks to ensure that all system values are in reasonable range
  takeMeasurements();                                               // Sets the sysStatus.stateOfCharge
  if (sysStatus.metricUnits < 0 || sysStatus.metricUnits >1) sysStatus.metricUnits = 0;
  if (sysStatus.connectedStatus < 0 || sysStatus.connectedStatus > 1) {
    if (Particle.connected()) sysStatus.connectedStatus = true;
    else sysStatus.connectedStatus = false;
  }
  if (sysStatus.sensorType > 2) {
    sysStatus.sensorType = 0;
    strcpy(sensorTypeConfigStr,"Pressure Sensor");
  }
  if (sysStatus.lowBatteryMode < 0 || sysStatus.lowBatteryMode > 1) sysStatus.lowBatteryMode = 0;
  if (sysStatus.stateOfCharge < 30) sysStatus.lowBatteryMode = true;
  else sysStatus.lowBatteryMode = false;
  if (sysStatus.resetCount < 0 || sysStatus.resetCount > 255) sysStatus.resetCount = 0;
  if (sysStatus.timezone < -12 || sysStatus.timezone > 12) sysStatus.timezone = -5;
  if (sysStatus.dstOffset < 0 || sysStatus.dstOffset > 2) sysStatus.dstOffset = 1;
  if (sysStatus.openTime < 0 || sysStatus.openTime > 12) sysStatus.openTime = 0;
  if (sysStatus.closeTime < 12 || sysStatus.closeTime > 24) sysStatus.closeTime = 24;

  systemStatusWriteNeeded = true;
}

 // These are the particle functions that allow you to configure and run the device
 // They are intended to allow for customization and control during installations
 // and to allow for management.

bool connectToParticle() {
  Cellular.on();
  Particle.connect();
  // wait for *up to* 5 minutes
  for (int retry = 0; retry < 300 && !waitFor(Particle.connected,1000); retry++) {
    if(sensorDetect) recordCount(); // service the interrupt every 10 seconds
    Particle.process();
  }
  if (Particle.connected()) {
    sysStatus.connectedStatus = true;
    systemStatusWriteNeeded = true;
    return 1;                               // Were able to connect successfully
  }
  else {
    return 0;                                                    // Failed to connect
  }
}

bool disconnectFromParticle()                                     // Ensures we disconnect cleanly from Particle
{
  Particle.disconnect();
  waitFor(notConnected, 15000);                                   // make sure before turning off the cellular modem
  Cellular.off();
  sysStatus.connectedStatus = false;
  systemStatusWriteNeeded = true;
  delay(2000);                                                    // Bummer but only should happen once an hour
  return true;
}

bool notConnected() {                                             // Companion function for disconnectFromParticle
  return !Particle.connected();
}

int resetFRAM(String command)                                     // Will reset the local counts
{
  if (command == "1")
  {
    fram.erase();
    return 1;
  }
  else return 0;
}

int resetCounts(String command)                                       // Resets the current hourly and daily counts
{
  if (command == "1")
  {
    if (Particle.connected()) Particle.publish("Reset","All counts reset",PRIVATE);

    current.dailyCount = 0;                              // Reset the counts in FRAM as well
    current.hourlyCount = 0;
    current.lastCountTime = Time.now();                      // Set the time context to the new day
    sysStatus.resetCount = current.alertCount = 0;           // Reset everything for the day

    hourlies.startingTimeStamp = Time.now();
    hourlies.maxStateOfCharge = 0;
    hourlies.minStateOfCharge = 100;
    hourlies.dailyCount = 0;
    for (int i=0; i < 24; i++) {
      hourlies.hourlyCount[i] = 0;
    }
    hourliesCountsWriteNeeded = true;
    currentCountsWriteNeeded=true;
    systemStatusWriteNeeded=true;
    return 1;
  }
  else return 0;
}

int hardResetNow(String command)   {                                    // Will perform a hard reset on the Electron
  if (command == "1")
  {
    Particle.publish("Reset","Hard Reset in 2 seconds",PRIVATE, WITH_ACK);
    delay(2000);
    digitalWrite(hardResetPin,HIGH);                                    // This will cut all power to the Electron AND the carrir board
    return 1;                                                           // Unfortunately, this will never be sent
  }
  else return 0;
}

int setDisconnectedMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    sysStatus.disconnectedLogger = true;
    sysStatus.connectedStatus = false;
    Particle.publish("Mode","Changing device to disconnected mode - User / Reset to recover",PRIVATE);
    delay (10000);
    disconnectFromParticle();
    return 1;
  }
  else if (command == "0") {
    sysStatus.disconnectedLogger = false;
    sysStatus.connectedStatus = true;
    connectToParticle();
    if (Particle.connected) Particle.publish("Mode","Changing device to connected mode - User / Reset to recover",PRIVATE);
    return 1;
  }
  else return 0;
}

int setSolarMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    sysStatus.solarPowerMode = true;
    setPowerConfig();                                               // Change the power management Settings
    systemStatusWriteNeeded=true;
    if (Particle.connected()) Particle.publish("Mode","Set Solar Powered Mode", PRIVATE);
    return 1;
  }
  else if (command == "0")
  {
    sysStatus.solarPowerMode = false;
    systemStatusWriteNeeded=true;
    setPowerConfig();                                                // Change the power management settings
    if (Particle.connected()) Particle.publish("Mode","Cleared Solar Powered Mode", PRIVATE);
    return 1;
  }
  else return 0;
}

int setSensorType(String command) // Function to force sending data in current hour
{
  if (command == "0")
  {
    sysStatus.sensorType = 0;
    strcpy(sensorTypeConfigStr,"Pressure Sensor");
    systemStatusWriteNeeded=true;
    if (Particle.connected()) Particle.publish("Mode","Set Sensor Mode to Pressure", PRIVATE);
    return 1;
  }
  else if (command == "1")
  {
    sysStatus.sensorType = 1;
    strcpy(sensorTypeConfigStr,"PIR Sensor");
    systemStatusWriteNeeded=true;
    if (Particle.connected()) Particle.publish("Mode","Set Sensor Mode to PIR", PRIVATE);
    return 1;
  }
  else if (command == "2")
  {
    sysStatus.sensorType = 2;
    strcpy(sensorTypeConfigStr,"Legacy Sensor");
    systemStatusWriteNeeded=true;
    if (Particle.connected()) Particle.publish("Mode","Set Sensor Mode to Legacy", PRIVATE);
    return 1;
  }

  else return 0;
}

int setverboseMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    sysStatus.verboseMode = true;
    systemStatusWriteNeeded = true;
    if (Particle.connected()) Particle.publish("Mode","Set Verbose Mode", PRIVATE);
    return 1;
  }
  else if (command == "0")
  {
    sysStatus.verboseMode = false;
    systemStatusWriteNeeded = true;
    if (Particle.connected()) Particle.publish("Mode","Cleared Verbose Mode", PRIVATE);
    return 1;
  }
  else return 0;
}

int setTimeZone(String command)
{
  char * pEND;
  char data[256];
  Particle.syncTime();                                                        // Set the clock each day
  waitFor(Particle.syncTimeDone,30000);                                       // Wait for up to 30 seconds for the SyncTime to complete
  int8_t tempTimeZoneOffset = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempTimeZoneOffset < -12) | (tempTimeZoneOffset > 12)) return 0;       // Make sure it falls in a valid range or send a "fail" result
  sysStatus.timezone = (float)tempTimeZoneOffset;
  Time.zone(sysStatus.timezone);
  systemStatusWriteNeeded = true;                                             // Need to store to FRAM back in the main loop
  snprintf(currentOffsetStr,sizeof(currentOffsetStr),"%2.1f UTC",(Time.local() - Time.now()) / 3600.0);
  if (Particle.connected()) {
    snprintf(data, sizeof(data), "Time zone offset %i",tempTimeZoneOffset);
    Particle.publish("Time",data, PRIVATE);
    Particle.publish("Time",Time.timeStr(Time.now()), PRIVATE);
  }

  return 1;
}

int setOpenTime(String command)
{
  char * pEND;
  char data[256];
  int tempTime = strtol(command,&pEND,10);                                    // Looks for the first integer and interprets it
  if ((tempTime < 0) || (tempTime > 23)) return 0;                            // Make sure it falls in a valid range or send a "fail" result
  sysStatus.openTime = tempTime;
  systemStatusWriteNeeded = true;                                            // Need to store to FRAM back in the main loop
  if (Particle.connected()) {
    snprintf(data, sizeof(data), "Open time set to %i",sysStatus.openTime);
    Particle.publish("Time",data, PRIVATE);
  }
  return 1;
}

int setCloseTime(String command)
{
  char * pEND;
  char data[256];
  int tempTime = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempTime < 0) || (tempTime > 24)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  sysStatus.closeTime = tempTime;
  systemStatusWriteNeeded = true;                          // Store the new value in FRAMwrite8
  snprintf(data, sizeof(data), "Closing time set to %i",sysStatus.closeTime);
  if (Particle.connected()) Particle.publish("Time",data, PRIVATE);
  return 1;
}

int setLowPowerMode(String command)                                   // This is where we can put the device into low power mode if needed
{
  if (command != "1" && command != "0") return 0;                     // Before we begin, let's make sure we have a valid input
  if (command == "1")                                                 // Command calls for setting lowPowerMode
  {
    if (Particle.connected()) {
      Particle.publish("Mode","Low Power Mode", PRIVATE);
    }
    sysStatus.lowPowerMode = true;
    strcpy(lowPowerModeStr,"True");
  }
  else if (command == "0")                                            // Command calls for clearing lowPowerMode
  {
    if (!Particle.connected()) {                                      // In case we are not connected, we will do so now.
      connectToParticle();
      sysStatus.connectedStatus = true;
    }
    Particle.publish("Mode","Normal Operations", PRIVATE);
    delay(1000);                                                      // Need to make sure the message gets out.
    sysStatus.lowPowerMode = false;                                   // update the variable used for console status
    strcpy(lowPowerModeStr,"False");                                  // Use capitalization so we know that we set this.
  }
  systemStatusWriteNeeded = true;
  return 1;
}

void publishStateTransition(void)
{
  char stateTransitionString[40];
  snprintf(stateTransitionString, sizeof(stateTransitionString), "From %s to %s", stateNames[oldState],stateNames[state]);
  oldState = state;
  if(Particle.connected()) Particle.publish("State Transition",stateTransitionString, PRIVATE);
  Serial.println(stateTransitionString);
}

void fullModemReset() {                                                 // Adapted form Rikkas7's https://github.com/rickkas7/electronsample
	Particle.disconnect(); 	                                              // Disconnect from the cloud
	unsigned long startTime = millis();  	                                // Wait up to 15 seconds to disconnect
	while(Particle.connected() && millis() - startTime < 15000) {
		delay(100);
	}
	// Reset the modem and SIM card
	// 16:MT silent reset (with detach from network and saving of NVM parameters), with reset of the SIM card
	Cellular.command(30000, "AT+CFUN=16\r\n");
	delay(1000);
	// Go into deep sleep for 10 seconds to try to reset everything. This turns off the modem as well.
	System.sleep(SLEEP_MODE_DEEP, 10);
}

int setDSTOffset(String command) {                                      // This is the number of hours that will be added for Daylight Savings Time 0 (off) - 2
  char * pEND;
  char data[256];
  time_t t = Time.now();
  int8_t tempDSTOffset = strtol(command,&pEND,10);                      // Looks for the first integer and interprets it
  if ((tempDSTOffset < 0) | (tempDSTOffset > 2)) return 0;              // Make sure it falls in a valid range or send a "fail" result
  Time.setDSTOffset((float)tempDSTOffset);                              // Set the DST Offset
  sysStatus.dstOffset = (float)tempDSTOffset;
  systemStatusWriteNeeded = true;
  snprintf(data, sizeof(data), "DST offset %2.1f",sysStatus.dstOffset);
  if (Time.isValid()) isDSTusa() ? Time.beginDST() : Time.endDST();     // Perform the DST calculation here
  snprintf(currentOffsetStr,sizeof(currentOffsetStr),"%2.1f UTC",(Time.local() - Time.now()) / 3600.0);
  if (Particle.connected()) {
    Particle.publish("Time",data, PRIVATE);
    Particle.publish("Time",Time.timeStr(t), PRIVATE);
  }
  return 1;
}

bool isDSTusa() {
  // United States of America Summer Timer calculation (2am Local Time - 2nd Sunday in March/ 1st Sunday in November)
  // Adapted from @ScruffR's code posted here https://community.particle.io/t/daylight-savings-problem/38424/4
  // The code works in from months, days and hours in succession toward the two transitions
  int dayOfMonth = Time.day();
  int month = Time.month();
  int dayOfWeek = Time.weekday() - 1; // make Sunday 0 .. Saturday 6

  // By Month - inside or outside the DST window
  if (month >= 4 && month <= 10)
  { // April to October definetly DST
    return true;
  }
  else if (month < 3 || month > 11)
  { // before March or after October is definetly standard time
    return false;
  }

  boolean beforeFirstSunday = (dayOfMonth - dayOfWeek < 0);
  boolean secondSundayOrAfter = (dayOfMonth - dayOfWeek > 7);

  if (beforeFirstSunday && !secondSundayOrAfter) return (month == 11);
  else if (!beforeFirstSunday && !secondSundayOrAfter) return false;
  else if (!beforeFirstSunday && secondSundayOrAfter) return (month == 3);

  int secSinceMidnightLocal = Time.now() % 86400;
  boolean dayStartedAs = (month == 10); // DST in October, in March not
  // on switching Sunday we need to consider the time
  if (secSinceMidnightLocal >= 2*3600)
  { //  In the US, Daylight Time is based on local time
    return !dayStartedAs;
  }
  return dayStartedAs;
}

bool isDSTnz() {
  // New Zealand Summer Timer calculation (2am Local Time - last Sunday in September/ 1st Sunday in April)
  // Adapted from @ScruffR's code posted here https://community.particle.io/t/daylight-savings-problem/38424/4
  // The code works in from months, days and hours in succession toward the two transitions
  int dayOfMonth = Time.day();
  int month = Time.month();
  int dayOfWeek = Time.weekday() - 1; // make Sunday 0 .. Saturday 6

  // By Month - inside or outside the DST window - 10 out of 12 months with April and Septemper in question
  if (month >= 10 || month <= 3)
  { // October to March is definetly DST - 6 months
    return true;
  }
  else if (month < 9 && month > 4)
  { // before September and after April is definetly standard time - - 4 months
    return false;
  }

  boolean beforeFirstSunday = (dayOfMonth - dayOfWeek < 6);
  boolean lastSundayOrAfter = (dayOfMonth - dayOfWeek > 23);

  if (beforeFirstSunday && !lastSundayOrAfter) return (month == 4);
  else if (!beforeFirstSunday && !lastSundayOrAfter) return false;
  else if (!beforeFirstSunday && lastSundayOrAfter) return (month == 9);

  int secSinceMidnightLocal = Time.now() % 86400;
  boolean dayStartedAs = (month == 10); // DST in October, in March not
  // on switching Sunday we need to consider the time
  if (secSinceMidnightLocal >= 2*3600)
  { //  In the US, Daylight Time is based on local time
    return !dayStartedAs;
  }
  return dayStartedAs;
}
