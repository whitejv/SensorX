#include  <stdio.h>
#include <string.h>
#include <Wire.h>
#include <IPAddress.h>
#include <PubSubClient.h>
//#include <ArduinoJson.h>
#include <cJSON.h>
#include <water.h>
#include <SparkFun_RV1805.h>
#include <SparkFun_Qwiic_OpenLog_Arduino_Library.h>
#include <SparkFun_Qwiic_Buzzer_Arduino_Library.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
// WebSerial removed - replaced with enhanced serial debugging

#if defined(ARDUINO_ESP8266_GENERIC) || defined(ARDUINO_ESP8266_WEMOS_D1MINI) || defined(ARDUINO_ESP8266_THING_DEV)
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
// ESPAsync libraries removed - replaced with enhanced serial debugging
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_ADS1X15.h>
#include <Arduino.h>
#include "SparkFun_TCA9534.h"
#include <BME280_Arduino_I2C.h> 
#include <Adafruit_MCP23X17.h>
#endif

#define fwVersion 1117

// Enhanced Serial Debugging System (WebSerial removed for stability)
// DEBUG mode control - enable with build flag: -D DEBUG_MODE=1 or 2
// DEBUG_MODE 1: Enhanced serial debugging with periodic status reports
// DEBUG_MODE 2: Level 1 + sensor data output to Serial with timestamps
// Serial Commands: status, diag, reset, heap
#ifndef DEBUG_MODE
#define DEBUG_MODE 0  // Default: no debug output to reduce serial traffic
#endif
/*
 * Revision Log
 *
 * 1111 - Original GenericSensor Code
 * 1112 - Fixed bug that resulted in bad flow meter pulse counts
 *        & changed Interrupt type to RISING
 * 1113 - Slowing Transmission to approx 1x sec instead of 2x sec
 * 1114 - Updated to Rev 2 IO I/F and added extended sensor logic
 * 1115 - Refactor Temp Sensor Read to one per frame
 * 1116 - Added RTC (RV1805), OpenLog, and Buzzer support for enhanced logging and alerts
 *         RTC initialized with NTP time instead of compiler time for accuracy
 *         Modified debug modes: DEBUG_MODE 0 eliminates SD card logging to reduce I2C traffic
 *         DEBUG_MODE 1+ enables SD card logging along with serial output
 * 1117 - Implemented sensor rate grouping with 100ms frames to distribute I2C sensor reads across time
 *         Added enhanced hardware watchdog timer protection for framed sensor operations
 *         Fixed sensor timing to 500ms intervals per sensor type instead of 200ms
 *         Prevented sensor reads during main loop execution to avoid interference with publishing
 *         Added buffered logging system to reduce I2C traffic: 50kHz bus speed, 10ms timeout, 5sec buffer flush
 *         Dramatically reduced I2C operations from ~20/sec to ~0.2/sec for improved sensor reliability
 *         Implemented sequential execution: publishing and logging operations separated within each 1000ms cycle
 *
 *
 */


/* Declare all constants and global variables */

IPAddress prodMqttServerIP(192, 168, 1, 250);
IPAddress devMqttServerIP(192, 168, 1, 249);

// NTP client for accurate RTC initialization
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", -18000); // UTC-5 timezone offset

// Enhanced serial debugging replaces WebSerial

// Add to global variables section

// Define I2C device addresses
const uint8_t ADS1115_ADDR = 0x49;  // 16-bit ADC
const uint8_t ADS1015_ADDR = 0x48;  // 12-bit ADC
const uint8_t GPIO_ADDR    = 0x20;  // GPIO expander
const uint8_t BME_ADDR     = 0x77;  // Atmospheric Sensor
const uint8_t RTC_ADDR     = 0x69;  // RV1805 RTC
const uint8_t OPENLOG_ADDR = 0x2A;  // OpenLog datalogger
const uint8_t BUZZER_ADDR  = 0x34;  // Qwiic Buzzer
Adafruit_ADS1015 ads1015;  // For 12-bit version
Adafruit_ADS1115 ads1115;  // For 16-bit version
TCA9534 gpio;
BME280_Arduino_I2C bme(0x77);
Adafruit_MCP23X17 mcp;
RV1805 rtc;
OpenLog openLog;
QwiicBuzzer buzzer;

bool ads1115_present = false;  // 16-bit ADC
bool ads1015_present = false;  // 12-bit ADC
bool gpio_expander_present = false;  // GPIO expander
bool gpioStatus[8] ;
bool BME280_present = false;
bool rtc_present = false;  // RV1805 RTC
bool openlog_present = false;  // OpenLog datalogger
bool openlog_healthy = true;   // OpenLog operational status
bool buzzer_present = false;  // Qwiic buzzer

// Loop timing performance monitoring
unsigned long loopStartTime = 0;
unsigned long loopExecutionTime = 0;
unsigned long minLoopTime = 999999;
unsigned long maxLoopTime = 0;

// Current log filename for diagnostics
char currentLogFilename[20] = "";

// Buffered Logging System
String logBuffer = "";
unsigned long lastLogFlush = 0;
#define LOG_FLUSH_INTERVAL 5000  // Flush every 5 seconds
#define MAX_LOG_BUFFER_SIZE 2048 // 2KB buffer limit

// Main loop execution flag (prevents sensor reads during publishing)
bool mainLoopActive = false;

// Watchdog Timer Monitoring
unsigned long lastWdtFeedTime = 0;
unsigned long wdtFeedCount = 0;

// Add to error code definitions
#define I2C_ERROR_NO_ADS1115    0x10
#define I2C_ERROR_NO_ADS1015    0x11
#define I2C_ERROR_NO_GPIO       0x12
#define I2C_ERROR_NO_BME280     0x13
#define I2C_ERROR_NO_DEVICES    0x14
#define I2C_ERROR_NO_RTC        0x15
#define I2C_ERROR_NO_OPENLOG    0x16
#define I2C_ERROR_NO_BUZZER     0x17

int extendedSensor = 0;
int sensor = 0;
int InitiateReset = 0;
int ErrState = 0;
int ErrCount = 0;
const int ERRMAX = 10;
unsigned int masterCounter = 0;
const int discInput1 = DISCINPUT1;
const int discInput2 = DISCINPUT2;
int ioInput = 0;
long currentMillis = 0;
long previousMillis1 = 0;
long previousMillis2 = 0;
long millisecond = 0;
int loopInterval = 1000; //changed from 500 to 1000 to slow loop
int flowInterval = 2000;
volatile byte pulseCount1;
volatile byte pulseCount2;
byte pulse1Sec = 0;
unsigned long timerOTA;
char messageName[50];
char messageNameJSON[50];
float temperatureF;
int fanControl = DISCINPUT1;
int tempSensorcount = 0;
WiFiClient espFlowClient;

PubSubClient P_client(espFlowClient);
PubSubClient D_client(espFlowClient);
PubSubClient client;

const int oneWireBus = TEMPSENSOR;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

/* Forward declaration of functions to solve circular dependencies */

void updateFlowData1();
void updateFlowData2();
void updateTemperatureData();
void publishFlowData();
void publishJsonData();
void printFlowData();
void printClientState(int state);
void readAnalogInput();
void readDigitalInput();
void readBME280();
void logSensorData();
void alertWiFiStatus(bool connected);
void alertMQTTStatus(bool connected);
void alertSDCardStatus();
void setupOTA();
void setupWiFi();
void connectToMQTTServer();
void checkConnectionAndLogState();

void IRAM_ATTR pulseCounter1() {
  pulseCount1++;
}void IRAM_ATTR pulseCounter2() {
  pulseCount2++;
}
const char buildTimestamp[] = __DATE__ " " __TIME__;

// Enhanced Serial Debugging Functions
void executeFramedSensorReads() {
  unsigned long currentMillis = millis();
  int frameIndex = (currentMillis / 100) % 10;  // Changes every 100ms, cycles 0-9 (500ms per sensor)

  static int lastFrameIndex = -1;

  // Skip sensor reads if main loop is currently executing (prevents interference with publishing)
  if (mainLoopActive) {
    return;
  }

  // Feed hardware watchdog before I2C operations (enhanced protection)
  ESP.wdtFeed();
  lastWdtFeedTime = millis();
  wdtFeedCount++;

  // Only execute if we're in a new frame (prevents multiple calls per 100ms frame)
  if (frameIndex != lastFrameIndex) {
    switch(frameIndex) {
      case 0:  // 0-100ms: Temperature sensors
        updateTemperatureData();
        break;

      case 1:  // 100-200ms: (unused - spacing)
        break;

      case 2:  // 200-300ms: Basic I/O (analog/digital)
        readAnalogInput();
        readDigitalInput();
        break;

      case 3:  // 300-400ms: (unused - spacing)
        break;

      case 4:  // 400-500ms: ADC sensors
        if (ads1015_present) readADS1015();
        if (ads1115_present) readADS1115();
        break;

      case 5:  // 500-600ms: (unused - spacing)
        break;

      case 6:  // 600-700ms: GPIO expander
        if (gpio_expander_present) readGPIOExpander();
        break;

      case 7:  // 700-800ms: (unused - spacing)
        break;

      case 8:  // 800-900ms: BME280 environmental sensor
        if (BME280_present) readBME280();
        break;

      case 9:  // 900-1000ms: (unused - spacing)
        break;
    }
    lastFrameIndex = frameIndex;
  }
}
// Buffered Logging Functions
void flushLogBuffer() {
  if (logBuffer.length() == 0) return;

  // Check buffer size limits
  if (logBuffer.length() > MAX_LOG_BUFFER_SIZE) {
    logMessageLn("WARNING: Log buffer overflow, forcing flush");
  }

  // Single bulk I2C write (much more efficient than individual writes)
  if (openLog.println(logBuffer)) {
    logBuffer = "";  // Clear buffer on successful write
    lastLogFlush = millis();
  } else {
    logMessageLn("WARNING: Log buffer flush failed");
  }
}

void printSystemStatus() {
  Serial.println("\n=== ESP8266 Sensor System Status ===");
  Serial.printf("Uptime: %lus, Cycle: %lu\n", millis() / 1000, masterCounter);
  Serial.printf("Loop Time: %lu/%lu/%lu us (min/current/max)\n",
                minLoopTime, loopExecutionTime, maxLoopTime);
  Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("WDT Status: %lu feeds, last feed %lu ms ago\n",
                wdtFeedCount, millis() - lastWdtFeedTime);
  #if DEBUG_MODE >= 1
  Serial.printf("Log Buffer: %d/%d bytes, last flush %lu sec ago\n",
                logBuffer.length(), MAX_LOG_BUFFER_SIZE, (millis() - lastLogFlush) / 1000);
  #endif
  Serial.printf("WiFi Status: %s (%s)\n",
                WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected",
                WiFi.localIP().toString().c_str());
  Serial.printf("MQTT Status: %s\n", client.connected() ? "Connected" : "Disconnected");

  // Send comprehensive I2C device status summary
  Serial.println("=== I2C Device Status ===");
  Serial.println("RTC (0x69): " + String(rtc_present ? "Connected" : "Not found"));
  Serial.println("OpenLog (0x2A): " + String(openlog_present ? (openlog_healthy ? "Connected" : "Connected but unhealthy") : "Not found"));
  Serial.println("Buzzer (0x34): " + String(buzzer_present ? "Connected" : "Not found"));
  Serial.println("ADS1115 (0x48): " + String(ads1115_present ? "Connected" : "Not found"));
  Serial.println("ADS1015 (0x49): " + String(ads1015_present ? "Connected" : "Not found"));
  Serial.println("GPIO Expander (0x20): " + String(gpio_expander_present ? "Connected" : "Not found"));
  Serial.println("BME280 (0x77): " + String(BME280_present ? "Connected" : "Not found"));
  Serial.println("Temperature Sensors: " + String(tempSensorcount));
  Serial.println("Sensor Configuration: " + String(sensor));
  Serial.println("Debug Mode: " + String(DEBUG_MODE));
  Serial.println("Firmware Version: " + String(fwVersion));
  Serial.println("=====================================");
}

void processSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();

    if (command == "status") {
      printSystemStatus();
    }
    else if (command == "diag" || command == "diagnose") {
      diagnoseOpenLog();
    }
    else if (command == "reset" || command == "resetstats") {
      minLoopTime = 999999;
      maxLoopTime = 0;
      wdtFeedCount = 0;
      lastWdtFeedTime = millis();
      Serial.println("Timing and WDT statistics reset!");
    }
    else if (command == "heap") {
      Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());
      Serial.printf("Heap Fragmentation: %d%%\n", ESP.getHeapFragmentation());
    }
    else if (command == "help" || command == "?") {
      Serial.println("Available commands:");
      Serial.println("  status  - Show comprehensive system status");
      Serial.println("  diag    - OpenLog diagnostics (includes file size check)");
      Serial.println("  reset   - Reset timing and WDT statistics");
      Serial.println("  heap    - Show memory usage");
      Serial.println("  help    - Show this help");
    }
    else if (command.length() > 0) {
      Serial.println("Unknown command: " + command);
      Serial.println("Type 'help' for available commands");
    }
  }
}
void setup() {
  Serial.begin(115200);
  Serial.println("Booting");
   // Print the build timestamp
  Serial.print("Program Build Timestamp: ");
  Serial.println(buildTimestamp);
  Serial.print("Debug Mode: ");
  Serial.println(DEBUG_MODE);

  // Initialize I2C with optimized settings for reliability
  Wire.begin();  // Using default ESP8266 pins - GPIO4 (SDA) and GPIO5 (SCL)
  Wire.setClock(50000);   // Reduced to 50kHz for better reliability with multiple devices
  Wire.setTimeout(10000); // 10ms timeout to prevent hanging operations
  
    // Initialize buzzer early for WiFi status feedback
  if (buzzer.begin(BUZZER_ADDR)) {
    buzzer_present = true;
    Serial.println("Buzzer initialized for WiFi feedback");
    buzzer.configureBuzzer(SFE_QWIIC_BUZZER_RESONANT_FREQUENCY, 0, SFE_QWIIC_BUZZER_VOLUME_MAX);
  } else {
    Serial.println("Buzzer not found - no audio feedback available");
  }

  // Connect to WiFi first to enable NTP time sync for RTC
  setupWiFi();

   // Initialize RTC (RV1805)
  if (rtc.begin()) {
  rtc_present = true;
  logMessageLn("RTC found at 0x69");
  // Sync with NTP time (WiFi is now connected)
  timeClient.begin();
  if (timeClient.update()) {
      time_t epochTime = timeClient.getEpochTime();
      struct tm *ptm = gmtime((time_t*)&epochTime);

          rtc.setTime(0, ptm->tm_sec, ptm->tm_min, ptm->tm_hour,
                      ptm->tm_mday, ptm->tm_mon + 1, ptm->tm_year + 1900,
                      ptm->tm_wday + 1);
          logMessageLn("RTC synced with NTP time");

      // Confirmation ping for NTP sync
      if (buzzer_present) {
        buzzer.configureBuzzer(2500, 50, SFE_QWIIC_BUZZER_VOLUME_MAX); // 2.5kHz, 50ms
        buzzer.on();
      }

          // Read back and display the RTC time to verify
          if (rtc.updateTime()) {
            logMessage("RTC time set to: ");
            logMessage(String(rtc.getYear() + 2000));
            logMessage("-");
            if (rtc.getMonth() < 10) logMessage("0");
            logMessage(String(rtc.getMonth()));
            logMessage("-");
            if (rtc.getDate() < 10) logMessage("0");
            logMessage(String(rtc.getDate()));
            logMessage(" ");
            if (rtc.getHours() < 10) logMessage("0");
            logMessage(String(rtc.getHours()));
            logMessage(":");
            if (rtc.getMinutes() < 10) logMessage("0");
            logMessage(String(rtc.getMinutes()));
            logMessage(":");
            if (rtc.getSeconds() < 10) logMessage("0");
            logMessageLn(String(rtc.getSeconds()));
          }
  } else {
      logMessageLn("NTP sync failed - RTC time not set");
  }
} else {
    Serial.println("RTC not found!");
    ErrState = I2C_ERROR_NO_RTC;
    ErrCount++;
    playErrorTone(I2C_ERROR_NO_RTC, ErrCount);
  }

  // Initialize OpenLog with enhanced device detection (after RTC for timestamp-based filenames)
  // First, check if any device responds at the OpenLog address with a simple I2C ACK check
  Wire.beginTransmission(OPENLOG_ADDR);
  uint8_t ackResult = Wire.endTransmission();
  if (ackResult == 0) {
    // Device acknowledged - now try the library initialization
    if (openLog.begin(OPENLOG_ADDR)) {
      // Additional validation: try multiple status reads for consistency
      int status1 = openLog.getStatus();
      delay(10);  // Small delay
      int status2 = openLog.getStatus();

      // Check if status is consistent and reasonable (bit 0 should be set for SD init good)
      if ((status1 == status2) && (status1 & 0x01) && (status1 <= 0x1F)) {
        openlog_present = true;
        logMessageLn("OpenLog found at 0x2A - Status: " + String(status1));

        // Create and open log file with RTC timestamp or millis fallback
        char filename[25];  // Increased size for longer filename

        if (rtc_present && rtc.updateTime()) {
          // Use RTC time for filename: LOG_YYYYMMDD_HHMMSS.TXT
          sprintf(filename, "LOG_%04d%02d%02d_%02d%02d%02d.TXT",
                  rtc.getYear() + 2000, rtc.getMonth(), rtc.getDate(),
                  rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
        } else {
          // Fallback to millis if RTC not available
          sprintf(filename, "LOG_%lu.TXT", millis()/1000);
        }

        // Create and open the file for writing
        if (openLog.append(filename)) {
          // Store the filename for diagnostics
          strcpy(currentLogFilename, filename);

          // Debug: Verify filename tracking
          Serial.print("Filename generated: ");
          Serial.println(filename);
          Serial.print("currentLogFilename set to: ");
          Serial.println(currentLogFilename);

          logMessageLn("Log file opened for writing: " + String(filename));

          // Give OpenLog time to initialize SD card operations
          delay(500);

            // Test write to verify SD card functionality
          logMessageLn("Starting OpenLog test write...");
          openLog.println("=== OpenLog Test Write ===");
          openLog.println("Timestamp: " + String(millis()));
          openLog.println("Firmware Version: " + String(fwVersion));
          openLog.println("Debug Mode: " + String(DEBUG_MODE));
          openLog.println("=== Test Write Complete ===");

          // Multiple sync attempts
          logMessageLn("Syncing test data...");
          openLog.syncFile();
          delay(100);
          openLog.syncFile(); // Second sync attempt

          // Check status after test write
          int testStatus = openLog.getStatus();
          logMessageLn("Status after test write: " + String(testStatus));

          // Test write verification completed
          logMessageLn("OpenLog initialization and test write completed");

          logMessageLn("OpenLog test write completed");
        } else {
          logMessageLn("ERROR: Failed to open log file for writing: " + String(filename));
          ErrState = I2C_ERROR_NO_OPENLOG;
          ErrCount++;
          playErrorTone(I2C_ERROR_NO_OPENLOG, ErrCount);
          openlog_present = false;
        }
      } else {
        Serial.println("OpenLog ACK check passed but status inconsistent or invalid!");
        Serial.println("Status1: " + String(status1) + ", Status2: " + String(status2));
        ErrState = I2C_ERROR_NO_OPENLOG;
        ErrCount++;
        playErrorTone(I2C_ERROR_NO_OPENLOG, ErrCount);
      }
    } else {
      Serial.println("OpenLog library begin() failed despite ACK!");
      ErrState = I2C_ERROR_NO_OPENLOG;
      ErrCount++;
      playErrorTone(I2C_ERROR_NO_OPENLOG, ErrCount);
    }
  } else {
    Serial.println("OpenLog not found - no ACK at 0x2A");
    ErrState = I2C_ERROR_NO_OPENLOG;
    ErrCount++;
    playErrorTone(I2C_ERROR_NO_OPENLOG, ErrCount);
  }

  // Enhanced serial debugging initialized
  #if DEBUG_MODE >= 1
  Serial.println("Enhanced Serial Debugging Active");
  Serial.println("Commands: status, diag, reset, heap, help");
  #endif

  /* hardcoded for now
    const int configPin1 = CONFIGPIN1;
    const int configPin3 = CONFIGPIN2;
    const int configPin2 = CONFIGPIN3;
    pinMode(configPin1, INPUT_PULLUP);
    pinMode(configPin2, INPUT_PULLUP);
    pinMode(configPin3, INPUT_PULLUP);
    pinMode(discInput1, INPUT); //Has external pull-up
    pinMode(discInput2, INPUT); //Has external pull-up
*/
    sensors.begin();// Start the DS18B20 for Temp Sensor

    pinMode(FLOWSENSOR, INPUT);
    attachInterrupt(digitalPinToInterrupt(FLOWSENSOR), pulseCounter1, RISING);
    pinMode(12, INPUT); //using one of the config pins due to too many conflicts
    attachInterrupt(digitalPinToInterrupt(12), pulseCounter2, RISING);


    // Read the config pins and get configuation data
    //sensor = digitalRead(configPin3) <<2 | digitalRead(configPin2)<<1 | digitalRead(configPin1) ;
    sensor = 6; //hardcoded unfortunately for now


  /*
   ^ Determine number of Temperature Sensors
   */
  // locate devices on the bus
  logMessage("Locating devices...");
  logMessage("Found ");
  tempSensorcount = sensors.getDeviceCount();
  logMessage(tempSensorcount, DEC);
  logMessageLn(" temperature sensors.");

  /* 
  * Sensors 0-3 are standard sensors
  * Sensors 4-7 are extended sensors with additional data words
  */
  logMessage("Sensor # ");
  logMessage(sensor);
  if (sensor >= 4){ 
    extendedSensor = 1;
    logMessage("- Extended") ;
  }
  logMessage(" Sensor ID: ");
  logMessageLn(flowSensorConfig[sensor].sensorName);

  strcpy(messageName, flowSensorConfig[sensor].messageid);
  strcpy(messageNameJSON, flowSensorConfig[sensor].jsonid);

  if ( extendedSensor == 1 ) {
  
 //Redifine Disc #2 (GPIO16) as an output for fan control
    
    pinMode(fanControl, OUTPUT); //Has external pull-up  
    
 
  // Initialize ADS1015
    if (ads1015.begin(ADS1015_ADDR)) {
        ads1015_present = true;
        logMessageLn("ADS1015 found at 0x49");
    } else {
        logMessageLn("ADS1015 not found!");
        ErrState = I2C_ERROR_NO_ADS1015;
        ErrCount++;
        playErrorTone(I2C_ERROR_NO_ADS1015, ErrCount);
    }

    // Initialize ADS1115
    if (ads1115.begin(ADS1115_ADDR)) {
        ads1115_present = true;
        logMessageLn("ADS1115 found at 0x48");
    } else {
        logMessageLn("ADS1115 not found at 0x48");
        ErrState = I2C_ERROR_NO_ADS1115;
        ErrCount++;
        playErrorTone(I2C_ERROR_NO_ADS1115, ErrCount);
    }

    if (mcp.begin_I2C()) {
        gpio_expander_present = true;
        mcp.pinMode(0, INPUT_PULLUP);
        mcp.pinMode(1, INPUT_PULLUP);
        mcp.pinMode(2, INPUT_PULLUP);
        mcp.pinMode(3, INPUT_PULLUP);
        mcp.pinMode(4, INPUT_PULLUP);
        mcp.pinMode(5, INPUT_PULLUP);
        mcp.pinMode(6, INPUT_PULLUP);
        mcp.pinMode(7, INPUT_PULLUP);
 
        logMessageLn("GPIO Expander found at 0x20");
    } else {
        logMessageLn("GPIO Expander not found at 0x20");
        ErrState = I2C_ERROR_NO_GPIO;
        ErrCount++;
        playErrorTone(I2C_ERROR_NO_GPIO, ErrCount);
    }

        // Initialize BME280
    if (bme.begin() == 0) {
        BME280_present = true;
        logMessageLn("BME280 found at 0x77");
    } else {
        logMessageLn("BME280 not found at 0x77");
        ErrState = I2C_ERROR_NO_BME280;
        ErrCount++;
        playErrorTone(I2C_ERROR_NO_BME280, ErrCount);
    }
  }

  setupOTA();
  connectToMQTTServer();

  if (client.setBufferSize(1024) == FALSE ) {
    Serial.println("Failed to allocate large MQTT send buffer - JSON messages may fail to send.");
  }
  genericSens_.generic.tempSensorcount = tempSensorcount;
  genericSens_.generic.fw_version = fwVersion ;

  // Write CSV header to log file now that setup is complete
  if (openlog_present) {
    logMessageLn("Writing CSV header to log file...");
    openLog.println("Timestamp,Millis,Flow1_Pulses,Flow1_ms,Flow1_NewData,Flow2_Pulses,Flow2_ms,Flow2_NewData,Temp1,Temp2,Temp3,Temp4,ADC0,ADC_x1,ADC_x2,ADC_x3,ADC_x4,ADC_x5,ADC_x6,ADC_x7,ADC_x8,GPIO,GPIO_x1,GPIO_x2,TempX,Humidity,Pressure,Cycle_Count,FW_Version");

    // Force sync and verify
    openLog.syncFile();

    // Small delay to ensure write completes
    delay(100);

    // Check status after header write
    int headerStatus = openLog.getStatus();
    if (headerStatus & 0x01) {  // SD init good bit
      //logMessageLn("CSV header written successfully");
    } else {
      logMessageLn("WARNING: CSV header write may have failed - Status: " + String(headerStatus));
      ErrState = I2C_ERROR_NO_OPENLOG;
      ErrCount++;
      playErrorTone(I2C_ERROR_NO_OPENLOG, ErrCount);
    }
  }
}

void loop() {
  // Measure loop execution time (start)
  loopStartTime = micros();

  // Execute framed sensor reads (distributed across 100ms frames)
  executeFramedSensorReads();

  ArduinoOTA.handle();

  // Enhanced Serial Debugging
  #if DEBUG_MODE >= 1
  // Process serial commands
  processSerialCommands();

  // Print status every 30 seconds
  static unsigned long last_status_print = 0;
  if ((unsigned long)(millis() - last_status_print) > 30000) {
    printSystemStatus();
    last_status_print = millis();
  }
  #endif

  if (millis() - timerOTA > loopInterval) {

     // Set flag to prevent sensor reads during main loop execution
     mainLoopActive = true;

     ESP.wdtFeed();
     lastWdtFeedTime = millis();
     wdtFeedCount++;

     // Flow sensors called every main loop (frequent, internal 2s timing)
     updateFlowData1();
     updateFlowData2();

     // I2C sensors now handled by executeFramedSensorReads() in 100ms frames

     // Fan control based on temperature (from framed sensor reads)
     if (genericSens_.generic.tempx >= 70.){
      //Serial.println("Temp exceeds 70 degrees fan should be on");
      digitalWrite(fanControl, HIGH);
     }
    else {
      //Serial.println("Temp below 70 degrees fan should be off");
      digitalWrite(fanControl, LOW);
    } 
     // Both operations execute every 1000ms main loop with temporal separation
     // Publishing first, then brief pause, then logging - maintains full data frequency

     // Phase 1: Publishing operations (MQTT data transmission)
     publishFlowData();
     publishJsonData();

     // Phase 2: Logging operations (Serial output and SD card logging)
     printFlowData();
     #if DEBUG_MODE >= 1
     logSensorData(); // Log all sensor data to SD card (only in debug modes 1+)
     #endif

    // Periodic buffer flush to ensure logged data is written to SD card (only when logging is active)
    #if DEBUG_MODE >= 1
    if (openlog_present && openlog_healthy) {
      // Force flush buffer every 10 seconds to ensure data persistence
      static unsigned long lastFlush = 0;
      if (millis() - lastFlush > 10000 || logBuffer.length() > 512) {
        flushLogBuffer();
        lastFlush = millis();
      }
    }
    #endif


     timerOTA = millis();

     // Clear flag - main loop execution complete, sensor reads can resume
     mainLoopActive = false;

     client.loop();
     
     ++masterCounter;

     genericSens_.generic.cycle_count = masterCounter;
  }
  checkConnectionAndLogState();

  // Measure loop execution time (end) and update statistics
  loopExecutionTime = micros() - loopStartTime;
  minLoopTime = min(minLoopTime, loopExecutionTime);
  maxLoopTime = max(maxLoopTime, loopExecutionTime);

}

/* Implementation of the functions */
void setupWiFi() {
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // WiFi connection attempt starting - short beep if buzzer available
  if (buzzer_present) {
    buzzer.configureBuzzer(1000, 100, SFE_QWIIC_BUZZER_VOLUME_MAX); // 1kHz, 100ms
    buzzer.on();
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Wait for connection with periodic status beeps
  unsigned long startTime = millis();
  unsigned long lastBeepTime = 0;
  const unsigned long CONNECTION_TIMEOUT = 30000; // 30 second timeout
  const unsigned long BEEP_INTERVAL = 250; // 200ms beep interval

  while (WiFi.status() != WL_CONNECTED) {
    unsigned long currentTime = millis();

    // Check for timeout
    if (currentTime - startTime > CONNECTION_TIMEOUT) {
      Serial.println("Connection timeout! Rebooting...");

      // WiFi connection failed - error tone if buzzer available
      if (buzzer_present) {
        buzzer.configureBuzzer(400, 500, SFE_QWIIC_BUZZER_VOLUME_MAX); // 400Hz, 500ms
        buzzer.on();
        delay(600); // Wait for tone to complete
      }

      delay(5000);
      ESP.restart();
    }

    // Periodic status beep during connection attempt
    if (buzzer_present && (currentTime - lastBeepTime > BEEP_INTERVAL)) {
      buzzer.configureBuzzer(600, 100, SFE_QWIIC_BUZZER_VOLUME_MAX); // 600Hz, 100ms (lower tone)
      buzzer.on();
      lastBeepTime = currentTime;
    }

    delay(100); // Small delay to prevent busy waiting
  }

  logMessageLn("");
  logMessage("WiFi connected -- ");
  logMessage("IP address: ");
  logMessage(WiFi.localIP().toString());
  logMessage(" MAC Address:  ");
  logMessageLn(WiFi.macAddress());

  // WiFi connection successful - robot "yes" sound effect
  if (buzzer_present) {
    buzzer.playSoundEffect(2, SFE_QWIIC_BUZZER_VOLUME_MAX); // "robot saying 'Yes'"
  }
}

void setupOTA(){
  /*
   * Adding OTA Support
   */
  // Port defaults to 8266
  ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
}

void connectToMQTTServer(){
  unsigned long connectAttemptStart = millis();
  bool connected = false;
  //Try connecting to the production MQTT server first

  P_client.setServer(prodMqttServerIP, PROD_MQTT_PORT);

  // Connect to the MQTT server

  while (!P_client.connected() && millis() - connectAttemptStart < 5000) { // Adjust the timeout as needed
    logMessage("Connecting to Production MQTT Server: ...");
    connected = P_client.connect(flowSensorConfig[sensor].clientid);
    if (connected) {
      client = P_client; // Assign the connected production client to the global client object
      logMessageLn("connected");

      // Production server success: higher→lower tones (1 sec each)
      if (buzzer_present) {
        buzzer.configureBuzzer(2000, 1000, SFE_QWIIC_BUZZER_VOLUME_MAX); // 2kHz, 1 sec
        buzzer.on();
        delay(1100); // Wait for tone + small gap
        buzzer.configureBuzzer(1500, 1000, SFE_QWIIC_BUZZER_VOLUME_MAX); // 1.5kHz, 1 sec
        buzzer.on();
      }
    } else {
      logMessage("failed with client state: ");
      printClientState(P_client.state());
      logMessageLn("");
      delay(2000);
    }
  }

  // If connection to the production server failed, try connecting to the development server
  if (!connected) {
    //PubSubClient client(devMqttServerIP, DEV_MQTT_PORT, espWellClient);
    D_client.setServer(devMqttServerIP, DEV_MQTT_PORT);
    while (!D_client.connected()) {
      logMessage("Connecting to Development MQTT Server...");
      connected = D_client.connect(flowSensorConfig[sensor].clientid);
      if (connected) {
        client = D_client; // Assign the connected development client to the global client object
        logMessageLn("connected");

        // Development server success: lower→higher tones (1 sec each)
        if (buzzer_present) {
          buzzer.configureBuzzer(1500, 1000, SFE_QWIIC_BUZZER_VOLUME_MAX); // 1.5kHz, 1 sec
          buzzer.on();
          delay(1100); // Wait for tone + small gap
          buzzer.configureBuzzer(2000, 1000, SFE_QWIIC_BUZZER_VOLUME_MAX); // 2kHz, 1 sec
          buzzer.on();
        }
      } else {
        logMessage("failed with client state: ");
        printClientState(D_client.state());
        logMessageLn("");

        // MQTT connection failure - play "NO" sound effect
        if (buzzer_present) {
          buzzer.playSoundEffect(4, SFE_QWIIC_BUZZER_VOLUME_MAX); // Sound effect "NO"
        }

        delay(2000);
      }
    }
  }

  // If still not connected after trying both servers, it's a complete failure
  if (!connected) {
    logMessageLn("Failed to connect to any MQTT server");

    // Final failure indication - play "NO" sound effect
    if (buzzer_present) {
      buzzer.playSoundEffect(4, SFE_QWIIC_BUZZER_VOLUME_MAX); // Sound effect "NO"
    }
  }
}
// Define the bit-packed structure
struct FlowData {
  uint32_t pulses : 12;       // 12 bits for pulse count
  uint32_t milliseconds : 19; // 19 bits for milliseconds
  uint32_t newData : 1;       // 1 bit for new data flag
};
/*
void updateFlowData1() {
  currentMillis = millis();
  if (((currentMillis - previousMillis1) > flowInterval) && pulseCount1 > 0 ) {
    pulse1Sec = pulseCount1;
    pulseCount1 = 0;
    millisecond = millis() - previousMillis1 ;
    genericSens_.generic.pulse_count = pulse1Sec ;
    genericSens_.generic.milliseconds = millisecond ;
    genericSens_.generic.new_data_flag = 1;
    previousMillis1 = millis();
  } else {
    genericSens_.generic.new_data_flag = 0 ;
  }
}

void updateFlowData2() {
  currentMillis = millis();
  if (((currentMillis - previousMillis2) > flowInterval) && pulseCount2 > 0 ) {
    pulse1Sec = pulseCount2;
    pulseCount2 = 0;
    millisecond = millis() - previousMillis2 ;
    //genericSens_.generic.pulse_countx = pulse1Sec ;
    //genericSens_.generic.millisecondsx = millisecond ;
   // genericSens_.generic.new_data_flagx = 1;
    genericSens_.generic.flowData2 = flow
    previousMillis2 = millis();
  } else {
    //genericSens_.generic.new_data_flagx = 0 ;
    genericSens_.generic.flowData2 
  }
}
*/
struct FlowData flowData1;
struct FlowData flowData2;
void updateFlowData1() {
  // Declare a variable of type FlowData
  
  static unsigned long previousMillis1 = 0;
  unsigned long currentMillis = millis();

  if (((currentMillis - previousMillis1) > flowInterval) && pulseCount1 > 0) {
    // Update the bit-packed structure
    flowData1.pulses = pulseCount1;  // Store the pulse count
    flowData1.milliseconds = currentMillis - previousMillis1;  // Store the elapsed time
    flowData1.newData = 1;  // Set the new data flag
    memcpy(&genericSens_.generic.flowData1, &flowData1, sizeof(int));
    // Reset for the next interval
    pulseCount1 = 0;
    previousMillis1 = currentMillis;
  } else {
    // No new data, clear the new data flag
    flowData1.newData = 0;
    memcpy(&genericSens_.generic.flowData1, &flowData1, sizeof(int));
  }
  //Serial.print("New Data: ");
  //Serial.println(flowData1.newData ? "Yes" : "No");

  //Serial.print("Milliseconds: ");
  //Serial.println(flowData1.milliseconds);

  //Serial.print("Pulses: ");
  //Serial.println(flowData1.pulses);
}
void updateFlowData2() {
  // Declare a variable of type FlowData
  
  static unsigned long previousMillis2 = 0;
  unsigned long currentMillis = millis();

  if (((currentMillis - previousMillis2) > flowInterval) && pulseCount2 > 0) {
    // Update the bit-packed structure
    flowData2.pulses = pulseCount2;  // Store the pulse count
    flowData2.milliseconds = currentMillis - previousMillis2;  // Store the elapsed time
    flowData2.newData = 1;  // Set the new data flag
    memcpy(&genericSens_.generic.flowData2, &flowData2, sizeof(int));
    // Reset for the next interval
    pulseCount2 = 0;
    previousMillis2 = currentMillis;
  } else {
    // No new data, clear the new data flag
    flowData2.newData = 0;
    memcpy(&genericSens_.generic.flowData2, &flowData2, sizeof(int));
  }
  //Serial.print("New Data: ");
  //Serial.println(flowData2.newData ? "Yes" : "No");

  //Serial.print("Milliseconds: ");
  //Serial.println(flowData2.milliseconds);

  //Serial.print("Pulses: ");
  //Serial.println(flowData2.pulses);
}

void updateTemperatureData() {
    // Static variable to keep track of the current sensor index. It's initialized to 0 only once.
    static int currentSensorIndex = 0;

    // Request temperature conversion for all sensors. This is a non-blocking call.
    sensors.requestTemperatures(); 

    // Read the temperature of the current sensor
    float temperatureF = sensors.getTempFByIndex(currentSensorIndex);

    // Update the correct structure member based on the current sensor index.
    switch (currentSensorIndex) {
        case 0:
            genericSens_.generic.temp1 = (int)temperatureF;
            memcpy(&genericSens_.generic.temp1_f, &temperatureF, sizeof(temperatureF));
            break;
        case 1:
            genericSens_.generic.temp2 = temperatureF;
            break;
        case 2:
            genericSens_.generic.temp3 = temperatureF;
            break;
        case 3:
            genericSens_.generic.temp4 = temperatureF;
            break;
        // Default case to handle any other sensor index, although we're only dealing with 4 here.
        default:
            break;
    }

    // Increment the index and use the modulo operator to loop back to 0.
    if (tempSensorcount > 0) {  // Check if there are any temperature sensors
      currentSensorIndex = (currentSensorIndex + 1) % tempSensorcount;
    }
}

void readAnalogInput() {
  const int analogInPin = A0; 
  genericSens_.generic.adc_sensor = analogRead(analogInPin);
  //Serial.println(flow_data_payload[3]);
}


void readDigitalInput() {

  // Read the config pins and get configuation data
  //Serial.print(digitalRead(discInput1));
  //Serial.print(digitalRead(discInput2));
  ioInput = digitalRead(discInput2)<<1 | digitalRead(discInput1) ;
  genericSens_.generic.gpio_sensor = ioInput ;
  //Serial.print("IO Input: ");
  //Serial.println(ioInput);
}
  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

void readADS1015(void) {
  int adc0, adc1, adc2, adc3;
  float volts0, volts1, volts2, volts3;
  adc0 = ads1015.readADC_SingleEnded(0);
  adc1 = ads1015.readADC_SingleEnded(1);
  adc2 = ads1015.readADC_SingleEnded(2);
  adc3 = ads1015.readADC_SingleEnded(3);
  volts0 = ads1015.computeVolts(adc0);
  volts1 = ads1015.computeVolts(adc1);
  volts2 = ads1015.computeVolts(adc2);
  volts3 = ads1015.computeVolts(adc3);
  genericSens_.generic.adc_x1 = volts0 ;
  genericSens_.generic.adc_x2 = volts1 ;
  genericSens_.generic.adc_x3 = volts2 ;
  genericSens_.generic.adc_x4 = volts3 ;
}
void readADS1115(void) {
  int adc0, adc1, adc2, adc3;
  float volts0, volts1, volts2, volts3;
  adc0 = ads1115.readADC_SingleEnded(0);
  adc1 = ads1115.readADC_SingleEnded(1);
  adc2 = ads1115.readADC_SingleEnded(2);
  adc3 = ads1115.readADC_SingleEnded(3);
  volts0 = ads1115.computeVolts(adc0);
  volts1 = ads1115.computeVolts(adc1);
  volts2 = ads1115.computeVolts(adc2);
  volts3 = ads1115.computeVolts(adc3);
  genericSens_.generic.adc_x5 = volts0 ;
  genericSens_.generic.adc_x6 = volts1 ;
  genericSens_.generic.adc_x7 = volts2 ;
  genericSens_.generic.adc_x8 = volts3 ;
}

void readGPIOExpander(void) {
  // Read all GPIO pins at once
  uint8_t portAValue = mcp.readGPIO(0);  // Using the simpler uint8_t return method
  genericSens_.generic.GPIO_x1 = portAValue;
  uint8_t portBValue = mcp.readGPIO(1);  // Using the simpler uint8_t return method
  genericSens_.generic.GPIO_x2 = portBValue;

}

void logSensorData() {
    // Only log if OpenLog is available and healthy
    if (!openlog_present || !openlog_healthy) return;

    // Quick status check before logging
    int currentStatus = openLog.getStatus();
    if (!(currentStatus & 0x01)) {  // SD init good bit not set
      logMessageLn("WARNING: OpenLog SD card not ready for logging - Status: " + String(currentStatus));
      return;
    }

    // Additional check - ensure file is still open (bit 3)
    if (!(currentStatus & 0x08)) {
      logMessageLn("WARNING: OpenLog file not open for logging - Status: " + String(currentStatus));
      // Try to continue anyway, but log the issue
    }

    // Get current timestamp from RTC
    String timestamp = "";
    if (rtc_present && rtc.updateTime() == true) {
        char timeStr[20];
        sprintf(timeStr, "%04d-%02d-%02d %02d:%02d:%02d",
                rtc.getYear() + 2000, rtc.getMonth(), rtc.getDate(),
                rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
        timestamp = String(timeStr);
    } else {
        // Fallback to millis if RTC not available
        timestamp = String(millis());
    }

    // Extract flow data
    struct FlowData flow1, flow2;
    memcpy(&flow1, &genericSens_.generic.flowData1, sizeof(int));
    memcpy(&flow2, &genericSens_.generic.flowData2, sizeof(int));

    // Build CSV line with all sensor data
    String csvLine = timestamp;
    csvLine += ",";
    csvLine += String(millis());  // Add millis for precise timestamp
    csvLine += ",";
    csvLine += String(flow1.pulses);
    csvLine += ",";
    csvLine += String(flow1.milliseconds);
    csvLine += ",";
    csvLine += String(flow1.newData);
    csvLine += ",";
    csvLine += String(flow2.pulses);
    csvLine += ",";
    csvLine += String(flow2.milliseconds);
    csvLine += ",";
    csvLine += String(flow2.newData);
    csvLine += ",";
    csvLine += String(genericSens_.generic.temp1);
    csvLine += ",";
    csvLine += String(genericSens_.generic.temp2, 2);
    csvLine += ",";
    csvLine += String(genericSens_.generic.temp3, 2);
    csvLine += ",";
    csvLine += String(genericSens_.generic.temp4, 2);
    csvLine += ",";
    csvLine += String(genericSens_.generic.adc_sensor);
    csvLine += ",";
    csvLine += String(genericSens_.generic.adc_x1, 3);
    csvLine += ",";
    csvLine += String(genericSens_.generic.adc_x2, 3);
    csvLine += ",";
    csvLine += String(genericSens_.generic.adc_x3, 3);
    csvLine += ",";
    csvLine += String(genericSens_.generic.adc_x4, 3);
    csvLine += ",";
    csvLine += String(genericSens_.generic.adc_x5, 3);
    csvLine += ",";
    csvLine += String(genericSens_.generic.adc_x6, 3);
    csvLine += ",";
    csvLine += String(genericSens_.generic.adc_x7, 3);
    csvLine += ",";
    csvLine += String(genericSens_.generic.adc_x8, 3);
    csvLine += ",";
    csvLine += String(genericSens_.generic.gpio_sensor);
    csvLine += ",";
    csvLine += String(genericSens_.generic.GPIO_x1);
    csvLine += ",";
    csvLine += String(genericSens_.generic.GPIO_x2);
    csvLine += ",";
    csvLine += String(genericSens_.generic.tempx, 2);
    csvLine += ",";
    csvLine += String(genericSens_.generic.humidity, 2);
    csvLine += ",";
    csvLine += String(genericSens_.generic.pressurex, 4);
    csvLine += ",";
    csvLine += String(genericSens_.generic.cycle_count);
    csvLine += ",";
    csvLine += String(genericSens_.generic.fw_version);

    // Buffer the CSV line instead of immediate write (for reduced I2C traffic)
    logBuffer += csvLine + "\n";

    // Flush buffer if it's getting large or periodic flush needed
    if (logBuffer.length() > 1024 || millis() - lastLogFlush > LOG_FLUSH_INTERVAL) {
      flushLogBuffer();
    }

    // Debug: Check file size occasionally to verify writes are working
    static int debugWriteCount = 0;
    debugWriteCount++;
    if (debugWriteCount >= 10) {  // Check every 10 writes
      debugWriteCount = 0;

      // We need to track the current filename - for now just check if we have data
      int postWriteStatus = openLog.getStatus();
      if (!(postWriteStatus & 0x01)) {
        logMessageLn("CRITICAL: SD card status bad after data write - Status: " + String(postWriteStatus));
      } else {
        // Only log in debug mode to avoid spam
        #if DEBUG_MODE >= 2
        logMessageLn("Data write OK - Cycle: " + String(masterCounter) + ", Status: " + String(postWriteStatus));
        #endif
      }
    }

    // Periodic detailed status check (every 50 cycles to avoid spam)
    static int logCycleCount = 0;
    logCycleCount++;
    if (logCycleCount >= 50) {
      logCycleCount = 0;
      int postWriteStatus = openLog.getStatus();
      if (!(postWriteStatus & 0x01)) {
        logMessageLn("WARNING: Potential SD card write issue detected - Status: " + String(postWriteStatus));
      } else {
        // Optional: Log successful writes in debug mode
        #if DEBUG_MODE >= 1
        logMessageLn("SD card write OK - Cycle: " + String(masterCounter));
        #endif
      }
    }
}

// Dual logging function - writes to Serial and OpenLog (if available)
void logMessage(const char* message) {
  Serial.print(message);
  if (openlog_present && openlog_healthy) {
    openLog.print(message);
    // Don't sync on partial messages for performance
  }
}

void logMessage(const String& message) {
  Serial.print(message);
  if (openlog_present && openlog_healthy) {
    openLog.print(message);
    // Don't sync on partial messages for performance
  }
}

void logMessageLn(const char* message) {
  Serial.println(message);
  if (openlog_present && openlog_healthy) {
    openLog.println(message);
    openLog.syncFile(); // Sync complete lines to SD card
  }
}

void logMessageLn(const String& message) {
  Serial.println(message);
  if (openlog_present && openlog_healthy) {
    openLog.println(message);
    openLog.syncFile(); // Sync complete lines to SD card
  }
}

void logMessage(int value) {
  Serial.print(value);
  if (openlog_present && openlog_healthy) {
    openLog.print(value);
  }
}

void logMessage(int value, int format) {
  Serial.print(value, format);
  if (openlog_present && openlog_healthy) {
    openLog.print(value, format);
  }
}

void logMessageLn(int value) {
  Serial.println(value);
  if (openlog_present && openlog_healthy) {
    openLog.println(value);
    openLog.syncFile(); // Sync complete lines to SD card
  }
}

void alertWiFiStatus(bool connected) {
    // Placeholder - implementation will be added later
}

void playErrorTone(int errorType, int errorCount) {
  if (!buzzer_present) return;  // No buzzer available

  // Different tones for different error types
  switch(errorType) {
    case I2C_ERROR_NO_OPENLOG:
    case I2C_ERROR_NO_RTC:
    case I2C_ERROR_NO_ADS1115:
    case I2C_ERROR_NO_ADS1015:
    case I2C_ERROR_NO_GPIO:
    case I2C_ERROR_NO_BME280:
      // I2C device errors - short, high-pitched beep
      buzzer.configureBuzzer(2000, 150, SFE_QWIIC_BUZZER_VOLUME_MAX); // 2kHz, 150ms
      buzzer.on();
      break;

    default:
      // MQTT/connection errors - longer, lower tone
      buzzer.configureBuzzer(800, 300, SFE_QWIIC_BUZZER_VOLUME_MAX); // 800Hz, 300ms
      buzzer.on();
      break;
  }

  // Additional beep for repeated errors (errorCount > 1)
  if (errorCount > 1) {
    delay(200);  // Brief pause
    buzzer.configureBuzzer(1200, 100, SFE_QWIIC_BUZZER_VOLUME_MAX); // 1.2kHz, 100ms
    buzzer.on();
  }
}

void alertMQTTStatus(bool connected) {
    // Placeholder - implementation will be added later
}

void alertSDCardStatus() {
    // Placeholder - implementation will be added later
}

// Diagnostic function to check OpenLog file contents (call from serial command)
void diagnoseOpenLog() {
  if (!openlog_present) {
    logMessageLn("OpenLog not available for diagnosis");
    return;
  }

  logMessageLn("=== OpenLog Diagnostic ===");
  logMessageLn("OpenLog Present: " + String(openlog_present ? "Yes" : "No"));
  logMessageLn("OpenLog Healthy: " + String(openlog_healthy ? "Yes" : "No"));

  // Debug: Show current filename variable contents
  Serial.print("currentLogFilename variable contains: '");
  Serial.print(currentLogFilename);
  Serial.println("'");

  if (strlen(currentLogFilename) > 0) {
    logMessageLn("Current Log File: " + String(currentLogFilename));
  } else {
    logMessageLn("Current Log File: None");
  }

  // Check current status
  int status = openLog.getStatus();
  logMessageLn("Current Status: " + String(status) + " (binary: " + String(status, BIN) + ")");

  // Check if SD card is initialized
  if (status & 0x01) {
    logMessageLn("SD Card: Initialized (bit 0 set)");
  } else {
    logMessageLn("SD Card: NOT initialized (bit 0 clear)");
  }

  // Check if file is open
  if (status & 0x08) {
    logMessageLn("File Status: File is currently open (bit 3 set)");
  } else {
    logMessageLn("File Status: No file currently open (bit 3 clear)");
  }

  // Check directory location
  if (status & 0x10) {
    logMessageLn("Directory: In root directory (bit 4 set)");
  } else {
    logMessageLn("Directory: In subdirectory (bit 4 clear)");
  }

  // File size checking removed for safety - causes system hangs

  logMessageLn("=== End Diagnostic ===");
}

void readBME280(void) {
      // Read measurements from the sensor
    BME280Data* bmedata = bme.read();
    float tempx;
    float pressurex;
    tempx = ((bmedata->temperature*9/5) + 32);
    pressurex = bmedata->pressure * 0.0002953 ;
    genericSens_.generic.tempx = tempx;
    genericSens_.generic.humidity = bmedata->humidity;
    genericSens_.generic.pressurex = pressurex ;

    // Check if data is received. If data could not be received, data would be a null pointer
    //if (bmedata != nullptr) {
    //    Serial.print("> Temperature (F): ");
    //    Serial.println((bmedata->temperature*9/5) + 32);
    //    Serial.print("> Humidity (%): ");
    //    Serial.println(bmedata->humidity);
    //    Serial.print("> Pressure (Pa): ");
    //    Serial.println(bmedata->pressure * 0.0002953 );
    //}
}
void publishFlowData() {

  client.publish(flowSensorConfig[sensor].messageid, (byte *)genericSens_.data_payload, flowSensorConfig[sensor].messagelen*4);
}

void publishJsonData() {

    int i;
    struct FlowData flow1;
    struct FlowData flow2;
    
    // Create a new cJSON object
    cJSON *jsonDoc = cJSON_CreateObject();

    // Access the bit-packed structure directly
    // Assuming genericSens_.generic.flowData2 is of type struct FlowData
    memcpy(&flow1, &genericSens_.generic.flowData1, sizeof(int));
    memcpy(&flow2, &genericSens_.generic.flowData2, sizeof(int));

    // Add the bit-field values to the JSON object
    cJSON_AddNumberToObject(jsonDoc, "X001D:W1_NewData ", flow1.newData);
    cJSON_AddNumberToObject(jsonDoc, "X001D:W1_mSeconds", flow1.milliseconds);
    cJSON_AddNumberToObject(jsonDoc, "X001D:W1_Pulses  ", flow1.pulses);
    cJSON_AddNumberToObject(jsonDoc, "X001D:W2_NewData ", flow2.newData);
    cJSON_AddNumberToObject(jsonDoc, "X001D:W2_mSeconds", flow2.milliseconds);
    cJSON_AddNumberToObject(jsonDoc, "X001D:W2_Pulses  ", flow2.pulses);
    //Serial.printf("message length   %d", flowSensorConfig[sensor].messagelen);
    for (i = 2; i < 12; i++) {
        // Add data to the cJSON object
        cJSON_AddNumberToObject(jsonDoc, genericsens_ClientData_var_name[i], genericSens_.data_payload[i]);
    }

    cJSON_AddNumberToObject(jsonDoc, genericsens_ClientData_var_name[12], genericSens_.generic.adc_x1);
    cJSON_AddNumberToObject(jsonDoc, genericsens_ClientData_var_name[13], genericSens_.generic.adc_x2);
    cJSON_AddNumberToObject(jsonDoc, genericsens_ClientData_var_name[14], genericSens_.generic.adc_x3);
    cJSON_AddNumberToObject(jsonDoc, genericsens_ClientData_var_name[15], genericSens_.generic.adc_x4);
    cJSON_AddNumberToObject(jsonDoc, genericsens_ClientData_var_name[16], genericSens_.generic.adc_x5);
    cJSON_AddNumberToObject(jsonDoc, genericsens_ClientData_var_name[17], genericSens_.generic.adc_x6);
    cJSON_AddNumberToObject(jsonDoc, genericsens_ClientData_var_name[18], genericSens_.generic.adc_x7);
    cJSON_AddNumberToObject(jsonDoc, genericsens_ClientData_var_name[19], genericSens_.generic.adc_x8);
    cJSON_AddNumberToObject(jsonDoc, genericsens_ClientData_var_name[20], genericSens_.generic.tempx);
    cJSON_AddNumberToObject(jsonDoc, genericsens_ClientData_var_name[21], genericSens_.generic.pressurex);
    cJSON_AddNumberToObject(jsonDoc, genericsens_ClientData_var_name[22], genericSens_.generic.humidity);
    cJSON_AddNumberToObject(jsonDoc, genericsens_ClientData_var_name[23], genericSens_.generic.temp2);
    cJSON_AddNumberToObject(jsonDoc, genericsens_ClientData_var_name[24], genericSens_.generic.temp3);
    cJSON_AddNumberToObject(jsonDoc, genericsens_ClientData_var_name[25], genericSens_.generic.temp4);
  
   cJSON_AddNumberToObject(jsonDoc, "Sensor: ", sensor);
   cJSON_AddStringToObject(jsonDoc, "Build: ", buildTimestamp);
    // Serialize the cJSON object to a string
    char *jsonBuffer = cJSON_Print(jsonDoc);
    if (jsonBuffer != NULL) {
        size_t n = strlen(jsonBuffer);
        //Serial.printf(flowSensorConfig[sensor].jsonid);
        //Serial.printf("   %d", n);
        //Serial.printf("\n");
        //Serial.printf(jsonBuffer);
        //Serial.printf("\n");
        
        // Publish the JSON data
        if (client.publish(flowSensorConfig[sensor].jsonid, jsonBuffer, n) == FALSE) {
          Serial.printf("JSON Message Failed to Publish");
          Serial.printf("\n");
        }
        // Free the serialized data buffer
        free(jsonBuffer);
    }

    // Delete the cJSON object
    cJSON_Delete(jsonDoc);
}

void printFlowData() {
  #if DEBUG_MODE >= 2
  // Print sensor data to Serial terminal when debug mode >= 2 with timestamps

  // Add timestamp
  if (rtc_present && rtc.updateTime() == true) {
    char timeStr[20];
    sprintf(timeStr, "%04d-%02d-%02d %02d:%02d:%02d",
            rtc.getYear() + 2000, rtc.getMonth(), rtc.getDate(),
            rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
    Serial.printf("[%s] ", timeStr);
  } else {
    Serial.printf("[%lu] ", millis());
  }

  Serial.printf(messageName);
  //Serial.printf("message length   %d",flowSensorConfig[sensor].messagelen);
  for (int i = 0; i<=5; ++i) {
    Serial.printf(" %x", genericSens_.data_payload[i]);
  }  for (int i = 5; i<=8; ++i) {
    Serial.printf(" %d", genericSens_.data_payload[i]);
  }
    for (int i = 9; i<=11; ++i) {
    Serial.printf(" %x", genericSens_.data_payload[i]);
  }
  Serial.printf(" %.3f", genericSens_.generic.adc_x1);
  Serial.printf(" %.3f", genericSens_.generic.adc_x2);
  Serial.printf(" %.3f", genericSens_.generic.adc_x3);
  Serial.printf(" %.3f", genericSens_.generic.adc_x4);
  Serial.printf(" %.3f", genericSens_.generic.adc_x5);
  Serial.printf(" %.3f", genericSens_.generic.adc_x6);
  Serial.printf(" %.3f", genericSens_.generic.adc_x7);
  Serial.printf(" %.3f", genericSens_.generic.adc_x8);
  Serial.printf(" %.3f", genericSens_.generic.tempx);
  Serial.printf(" %.3f", genericSens_.generic.pressurex);
  Serial.printf(" %.3f", genericSens_.generic.humidity);
  Serial.printf(" %.3f", genericSens_.generic.temp2);
  Serial.printf(" %.3f", genericSens_.generic.temp3);
  Serial.printf(" %.3f", genericSens_.generic.temp4);
  Serial.printf("\n");
  #endif

  #if DEBUG_MODE >= 3
  // Also output sensor data to WebSerial when debug mode >= 3
  WebSerial.printf(messageName);
  for (int i = 0; i<=5; ++i) {
    WebSerial.printf(" %x", genericSens_.data_payload[i]);
  }  for (int i = 5; i<=8; ++i) {
    WebSerial.printf(" %d", genericSens_.data_payload[i]);
  }
    for (int i = 9; i<=11; ++i) {
    WebSerial.printf(" %x", genericSens_.data_payload[i]);
  }
  WebSerial.printf(" %.3f", genericSens_.generic.adc_x1);
  WebSerial.printf(" %.3f", genericSens_.generic.adc_x2);
  WebSerial.printf(" %.3f", genericSens_.generic.adc_x3);
  WebSerial.printf(" %.3f", genericSens_.generic.adc_x4);
  WebSerial.printf(" %.3f", genericSens_.generic.adc_x5);
  WebSerial.printf(" %.3f", genericSens_.generic.adc_x6);
  WebSerial.printf(" %.3f", genericSens_.generic.adc_x7);
  WebSerial.printf(" %.3f", genericSens_.generic.adc_x8);
  WebSerial.printf(" %.3f", genericSens_.generic.tempx);
  WebSerial.printf(" %.3f", genericSens_.generic.pressurex);
  WebSerial.printf(" %.3f", genericSens_.generic.humidity);
  WebSerial.printf(" %.3f", genericSens_.generic.temp2);
  WebSerial.printf(" %.3f", genericSens_.generic.temp3);
  WebSerial.printf(" %.3f", genericSens_.generic.temp4);
  WebSerial.printf("\n");
  #endif
}
void printClientState(int state) {
  switch (state) {
    case -4:
      Serial.println("MQTT_CONNECTION_TIMEOUT");
      break;
    case -3:
      Serial.println("MQTT_CONNECTION_LOST");
      break;
    case -2:
      Serial.println("MQTT_CONNECT_FAILED");
      break;
    case -1:
      Serial.println("MQTT_DISCONNECTED");
      break;
    case  0:
      Serial.println("MQTT_CONNECTED");
      break;
    case  1:
      Serial.println("MQTT_CONNECT_BAD_PROTOCOL");
      break;
    case  2:
      Serial.println("MQTT_CONNECT_BAD_CLIENT_ID");
      break;
    case  3:
      Serial.println("MQTT_CONNECT_UNAVAILABLE");
      break;
    case  4:
      Serial.println("MQTT_CONNECT_BAD_CREDENTIALS");
      break;
    case  5:
      Serial.println("MQTT_CONNECT_UNAUTHORIZED");
      break;
  }
}
void checkConnectionAndLogState(){
    if (client.connected() == FALSE) {
    ErrState = client.state() ;
    ++ErrCount;
    Serial.printf(messageName);
    Serial.print("-- Disconnected from MQTT:");
    Serial.print("Error Count:  ");
    Serial.print(ErrCount);
    Serial.print("Error Code:  ");
    Serial.println(ErrState);

    // Audio cue for MQTT disconnection
    playErrorTone(ErrState, ErrCount);
  }

  if ( ErrCount > ERRMAX ) {
    //Initiate Reset - use a distinctive error pattern
    Serial.println("Initiate board reset!!") ;
    if (buzzer_present) {
      // Three rapid beeps before reset
      for (int i = 0; i < 3; i++) {
        buzzer.configureBuzzer(1500, 200, SFE_QWIIC_BUZZER_VOLUME_MAX);
        buzzer.on();
        delay(300);
      }
    }
    while(1);
  }
}