#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <SD.h>
#include <DHT.h>
#include <RTClib.h>
#include <OneButton.h>
#include <EEPROM.h>
#include <Servo.h>
#include "Relay.h"

/* ===================== EEPROM ADDRESSES  ===================== */
#define LAST_SERVO_UNIXTIME_ADDRESS 10 // EEPROM address to store last servo action time
#define LAST_SERVO_POSITION_ADDRESS 15 // EEPROM address to store last servo position
#define LAST_PUMP_UNIXTIME_ADDRESS 20  // EEPROM address to store last pump action time
#define LAST_PUMP_STATE_ADDRESS 25// EEPROM address to store last pump state
#define LAST_REF_UNIXTIME_ADDRESS 30   // EEPROM address to store last ref action time
#define LAST_DRUM_UNIXTIME_ADDRESS 40  // EEPROM address to store last drum action time
#define LAST_DRUM_STATE_ADDRESS 45    // EEPROM address to store last drum state
#define LAST_TREATMENT_ADDRESS 70     // EEPROM address to store last treatment selection
#define LAST_LOGGED_TIME_ADDRESS 80    // EEPROM address to store last data logged time

/* ===================== OPERATION VARIABLES ===================== */
#define UNDER_DEVELOPMENT 1    // 1 = dev, 0 = prod

#if UNDER_DEVELOPMENT

  #define EXHAUST_OPEN_INTERVAL_MINUTES 1
  #define EXHAUST_OPEN_DURATION_MINUTES 1
  #define DRUM_ROTATION_INTERVAL_MINUTES 1
  #define DRUM_ROTATION_DURATION_MINUTES 1
  #define DATA_LOG_INTERVAL_MINUTES 3
  #define SETPOINT_TEMPERATURE_C 50
  #define SERVO_OPEN_ANGLE 90
  #define SERVO_CLOSE_ANGLE 180

#else

  #define EXHAUST_OPEN_INTERVAL_MINUTES 30
  #define EXHAUST_OPEN_DURATION_MINUTES 1
  #define DRUM_ROTATION_INTERVAL_MINUTES 15
  #define DRUM_ROTATION_DURATION_MINUTES 1
  #define DATA_LOG_INTERVAL_MINUTES 10
  #define SETPOINT_TEMPERATURE_C 23.0
  #define SERVO_OPEN_ANGLE
  #define SERVO_CLOSE_ANGLE 90

#endif

const char *FILENAME = "DATA.CSV"; // SD card filename
const char *HEADER_LINE = "TIMESTAMP(MM/DD/YYYY-HH:MM:SS), TREATMENT, AVERAGE_TEMPERATURE, AVERAGE_HUMIDITY";
char timestamp[20];               // Formatted time string
char logLine[100];                // Full CSV log line

#define TEST_DELAY 2000UL   // 2 seconds delay for testing
#define LCD_WAIT 2000UL     // 2 seconds delay for LCD messages
#define LCD_ACTIVITY_LINE 1 // 2 seconds delay for LCD messages



/* ===================== MENU VARIABLES ===================== */
uint8_t treatment = 1;     // currently selected treatmentEEPROM ( saved)
uint8_t menuSelection = 1; // cursor in menu
bool inSettings = false;   // are we in treatment menu?


unsigned long lastUnixTime = 0;
unsigned long lastServoUnix = 0;
unsigned long lastMotorUnix = 0;
unsigned long lastLoggedUnix = 0;
unsigned long lastServoMillis = 0;
bool servoMoved = false;

bool isExhaustOpen = false;
bool isMistingPumpOn = false;
bool isRefOn = false;
bool isFanOn = false;
bool isDrumMotorOn = false;

unsigned long MistPumpStartTime = 0;
unsigned long MistPumpPulseOnDurationMs = 1000;
bool mistPulseActive = false;
unsigned long MistPumpOffTime = 0;
const unsigned long MistPumpPulseOffDurationMs = 1000;

bool lastExhaustState = false;
bool lastMistingPumpState = false;
bool lastRefState = false;
bool lastDrumMotorState = false;

String Temperature1, Temperature2, Humidity1, Humidity2;
float averageTemperature = 0.0;
float averageHumidity = 0.0;

//static unsigned long SerialLastPrint = 0;//speed limiters
 static unsigned long dhtLineLastPrint = 0;//anti flicker on lcd
// static unsigned long dhtLastRead = 0;
bool tempLineInitiated = false;
/* ===================== TREATMENT RANGES ===================== */
uint8_t treatmentMin[4] = {0, 65, 75, 85}; // index 1-3 corresponds to T1-T3
uint8_t treatmentMax[4] = {0, 75, 85, 95}; // humidity ranges maxmin


#define TREATMENT_

/* ===================== MENU TIMEOUT ===================== */
#define MENU_TIMEOUT_MS 10000UL // 10 seconds timeout

unsigned long menuLastActive = 0; // timestamp of last menu activity
/* ===================== PIN DEFINITIONS ===================== */
#define DHT1_PIN 48
#define DHT2_PIN 49

#define PUMP_RELAY_PIN 4  // active LOW
#define FAN_RELAY_PIN 5   // active LOW (always on)
#define MOTOR_RELAY_PIN 6 // active LOW
#define SERVO_RELAY_PIN 7 // active LOW
#define REF_SSR_PIN 8     // active HIGH

#define SERVO_PWM_PIN 9
#define BUTTON_PIN 3
#define SD_CS_PIN 53

/* ===================== LIB OBJECTS ===================== */
OneButton button;
Servo exhaustServo;
RTC_DS3231 rtc;

/* ===================== RELAY OBJECTS ===================== */
Relay pumpRelay(PUMP_RELAY_PIN, LOW);
Relay fanRelay(FAN_RELAY_PIN, LOW);
Relay motorRelay(MOTOR_RELAY_PIN, LOW);
Relay servoRelay(SERVO_RELAY_PIN, LOW);
Relay refSSR(REF_SSR_PIN, HIGH);

/* ===================== LCD ===================== */
#define LCD_I2C_ADDRESS 0x27
#define LCD_COLUMNS 20
#define LCD_ROWS 4
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, LCD_COLUMNS, LCD_ROWS);

byte buttonPressTimes = 0; //UNUSED
bool isLongPressed = false;
bool buttonPressed = false;
uint8_t lastTreatment = 0;

DHT dht1(DHT1_PIN, DHT22);
DHT dht2(DHT2_PIN, DHT22);

/* ===================== SETUP ===================== */

void setupOutputs()
{
  pumpRelay.begin();
  fanRelay.begin();
  motorRelay.begin();
  servoRelay.begin();
  refSSR.begin();
}

void lcdPrintLine(uint8_t line, const String &text, bool center = true, bool autoClear = false)
{
  const byte LCD_COLS = 20;
  lcd.setCursor(0, line);
  for (uint8_t i = 0; i < LCD_COLS; i++) lcd.print(' ');

  int len = min((int)text.length(), LCD_COLS);
  int pos = center ? max((LCD_COLS - len) / 2, 0) : 0;

  lcd.setCursor(pos, line);
  lcd.print(text.substring(0, len));

  if (autoClear)
  {
    delay(LCD_WAIT);
    lcd.setCursor(0, line);
    for (uint8_t i = 0; i < LCD_COLS; i++) lcd.print(' ');
  }
}

void lcdGreetings(){

  lcdPrintLine(0, "IABE-MICROCLIMATE");
  lcdPrintLine(1, "CONTROLLED CHAMBER");
  lcdPrintLine(2, "FOR MULBERRY LEAF");
  lcdPrintLine(3, "STORAGE");
}

void setupLcd()
{
  lcd.init();
  lcd.backlight();
  lcd.clear();

  delay(LCD_WAIT);
  lcd.clear();

  lcdGreetings();
  delay(5000);

  lcd.clear();
}


/* ===================== UPDATE MENU ===================== */
void updateTreatmentMenu()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SELECT TREATMENT");

  for (uint8_t i = 1; i <= 3; i++)
  {
    lcd.setCursor(0, i);
    if (menuSelection == i)
      lcd.print(">TREATMENT " + String(i) + " " + String(treatmentMin[i]) + "-" + String(treatmentMax[i]) + "%");
    else
      lcd.print(" TREATMENT " + String(i) + " " + String(treatmentMin[i]) + "-" + String(treatmentMax[i]) + "%");
  }
}

byte minimumHumidity()
{
  return treatmentMin[treatment];
}
byte maximumHumidity()
{
  return treatmentMax[treatment];
}

void showCurrentTreatment()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("CURRENT TREATMENT");
  lcd.print(" T" + String(treatment));
  //lcd.setCursor(0,1);
  //lcd.print("T" + String(treatment) + " " + String(treatmentMin[treatment]) + "-" + String(treatmentMax[treatment]) + "%");
}

/* ===================== BUTTON CALLBACKS ===================== */
void handleShortPress()
{
  if (inSettings)
  {
    {
      menuSelection++;
      if (menuSelection > 3) menuSelection = 1;
      updateTreatmentMenu();
      menuLastActive = millis(); // reset menu timeout
      Serial.println("Next");    // Serial feedback
    }
  }
}
void handleLongPress()
{
  if (!inSettings)
  {
    // Enter settings menu
    inSettings = true;
    menuSelection = treatment; // start cursor at current treatment
    updateTreatmentMenu();
    menuLastActive = millis(); // reset menu timeout
    Serial.println("Select");  // entering menu
  }
  else
  {
    // Select treatment
    treatment = menuSelection;
    EEPROM.write(LAST_TREATMENT_ADDRESS, treatment); // save selection to EEPROM
    inSettings = false;
    showCurrentTreatment();
    lastTreatment = treatment; // sync lastTreatment
    Serial.println("Select");  // selection made
  }
}
void setupButton(){
  button.setup(BUTTON_PIN, INPUT, false);       // ACTIVE LOW TRUE, ACTIVE HIGH FALSE
  button.setClickMs(50);                        // max duration for a short click
  button.setPressMs(1000);                      // min duration for long press
  button.attachClick(handleShortPress);         // short press (on release)
  button.attachLongPressStart(handleLongPress); // long press (fires once at start of long press)
}

float readDHTTemperature(DHT &dht)
{
  float temp = dht.readTemperature();
  if (isnan(temp))
  {
    //Serial.println("Failed to read temperature from DHT sensor!");
    return -999.0; // error value
  }
  return temp;
}
float readDHTHumidity(DHT &dht)
{
  float hum = dht.readHumidity();
  if (isnan(hum))
  {
    //Serial.println("Failed to read humidity from DHT sensor!");
    return -1.0; // error value
  }
  return hum;
}
float readDHT1Temperature()
{
  return readDHTTemperature(dht1);
}
float readDHT2Temperature()
{
  return readDHTTemperature(dht2);
}
float readDHT1Humidity()
{
  return readDHTHumidity(dht1);
}
float readDHT2Humidity()
{
  return readDHTHumidity(dht2);
}
bool readDHTSensorsAndSetAverage() {
  //  unsigned long now = millis();

  //if (now - dhtLastRead < 500) return false; // 1s rate limit (non-blocking)

  // dhtLastRead = now;
  float temp1 = readDHT1Temperature();
  float hum1  = readDHT1Humidity();
  float temp2 = readDHT2Temperature();
  float hum2  = readDHT2Humidity();

  // both sensors OK
  bool dht1Ok = (temp1 != -999.0 && hum1 != -1.0);
  bool dht2Ok = (temp2 != -999.0 && hum2 != -1.0);

  // both sensors failed
  if (!dht1Ok && !dht2Ok) {

    if (!inSettings) lcdPrintLine(LCD_ACTIVITY_LINE, "DHT1 & DHT2 FAIL", true);
    // Serial.println("Both DHT sensors failed!");
    Temperature1 = "ERR";
    Humidity1    = "ERR";
    Temperature2 = "ERR";
    Humidity2    = "ERR";
    return false;
  }

  // only DHT1 works
  if (dht1Ok && !dht2Ok) {
    averageTemperature = temp1;
    averageHumidity    = hum1;
    //Serial.println("DHT2 sensor failed!");
    Temperature1 = String(temp1);
    Humidity1    = String(hum1);
    Temperature2 = "ERR";
    Humidity2    = "ERR";
    return true;
  }

  // only DHT2 works
  if (!dht1Ok && dht2Ok) {
    averageTemperature = temp2;
    averageHumidity    = hum2;
    //Serial.println("DHT1 sensor failed!");
    Temperature1 = "ERR";
    Humidity1    = "ERR";
    Temperature2 = String(temp2);
    Humidity2    = String(hum2);
    return true;
  }

  // both sensors OK
  averageTemperature = (temp1 + temp2) / 2.0;
  averageHumidity    = (hum1 + hum2) / 2.0;

  Temperature1 = String(temp1);
  Humidity1    = String(hum1);
  Temperature2 = String(temp2);
  Humidity2    = String(hum2);
  // Serial.print("Average Temp: ");
  // Serial.print(averageTemperature);
  // Serial.print(" °C, Average Humidity: ");
  // Serial.print(averageHumidity);
  // Serial.println(" %");
  return true;
}

void sdCardSetup()
{
  if (!SD.begin(53))              // Initialize SD (CS pin 53 on Mega)
  {
    Serial.println("SD CARD INITIALIZATION FAILED");
    lcd.print("SD INIT FAILED");
    delay(3000);
    return;                       // Stop setup if SD fails
  }

  Serial.println("SD INITIALIZED SUCCESSFULLY");

  if (!SD.exists(FILENAME))       // If file does not exist
  {
    Serial.println("CREATING DATA FILE");
    File myFile = SD.open(FILENAME, FILE_WRITE); // Create file
    if (myFile)
    {
      myFile.println(HEADER_LINE); // Write CSV header
      myFile.close();              // Close file
    }
  }
}
void ftoa_safe(float v, char *out, size_t len)
{
  if (isnan(v))                    // If value invalid
  {
    strncpy(out, "ERR", len);      // Write ERR
    out[len - 1] = '\0';           // Ensure null-termination
  }
  else
  {
    dtostrf(v, 0, 1, out);         // Convert float to string
  }
}
void updateTimestamp() {
  DateTime now = rtc.now();
  sprintf(timestamp, "%02d/%02d/%04d-%02d:%02d:%02d",
          now.month(), now.day(), now.year(),
          now.hour(), now.minute(), now.second());
}
void updateLogLine() {
  char t[8], h[8]; // Buffers for CSV values
  //Cleaning up
  ftoa_safe(averageTemperature, t, sizeof(t)); // Convert T1
  ftoa_safe(averageHumidity, h, sizeof(h));    // Convert H1
 // Build CSV log line
  snprintf(logLine, sizeof(logLine),
        "%s,T%d,%s,%s",
        timestamp, treatment, t, h); // Build CSV line
}
void saveLogLine() {
  updateTimestamp();
  updateLogLine();
  File logFile = SD.open(FILENAME, FILE_WRITE); // Open file append mode
  if (logFile)
  {
    logFile.println(logLine);      // Write log line
    Serial.print("LOGGED: ");
    Serial.println(logLine);       // Debug output
    logFile.close();               // Close file
  }
  else
  {
    Serial.println("ERROR OPENING LOG FILE");
    lcdPrintLine (LCD_ACTIVITY_LINE, "SD WRITE ERROR", true);
    delay(2000);
  }
}

void turnOnDrumMotor()
{
  Serial.println("Turning on drum motor");
  lcdPrintLine(LCD_ACTIVITY_LINE, "TURNING ON DRUM", true);
  delay(LCD_WAIT);
  motorRelay.on();
  lcdPrintLine(LCD_ACTIVITY_LINE, "DRUM IS ON", true);
}
void turnOffDrumMotor()
{
  Serial.println("Turning off drum motor");
  lcdPrintLine(LCD_ACTIVITY_LINE, "TURNING OFF DRUM", true);
  delay(LCD_WAIT);
  motorRelay.off();
  lcdPrintLine(LCD_ACTIVITY_LINE, "DRUM IS OFF", true);
}
void turnOnMistingPump()
{
  Serial.println("Turning on misting pump");
  lcdPrintLine(LCD_ACTIVITY_LINE, "TURNING ON PUMP", true);
  delay(LCD_WAIT);
  pumpRelay.on();
  lcdPrintLine(LCD_ACTIVITY_LINE, "PUMP IS ON", true);
}

void turnOffMistingPump()
{
  Serial.println("Turning off misting pump");
  lcdPrintLine(LCD_ACTIVITY_LINE, "TURNING OFF PUMP", true);
  delay(LCD_WAIT);
  pumpRelay.off();
  lcdPrintLine(LCD_ACTIVITY_LINE, "PUMP IS OFF", true);
}

void MistPumpStart() {
  if (!isMistingPumpOn) {
  MistPumpStartTime = millis();
  turnOnMistingPump();
  isMistingPumpOn = true;
  }
}

void turnOnCoolingSystem()
{
  Serial.println("Turning on cooling system");
  lcdPrintLine(LCD_ACTIVITY_LINE, "TURNING ON COOLER", true);
  delay(LCD_WAIT);
  refSSR.on();
  fanRelay.on();
  lcdPrintLine(LCD_ACTIVITY_LINE, "COOLER IS ON", true);
}
void turnOffCoolingSystem()
{
  Serial.println("Turning off cooling system");
  lcdPrintLine(LCD_ACTIVITY_LINE, "TURNING OFF COOLER", true);
  delay(LCD_WAIT);
  refSSR.off();
  fanRelay.off();
  lcdPrintLine(LCD_ACTIVITY_LINE, "COOLER IS OFF", true);
}
//uncomment if ever needed to control fans separately
// void turnOnRecirculatingFans()
// {
//   Serial.println("Turning on recirculating fans");
//   lcdPrintLine(LCD_ACTIVITY_LINE, "TURNING ON FANS", true);
//   delay(LCD_WAIT);
//   fanRelay.on();
//   lcdPrintLine(LCD_ACTIVITY_LINE, "FANS ARE ON", true);
// }
// void turnOffRecirculatingFans()
// {
//   Serial.println("Turning off recirculating fans");
//   lcdPrintLine(LCD_ACTIVITY_LINE, "TURNING OFF FANS", true);
//   delay(LCD_WAIT);
//   fanRelay.off();
//   lcdPrintLine(LCD_ACTIVITY_LINE, "FANS ARE OFF", true);
// }
void openServo() {
    servoRelay.on();
    Serial.println("Opening Exhaust Servo");
    lcdPrintLine(LCD_ACTIVITY_LINE, "OPENING EXHAUST", true);
    delay(LCD_WAIT);

    for (int angle = SERVO_CLOSE_ANGLE; angle >= SERVO_OPEN_ANGLE; angle--) { // 180 → 0
        exhaustServo.write(angle);
        delay(20); // smooth movement
    }

    lcdPrintLine(LCD_ACTIVITY_LINE, "EXHAUST IS OPEN", true);
    servoMoved = true;
    lastServoMillis = millis();
}

void closeServo() {
    Serial.println("Closing Exhaust Servo");
    servoRelay.on();
    lcdPrintLine(LCD_ACTIVITY_LINE, "CLOSING EXHAUST", true);
    delay(LCD_WAIT);

    for (int angle = SERVO_OPEN_ANGLE; angle <= SERVO_CLOSE_ANGLE; angle++) { // 0 → 180
        exhaustServo.write(angle);
        delay(20); // smooth movement
    }

    delay(1000); // allow servo to settle
    lcdPrintLine(LCD_ACTIVITY_LINE, "EXHAUST IS CLOSED", true);
    servoMoved = true;
    lastServoMillis = millis();
}


void setupExhaustServo()
{
  exhaustServo.attach(SERVO_PWM_PIN);
  if (isExhaustOpen) {
    openServo();
  } else {
    closeServo();
  }
}
void setupDht()
{
  dht1.begin();
  dht2.begin();
}
bool statusChanged() {
  return (lastExhaustState != isExhaustOpen) ||
         (lastMistingPumpState != isMistingPumpOn) ||
         (lastRefState != isRefOn) ||
         (lastDrumMotorState != isDrumMotorOn);
}

void updateDhtLine()
{
  if (inSettings) return; // do not update DHT line if in settings menu

  unsigned long now = millis();
  if (now - dhtLineLastPrint < 1000) return;
  dhtLineLastPrint = now;

  char temperatureString[4];
  char humidityString[4];

  //if temperature reading -999, show ERR. if humidity reading -1, show ERR
  if (averageTemperature == -999) {
    strcpy(temperatureString, "ERR");
  } else {
    dtostrf(averageTemperature, 4, 1, temperatureString);
  }

  if (averageHumidity == -1) {
    strcpy(humidityString, "ERR");
  } else {
    dtostrf(averageHumidity, 4, 1, humidityString);
  }
  if(!tempLineInitiated) {
    lcd.setCursor(0, 2);
    lcd.print("TMP:    ");
    lcd.print(char(223)); // degree symbol
    lcd.print("C HUM:    %");
    tempLineInitiated = true; // mark as initiated
  }
 //print values
  lcd.setCursor(4, 2);
  lcd.print(temperatureString);
  lcd.setCursor(15, 2);
  lcd.print(humidityString);
}

  void updateStatusLine()
  {
    if (inSettings) return; // do not update status line if in settings menu

    char line[21]; // 20 chars + null terminator

    snprintf(line, sizeof(line),
    "EX:%d REF:%d MP:%d DM:%d",
    isExhaustOpen ? 1 : 0,
    refSSR.isOn() ? 1 : 0,
    pumpRelay.isOn() ? 1 : 0,
    isDrumMotorOn ? 1 : 0);
    lcdPrintLine(3, String(line), false);
  }

void rtcSetup() {
    if (!rtc.begin()) {
      Serial.println("Couldn't find RTC");
      // while (1);
    }
    if (rtc.lostPower())
    {
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }

    DateTime now = rtc.now();
    Serial.println(now.timestamp());
}

void getLastTreatmentFromEEPROM() {
  // Load treatment from EEPROM (address 1)
  uint8_t storedTreatment = EEPROM.read(LAST_TREATMENT_ADDRESS);
  if (storedTreatment >= 1 && storedTreatment <= 3)
    treatment = storedTreatment;
  else
    treatment = 1; // default if invalid

  Serial.println("Loaded Treatment: T" + String(treatment));
  // show current treatment on LCD line 2
  showCurrentTreatment();
  lastTreatment = treatment;
}

bool getLastState(byte _address){
  bool state = false;
  EEPROM.get(_address, state);
  return state;
}

unsigned long getLastUnixTime(byte _address){
  unsigned long unixTime = 0;
  EEPROM.get(_address, unixTime);
  return unixTime;
}

float minutesElapsed(unsigned long startUnix, unsigned long currentUnix) {
    unsigned long elapsedSeconds = currentUnix - startUnix;
    float elapsedMinutes = elapsedSeconds / 60;

    return elapsedMinutes;
}
void printTimestamp(unsigned long unixTime, bool newLine = false)//to Serial
{
  unsigned long seconds = unixTime % 60;
  unsigned long minutes = (unixTime / 60) % 60;
  unsigned long hours   = (unixTime / 3600) % 24;

  if (hours < 10)   Serial.print('0');
  Serial.print(hours);
  Serial.print(':');

  if (minutes < 10) Serial.print('0');
  Serial.print(minutes);
  Serial.print(':');

  if (seconds < 10) Serial.print('0');
  Serial.print(seconds);

  if (newLine)
    Serial.println();
}

void getEepromData() {



  lastMotorUnix = getLastUnixTime(LAST_DRUM_UNIXTIME_ADDRESS);
  lastLoggedUnix = getLastUnixTime(LAST_LOGGED_TIME_ADDRESS);
  // lastMistingPumpState
  isExhaustOpen = getLastState(LAST_SERVO_POSITION_ADDRESS);
  isDrumMotorOn = getLastState(LAST_DRUM_STATE_ADDRESS);

  Serial.print("LAST LOGGED TIME: ");
  printTimestamp(lastLoggedUnix);
  Serial.print(" DRUM MOTOR ");
  Serial.print(isDrumMotorOn ? "TURNED ON AT " : "TURNED OFF AT ");
  printTimestamp(lastMotorUnix);
  Serial.print(" EXHAUST ");
  Serial.print(isExhaustOpen ? "OPENED AT " : "CLOSED AT ");
  printTimestamp(lastServoUnix, true);

  // Serial.print(last);

  getLastTreatmentFromEEPROM();
}

void updateStates(){
  lastExhaustState = isExhaustOpen;
  lastMistingPumpState = isMistingPumpOn;
  lastRefState = isRefOn;
  lastDrumMotorState = isDrumMotorOn;
}

void setup()
{
  Serial.begin(9600);
  Wire.begin();

  setupButton();
  getEepromData();
  setupOutputs();
  setupLcd();
  setupDht();
  setupExhaustServo();
  rtcSetup();
  sdCardSetup();

  pinMode(LED_BUILTIN, OUTPUT);

  delay(3000);
  showCurrentTreatment();
  updateStatusLine();
}
void exhaustController(){

  float servoElapsedMinutes = minutesElapsed(lastServoUnix, lastUnixTime);
  //Serial.print("Servo Elapsed Minutes: ");
  //Serial.println(servoElapsedMinutes, 2);
  if (isExhaustOpen) {
    if (servoElapsedMinutes >= EXHAUST_OPEN_DURATION_MINUTES) {
        closeServo();
        isExhaustOpen = false;
        lastServoUnix = lastUnixTime; // reset timer
        //SAVE UNIX TIME TO EEPROM
        EEPROM.update(LAST_SERVO_UNIXTIME_ADDRESS, lastServoUnix);
        EEPROM.update(LAST_SERVO_POSITION_ADDRESS, isExhaustOpen);
    }
  } else {
      if (servoElapsedMinutes >= EXHAUST_OPEN_INTERVAL_MINUTES) {
          openServo();
          isExhaustOpen = true;
          lastServoUnix = lastUnixTime; // reset timer
          //SAVE UNIX TIME TO EEPROM
          EEPROM.update(LAST_SERVO_UNIXTIME_ADDRESS, lastServoUnix);
          EEPROM.update (LAST_SERVO_POSITION_ADDRESS, isExhaustOpen);
      }
  }
  if (servoMoved){
    if (millis() - lastServoMillis >= 10000){
      servoRelay.off();
      servoMoved = false;
      lastServoMillis = 0;
    }
  }
}

void drumMotorController(){
  float motorElapsedMinutes = minutesElapsed(lastMotorUnix, lastUnixTime);
  //Serial.print("Motor Elapsed Minutes: ");
  //Serial.println(motorElapsedMinutes, 2);

  if (!isDrumMotorOn) {
      if (motorElapsedMinutes >= DRUM_ROTATION_INTERVAL_MINUTES) {
          turnOnDrumMotor();
          isDrumMotorOn = true;
          lastMotorUnix = lastUnixTime; // reset timer
          //SAVE UNIX TIME TO EEPROM
          EEPROM.update(LAST_DRUM_UNIXTIME_ADDRESS, lastMotorUnix);
          EEPROM.update(LAST_DRUM_STATE_ADDRESS, isDrumMotorOn);
      }
  } else {
      float motorElapsedMinutes = minutesElapsed(lastMotorUnix, lastUnixTime);
      if (motorElapsedMinutes >= DRUM_ROTATION_DURATION_MINUTES) {
          turnOffDrumMotor();
          isDrumMotorOn = false;
          lastMotorUnix = lastUnixTime; // reset timer
          //SAVE UNIX TIME TO EEPROM
          EEPROM.update(LAST_DRUM_UNIXTIME_ADDRESS, lastMotorUnix);
          EEPROM.update(LAST_DRUM_STATE_ADDRESS, isDrumMotorOn);

      }
  }

}

void menuController(){
    button.tick();
    if (!inSettings && lastTreatment != treatment)
    {
      tempLineInitiated = false;
      lcd.setCursor(0, 1);
      lcd.print("                    "); // clear previous text
      lcd.setCursor(0, 1);
      lcd.print("T" + String(treatment) + " " + String(treatmentMin[treatment]) + "-" + String(treatmentMax[treatment]) + "%");
      lastTreatment = treatment; // update lastTreatment
    }

    // Exit menu if timeout reached
    if (inSettings && millis() - menuLastActive >= MENU_TIMEOUT_MS)
    {
      tempLineInitiated = false;
      inSettings = false;
      lcd.noCursor();
      lcd.noBlink();
      showCurrentTreatment();
      lastTreatment = treatment;
      Serial.println("Menu Timeout - Returning to Current Treatment");
    }

    if (!inSettings && tempLineInitiated) {
      tempLineInitiated = false;
      updateStatusLine();
    }
}

void coolerController(){
  // COOLING CONTROL BASED ON TEMPERATURE
  if (averageTemperature > SETPOINT_TEMPERATURE_C) {
    if (!isRefOn) {
      turnOnCoolingSystem();
      isRefOn = true;
    }
  } else {
    if (isRefOn) {
      turnOffCoolingSystem();
      isRefOn = false;
    }
  }

}

void mistingController(){
  // MISTING CONTROL BASED ON HUMIDITY
  float minHumidity = treatmentMin[treatment];
  float maxHumidity = treatmentMax[treatment] - 1;

  // START misting if humidity too low
  if (averageHumidity < minHumidity) {

    // start pulse
    if (!mistPulseActive) {
      MistPumpStart();
      mistPulseActive = true;
    }

    // turn OFF after ON duration
    if (isMistingPumpOn && millis() - MistPumpStartTime >= MistPumpPulseOnDurationMs) {
      turnOffMistingPump();
      isMistingPumpOn = false;
      MistPumpOffTime = millis();
    }

    // after OFF delay → read again
    if (!isMistingPumpOn && mistPulseActive && millis() - MistPumpOffTime >= MistPumpPulseOffDurationMs) {
       readDHTSensorsAndSetAverage();

      if (averageHumidity >= maxHumidity) {
        mistPulseActive = false;
      }
    }
  }
// humidity already OK → ensure pump OFF
else {
  mistPulseActive = false;
  if (isMistingPumpOn) {
    turnOffMistingPump();
    isMistingPumpOn = false;
  }
}


  // Stop misting pump if humidity is above maximum
  if (averageHumidity >= maximumHumidity()) {
    if (isMistingPumpOn) {
      turnOffMistingPump();
      isMistingPumpOn = false;
      mistPulseActive = false;
    }
  }
}

void dataLoggingControler(){
  float logElapsedMinutes = minutesElapsed(lastLoggedUnix, lastUnixTime);
  if (logElapsedMinutes >= DATA_LOG_INTERVAL_MINUTES) {
      lcdPrintLine(LCD_ACTIVITY_LINE, "LOGGING DATA", true);
      saveLogLine();
      //SAVE UNIX TIME TO EEPROM
      EEPROM.put(LAST_LOGGED_TIME_ADDRESS, lastUnixTime);
      lastLoggedUnix = lastUnixTime; // reset timer
  }
}

void serialMonitorControler(){
  //unsigned long now = millis();

  //if (now - SerialLastPrint < 500) return; // 1s rate limit (non-blocking)
  //SerialLastPrint = now;

  Serial.print("TIME: ");
  printTimestamp(lastUnixTime);

  Serial.print(" | Treatment: T" + String(treatment));
  // CURRENT HUMIDITY SETTINGS: MIN - MAX
  Serial.print("\t T1: " + Temperature1 + "\t T2: " + Temperature2 );
  Serial.print("\t H1: " + Humidity1 + "\t H2: " + Humidity2);
  Serial.print("\t T_AVE: " + String(averageTemperature) + " °C\t H_AVE: " + String(averageHumidity) + " %\t");
  Serial.print("\t EXH: " + String(isExhaustOpen));
  Serial.print(" | DRUM: " + String(isDrumMotorOn));
  Serial.print(" | COOL: " + String(isRefOn));
  Serial.print(" | MIST PUMP: " + String(isMistingPumpOn));
  //Serial.println();





}

/* ===================== LOOP ===================== */
void loop()
{
  // exhaustServo.write(0);
  // delay(5000);
  // exhaustServo.write(180);
  // delay(5000);

  return;

  unsigned long currentMillis = millis();
  //TODO:
  //timestamp treatment t1 t2 h1 h2 tave have  outpput
  DateTime now = rtc.now();
  lastUnixTime = now.unixtime();

  // Temperature and Humidity control
  menuController();

  if (!inSettings){
    readDHTSensorsAndSetAverage();
    exhaustController();
    drumMotorController();
    coolerController();
    mistingController();
    dataLoggingControler();

    serialMonitorControler();
    updateDhtLine();
    if (statusChanged())
    {
      updateStatusLine();
      updateStates();
    }
  }
  unsigned long loopDuration = millis() - currentMillis;
  Serial.print("\t | LOOP DURATION (ms): ");
  Serial.println(loopDuration);
}