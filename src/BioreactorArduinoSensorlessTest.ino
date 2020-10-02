/*Bioreactor Control
 * AUTHOR: Kyle Beyke & Michelob Fedusenko
 */

/*      NEEDED
 * Implement stall detection for feeder (examples in antiquated plunger
 * functions)
 * Timeouts for validations and handshakes
 * Ability for raspi to set airflow durations for sensor calibrations and reads
 */

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <NDIR_SoftwareSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TMC2130Stepper.h>
#include <TMC2130Stepper_REGDEFS.h>
#include <EEPROM.h>
  ///* DEFINITIONS *///
    /* EEPROM MEMORY ADDRESS DEFINITIONS */
#define FEED_ROTATIONS_MEM  2   // EEPROM address for feed_rotations
#define CO2_MINIMUM_MEM     4   // EEPROM address for co2Minimum
#define EEPROM_SET_MEM      6   //EEPROM address for previous settings check
    /* TEMP SENSOR PIN DEFINITIONS */
#define ONE_WIRE_BUS  2   //RMF changed pin 11 > 2
    /* CO2 SENSOR PIN DEFINITIONS */
#define NDIR_TX_PIN   3   //RMF changed pin 13 > 3
#define NDIR_RX_PIN   4   //RMF changed pin 12 > 4
    /* RELAY PIN DEFINITIONS */
#define AIR_VALVE_RELAY_PIN   7  //goes to relay board
#define UPPER_FEED_VALVE_PIN  5  //RMF changed pin 3 > 5
#define LOWER_FEED_VALVE_PIN  6  //RMF changed pin  4 > 6
    /* CONDITIONAL DEFINITIONS */
#define OPEN                  0  // For reading
#define CLOSE                 1  // For reading
    /* MOTOR CONTROL DEFINTIONS */
#define STEP_ANGLE        0.094
#define MICROSTEPS        0  //255, 128, 64, 32, 16, 8, 4, 2, 0 (FULLSTEP)
#define STALL_LIMIT       100
#define STALL_TIMEOUT     6000
#define STALL_VALUE       63  // [-64..63]
#define INTERVAL_LIMIT    1550
#define CLOCKWISE         LOW
#define COUNTERCLOCKWISE  HIGH
    /* MOTOR PIN DEFINITIONS */
#define EN_PIN    9
#define DIR_PIN   10
#define STEP_PIN  11
#define CS_PIN    12
    /* BIOREACTOR CONTROL DEFINITIONS */
#define OPERATION_INTERVAL  1  // minutes
#define FEED_RPM            60  // feed auger revolutions per minute
#define SENSOR_TIMEOUT      10  // seconds
  ///* GLOBALS *///
    // Connections
OneWire oneWire(ONE_WIRE_BUS); //One wire library to avoid a  pullup resistor
DallasTemperature tempSensor(&oneWire); //Temp sensor connection
    //Select 2 digital pins as SoftwareSerial's Tx and Rx. For example, Rx=12 Tx=13
NDIR_SoftwareSerial mySensor(NDIR_TX_PIN, NDIR_RX_PIN);  // CO2 sensor connection
TMC2130Stepper driver = TMC2130Stepper(CS_PIN);  // TMC2130 driver connection
    // Sensors read variables
int co2Current = 0;
int tempCurrent = 0;
    // Time keeping
    // Avoid problems with millis() rollover without resetting millis()
    // DON'T reset millis()
unsigned long counter = 0;
unsigned long previousCounter = 0;
unsigned long interval = OPERATION_INTERVAL * 60;  // ~seconds
unsigned long controlStart = 0;
unsigned long controlStop = 0;
    // Feed defaults
int co2_minimum = 500;
int feed_rotations = 1;
    // Driver control
bool vsense;

  ///* FUNCTIONS *///
    // Reset function used for resetting microcontroller
void (* resetFunc) (void) = 0;  // Declare reset function @ address 0
    /* EEPROM OPERATIONS*/
boolean setMemCheck() {  // Set EEPROM char value to confirm previous write
  char character = 't';
  int value = int(character);
  EEPROM.put(EEPROM_SET_MEM, value);
  if (getMemCheck()) {
    return true;
  }
  return false;
}

boolean getMemCheck() {  // Confirm if previous EEPROM wrtie was performed
  int value;
  EEPROM.get(EEPROM_SET_MEM, value);
  char character = value;
  if (character == 't') {
    return true;
  }
  return false;
}

int getFeedRotations() {  // Gets stored EEPROM value for feed_rotations
  int mem_value;
  EEPROM.get(FEED_ROTATIONS_MEM, mem_value);
  return mem_value;
}

void loadFeedRotations() {  // Loads stored EEPROM value into global
  feed_rotations = getFeedRotations();
}

boolean setFeedRotations(int value) {  // Updates EEPROM and loads into global
    EEPROM.put(FEED_ROTATIONS_MEM, value);
    if (getFeedRotations() == value) {
      loadFeedRotations();
      return true;
    }
    return false;
}

int getCO2Minimum() {  // Gets stored EEPROM value for co2_minimum
  int mem_value;
  EEPROM.get(CO2_MINIMUM_MEM, mem_value);
  return mem_value;
}

void loadCO2Minimum() {  // Loads EEPROM value into global
  co2_minimum = getCO2Minimum();
}

boolean setCO2Minimum(int value) {  // Updates eeprom and redefines global
  EEPROM.put(CO2_MINIMUM_MEM, value);
  if (getCO2Minimum() == value) {
    loadCO2Minimum();
    return true;
  }
  return false;
}

void memStateHandler() {  // Checks if previous EEPROM write exists, updates
  if(getMemCheck()) {  // use previously stored settings
    loadFeedRotations();
    loadCO2Minimum();
  }
  else {  // Save default conditions initialized globally at startup
    setFeedRotations(feed_rotations);
    setCO2Minimum(co2_minimum);
    setMemCheck();
  }
}

    /* VALVE AND RELAY CONTROL FUNCTIONS */
void relay_init() {
  pinMode(AIR_VALVE_RELAY_PIN, OUTPUT);   // Initiate relay with normal conditions
  pinMode(UPPER_FEED_VALVE_PIN, OUTPUT);
  pinMode(LOWER_FEED_VALVE_PIN, OUTPUT);
  digitalWrite(AIR_VALVE_RELAY_PIN, CLOSE);
  digitalWrite(UPPER_FEED_VALVE_PIN, OPEN);
  digitalWrite(LOWER_FEED_VALVE_PIN, CLOSE);
}

void airFlowCalibration() {
  // Invert air flow condition, Fresh air -> Sensor
  digitalWrite(AIR_VALVE_RELAY_PIN, OPEN);
  }

void calibrateCO2Sensor() {
  mySensor.calibrateZero(); //Calibrate zero (400ppm)
  delay(1000);  // Wait to ensure calibration is complete
}

void airFlowBioreactor() {
  digitalWrite(AIR_VALVE_RELAY_PIN, CLOSE); // sets the digital pin 7 off, opening circuits to both valves and returning to normal flow
}

void upperFeedValveControl(int state) {
  digitalWrite(UPPER_FEED_VALVE_PIN, state);
}

void lowerFeedValveControl(int state) {
  digitalWrite(LOWER_FEED_VALVE_PIN, state);
}

    /* SENSOR FUNCTIONS */
int getAveragePPM(int delay_interval_sec) {
  int i = 0;
  int average = 0;
  int total = 0;
  int readings[10];
  unsigned long startMillis = millis(); //Get time
  while (i < 10) {
      unsigned long currentMillis = millis(); //compare time since last attempt
      if (currentMillis - startMillis <= delay_interval_sec) { //If we are still within the allowable interval,
        if (mySensor.measure()) { //Try to connect get measurements from the sensor
            readings[i] = mySensor.ppm;
            i++;
        }
      } else {
        Serial.println("ERROR,CO2 Sensor failed! Initiating controller reset...");
        delay(1000);
        resetFunc(); //Reset the arduino and peripherals if it takes longer than the assigned interval to read the sensor
      }
  }
  for (int j = 0; j < 10; j++) {
    total = total + readings[j];
  }
  average = total / (i+1);
  return average;
}

float getTemp() {
  float celcius = 0;
  tempSensor.requestTemperatures();
  celcius = tempSensor.getTempCByIndex(0);
  return celcius;
}

    /* FEED MOTOR DRIVER CONTROL FUNCTIONS */
uint16_t rms_current(uint8_t CS, float Rsense = 0.11) {
  return (float)(CS+1)/32.0 * (vsense?0.180:0.325)/(Rsense+0.02) / 1.41421 * 1000;
}

void initFeedMotor() {
  pinMode(EN_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH); //deactivate driver (LOW active)
  digitalWrite(DIR_PIN, LOW); //LOW or HIGH
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(CS_PIN, HIGH);
  SPI.begin();
  pinMode(MISO, INPUT_PULLUP);
}

void configDriver() {
  driver.push();
  driver.toff(3);
  driver.tbl(1);
  driver.hysteresis_start(4);
  driver.hysteresis_end(-2);
  driver.rms_current(1500); // mA
  driver.microsteps(MICROSTEPS);
  driver.diag1_stall(1);
  driver.diag1_active_high(1);
  driver.coolstep_min_speed(0xFFFFF); // 20bit max
  driver.THIGH(0);
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.sg_stall_value(STALL_VALUE);
}

void setStepperInterrupts() {
  cli();//stop interrupts
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  OCR1A = 256;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bits for 8 prescaler
  TCCR1B |= (1 << CS11);// | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();//allow interrupts
}

ISR(TIMER1_COMPA_vect) {
  PORTF |= 1 << 0;
  PORTF &= ~(1 << 0);
}

    /* FEED MOTOR CONTROL FUNCTIONS */
void shaftDirection(int value) {
  digitalWrite(DIR_PIN, value);
}

int calcSteps(int steps) {
  if (MICROSTEPS != 0) {
    steps = steps * MICROSTEPS;
    return steps;
  }
  return steps;
}

int calcRotationSteps(float rotations) {
  int steps = rotations * (360 / STEP_ANGLE);
  if (MICROSTEPS != 0) {
    int steps = steps * MICROSTEPS;  //NOT a div-by-0
  }
  return steps;
}

int calcStepSpeed(int steps_per_second) { // per second
  unsigned int step_speed;
  step_speed = (1000000 / steps_per_second) / 2;
  if (MICROSTEPS != 0) {
    step_speed = step_speed / MICROSTEPS;
  }
  return step_speed;
}

int calcRotationStepSpeed(int rotations_per_minute) {
  // Return microseconds per half step
  return calcStepSpeed(calcRotationSteps(rotations_per_minute / 60));
}

void stepMotor(unsigned int step_speed, unsigned int steps) {
  for (unsigned int i = 0; i <= steps; i++) {
    int step_state = HIGH;
    digitalWrite(STEP_PIN, step_state);
    delayMicroseconds(step_speed);
    step_state = LOW;
    digitalWrite(STEP_PIN, step_state);
    delayMicroseconds(step_speed);
  }
}

    /* FEED OPERATION */
void feedOperation(int delayInterval) {
  int step_speed = calcRotationStepSpeed(FEED_RPM);
    // uint32_t drv_status = driver.DRV_STATUS();
  int rotation_steps = calcRotationSteps(feed_rotations);
    // Ensure feed condition for valves
  lowerFeedValveControl(CLOSE);
  delay(delayInterval);
  upperFeedValveControl(OPEN);
  delay(delayInterval);
    // Begin feed operation rotations
  stepMotor(step_speed, rotation_steps);
  delay(delayInterval);
    // Close top valve and open lower valve
  upperFeedValveControl(CLOSE);
  delay(delayInterval);
  lowerFeedValveControl(OPEN);
  delay(delayInterval);
    // Revert to feed condition
  lowerFeedValveControl(CLOSE);
  delay(delayInterval);
  upperFeedValveControl(OPEN);
  delay(delayInterval);
}

    /* COMMUNICATION FUNCTIONS */
    // ADD TIMEOUT
boolean handshake() {
  Serial.println("FLASH");
  while(Serial.available() == 0) {}
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    if (data == "THUNDER") {
      sendValidation(true);
      return true;
    }
  }
  sendValidation(false);
  return false;
}

    // ADD TIMEOUT
 boolean getValidation() {
  while(Serial.available() == 0) {}
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    if (data == "CONFIRMED") {
      return true;
    }
    else if (data == "FAILED") {
      return false;
    }
  }
  Serial.println("ERROR, Validation failed!");
  while(!getValidation());
  return false;
}

void sendValidation(boolean status) {
  if (status == true){
    Serial.println("CONFIRMED");
  }
  else if (status == false) {
    Serial.println("FAILED");
  }
}

void executeCommandCharArray() {
  char command_raw[25];
  if (Serial.available() > 0) {
    Serial.readBytesUntil('\n', command_raw, sizeof(command_raw));
    if (strcmp(command_raw, "get-feed-ppm") == 0) {
      sendValidation(true);
      String response = "CURFEEDPPM,";
      response = response + getCO2Minimum();
      Serial.println(response);
      while(!getValidation()) {}
    } else if (strcmp(command_raw, "get-feed-rotations") == 0) {
      sendValidation(true);
      String response = "CURFEEDROT,";
      response = response + getFeedRotations();
      Serial.println(response);
      while (!getValidation()) {}
    } else {
      int value = 0;
      char command[20];
      char separator[] = ",";
      char *token = strtok(command_raw, separator);
      int size = sizeof(token);
      strncpy(command, token, size);
      token = strtok(NULL, separator);
      value = atoi(token);
      if (strcmp(command, "reset") == 0 && value == 1) {
        sendValidation(true);
        Serial.println("SUCCESS,Controller resetting");
        while(!getValidation()) {}
        delay(1000);
        resetFunc();
      } else if (strcmp(command, "set-feed-rotations") == 0 && value > 0 && value < 101) {
        sendValidation(true);
        if(setFeedRotations(value)){
          Serial.println("SUCCESS,Feed rotations set");
          while(!getValidation()) {}
        }
      } else if (strcmp(command, "set-feed-ppm") == 0 && value > 499 && value < 5001) {
        sendValidation(true);
        if (setCO2Minimum(value)) {
          Serial.println("SUCCESS,Feed PPM threshhold set");
          while(!getValidation()) {}
        }
      } else {
        sendValidation(false);
        Serial.println("ERROR,Command failed to execute");
        while(!getValidation()) {}
      }
    }
  }
}


String getValue(String data, char separator[], int index) //function needed for string parsing
{
  String values[2];
  int value_index = 0;
  char str[data.length()+1];
  data.toCharArray(str, data.length()+1);
  char *token = strtok(str, separator);
  while (token != NULL) {
    String value_string = token;
    values[value_index] = value_string;
    value_index++;
    token = strtok(NULL, separator);
  }
  return values[index];
}

boolean sendData(int ppm, float temp) {
  Serial.print("DATA,");
  Serial.print(ppm);
  Serial.print(",");
  Serial.println(temp);
  return getValidation();
}

boolean sendDataWithFeed(int ppm, float temp) {
  Serial.print("DATAFEED,");
  Serial.print(ppm);
  Serial.print(",");
  Serial.print(temp);
  Serial.print(",");
  Serial.println(feed_rotations);
  return getValidation();
}

    // Command structure, uses EEPROM functions to implement commands parsed from passed strings
    // This function utilizes String() objects which are not memory efficient
void executeCommand(String command_raw) {
  if (command_raw == "get-feed-ppm") {
    sendValidation(true);
    String response = "CURFEEDPPM,";
    response = response + getCO2Minimum();
    Serial.println(response);
    while(!getValidation()) {}
  } else if (command_raw == "get-feed-rotations") {
    sendValidation(true);
    String response = "CURFEEDROT,";
    response = response + getFeedRotations();
    Serial.println(response);
    while (!getValidation()) {}
  } else {
    char separator[] = ",";
    String command = getValue(command_raw, separator, 0);
    String valueString = getValue(command_raw, separator, 1);
    int value = valueString.toInt();
    if (command == "reset" && value == 1) {
      sendValidation(true);
      Serial.println("SUCCESS,Controller resetting");
      while(!getValidation()) {}
      delay(1000);
      resetFunc();
    } else if (command == "set-feed-rotations" && value > 0 && value < 101) {
      sendValidation(true);
      if(setFeedRotations(value)){
        Serial.println("SUCCESS,Feed rotations set");
        while(!getValidation()) {}
      }
    } else if (command == "set-feed-ppm" && value > 499 && value < 5001) {
      sendValidation(true);
      if (setCO2Minimum(value)) {
        Serial.println("SUCCESS,Feed PPM threshhold set");
        while(!getValidation()) {}
      }
    } else {
      sendValidation(false);
      Serial.println("ERROR,Command failed to execute");
      while(!getValidation()) {}
    }
  }
}

void checkCommand() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    executeCommand(command);
  }
}

void notifyRead(boolean state) {
  if (state == true) {
    Serial.println("READING");
    while(!getValidation()) {}
  }
  if (state == false) {
    Serial.println("READDONE");
    while(!getValidation()) {}
  }
}

void setup() {
  //pinMode(AIR_VALVE_RELAY_PIN, OUTPUT);  //Pin to control relay board, pin is split into two outputs
  //digitalWrite(AIR_VALVE_RELAY_PIN, HIGH);  //Make sure valves are at a normal condition (circuits opened)
  Serial.begin(115200);  // Init serial port and set baudrate
  delay(1000);
  if(!Serial) {  // If the serial fails, reset
    resetFunc();
  }
  // tempSensor.begin();  // Init the temp sensor
  // if (mySensor.begin()) {  // Init the CO2 sensor
  //   delay(10000); //wait for sensor to come up
  // } else {
  //   Serial.println("ERROR,CO2 Sensor did not initialize. Reseting microcontroller...");
  //   delay(1000);
  //   resetFunc();
  // }
  // initFeedMotor();  // Init feed motor pin control
  // configDriver();  // Configure feed motor driver
  // setStepperInterrupts();  // Set stepper interrupts
  // digitalWrite(EN_PIN, LOW);  // TMC2130 outputs on (LOW active)
  // vsense = driver.vsense();  // Set vsense
  // shaftDirection(CLOCKWISE);  // Set shaft direction
  // relay_init();  // Initialize relay
  delay(1000);  // Allow for everything to initialize...
  while(!handshake()) {} // Wait for Raspi to confirm it is connected
  executeCommandCharArray();   // Check for command, if no command in waiting, ensure
  memStateHandler();  // Check EEPROM for previously set feed defaults, update accordingly
}

void loop() {
  boolean rollover = false;
  controlStart = millis();
  executeCommandCharArray();
  airFlowBioreactor();  // Return to normal flow conditions, CO2 -> Sensor
  if ((counter - previousCounter) >= (interval)) {
    notifyRead(true);
    co2Current = random(400,5001);
    tempCurrent = random(10,31);
    if(co2Current < co2_minimum) {  // If current CO2 levels are below minimum
      delay(5000); // Perform a feed operation
      while (!sendDataWithFeed(co2Current, tempCurrent)) {}  // Send data including current feed rotations
    } else {
      while (!sendData(co2Current, tempCurrent)) {}  // Send data sans feed rotations
    }
    notifyRead(false);
    previousCounter = 0;
    counter = 0;
    rollover = true;
  }
  controlStop = millis();
  if (!rollover) {
    counter = counter + ((controlStop - controlStart) / 1000);
  }
  delay(1000);
  counter++;
}

    /*ANTIQUATED FUNCTION EXAMPLES FOR REFERENCE*/
      /*These provie examples of how to use stall detection */
/*boolean lowerPlunger() {
  digitalWrite(DIR_PIN, HIGH); //Set rotation counter clockwise

  uint32_t drv_status = driver.DRV_STATUS();
  unsigned long init = millis();
  for (int i = 0; i <= 255; i++) {
    drv_status = driveMotor(drv_status);
  }
  unsigned long start = millis();
  while (millis() <= start + STALL_TIMEOUT) {
    drv_status = driver.DRV_STATUS();
    while((drv_status & SG_RESULT_bm)>>SG_RESULT_bp > STALL_LIMIT){
      drv_status = driveMotor(drv_status);
    }
    for (int i = 0; i <= 63; i++) {
      drv_status = driveMotor(drv_status);
    }
    unsigned long interval = millis() - init;
    Serial.println(interval);
    if (interval >= INTERVAL_LIMIT) {
      return true;
      break;
    }
    else {
      drv_status = driver.DRV_STATUS();
      while((drv_status & SG_RESULT_bm)>>SG_RESULT_bp > STALL_LIMIT){
        drv_status = driveMotor(drv_status);
      }
    }
  }
  return false;
}

boolean raisePlunger() {
  digitalWrite(DIR_PIN, LOW); //Set rotation clockwise

  uint32_t drv_status = driver.DRV_STATUS();
  unsigned long init = millis();
  for (int i = 0; i <= 255; i++) {
    drv_status = driveMotor(drv_status);
  }

  unsigned long start = millis();
  while (millis() <= start + STALL_TIMEOUT) {
    drv_status = driver.DRV_STATUS();
    while((drv_status & SG_RESULT_bm)>>SG_RESULT_bp > STALL_LIMIT){
      drv_status = driveMotor(drv_status);
    }
    for (int i = 0; i <= 63; i++) {
      drv_status = driveMotor(drv_status);
    }
    unsigned long interval = millis() - init;
    Serial.println(interval);
    if (interval >= INTERVAL_LIMIT) {
      return true;
      break;
    }
    else {
      drv_status = driver.DRV_STATUS();
      while((drv_status & SG_RESULT_bm)>>SG_RESULT_bp > STALL_LIMIT){
        drv_status = driveMotor(drv_status);
      }
    }
  }

  return false;
}*/
