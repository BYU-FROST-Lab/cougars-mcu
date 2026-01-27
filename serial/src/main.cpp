#include <Arduino.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "MS5837.h"
#include "/home/frostlab/config/teensy_params.h"

#define ENABLE_SERVOS
#define ENABLE_THRUSTER
#define ENABLE_BATTERY
#define ENABLE_LEAK
#define ENABLE_PRESSURE

#define SERIAL_BAUD_RATE 115200

// hardware pin values
#define DBG_LED PA0
#define BATT_V_SENSE PA1
#define SRV1 PA2
#define SRV2 PA3
#define SRV3 PA4
#define SRV4 PA6
#define ESC PA7
#define LEAK PA8
#define STROBE PA11
#define PWR_RELAY PA15
#define CURR_SENSE PB0

// default actuator positions


// actuator conversion values
#define SERVO_OUT_US_MAX 2000   //servo output will lerp between min and max microseconds
#define SERVO_OUT_US_MAX 1000
#define THRUSTER_OUT_US_MAX 1900 //thruster output will lerp between min and max microseconds
#define THRUSTER_OUT_US_MIN 1100 //thruster output will lerp between min and max microseconds
#define THRUSTER_CMD_RANGE 200 //controls how the uC interprets thruster values. Will lerp from -range/2 to range/2

// sensor baud rates
#define I2C_RATE 400000

// sensor update rates
#define BATTERY_MS 1000 // arbitrary
#define LEAK_MS 1000    // arbitrary
#define PRESSURE_MS 20  // fastest update speed is 50 Hz

#define ACTUATOR_TIMEOUT 5000
// time of last received command (used as a fail safe)
unsigned long last_received = 0;

// sensor objects
MS5837 myPressure;

// actuator objects
Servo myServo1;
Servo myServo2;
Servo myServo3;
Servo myThruster;

// Global sensor variables
float pressure = 0;
float conversionFactor = 100.0f;
float temperature = 0.0;

// Buffer size for serial input
const int BUFFER_SIZE = 50;
char inputBuffer[BUFFER_SIZE];
int bufferIndex = 0;
bool newData = false;

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.print("microcontroller begun");

  // set up the indicator light
  pinMode(LED_PIN, OUTPUT);

  #ifdef ENABLE_SERVOS
    // Set up the servo and thruster pins
    pinMode(SERVO_PIN1, OUTPUT);
    pinMode(SERVO_PIN2, OUTPUT);
    pinMode(SERVO_PIN3, OUTPUT);

    myServo1.attach(SERVO_PIN1,1000,2000);
    myServo2.attach(SERVO_PIN2,1000,2000);
    myServo3.attach(SERVO_PIN3, 1000, 2000);

    myServo1.write(DEFAULT_SERVO);
    myServo2.write(DEFAULT_SERVO);
    myServo3.write(DEFAULT_SERVO);

    #ifdef ENABLE_BT_DEBUG
      Serial.println("[INFO] Servos enabled");
    #endif // ENABLE_BT_DEBUG
  #endif // ENABLE_SERVOS

  #ifdef ENABLE_THRUSTER
    pinMode(THRUSTER_PIN, OUTPUT);
    myThruster.attach(THRUSTER_PIN, 1000, 2000);
    myThruster.writeMicroseconds(THRUSTER_OFF);
    delay(7000);

    #ifdef ENABLE_BT_DEBUG
        Serial.println("[INFO] Thruster enabled");
    #endif // ENABLE_BT_DEBUG
  #endif // ENABLE_THRUSTER

  // Setup Depth sensor
  Wire.begin();
  Wire.setClock(I2C_RATE);

  #ifdef ENABLE_BATTERY
  pinMode(CURRENT_PIN, INPUT);
  pinMode(VOLT_PIN, INPUT);

    #ifdef ENABLE_BT_DEBUG
      Serial.println("[INFO] Battery Sensor enabled");
    #endif // ENABLE_BT_DEBUG
  #endif // ENABLE_BATTERY

  #ifdef ENABLE_LEAK
  pinMode(LEAK_PIN, INPUT);

    #ifdef ENABLE_BT_DEBUG
      Serial.println("[INFO] Leak Sensor enabled");
    #endif // ENABLE_BT_DEBUG
  #endif // ENABLE_LEAK
  
  #ifdef ENABLE_PRESSURE
    while (!myPressure.init()) {
      Serial.println("ERROR: Could not connect to Pressure Sensor over I2C");
      delay(1000);
    }
    
  #ifdef PRESSURE_10M
    myPressure.setModel(MS5837::MS5837_02BA);
    // myPressure.setModel(MS5837::MS5837_30BA);
  #endif 

  #ifdef PRESSURE_30M
  myPressure.setModel(MS5837::MS5837_30BA);
  #endif

    #ifdef ENABLE_BT_DEBUG
      Serial.println("[INFO] Pressure Sensor enabled");
    #endif // ENABLE_BT_DEBUG
    //TODO: Make define for the other model of depth sensor
  #endif // ENABLE_PRESSURE
}

// Function to receive serial data with end marker
void recvWithEndMarker() {
  static const char endMarker = '\n';
  char receivedChar;

  while (Serial.available() > 0 && !newData) {
    receivedChar = Serial.read();

    if (receivedChar != endMarker) {
      inputBuffer[bufferIndex] = receivedChar;
      bufferIndex++;
      if (bufferIndex >= BUFFER_SIZE) {
        bufferIndex = BUFFER_SIZE - 1;
      }
    } else {
      inputBuffer[bufferIndex] = '\0'; // terminate the string
      bufferIndex = 0;
      newData = true;
    }
  }
}

// Function to convert float (-90 to 90) to int centered around positive 90
int convertToInt(float value) {
  int intValue = static_cast<int>(value + DEFAULT_SERVO);
  return intValue;
}

void control_callback(float servo1, float servo2, float servo3, int thruster){
  // Convert float (-90 to 90) to int centered around positive 90
  last_received = millis();

  #ifdef ENABLE_SERVOS
    int intFin1 = DEFAULT_SERVO, intFin2 = DEFAULT_SERVO, intFin3 = DEFAULT_SERVO;
    
    intFin1 = convertToInt(servo1);
    intFin2 = convertToInt(servo2);
    intFin3 = convertToInt(servo3);

    //TODO make sure this matches the fin convention for pitch up and yaw starboard for positive
    //DECIDE WHETHERE TO MAX OUT THE FINS HERE OR IN THE NODE?

    myServo1.write(intFin1);
    myServo2.write(intFin2);
    myServo3.write(intFin3);
  #endif
  
  #ifdef ENABLE_THRUSTER
    int usecThruster = map(thruster, THRUSTER_IN_LOW, THRUSTER_IN_HIGH, THRUSTER_OUT_LOW, THRUSTER_OUT_HIGH);
    myThruster.writeMicroseconds(usecThruster); //Thruster Value from 1100-1900
  #endif // ENABLE_THRUSTER

  
  // Print the results for debugging
  #ifdef ENABLE_BT_DEBUG
      Serial.print("Received servo1: ");
      Serial.print(servo1);
      Serial.print(", servo2: ");
      Serial.print(servo2);
      Serial.print(", servo3: ");
      Serial.print(servo3);
      Serial.print(", thruster: ");
      Serial.println(thruster);
    #endif
}

// Function to parse and execute NMEA command
void parseData() {
  float servo1, servo2, servo3;
  int thruster;

  if (sscanf(inputBuffer, "$CONTR,%f,%f,%f,%d", &servo1, &servo2, &servo3, &thruster) == 4) {
    control_callback(servo1, servo2, servo3, thruster);
  }
}

/**
 * Reads the pressure sensor data. This function reads the pressure sensor data
 * and publishes it to the micro-ROS agent.
 */
void read_pressure() {

  myPressure.read();
  pressure = myPressure.pressure(conversionFactor);
  temperature = myPressure.temperature();

  Serial.print("$DEPTH,");
  Serial.print(pressure,1);
  Serial.print(",");
  Serial.println(temperature, 1);
}


/**
 * Reads the battery sensor data. This function reads the battery sensor
 * data (voltage and current) and publishes it.
 */
void read_battery() {

  // we did some testing to determine the below params, but
  // it's possible they are not completely accurate
  float voltage = (analogRead(VOLT_PIN) * 0.03437) + 0.68;
  float current = (analogRead(CURRENT_PIN) * 0.122) - 11.95;

  // publish the battery data
  Serial.print("$BATTE,");
  Serial.print(voltage,1);
  Serial.print(",");
  Serial.println(current, 1);
}


/**
 * Reads the leak sensor data. This function reads the leak sensor data
 * and publishes it to the micro-ROS agent.
 */
void read_leak() {

  int leak = digitalRead(LEAK_PIN);

  // publish the leak data
  Serial.print("$LEAK,");
  Serial.println(leak,1);
}


void full_loop() {
  recvWithEndMarker();
  if (newData) {
    parseData();
    newData = false;
  }
  delay(5); // Adjust the delay as needed

  #ifdef ENABLE_LEAK
    read_leak();
  #endif // ENABLE_LEAK

  #ifdef ENABLE_PRESSURE
    read_pressure();
  #endif // ENABLE_PRESSURE

  #ifdef ENABLE_BATTERY
    read_battery();
  #endif // ENABLE_BATTERY

      // fail safe for agent disconnect
  if (millis() - last_received > ACTUATOR_TIMEOUT) {

    #ifdef ENABLE_SERVOS
        myServo1.write(DEFAULT_SERVO);
        myServo2.write(DEFAULT_SERVO);
        myServo3.write(DEFAULT_SERVO);
    #endif // ENABLE_SERVOS

    #ifdef ENABLE_THRUSTER
        myThruster.writeMicroseconds(THRUSTER_OFF);
    #endif // ENABLE_THRUSTER

    #ifdef ENABLE_BT_DEBUG
        Serial.println(
            "[INFO] No command received in timeout, stopping actuators");
    #endif // ENABLE_BT_DEBUG
  }
}


void sweep_loop(){
  int angle = 0;
  myServo1.write(angle);
  myServo2.write(angle);
  myServo3.write(angle);

  delay(1000);
  angle = 180;
  myServo1.write(angle);
  myServo2.write(angle);
  myServo3.write(angle);
  delay(1000);
}


void loop(){

  // blink the indicator light
  if (millis() % 1000 < 250) {
    digitalWrite(LED_PIN, LOW);
  } else {
    digitalWrite(LED_PIN, HIGH);
  }


  // sweep_loop();
  full_loop();
}
