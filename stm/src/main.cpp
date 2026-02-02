#include <Arduino.h>
#include <Servo.h> //arduinostm32 framework default library
#include <SoftwareSerial.h>
#include <Wire.h>

#ifdef ONBOARD //build arg passed to platformio in the upload script

  #include "/home/frostlab/config/teensy_params.h"

#else // for testing on PC, should be copy of the template teensy_params.h

  #define PRESSURE_30M   // COUG1 has 30m depth pressure sensor (Comment out one you don't need)
  #define PRESSURE_10M      // COUG2 has 10m depth pressure sensor
  #define DEFAULT_SERVO_POSITION 90 //default servo position before any commands
  #define THRUSTER_DEFAULT_OUT 1500 //default value written to the thruster
  //note: servo timings are currently not implemented and do nothing
  #define SERVO_OUT_US_MAX 2000 // check servo ratings for pwm microsecond values
  #define SERVO_OUT_US_MIN 1000 // change per servo type COUG1 is 500-2500 COUG2 is 1000-2000

  #define ENABLE_SERVOS true
  #define ENABLE_THRUSTER true
  #define ENABLE_BATTERY true
  #define ENABLE_LEAK true
  #define ENABLE_PRESSURE true
// unused by new board
  #define VOLT_PIN 27   //pins on the teensy for the battery monitor
  #define CURRENT_PIN 22   //pins on the teensy for the battery monitor
  #define LEAK_PIN 26       //pins on the teensy for the leak sensor

#endif


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


// actuator conversion values
#define SERVO_IN_MIN -90
#define SERVO_IN_MAX 90
#define THRUSTER_IN_MAX 100
#define THRUSTER_IN_MIN -100
#define THRUSTER_OUT_US_MAX 1900 //thruster output will lerp between min and max microseconds
#define THRUSTER_OUT_US_MIN 1100 //thruster output will lerp between min and max microseconds
#define THRUSTER_CMD_RANGE 200 //controls how the uC interprets thruster values. Will lerp from -range/2 to range/2

// sensor baud rates
#define I2C_RATE 400000

// sensor update rates
#define BATTERY_MS 1000 // arbitrary
#define LEAK_MS 1000    // arbitrary

#define ACTUATOR_TIMEOUT 5000
// time of last received command (used as a fail safe)
unsigned long last_received = 0;



// actuator objects
Servo myServo1;
Servo myServo2;
Servo myServo3;
Servo myThruster;

// Global sensor variables
float conversionFactor = 100.0f;
float temperature = 0.0;

// Buffer size for serial input
const int BUFFER_SIZE = 50;
char inputBuffer[BUFFER_SIZE];
int bufferIndex = 0;
bool newData = false;

uint16_t blink_tmr=0;
uint8_t b_code=0;
uint8_t b_state=0;

void blink_code(uint8_t code);
void blink_callback();

void setup() {
    blink_code(0);
    blink_callback();
    Serial.begin(SERIAL_BAUD_RATE);
    Serial.print("microcontroller begun");
  pinMode(DBG_LED, OUTPUT);
  pinMode(PWR_RELAY,OUTPUT);
  digitalWrite(PWR_RELAY,0);

  if(ENABLE_SERVOS){
    // Set up the servo and thruster pins
    pinMode(SRV1, OUTPUT);
    pinMode(SRV2, OUTPUT);
    pinMode(SRV3, OUTPUT);

    myServo1.attach(SRV1,SERVO_OUT_US_MIN,SERVO_OUT_US_MAX);
    myServo2.attach(SRV2,SERVO_OUT_US_MIN,SERVO_OUT_US_MAX);
    myServo3.attach(SRV3,SERVO_OUT_US_MIN,SERVO_OUT_US_MAX);

    myServo1.write(DEFAULT_SERVO_POSITION);
    myServo2.write(DEFAULT_SERVO_POSITION);
    myServo3.write(DEFAULT_SERVO_POSITION);
  }

  if(ENABLE_THRUSTER){
    pinMode(ESC, OUTPUT);
    myThruster.attach(ESC, 1000, 2000);
    myThruster.writeMicroseconds(THRUSTER_DEFAULT_OUT);
    delay(7000);
  }

  // Setup Depth sensor
  Wire.begin();
  Wire.setClock(I2C_RATE);

  if(ENABLE_BATTERY){
      pinMode(CURR_SENSE, INPUT);
  pinMode(BATT_V_SENSE, INPUT);
  }

  if(ENABLE_LEAK){
  pinMode(LEAK, INPUT);
  }
  
}

void blink_code(uint8_t code){
  b_code=code;
  b_state=0;
  blink_tmr=0;
}

void blink_callback(){
  switch(b_code){
    case 0: //nominal, const on
    digitalWrite(DBG_LED,HIGH);
    break;
    case 1: //blink once, return to state 0
    digitalWrite(DBG_LED,LOW);
    if(blink_tmr>200){
      b_code=0;
      blink_tmr=0;
    }
    blink_tmr++;
    break;
    case 2: // const slow blink
    if(b_state){
      digitalWrite(DBG_LED,LOW);
    } else{
      digitalWrite(DBG_LED,HIGH);
    }
    if(blink_tmr>400){
      b_state=!b_state;
      blink_tmr=0;
    }
    blink_tmr++;
    break; 
    case 3://fast blink
        if(b_state){
      digitalWrite(DBG_LED,LOW);
    } else{
      digitalWrite(DBG_LED,HIGH);
    }
    if(blink_tmr>200){
      b_state=!b_state;
      blink_tmr=0;
    }
    blink_tmr++;
    break;
    default:
    b_code=0;
    break;
  }
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
  int intValue = static_cast<int>(value + DEFAULT_SERVO_POSITION);
  return intValue;
}

void control_callback(float servo1, float servo2, float servo3, int thruster,char sw_state){
  // Convert float (-90 to 90) to int centered around positive 90
  last_received = millis();

  if(ENABLE_SERVOS){
    int intFin1 = DEFAULT_SERVO_POSITION, intFin2 = DEFAULT_SERVO_POSITION, intFin3 = DEFAULT_SERVO_POSITION;
    
    intFin1 = convertToInt(servo1);
    intFin2 = convertToInt(servo2);
    intFin3 = convertToInt(servo3);

    //TODO make sure this matches the fin convention for pitch up and yaw starboard for positive
    //DECIDE WHETHERE TO MAX OUT THE FINS HERE OR IN THE NODE?
    
    myServo1.writeMicroseconds(intFin1);
    myServo2.writeMicroseconds(intFin2);
    myServo3.writeMicroseconds(intFin3);
  }
  
  if(ENABLE_THRUSTER){
    int usecThruster = map(thruster, THRUSTER_IN_MIN, THRUSTER_IN_MAX, THRUSTER_OUT_US_MIN, THRUSTER_OUT_US_MAX);
    myThruster.writeMicroseconds(usecThruster); //Thruster Value from 1100-1900
  }

  digitalWrite(PWR_RELAY,sw_state==1);
}

// Function to parse and execute NMEA command
void parseData() {
  float servo1, servo2, servo3;
  int thruster;
  char sw_state;
  if (sscanf(inputBuffer, "$CONTR,%f,%f,%f,%d,%c", &servo1, &servo2, &servo3, &thruster,&sw_state) == 5) {
    control_callback(servo1, servo2, servo3, thruster,sw_state);
  }
}



/**
 * Reads the battery sensor data. This function reads the battery sensor
 * data (voltage and current) and publishes it.
 */
void read_battery() {

  // we did some testing to determine the below params, but
  // it's possible they are not completely accurate
  float voltage = (analogRead(BATT_V_SENSE) * 5.7);
  float current = (analogRead(CURR_SENSE) * 0.0264);

  // publish the battery data
  Serial.print("$BATTE,");
  Serial.print(voltage,1);
  Serial.print(",");
  Serial.println(current, 1);
}


/**
 * Reads the leak sensor data. This function reads the leak sensor data and prints it over serial
 */
void read_leak() {

  int leak = digitalRead(LEAK);

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

  if(ENABLE_LEAK){
    read_leak();
  }


  if(ENABLE_BATTERY){
    read_battery();
  }

      // fail safe for agent disconnect
  if (millis() - last_received > ACTUATOR_TIMEOUT) {
    if(ENABLE_SERVOS){
        myServo1.write(DEFAULT_SERVO_POSITION);
        myServo2.write(DEFAULT_SERVO_POSITION);
        myServo3.write(DEFAULT_SERVO_POSITION);
    }
    if(ENABLE_THRUSTER){
        myThruster.writeMicroseconds(THRUSTER_DEFAULT_OUT);
    }
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
  blink_callback();
  // sweep_loop();
  full_loop();
}
