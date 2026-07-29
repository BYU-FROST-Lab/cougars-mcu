#include <Arduino.h>
#include <Servo.h> //arduinostm32 framework default library

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
#define WATER_SENSOR PB1 // underwater/submerged sensor, drives the DVL/modem relay and strobe in AUTO mode

// underwater sensor polarity: digitalRead(WATER_SENSOR) reads this value when submerged
#define WATER_SENSOR_WET_STATE HIGH

// relay/strobe mode selection, settable over serial with $RELAY,<c> and $STROBE,<c> (c = '0' off, '1' on, 'A' auto)
#define MODE_OFF 0
#define MODE_ON 1
#define MODE_AUTO 2

#define STROBE_BLINK_MS 500 // nav light blink half-period when active
#define STROBE_LEAK_BLINK_MS 50 // fast blink half-period when a leak is detected, overrides strobe_mode entirely


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

// DVL/modem relay and strobe (nav light) mode state, default to following the underwater sensor
uint8_t relay_mode = MODE_AUTO;
uint8_t strobe_mode = MODE_AUTO;
bool strobe_state = false;
unsigned long strobe_last_toggle = 0;

void update_relay_and_strobe();

void setup() {
  // Custom F030K6 board routes UART to PA9/PA10, not the Nucleo default pins.
  Serial.setTx(PA9);
  Serial.setRx(PA10);
    Serial.begin(SERIAL_BAUD_RATE);
    Serial.print("microcontroller begun");
  pinMode(DBG_LED, OUTPUT);
  pinMode(PWR_RELAY,OUTPUT);
  digitalWrite(PWR_RELAY,0);
  pinMode(WATER_SENSOR, INPUT);
  pinMode(STROBE, OUTPUT);
  digitalWrite(STROBE, LOW);

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


  if(ENABLE_BATTERY){
      pinMode(CURR_SENSE, INPUT);
  pinMode(BATT_V_SENSE, INPUT);
  }

  if(ENABLE_LEAK){
  pinMode(LEAK, INPUT);
  }
  
}

// Drives the DVL/modem relay, its DBG_LED indicator, and the strobe nav light
// off of relay_mode/strobe_mode, falling back to the underwater sensor (WATER_SENSOR)
// whenever a mode is set to MODE_AUTO.
void update_relay_and_strobe(){
  bool submerged = (digitalRead(WATER_SENSOR) == WATER_SENSOR_WET_STATE);

  bool relay_on;
  switch(relay_mode){
    case MODE_ON: relay_on = true; break;
    case MODE_OFF: relay_on = false; break;
    default: relay_on = submerged; break; // MODE_AUTO
  }
  digitalWrite(PWR_RELAY, relay_on);
  digitalWrite(DBG_LED, relay_on); // debug LED mirrors the DVL/modem relay state

  bool strobe_active;
  switch(strobe_mode){
    case MODE_ON: strobe_active = true; break;
    case MODE_OFF: strobe_active = false; break;
    default: strobe_active = submerged; break; // MODE_AUTO
  }

  // A detected leak always wins: force the strobe on and blinking fast,
  // ignoring strobe_mode/relay logic entirely.
  unsigned long blink_period = STROBE_BLINK_MS;
  bool leak_detected = ENABLE_LEAK && (digitalRead(LEAK) == HIGH);
  if(leak_detected){
    strobe_active = true;
    blink_period = STROBE_LEAK_BLINK_MS;
  }

  if(strobe_active){
    if(millis() - strobe_last_toggle >= blink_period){
      strobe_last_toggle = millis();
      strobe_state = !strobe_state;
      digitalWrite(STROBE, strobe_state);
    }
  } else {
    strobe_state = false;
    digitalWrite(STROBE, LOW);
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

void control_callback(float servo1, float servo2, float servo3, int thruster){
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

  
}

// Applies a mode char ('0' off, '1' on, 'A' auto/follow underwater sensor) to a mode variable.
// Unrecognized chars are ignored, leaving the mode unchanged.
void apply_mode_char(uint8_t &mode, char mode_char){
  switch(mode_char){
    case '0': mode = MODE_OFF; break;
    case '1': mode = MODE_ON; break;
    case 'A': mode = MODE_AUTO; break;
  }
}

// Function to parse and execute NMEA command
void parseData() {
  float servo1, servo2, servo3;
  int thruster;
  char mode_char;
  if (sscanf(inputBuffer, "$CONTR,%f,%f,%f,%d", &servo1, &servo2, &servo3, &thruster) == 4) {
    control_callback(servo1, servo2, servo3, thruster);
  }
  // $RELAY,<c> sets the DVL/modem power relay mode: '0' off, '1' on, 'A' auto (follow WATER_SENSOR)
  if(sscanf(inputBuffer, "$RELAY,%c", &mode_char) == 1){
    apply_mode_char(relay_mode, mode_char);
  }
  // $STROBE,<c> sets the nav light strobe mode: '0' off, '1' on (blinking), 'A' auto (follow WATER_SENSOR)
  if(sscanf(inputBuffer, "$STROBE,%c", &mode_char) == 1){
    apply_mode_char(strobe_mode, mode_char);
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
  update_relay_and_strobe();
  // sweep_loop();
  full_loop();
}
