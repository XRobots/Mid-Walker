// include the library for the AS5047P sensor.
#include <AS5047P.h>

// define the chip select port.
#define AS5047P_CHIP_SELECT_PORT_1 36

// define the spi bus speed 
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000

// initialize a new AS5047P sensor object.
AS5047P as5047p1(AS5047P_CHIP_SELECT_PORT_1, AS5047P_CUSTOM_SPI_BUS_SPEED);

//PID
#include <PID_v1.h>

// PID1           // mail back wheel sideways
double Pk1 = 10;
double Ik1 = 0;
double Dk1 = 0.1;

double Setpoint1, Input1, Output1, Output1a;    // PID variables
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID Setup

unsigned long currentMillis;
unsigned long previousMillis = 0;        // set up timers
long interval = 5;             // time constant for timer

float angle;  // encoder angle
int marker;
int jog;
int def;
int walk;
int legJog1;
int legJog2;
int stick;
float legPos = 0;
float legPos1 = 0;

int first = 0;  // flag to see if we just powered up

// arduino setup routine
void setup() {

  pinMode(2, OUTPUT);     // motor PWMs
  pinMode(3, OUTPUT);

  Serial.begin(115200);
  Serial1.begin(115200);

  // initialize the AS5047P sensor and hold if sensor can't be initialized.
  while (!as5047p1.initSPI()) {
    Serial.println(F("Can't connect to the AS5047P_1 sensor! Please check the connection..."));
    delay(10);
    }

  PID1.SetMode(AUTOMATIC);              
  PID1.SetOutputLimits(-127,127);
  PID1.SetSampleTime(5);
}

// arduino loop routine
void loop() {

  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {   // start of timed loop
      previousMillis = currentMillis;

      // read the sensor
      //Serial.println(as5047p1.readAngleDegree());
      angle = as5047p1.readAngleDegree();

      angle = angle - 259 + 17; // calibrate for zero
      angle = angle *- 1;     // reverse for legs 1 and 4

      if (Serial1.available() > 0){           // receive IMU data from serial IMU
        //Serial.print("data  ");
        marker = Serial1.parseInt();
          if (marker == 9000) {                   // look for check value to check it's the start of the data
              jog = Serial1.parseInt();
              def = Serial1.parseInt();
              walk = Serial1.parseInt();
              legJog1 = Serial1.parseInt();
              legJog2 = Serial1.parseInt();
              stick = Serial1.parseInt();
              if (Serial1.read() == '\n') {     // end of IMU data 
              }
          }
     }

     Serial.print(angle);
     Serial.print(" , ");
     Serial.print(jog);
     Serial.print(" , ");
     Serial.print(def);
     Serial.print(" , ");
     Serial.print(walk);
     Serial.print(" , ");
     Serial.print(legJog1);
     Serial.print(" , ");
     Serial.print(legJog2);
     Serial.print(" , ");
     Serial.print(legPos);
     Serial.print(" , ");
     Serial.print(first);
     Serial.println();

     if (jog == 1) {
        if (legJog1 == 1) {
          legPos = legPos - 0.1;
        }
        else if (legJog2 == 1) {
          legPos = legPos + 0.1;
        }
     first = 1;
     }     
     else if (def == 1) {
        legPos = 0;
        first = 1;
     }
     else if (walk == 1) {
        legPos = stick/8;
        first = 1;
     }
     else if (first == 0) {
        legPos = angle;
     } 
     
     legPos = constrain(legPos, -40,20);

     Input1 = angle;
     Setpoint1 = legPos;
     PID1.Compute();
     
     if (Output1 > 0) {
       analogWrite(2, 0);
       analogWrite(3, Output1);
     }
     else if (Output1 < 0) {
      Output1a = abs(Output1);
      analogWrite(2, Output1a);
      analogWrite(3, 0);
     }
     else {
      analogWrite(2, 0);
      analogWrite(3, 0);
     }

  }  // end of timed loop


} // end of main loop
