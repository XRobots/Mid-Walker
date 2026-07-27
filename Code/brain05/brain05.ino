#include <Arduino.h>
#include "ODriveCAN.h"
#include <Wire.h>

// include the library for the AS5047P sensor.
#include <AS5047P.h>
// define the chip select port.
#define AS5047P_CHIP_SELECT_PORT 10
// define the spi bus speed 
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000
// initialize a new AS5047P sensor object.
AS5047P as5047p(AS5047P_CHIP_SELECT_PORT, AS5047P_CUSTOM_SPI_BUS_SPEED);

// CAN bus baudrate. Make sure this matches for every device on the bus
#define CAN_BAUDRATE 250000

// ODrive node_id for odrvives
#define ODRV0_NODE_ID 0
#define ODRV1_NODE_ID 1
// Uncomment below the line that corresponds to your hardware.
// See also "Board-specific settings" to adapt the details for your hardware setup.

#define IS_TEENSY_BUILTIN // Teensy boards with built-in CAN interface (e.g. Teensy 4.1). See below to select which interface to use.
// #define IS_ARDUINO_BUILTIN // Arduino boards with built-in CAN interface (e.g. Arduino Uno R4 Minima)
// #define IS_MCP2515 // Any board with external MCP2515 based extension module. See below to configure the module.

/* Board-specific includes ---------------------------------------------------*/

#if defined(IS_TEENSY_BUILTIN) + defined(IS_ARDUINO_BUILTIN) + defined(IS_MCP2515) != 1
#warning "Select exactly one hardware option at the top of this file."

#if CAN_HOWMANY > 0 || CANFD_HOWMANY > 0
#define IS_ARDUINO_BUILTIN
#warning "guessing that this uses HardwareCAN"
#else
#error "cannot guess hardware version"
#endif

#endif

#ifdef IS_ARDUINO_BUILTIN
// See https://github.com/arduino/ArduinoCore-API/blob/master/api/HardwareCAN.h
// and https://github.com/arduino/ArduinoCore-renesas/tree/main/libraries/Arduino_CAN

#include <Arduino_CAN.h>
#include <ODriveHardwareCAN.hpp>
#endif // IS_ARDUINO_BUILTIN

#ifdef IS_MCP2515
// See https://github.com/sandeepmistry/arduino-CAN/
#include "MCP2515.h"
#include "ODriveMCPCAN.hpp"
#endif // IS_MCP2515

# ifdef IS_TEENSY_BUILTIN
// See https://github.com/tonton81/FlexCAN_T4
// clone https://github.com/tonton81/FlexCAN_T4.git into /src
#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"
struct ODriveStatus; // hack to prevent teensy compile error
#endif // IS_TEENSY_BUILTIN

/* Board-specific settings ---------------------------------------------------*/

/* Teensy */

#ifdef IS_TEENSY_BUILTIN

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;

bool setupCan() {
  can_intf.begin();
  can_intf.setBaudRate(CAN_BAUDRATE);
  can_intf.setMaxMB(16);
  can_intf.enableFIFO();
  can_intf.enableFIFOInterrupt();
  can_intf.onReceive(onCanMessage);
  return true;
}

#endif // IS_TEENSY_BUILTIN

/* MCP2515-based extension modules -*/

#ifdef IS_MCP2515

MCP2515Class& can_intf = CAN;

// chip select pin used for the MCP2515
#define MCP2515_CS 10

// interrupt pin used for the MCP2515
// NOTE: not all Arduino pins are interruptable, check the documentation for your board!
#define MCP2515_INT 2

// freqeuncy of the crystal oscillator on the MCP2515 breakout board. 
// common values are: 16 MHz, 12 MHz, 8 MHz
#define MCP2515_CLK_HZ 8000000


static inline void receiveCallback(int packet_size) {
  if (packet_size > 8) {
    return; // not supported
  }
  CanMsg msg = {.id = (unsigned int)CAN.packetId(), .len = (uint8_t)packet_size};
  CAN.readBytes(msg.buffer, packet_size);
  onCanMessage(msg);
}

bool setupCan() {
  // configure and initialize the CAN bus interface
  CAN.setPins(MCP2515_CS, MCP2515_INT);
  CAN.setClockFrequency(MCP2515_CLK_HZ);
  if (!CAN.begin(CAN_BAUDRATE)) {
    return false;
  }

  CAN.onReceive(receiveCallback);
  return true;
}

#endif // IS_MCP2515


/* Arduinos with built-in CAN */

#ifdef IS_ARDUINO_BUILTIN

HardwareCAN& can_intf = CAN;

bool setupCan() {
  return can_intf.begin((CanBitRate)CAN_BAUDRATE);
}

#endif

// Instantiate ODrive objects
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID); // Standard CAN message ID
ODriveCAN odrv1(wrap_can_intf(can_intf), ODRV1_NODE_ID); // Standard CAN message ID
ODriveCAN* odrives[] = {&odrv0, &odrv1}; // Make sure all ODriveCAN instances are accounted for here

struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

// Keep some application-specific user data for every ODrive.
ODriveUserData odrv0_user_data;
ODriveUserData odrv1_user_data;

// Called every time a Heartbeat message arrives from the ODrive
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data_ptr) {
  ODriveUserData* odrv_user_data_ptr = static_cast<ODriveUserData*>(user_data_ptr);
  odrv_user_data_ptr->last_heartbeat = msg;
  odrv_user_data_ptr->received_heartbeat = true;
}

// Called every time a feedback message arrives from the ODrive
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data_ptr) {
  ODriveUserData* odrv_user_data_ptr = static_cast<ODriveUserData*>(user_data_ptr);
  odrv_user_data_ptr->last_feedback = msg;
  odrv_user_data_ptr->received_feedback = true;
}

// Called for every message that arrives on the CAN bus
void onCanMessage(const CanMsg& msg) {
  for (auto odrive: odrives) {
    onReceive(msg, *odrive);
  }

}
#include "SparkFun_BNO08x_Arduino_Library.h"  // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x
BNO08x myIMU;

#define BNO08X_INT  2
#define BNO08X_RST  3
#define BNO08X_ADDR 0x4B  // SparkFun BNO08x Breakout (Qwiic) defaults to 0x4B

int marker;
int joy1Th;
int joy2Th;
int joy1ThScaled;
int joy2ThScaled;
int joy2ThScaledBM;
int joy2ThScaledR;
int joy2ThScaledL;
int total1;
int total2;

int sw1;
int sw2;
int sw3;
int sw4;

int but1;
int but2;
int but3;
int but4;
int but5;
int but6;
int but7;
int but8;
int but9;
int but10;
int but11;
int but12;
int but13;
int but14;

int rot;

int jog = 0;
int def = 0;
int walk = 0;

int leg1Pos;      // up and down positions for the legs
int leg2Pos;
int leg3Pos;
int leg4Pos;

int leg1PosI;      // interpolated lef postions
int leg2PosI;
int leg3PosI;
int leg4PosI;

float axis1Pos;
float axis1PosI;
float axis1PosIScaled;
float axis1PosIScaledFiltered;

float axis2Pos;
float axis2PosI;
float axis2PosIScaled;
float axis2PosIScaledFiltered;

int rot1 = 0;     // move rotational axis in each direction
int rot2 = 0;

int rotOpen = 0;    // set off opening or closing. 0 = closed / 1 = open

int leg1Jog1 = 0;     // jog up;
int leg1Jog2 = 0;     // jog down;
int leg2Jog1 = 0;     // jog up;
int leg2Jog2 = 0;     // jog down;
int leg3Jog1 = 0;     // jog up;
int leg3Jog2 = 0;     // jog down;
int leg4Jog1 = 0;     // jog up;
int leg4Jog2 = 0;     // jog down;

int clFlag = 0;   // ODrive init flag
float ODrive0Pos;   // ODrive output pos
float ODrive0PosI;   // Interpolated ODrive output pos

float ODrive1Pos;   // ODrive output pos
float ODrive1PosI;   // Interpolated ODrive output pos

unsigned long currentMillis;
unsigned long previousMillis = 0;        // set up timers
unsigned long previousWalkMillis = 0;        // set up timers
unsigned long previousRotMillis = 0;        // set up timers
int rotState = 0;
long interval = 10;             // time constant for timer
int walkState = 0;

#include <Ramp.h>

class Interpolation {
  public:
    rampInt myRamp;
    int interpolationFlag = 0;
    int savedValue;

    int go(int input, int duration) {

      if (input != savedValue) {   // check for new data
        interpolationFlag = 0;
      }
      savedValue = input;          // bookmark the old value

      if (interpolationFlag == 0) {                                   // only do it once until the flag is reset
        myRamp.go(input, duration, LINEAR, ONCEFORWARD);              // start interpolation (value to go to, duration)
        interpolationFlag = 1;
      }

      int output = myRamp.update();
      return output;
    }
};    // end of class

Interpolation interpLeg1;
Interpolation interpLeg2;
Interpolation interpLeg3;
Interpolation interpLeg4;
Interpolation interpAxis1;
Interpolation interpAxis2;

// motion filter to filter motions and compliance

float filter(float prevValue, float currentValue, int filter) {  
  float lengthFiltered =  (prevValue + (currentValue * filter)) / (filter + 1);  
  return lengthFiltered;  
}

void setup() {

  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);
  Serial4.begin(115200);
  Serial5.begin(115200);

  pinMode(13, OUTPUT);  // LED

  // Register callbacks for the heartbeat and encoder feedback messages
  odrv0.onFeedback(onFeedback, &odrv0_user_data);
  odrv0.onStatus(onHeartbeat, &odrv0_user_data);
  odrv1.onFeedback(onFeedback, &odrv1_user_data);
  odrv1.onStatus(onHeartbeat, &odrv1_user_data);

  // Configure and initialize the CAN bus interface. This function depends on
  // your hardware and the CAN stack that you're using.
  if (!setupCan()) {
    Serial.println("CAN failed to initialize: reset required");
    while (true); // spin indefinitely
  }

  Wire.begin();

  if (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false) {
    Serial.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1);
  }
  Serial.println("BNO08x found!");
  // Wire.setClock(400000); //Increase I2C data rate to 400kHz
  setReports();
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");
  if (myIMU.enableRotationVector() == true) {
    Serial.println(F("Rotation vector enabled"));
    Serial.println(F("Output in form roll, pitch, yaw"));
  } else {
    Serial.println("Could not enable rotation vector");
  }
}

void loop() {

  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {   // start of timed loop
      previousMillis = currentMillis;

    if (myIMU.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
    }

    // Has a new event come in on the Sensor Hub Bus?
    if (myIMU.getSensorEvent() == true) {
      // is it the correct sensor data we want?
      if (myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {  
      float roll = (myIMU.getRoll()) * 180.0 / PI; // Convert roll to degrees
      float pitch = (myIMU.getPitch()) * 180.0 / PI; // Convert pitch to degrees
      float yaw = (myIMU.getYaw()) * 180.0 / PI; // Convert yaw / heading to degrees
  
      }
    }

    if (Serial5.available() > 0){           // receive IMU data from serial IMU
        //Serial.println("data");
        digitalWrite(13, HIGH);
        marker = Serial5.parseInt();
          if (marker == 9000) {                   // look for check value to check it's the start of the data
              total1 = Serial5.parseInt();
              total2 = Serial5.parseInt();
              joy1Th = Serial5.parseInt();
              joy2Th = Serial5.parseInt();
              if (Serial5.read() == '\n') {     // end of IMU data 
              }
          }
     }
     else {
        digitalWrite(13, LOW);
     }

     // unpack pendant values

     sw1 = bitRead(total1, 0);
     sw2 = bitRead(total1, 1);
     sw3 = bitRead(total1, 2);
     sw4 = bitRead(total1, 3);

     but1 = bitRead(total1, 4);
     but2 = bitRead(total1, 5);
     but3 = bitRead(total1, 6);
     but4 = bitRead(total1, 7);
     but5 = bitRead(total1, 8);
     but6 = bitRead(total1, 9);
     but7 = bitRead(total1, 10);
     but8 = bitRead(total1, 11);
     
     but9 = bitRead(total2, 0);
     but10 = bitRead(total2, 1);
     but11 = bitRead(total2, 2);
     but12 = bitRead(total2, 3);
     but13 = bitRead(total2, 4);
     but14 = bitRead(total2, 5);

     if (sw1 == 1 && sw2 == 0 && sw3 == 0 && sw4 == 0) {      // jog mode
        jog = 1;
        def = 0;
        walk = 0;
        //Serial.println("jog mode");      
     }
     else if (sw2 == 1 && sw1 == 0 && sw3 == 0 && sw4 == 0) { // default positions
        jog = 0;
        def = 1;
        walk = 0;
        //Serial.println("default positions");      
     }
     else if (sw3 == 1 && sw1 == 0 && sw2 == 0) { // walk mode
        jog = 0;
        def = 0;
        walk = 1;
        //Serial.println("walk mode");      
     }
     else {     // do nothing
        jog = 0;
        def = 0;
        walk = 0;
     }

     // ODrive stuff

     if (sw1 == 0 && sw2 == 0 && sw3 == 0 && sw4 == 0 && but3 == 1 && but4 == 1 && clFlag == 0) {   // init odrives
        odrv0.setControllerMode(CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_PASSTHROUGH);
        odrv1.setControllerMode(CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_PASSTHROUGH);

        for (int i = 0; i < 15; ++i) {
          delay(10);
          pumpEvents(can_intf);
        }

        //Init Odrive 0

        Serial.println("Waiting for ODrive 0...");
        while (!odrv0_user_data.received_heartbeat) {
          pumpEvents(can_intf);
          delay(100);
        }
        Serial.println("found ODrive 0");

        Serial.println("Enabling closed loop control 0...");
          while (odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
          odrv0.clearErrors();
          delay(1);
          odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        }

        //Init Odrive 1

        Serial.println("Waiting for ODrive 1...");
        while (!odrv1_user_data.received_heartbeat) {
          pumpEvents(can_intf);
          delay(100);
        }
        Serial.println("found ODrive 0");

        Serial.println("Enabling closed loop control 0...");
          while (odrv1_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
          odrv1.clearErrors();
          delay(1);
          odrv1.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        }
        
        clFlag = 1;          // set flag
     }
     else if (jog == 0 && but3 == 0 && but4 == 0 && clFlag == 1) {
        clFlag = 0;          // reset flag
     }
     
     if (jog == 1 && def == 0 && walk == 0) {                            // jog axis
         if (but3 == 1 && but4 == 0) {
            ODrive0Pos = ODrive0Pos + 0.1;
         }
         else if (but3 == 0 && but4 == 1) {
            ODrive0Pos = ODrive0Pos - 0.1;
         }
         if (but7 == 1 && but8 == 0) {
            ODrive1Pos = ODrive1Pos + 0.1;
         }
         else if (but7 == 0 && but8 == 1) {
            ODrive1Pos = ODrive1Pos - 0.1;
         }
     }

     if (jog == 1 && def == 0 && walk == 0 && but3 == 1 && but4 == 1) {   // zero ODrive axis
          Set_Absolute_Position_msg_t abs_pos_msg;    //zero ODrives.
          abs_pos_msg.Position = 0.0f;
          odrv0.send(abs_pos_msg);
          ODrive0Pos = 0;       // zero pos variable         
     }
     if (jog == 1 && def == 0 && walk == 0 && but7 == 1 && but8 == 1) {   // zero ODrive axis
          Set_Absolute_Position_msg_t abs_pos_msg;    //zero ODrives.
          abs_pos_msg.Position = 0.0f;
          odrv1.send(abs_pos_msg);
          ODrive1Pos = 0;       // zero pos variable          
     } 
      
     if (def == 1 && jog ==0 && walk == 0) {
          ODrive0Pos = 0;  // zero position variable
          ODrive1Pos = 0;  // zero position variable
     }

     if (jog == 1) {
         if (but13 == 1) {      // rotation axis
            rot = 1;
         }
         else if (but14 == 1) {
            rot = 2;
         }
         else {
            rot = 0;
         }
     }
     
     joy2ThScaled = joy2Th*4;

     if (joy2ThScaled > 0) {
        joy2ThScaledR = joy2ThScaled;
     }
     else if (joy2ThScaled < 0) {
        joy2ThScaledL = abs(joy2ThScaled);
     }
     else {
        joy2ThScaledR = 0;
        joy2ThScaledL = 0;
     }

     leg1Jog1 = but1;
     leg1Jog2 = but2;
     leg2Jog1 = but5;
     leg2Jog2 = but6;
     leg3Jog1 = but9;
     leg3Jog2 = but10;
     leg4Jog1 = but11;
     leg4Jog2 = but12;

     // *********************************
     // *** start of walking sequence ***
     // *********************************

     // 0 rest state
     // 1 pick up two legs
     // 2 slide +/- or rotate +/-
     // 3 put feet down
     // 4 pick up other legs
     // 5 slide +/- or rotate +/-
     // 6 put feet down

     if (walk == 1 && sw4 == 1 && walkState == 0) {       // trigger
        leg1Pos = 0;
        leg2Pos = 0; 
        leg3Pos = 0; 
        leg4Pos = 0;
        //rot = 0;    
        previousWalkMillis = currentMillis;
        walkState = 1;                          // trigger walk sequence
     }
     else if (walkState == 1 && currentMillis - previousWalkMillis >= 1000) {           // start sequence
        leg1Pos = 0;
        leg2Pos = -250; 
        leg3Pos = 0; 
        leg4Pos = -250;
        //rot = 0;  
        previousWalkMillis = currentMillis;
        walkState = 2;
     }
     else if (walkState == 2 && currentMillis - previousWalkMillis >= 500) {
        axis1Pos = joy1Th*-0.75;
        axis2Pos = (joy1Th*-0.75);
        if (joy2ThScaledR > 0) {
            rotOpen = 1;
            joy2ThScaledBM = (abs(joy2ThScaled));
        }
        else if (joy2ThScaledL > 0 || rotOpen ==  1) {
            rotOpen = 2;
        }
        else {
            rotOpen = 0;
        }
        rotState = 0;        
        previousWalkMillis = currentMillis;
        walkState = 3;
     }
     else if (walkState == 3 && currentMillis - previousWalkMillis >= 1000 + (joy2ThScaledBM/6)) {
        leg1Pos = 0;
        leg2Pos = 0; 
        leg3Pos = 0; 
        leg4Pos = 0; 
        //rot = 0; 
        previousWalkMillis = currentMillis;
        walkState = 4;
     }
     else if (walkState == 4 && currentMillis - previousWalkMillis >= 1000) {
        leg1Pos = -250;
        leg2Pos = 0; 
        leg3Pos = -250; 
        leg4Pos = 0; 
        //rot = 0;
        previousWalkMillis = currentMillis;
        walkState = 5;
     } 
     else if (walkState == 5 && currentMillis - previousWalkMillis >= 500) {
        axis1Pos = joy1Th;
        axis2Pos = 0;
        if (joy2ThScaledR > 0 || rotOpen == 1) {
            rotOpen = 2;
        }
        else if (joy2ThScaledL > 0) {
            rotOpen = 1;
            joy2ThScaledBM = (abs(joy2ThScaled));
        }
        else {
            rotOpen = 0;
        }
        rotState = 0;        
        previousWalkMillis = currentMillis;
        walkState = 6;
     }
     else if (walkState == 6 && currentMillis - previousWalkMillis >= 1000 + (joy2ThScaledBM/6)) {
        leg1Pos = 0;
        leg2Pos = 0; 
        leg3Pos = 0; 
        leg4Pos = 0; 
        //rot = 0;
        previousWalkMillis = currentMillis;
        walkState = 0;
     }

     // *** END OF WALKING ***

     // *** START OF ROTATING = triggered above *** 

     if (rotOpen == 1 && rotState == 0) { // opening rotation move
        rot = 1;              //send opening command
        rotState = 1;         // increment state
        previousRotMillis = currentMillis;  // reset clock   
     }
     else if (rotState == 1 && currentMillis - previousRotMillis > joy2ThScaledBM) {
      rot = 0;    // turn off motors once time ahs expired
      rotState = 5;  // reset state
      previousRotMillis = currentMillis;
     }
     
     if (rotOpen == 2 && rotState == 0) {  // closing rotation move
        rot = 2;              // send closing command
        rotState = 3;         // increment state
        previousRotMillis = currentMillis;   // reset clock     
     }
     else if (rotState == 3 && currentMillis - previousRotMillis > joy2ThScaledBM + 500) {
        rot = 0;      // turn off motors
        rotState = 4; // reset state
        joy2ThScaledBM = 0;
        previousRotMillis = currentMillis;
     }

     Serial.print(walkState);
     Serial.print(" , ");
     Serial.print(rotOpen);
     Serial.print(" , ");
     Serial.print(rotState);
     Serial.print(" , ");
     Serial.print(rot);
     Serial.print(" , ");
     Serial.print(joy2ThScaledBM);
     Serial.println();

     

     // run interpolation on step values
     leg1PosI = interpLeg1.go(leg1Pos, 900);
     leg2PosI = interpLeg2.go(leg2Pos, 900);
     leg3PosI = interpLeg3.go(leg3Pos, 900);
     leg4PosI = interpLeg4.go(leg4Pos, 900);
     axis1PosI = interpAxis1.go(axis1Pos, 900);
     axis2PosI = interpAxis2.go(axis2Pos, 2500);

     axis1PosIScaled = axis1PosI/20;
     axis2PosIScaled = axis2PosI/20;

     axis1PosIScaledFiltered = filter(axis1PosIScaled, axis1PosIScaledFiltered, 20);
     axis2PosIScaledFiltered = filter(axis2PosIScaled, axis2PosIScaledFiltered, 20);

     if (walk == 1) {
        ODrive0Pos = axis1PosIScaledFiltered;
        ODrive1Pos = axis2PosIScaledFiltered;
     }
     
     /*
     Serial.print(walkState);
     Serial.print(" , ");
     Serial.print(joy2ThScaledR);
     Serial.print(" , ");
     Serial.print(joy2ThScaledL);
     Serial.print(" , ");
     Serial.print(rot);
     Serial.println();
     */
     
     // *** write to Odrive ***
     odrv0.setPosition(ODrive0Pos);
     odrv1.setPosition(ODrive1Pos);

     // *** send data to legs ***
     // *** LEG 1 ***
     Serial1.print(9000);     // start of data marker
     Serial1.print(" , ");
     Serial1.print(jog);
     Serial1.print(" , ");
     Serial1.print(def);
     Serial1.print(" , ");
     Serial1.print(walk);
     Serial1.print(" , ");
     Serial1.print(leg1Jog1);
     Serial1.print(" , ");
     Serial1.print(leg1Jog2);
     Serial1.print(" , ");
     Serial1.print(leg1PosI);
     Serial1.print(" , ");
     Serial1.print(rot);
     Serial1.print('\n');
     // *** LEG 2 ***
     Serial2.print(9000);     // start of data marker
     Serial2.print(" , ");
     Serial2.print(jog);
     Serial2.print(" , ");
     Serial2.print(def);
     Serial2.print(" , ");
     Serial2.print(walk);
     Serial2.print(" , ");
     Serial2.print(leg2Jog1);
     Serial2.print(" , ");
     Serial2.print(leg2Jog2);
     Serial2.print(" , ");
     Serial2.print(leg2PosI);
     Serial2.print('\n');
     // *** LEG 3 ***
     Serial3.print(9000);     // start of data marker
     Serial3.print(" , ");
     Serial3.print(jog);
     Serial3.print(" , ");
     Serial3.print(def);
     Serial3.print(" , ");
     Serial3.print(walk);
     Serial3.print(" , ");
     Serial3.print(leg3Jog1);
     Serial3.print(" , ");
     Serial3.print(leg3Jog2);
     Serial3.print(" , ");
     Serial3.print(leg3PosI);
     Serial3.print('\n');
     // *** LEG 4 ***
     Serial4.print(9000);     // start of data marker
     Serial4.print(" , ");
     Serial4.print(jog);
     Serial4.print(" , ");
     Serial4.print(def);
     Serial4.print(" , ");
     Serial4.print(walk);
     Serial4.print(" , ");
     Serial4.print(leg4Jog1);
     Serial4.print(" , ");
     Serial4.print(leg4Jog2);
     Serial4.print(" , ");
     Serial4.print(leg4PosI);
     Serial4.print('\n');
             
  }   // end of timed loop

  
} // end of main loop
