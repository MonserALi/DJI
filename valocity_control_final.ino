#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include<PS4Controller.h>

// ==========================
// MCP2515 PIN & CAN SETUP
// ==========================
#define CAN_CS 5
#define CAN_INT 4
MCP_CAN CAN(CAN_CS);

// ==========================
// MOTOR VARIABLES
// ==========================
int LF = 0, LB = 0, RF = 0, RB = 0;
int xData = 0, yData = 0;
int X = 0, Y = 0;
int mode = 0;
int speed = 800;  // Default speed

// ==========================
// TIMING
// ==========================
unsigned long lastSend = 0;
const unsigned long sendInterval = 5;

// ==========================
// CAN IDs
// ==========================
const int sendID = 0x200;
const int feedbackID_LF = 0x202;
const int feedbackID_LB = 0x204;
const int feedbackID_RF = 0x203;
const int feedbackID_RB = 0x201;

// ==========================
// SETUP
// ==========================
void setup() {
  Serial.begin(115200);

  if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("✅ CAN Initialized Successfully");
  else {
    Serial.println("❌ CAN Initialization Failed");
    while (1);
  }

  CAN.setMode(MCP_NORMAL);
  pinMode(CAN_INT, INPUT);
  PS4.begin("18:67:B0:51:0E:1D");
  Serial.println("PS4 ready ");

}
// ==========================
// MOVEMENT FUNCTIONS
// ==========================
void backward(int speed) {
 LF = -speed;  //unchanged
  LB = speed;//rb LB will be rb   1 for LB
  LB = -speed;//lb RF will be lb  2 for lb
  RB = speed;//rf   RB will be rf 3 for rb
}

void forward(int speed) {
  LF = speed;
 RB = -speed;//rb LB will be rb 
  LB = speed;//lb  RF will be lb
  RF = -speed;//rf RB wiull be rf
}

void left(int speed) {
  LF = -speed;//unchanged 
  LB = -speed;//lb RF will be LB
  RB =  -speed;
  RF =  -speed;
}

void right(int speed) {
  RB = speed;//rb LB will be rb
  RF = speed;//rf RB will be rf
  LF = speed;//unchanged 
  LB = speed;//lb RF will be LB

}

void stopAll() {
  LF = LB = RF = RB = 0;
}

void axis_logic(int X, int Y){
  int deadzone = 20;

  if(abs(X)<deadzone && abs(Y)<deadzone){
    stopAll();
    return;
  }
  if (Y > deadzone && abs(X) < Y) {
    forward(Y);
  } 
  else if (Y < -deadzone && abs(X) < abs(Y)) {
    backward(-Y);
  }
  // Turning
  else if (X > deadzone && abs(X) > abs(Y)) {
    right(X);
  } 
  else if (X < -deadzone && abs(X) > abs(Y)) {
    left(-X);
  } 
  else {
    stopAll();
  }

}

// ==========================
// CAN SEND FUNCTION
// ==========================
void sendMotorData() {
  byte data[8];
  data[0] = (RB >> 8) & 0xFF;// LB rb hoyeche
  data[1] = RF & 0xFF;//rb to rf
  data[2] = (LF >> 8) & 0xFF;
  data[3] = LF & 0xFF;
  data[4] = (RB >> 8) & 0xFF;// LB theke rb hoyeche
  data[5] = RF & 0xFF;// rb to rf 
  data[6] = (LB >> 8) & 0xFF;//rf thrkr lb
  data[7] = LB & 0xFF;//rf theke lb hoyeche

  CAN.sendMsgBuf(sendID, 0, 8, data);
  Serial.printf("LF=%d  LB=%d  RF=%d  RB=%d\n", LF, LB, RF, RB);
}



// ==========================
// LOOP
// ==========================
void loop() {
  // Read joystick
  xData = PS4.RStickX();
  yData = PS4.RStickY();

  X = map(xData,-128,127,-1500,1500);
  Y = map(yData,-128,127,-1500,1500);

  if(mode == 0){
    axis_logic(X,Y);
  }

  // Send data at interval
  if (millis() - lastSend >= sendInterval) {
    sendMotorData();
    lastSend = millis();
  }

  // Optional feedback handling
  if (!digitalRead(CAN_INT)) {
    long unsigned int rxId;
    byte len;
    byte rxBuf[8];
    if (CAN.readMsgBuf(&rxId, &len, rxBuf) == CAN_OK) {
      if (rxId == feedbackID_LF) {
        int16_t rpm = (rxBuf[2] << 8) | rxBuf[3];
        int16_t current = (rxBuf[4] << 8) | rxBuf[5];
        int8_t temp = rxBuf[6];
        // Optional: Serial.printf("RPM:%d Current:%d Temp:%d\n", rpm, current, temp);
      }
    }
  }
}
