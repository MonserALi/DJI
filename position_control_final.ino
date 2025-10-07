#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include <PS4Controller.h>

// ==========================
// MCP2515 PIN & CAN SETUP
// ==========================
#define CAN_CS 5
#define CAN_INT 4
MCP_CAN CAN(CAN_CS);

// ==========================
// ROBOT GEOMETRY CONFIG
// ==========================
const float WHEEL_RADIUS = 0.065f;        // meters (5 cm)
const float TRACK_WIDTH = 0.3f;         // meters (distance between left/right wheels)
const float GEAR_RATIO = 19.0f;           // Set to your actual gear ratio
const int TICKS_PER_MOTOR_REV = 8192;    // C620 encoder resolution
const float TICKS_PER_WHEEL_REV = TICKS_PER_MOTOR_REV * GEAR_RATIO;

// ==========================
// PID PARAMETERS
// ==========================
float Kp = 0.f;      // Reduced from 0.89 - less aggressive
float Ki = 0.001f;    // Reduced from 0.01 - less integral windup
float Kd = 0.05f;     // Reduced from 0.08 - less derivative action
const int16_t MAX_RPM = 700;
const float INTEGRAL_LIMIT = 5e5f;
const int32_t POSITION_DEADBAND = 130;  // Larger deadband to stop oscillation (was 30)

// ==========================
// CONTROL TIMING
// ==========================
unsigned long lastSend = 0;
unsigned long lastControl = 0;
const unsigned long sendInterval = 20;   // 50 Hz control loop

// ==========================
// MOTOR STRUCTURE
// ==========================
struct Motor {
  // Encoder tracking
  uint16_t last_raw_angle = 0;
  int64_t total_ticks = 0;
  int64_t target_ticks = 0;
  int16_t latest_rpm_feedback = 0;
  
  // PID state
  float integral = 0.0f;
  int64_t last_error = 0;
  int64_t last_position = 0;
  
  // Status
  bool position_reached = true;
};

// Motor indices: 0=LF, 1=LB, 2=RF, 3=RB
Motor motors[4];

// ==========================
// CAN IDs
// ==========================
const int sendID = 0x200;
const int feedbackIDs[4] = {0x202, 0x204, 0x203, 0x201}; // LF, LB, RF, RB

// ==========================
// CONTROL MODE
// ==========================
enum ControlMode {
  MODE_VELOCITY,   // Direct joystick control (active stick)
  MODE_POSITION    // Position hold (stick released)
};
ControlMode controlMode = MODE_POSITION;  // Start in position mode

// ==========================
// PS4 VARIABLES
// ==========================
int xData = 0, yData = 0;
int X = 0, Y = 0;

// ==========================
// UTILITY FUNCTIONS
// ==========================
inline float wheelCircumference() {
  return 2.0f * M_PI * WHEEL_RADIUS;
}

inline float metersToTicks(float meters) {
  float revs = meters / wheelCircumference();
  return revs * TICKS_PER_WHEEL_REV;
}

inline float ticksToMeters(int64_t ticks) {
  return (float)ticks / TICKS_PER_WHEEL_REV * wheelCircumference();
}

// Handle encoder wraparound (0..8191)
int wrapDiff(int new_raw, int old_raw) {
  int diff = new_raw - old_raw;
  if (diff > (TICKS_PER_MOTOR_REV / 2)) diff -= TICKS_PER_MOTOR_REV;
  else if (diff < -(TICKS_PER_MOTOR_REV / 2)) diff += TICKS_PER_MOTOR_REV;
  return diff;
}

// ==========================
// ENCODER FEEDBACK HANDLER
// ==========================
void handleCANFeedback(long unsigned int rxId, byte rxBuf[]) {
  // Find which motor this feedback is for
  int motorIdx = -1;
  for (int i = 0; i < 4; i++) {
    if (rxId == feedbackIDs[i]) {
      motorIdx = i;
      break;
    }
  }
  if (motorIdx == -1) return;
  
  Motor &m = motors[motorIdx];
  
  // Parse encoder angle (bytes 0-1, big endian, 0..8191)
  uint16_t raw_angle = ((uint16_t)rxBuf[0] << 8) | rxBuf[1];
  int diff = wrapDiff((int)raw_angle, (int)m.last_raw_angle);
  m.total_ticks += diff;
  m.last_raw_angle = raw_angle;
  
  // Parse RPM feedback (bytes 2-3, signed)
  int16_t rpm_fb = (int16_t)(((uint16_t)rxBuf[2] << 8) | rxBuf[3]);
  m.latest_rpm_feedback = rpm_fb;
  
  // Check if position reached
  int64_t error = m.target_ticks - m.total_ticks;
  m.position_reached = (abs(error) < POSITION_DEADBAND);
}

// ==========================
// POSITION PID CONTROLLER
// ==========================
int16_t computePIDandGetRPM(Motor &m) {
  int64_t error = m.target_ticks - m.total_ticks;
  
  // Larger deadband to prevent oscillation
  if (abs(error) < POSITION_DEADBAND) {
    m.integral = 0;
    m.last_error = error;
    m.last_position = m.total_ticks;  // Update last position even in deadband
    return 0;
  }
  
  // Conditional integration - only when error is moderate
  if (abs(error) < 3000) {
    m.integral += (float)error * (sendInterval / 1000.0f);
    m.integral = constrain(m.integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  } else {
    // Large errors - reset integral to prevent windup
    m.integral = 0;
  }
  
  // Derivative on measurement (avoids derivative kick)
  int64_t position_change = m.total_ticks - m.last_position;
  float derivative = -(float)position_change / (sendInterval / 1000.0f);
  m.last_position = m.total_ticks;
  
  // PID calculation
  float pid_out = Kp * (float)error + Ki * m.integral + Kd * derivative;
  
  // Convert to RPM and clamp
  int16_t rpm_cmd = (int16_t)round(pid_out);
  rpm_cmd = constrain(rpm_cmd, -MAX_RPM, MAX_RPM);
  
  m.last_error = error;
  return rpm_cmd;
}

// ==========================
// DIFFERENTIAL DRIVE KINEMATICS
// ==========================
void moveByBodyDelta(float dx_meters, float dtheta_radians) {
  // Calculate left and right wheel displacements
  float s_left = dx_meters - dtheta_radians * (TRACK_WIDTH / 2.0f);
  float s_right = dx_meters + dtheta_radians * (TRACK_WIDTH / 2.0f);
  
  int32_t ticks_left = (int32_t)round(metersToTicks(s_left));
  int32_t ticks_right = (int32_t)round(metersToTicks(s_right));
  
  // Apply to motors - match your velocity control directions
  // Forward: LF+, LB+, RF-, RB-
  // So left side is positive forward, right side is negative forward
  motors[0].target_ticks += ticks_left;    // LF (positive = forward)
  motors[1].target_ticks += ticks_left;    // LB (positive = forward)
  motors[2].target_ticks -= ticks_right;   // RF (negative = forward)
  motors[3].target_ticks -= ticks_right;   // RB (negative = forward)
  
  // Reset position_reached flags
  for (int i = 0; i < 4; i++) {
    motors[i].position_reached = false;
    motors[i].integral = 0;  // Reset integral on new move
  }
  
  Serial.printf("Move cmd: dx=%.3f m, dθ=%.3f rad (L=%d R=%d ticks)\n", 
                dx_meters, dtheta_radians, ticks_left, ticks_right);
}

void stopAllPosition() {
  for (int i = 0; i < 4; i++) {
    motors[i].target_ticks = motors[i].total_ticks;
    motors[i].integral = 0;
    motors[i].last_error = 0;
  }
  Serial.println("Position STOP - targets locked to current");
}

// ==========================
// VELOCITY MODE FUNCTIONS (Original behavior)
// ==========================
int16_t LF_vel = 0, LB_vel = 0, RF_vel = 0, RB_vel = 0;

void forward_vel(int speed) {
  LF_vel = speed;
  RB_vel = -speed;
  LB_vel = speed;
  RF_vel = -speed;
}

void backward_vel(int speed) {
  LF_vel = -speed;
  LB_vel = -speed;
  RF_vel = speed;
  RB_vel = speed;
}

void left_vel(int speed) {
  LF_vel = -speed;
  LB_vel = -speed;
  RB_vel = -speed;
  RF_vel = -speed;
}

void right_vel(int speed) {
  RB_vel = speed;
  RF_vel = speed;
  LF_vel = speed;
  LB_vel = speed;
}

void stopAll_vel() {
  LF_vel = LB_vel = RF_vel = RB_vel = 0;
}

void axis_logic_velocity(int X, int Y) {
  int deadzone = 50;  // Match the mode-switching deadzone
  
  if (abs(X) < deadzone && abs(Y) < deadzone) {
    stopAll_vel();
    return;
  }
  
  if (Y > deadzone && abs(X) < Y) {
    forward_vel(Y);
  } else if (Y < -deadzone && abs(X) < abs(Y)) {
    backward_vel(-Y);
  } else if (X > deadzone && abs(X) > abs(Y)) {
    right_vel(X);
  } else if (X < -deadzone && abs(X) > abs(Y)) {
    left_vel(-X);
  } else {
    stopAll_vel();
  }
}

// ==========================
// POSITION MODE JOYSTICK LOGIC
// ==========================
void axis_logic_position(int X, int Y) {
  static unsigned long lastMoveTime = 0;
  const unsigned long moveInterval = 200;  // Min time between moves (ms)
  int deadzone = 30;
  
  // Check if all motors reached their targets
  bool allReached = true;
  for (int i = 0; i < 4; i++) {
    if (!motors[i].position_reached) {
      allReached = false;
      break;
    }
  }
  
  // Only accept new commands if motors are done AND enough time passed
  if (!allReached || (millis() - lastMoveTime < moveInterval)) {
    return;
  }
  
  if (abs(X) < deadzone && abs(Y) < deadzone) {
    return;  // No movement
  }
  
  // Convert joystick to small incremental moves
  float scale = 0.02f;  // meters per command (tune this!)
  float dx = (Y / 1500.0f) * scale;      // Forward/backward
  float dtheta = (X / 1500.0f) * 0.3f;   // Turn (radians, tune this!)
  
  moveByBodyDelta(dx, dtheta);
  lastMoveTime = millis();
}

// ==========================
// CAN SEND FUNCTION
// ==========================
void sendMotorData(int16_t rpm_LF, int16_t rpm_LB, int16_t rpm_RF, int16_t rpm_RB) {
  byte data[8];
  
  // Pack RPMs for 4 motors (2 bytes each, big endian)
  // Order matches your feedback IDs: LF=0x202, LB=0x204, RF=0x203, RB=0x201
  // But 0x200 expects motors in order: 0x201, 0x202, 0x203, 0x204
  // So: RB, LF, RF, LB
  
  data[0] = (rpm_RB >> 8) & 0xFF;
  data[1] = rpm_RB & 0xFF;
  data[2] = (rpm_LF >> 8) & 0xFF;
  data[3] = rpm_LF & 0xFF;
  data[4] = (rpm_RF >> 8) & 0xFF;
  data[5] = rpm_RF & 0xFF;
  data[6] = (rpm_LB >> 8) & 0xFF;
  data[7] = rpm_LB & 0xFF;
  
  CAN.sendMsgBuf(sendID, 0, 8, data);
}

// ==========================
// SETUP
// ==========================
void setup() {
  Serial.begin(115200);
  delay(200);
  
  Serial.println("=== Position Control Robot Starting ===");
  
  // Initialize CAN
  if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("✅ CAN Initialized");
  } else {
    Serial.println("❌ CAN Init Failed!");
    while (1);
  }
  CAN.setMode(MCP_NORMAL);
  pinMode(CAN_INT, INPUT);
  
  // Initialize PS4
  PS4.begin("18:67:B0:51:0E:1D");
  Serial.println("✅ PS4 Controller Ready");
  
  Serial.println("⏳ Reading initial encoder positions...");
  delay(500);  // Wait for CAN to stabilize
  
  // Read initial encoder positions from motors
  unsigned long startTime = millis();
  bool allMotorsRead = false;
  bool motorReadFlags[4] = {false, false, false, false};
  
  while (millis() - startTime < 2000 && !allMotorsRead) {  // 2 second timeout
    if (!digitalRead(CAN_INT)) {
      long unsigned int rxId;
      byte len;
      byte rxBuf[8];
      if (CAN.readMsgBuf(&rxId, &len, rxBuf) == CAN_OK) {
        // Check which motor this is
        for (int i = 0; i < 4; i++) {
          if (rxId == feedbackIDs[i] && !motorReadFlags[i]) {
            uint16_t raw_angle = ((uint16_t)rxBuf[0] << 8) | rxBuf[1];
            motors[i].last_raw_angle = raw_angle;
            motors[i].total_ticks = 0;  // Start from zero
            motors[i].target_ticks = 0;  // Target is also zero
            motors[i].integral = 0;
            motors[i].last_error = 0;
            motors[i].last_position = 0;
            motors[i].position_reached = true;
            motorReadFlags[i] = true;
            Serial.printf("  Motor %d initialized at angle %d\n", i, raw_angle);
          }
        }
      }
    }
    
    // Check if all motors have been read
    allMotorsRead = true;
    for (int i = 0; i < 4; i++) {
      if (!motorReadFlags[i]) {
        allMotorsRead = false;
        break;
      }
    }
  }
  
  if (!allMotorsRead) {
    Serial.println("⚠ Warning: Not all motors detected!");
    for (int i = 0; i < 4; i++) {
      if (!motorReadFlags[i]) {
        Serial.printf("  Motor %d not responding\n", i);
      }
    }
  } else {
    Serial.println("✅ All motors initialized");
  }
  
  Serial.println("\nControls:");
  Serial.println("  Right Stick = Move (auto velocity mode)");
  Serial.println("  Release Stick = Hold position (auto position mode)");
  Serial.println("  Circle = Emergency stop");
  Serial.println("  Triangle = Print status");
  
  lastControl = millis();
}

// ==========================
// MAIN LOOP
// ==========================
void loop() {
  // Read CAN feedback
  if (!digitalRead(CAN_INT)) {
    long unsigned int rxId;
    byte len;
    byte rxBuf[8];
    if (CAN.readMsgBuf(&rxId, &len, rxBuf) == CAN_OK) {
      handleCANFeedback(rxId, rxBuf);
    }
  }
  
  // Read PS4 controller
  if (PS4.isConnected()) {
    xData = PS4.RStickX();
    yData = PS4.RStickY();
    X = map(xData, -128, 127, -MAX_RPM, MAX_RPM);
    Y = map(yData, -128, 127, -MAX_RPM, MAX_RPM);
    
    // Auto-switch modes based on joystick position
    int deadzone = 50;  // Increased deadzone for mode switching (was 20)
    if (abs(X) > deadzone || abs(Y) > deadzone) {
      // Stick is active -> velocity mode
      if (controlMode == MODE_POSITION) {
        controlMode = MODE_VELOCITY;
        Serial.println("→ VELOCITY mode");
      }
    } else {
      // Stick released -> position mode (hold position)
      if (controlMode == MODE_VELOCITY) {
        controlMode = MODE_POSITION;
        // IMMEDIATE BRAKE: Lock current positions as targets
        for (int i = 0; i < 4; i++) {
          motors[i].target_ticks = motors[i].total_ticks;
          motors[i].integral = 0;
          motors[i].last_error = 0;
          motors[i].position_reached = true;
        }
        // Send ZERO RPM command immediately for instant brake
        sendMotorData(0, 0, 0, 0);
        Serial.println("→ POSITION hold (BRAKE)");
      }
    }
    
    // Emergency stop button (Circle)
    if (PS4.Circle()) {
      for (int i = 0; i < 4; i++) {
        motors[i].target_ticks = motors[i].total_ticks;
        motors[i].integral = 0;
      }
      Serial.println("⚠ EMERGENCY STOP");
    }
    
    // Status button (Triangle)
    if (PS4.Triangle()) {
      Serial.println("\n=== Motor Status ===");
      for (int i = 0; i < 4; i++) {
        const char* names[] = {"LF", "LB", "RF", "RB"};
        Serial.printf("%s: pos=%lld tgt=%lld err=%lld rpm=%d %s\n",
                      names[i],
                      motors[i].total_ticks,
                      motors[i].target_ticks,
                      motors[i].target_ticks - motors[i].total_ticks,
                      motors[i].latest_rpm_feedback,
                      motors[i].position_reached ? "✓" : "...");
      }
      delay(300);
    }
  }
  
  // Control loop
  unsigned long now = millis();
  if (now - lastControl >= sendInterval) {
    lastControl = now;
    
    int16_t rpm_cmds[4] = {0, 0, 0, 0};
    
    if (controlMode == MODE_VELOCITY) {
      // Original velocity control
      axis_logic_velocity(X, Y);
      rpm_cmds[0] = LF_vel;
      rpm_cmds[1] = LB_vel;
      rpm_cmds[2] = RF_vel;
      rpm_cmds[3] = RB_vel;
      
    } else {
      // Position control
      axis_logic_position(X, Y);
      
      // Compute PID for each motor
      for (int i = 0; i < 4; i++) {
        rpm_cmds[i] = computePIDandGetRPM(motors[i]);
      }
    }
    
    // Send commands
    sendMotorData(rpm_cmds[0], rpm_cmds[1], rpm_cmds[2], rpm_cmds[3]);
    
    lastSend = now;
  }
  
  delay(1);
}
