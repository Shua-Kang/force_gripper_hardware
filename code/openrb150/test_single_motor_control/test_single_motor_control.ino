/*
 * OpenRB-150 PWM Control Receiver
 * Function: Receive digital strings from serial port, control motor PWM values
 */

 #include <Dynamixel2Arduino.h>

// --- User Configuration Area ---
#define DXL_SERIAL   Serial1 // OpenRB DXL port
#define DEBUG_SERIAL Serial  // USB serial (connects to computer)
const uint8_t DXL_ID = 2;    // Specify Motor ID here
const float DXL_PROTOCOL_VERSION = 2.0;
const int32_t BAUDRATE = 57600; // Motor baud rate (default is usually 57600)
// ----------------

// Initialize DXL instance
Dynamixel2Arduino dxl(DXL_SERIAL, -1); // OpenRB doesn't need DIR pin

void setup() {
  // 1. Initialize serial ports
  DEBUG_SERIAL.begin(115200); // Computer communication baud rate
   dxl.begin(BAUDRATE);
   dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
 
  // 2. Ping motor to ensure connection is normal
  if (dxl.ping(DXL_ID)) {
    DEBUG_SERIAL.print("Found Dynamixel ID: ");
    DEBUG_SERIAL.println(DXL_ID);
  } else {
    DEBUG_SERIAL.println("Failed to find Dynamixel. Check connections/ID.");
    while(1); // Stop execution
  }

  // 3. Configure motor mode
  // Must turn off torque before changing operating mode
  dxl.torqueOff(DXL_ID);

  // Set to PWM Control Mode (mode 16)
  // Note: In this mode, voltage ratio is controlled directly, suitable for wheels or fans, but no speed closed-loop control
  if(dxl.setOperatingMode(DXL_ID, OP_PWM)) {
      DEBUG_SERIAL.println("Mode set to PWM Control.");
  } else {
      DEBUG_SERIAL.println("Failed to set PWM Mode.");
  }

  // Turn on torque
  dxl.torqueOn(DXL_ID);
   DEBUG_SERIAL.println("Ready to receive PWM commands...");
 }
 
void loop() {
  // Check if computer has sent data
  if (DEBUG_SERIAL.available() > 0) {
    // Read string until newline
    String inputString = DEBUG_SERIAL.readStringUntil('\n');

    // Clean whitespace
    inputString.trim();

    if (inputString.length() > 0) {
      // Convert string to integer
      int pwm_value = inputString.toInt();

      // XL430 PWM limits are typically between +/- 885 (~100% duty cycle)
      // For safety, you could add clamping here, or send directly

      // Set PWM value
      // Positive: forward rotation, Negative: reverse rotation, 0: stop
      dxl.setGoalPWM(DXL_ID, pwm_value);
 
       DEBUG_SERIAL.print("Set PWM to: ");
       DEBUG_SERIAL.println(pwm_value);
     }
   }
 }