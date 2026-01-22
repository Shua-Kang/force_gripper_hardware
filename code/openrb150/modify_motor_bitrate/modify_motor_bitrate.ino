#include <Dynamixel2Arduino.h>
using namespace ControlTableItem;

// === User Settings ===
const uint8_t  DXL_ID     = 2;         // Your motor ID
const uint32_t OLD_BAUD   = 57600;     // Current baud rate
const uint32_t NEW_BAUD   = 1000000;   // Target baud rate (1 Mbps)

// === OpenRB-150 Interface Definition ===
#define DXL_SERIAL Serial1
const int DXL_DIR_PIN = -1;             // OpenRB onboard direction control, no external connection needed

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("=== Dynamixel Baud Rate Change ===");

  // Initialize with old baud rate
  dxl.begin(OLD_BAUD);
  dxl.setPortProtocolVersion(2.0);

  Serial.print("Connecting to DXL ID "); Serial.println(DXL_ID);

  // Turn off torque
  dxl.torqueOff(DXL_ID);
  delay(50);

  // Set to new baud rate
  // XL series: Control table BAUD_RATE = 1 → 57,600bps; 3 → 1,000,000bps
  dxl.writeControlTableItem((uint8_t)BAUD_RATE,
                            (uint8_t)DXL_ID,
                            (int32_t)3,         // 3 = 1 Mbps
                            (uint32_t)10);

  Serial.println("Baud rate changed to 1 Mbps (value=3).");

  // Re-initialize communication port to new baud rate
  dxl.begin(NEW_BAUD);
  delay(100);

  // Re-enable torque
  dxl.torqueOn(DXL_ID);
  Serial.println("Torque re-enabled.");
  Serial.println("✅ Done. Please remember to use 1 Mbps next time!");
}

void loop() {
  // Empty loop
}
