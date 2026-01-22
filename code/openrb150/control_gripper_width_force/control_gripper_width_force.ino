#include <Dynamixel2Arduino.h>
using namespace ControlTableItem;

// ======================= Ports/Baud Rates =======================
#define DXL_SERIAL   Serial1              // DYNAMIXEL bus fixed to Serial1
const int      DXL_DIR_PIN   = -1;
const uint32_t DXL_BAUD      = 1000000;   // If you changed to 1Mbps, set to 1000000

#define CMD_PORT     Serial               // Command input port: default USB. Change to Serial2 if using CH340
#define STAT_PORT    Serial2              // Status output port: default CH340. Change back to Serial if above is changed
const uint32_t CMD_BAUD   = 1000000;
const uint32_t STAT_BAUD  = 1000000;

// ======================= Motors & Control ========================
const uint8_t  DXL_ID_1    = 1;
const uint8_t  DXL_ID_2    = 2;
const int16_t  PWM_MIN     = -885;
const int16_t  PWM_MAX     =  885;

const float    KP          = 0.6f;        // Position error->PWM proportional coefficient (tunable)
const int32_t  OFFSET_COUNTS = 20;        // Initial position offset relative to open (counts)

// Initialization stability parameters (adjustable based on mechanics)
const int16_t  INIT_PWM_OPEN   =  200;    // PWM used when finding open-side limit (your definition: positive = open)
const int16_t  INIT_PWM_CLOSE  = -200;    // PWM used when finding close-side limit (your definition: negative = close)
const uint16_t STABLE_WINDOW   = 8;       // Consecutive N sampling windows
const int32_t  STABLE_EPS      = 2;       // Stability threshold (counts)
const uint32_t INIT_TIMEOUT_MS = 10000;   // Single segment search timeout

// 50Hz main loop
const uint32_t LOOP_HZ    = 50;
const uint32_t LOOP_DT_MS = 1000 / LOOP_HZ;

// ---- RAW addresses for X/XL series ----
const uint16_t ADDR_PRESENT_POSITION  = 132;
const uint16_t LEN_PRESENT_POSITION   = 4;

const int32_t  RETURN_OPEN_EPS      = 5;     // Tolerance for returning to open_limit (counts)
const uint32_t RETURN_OPEN_TIMEOUT  = 10000; // Timeout for returning to open_limit (ms)

// Write method switch: 1=bulkWrite; 0=twice writeControlTableItem (default 0, keeps your existing logic)
#define USE_BULK 0
// Read method switch: 1=syncRead; 0=twice readControlTableItem (comparison/fallback)
#define USE_SYNC_READ 1

// ======================= Control Modes ========================
// MODE_POS: Host sends "position target(0..1) + feedforward PWM", board uses KP to convert error to PWM
// MODE_PWM: Host sends PWM target directly, board forwards as-is
enum ControlMode : uint8_t { MODE_POS = 0, MODE_PWM = 1 };
volatile ControlMode g_mode = MODE_POS;  // Default position mode

// ======================= Global Objects/Cache ========================
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// syncRead structure (following new interface)
ParamForSyncReadInst_t    sync_read_param;
RecvInfoFromStatusInst_t  sync_read_result;

// Optional: bulkWrite structure (only when USE_BULK=1)
#if USE_BULK
const uint16_t ADDR_GOAL_PWM = 100;   // X/XL series Goal PWM address
const uint16_t LEN_GOAL_PWM  = 2;
ParamForBulkWriteInst_t   bulk_write_param;
#endif

// Position cache
volatile int32_t g_pos1 = 0, g_pos2 = 0;

// Limits and mapping
int32_t open_limit[2]  = {0, 0};
int32_t close_limit[2] = {4095, 4095};
bool    limits_valid   = false;

// Targets (position mode: 0~1) and feedforward PWM (used in position mode)
float   goal_norm_1 = 0.0f;
float   goal_norm_2 = 0.0f;
int16_t feed_pwm_1  = 0;
int16_t feed_pwm_2  = 0;

// Targets (PWM mode: direct output)
int16_t pwm_goal_1  = 0;
int16_t pwm_goal_2  = 0;

// ======================= Utility Functions ========================
static inline int16_t clamp_pwm(long v){
  if (v < PWM_MIN) return PWM_MIN;
  if (v > PWM_MAX) return PWM_MAX;
  return (int16_t)v;
}
static inline int32_t lerp_counts(int32_t a, int32_t b, float t){
  if (t < 0.f) t = 0.f; if (t > 1.f) t = 1.f; return a + (int32_t)((b - a) * t);
}

// ======================= DXL Initialization =========================
void setup_motors_to_pwm_mode(){
  const uint8_t ids[2] = {DXL_ID_1, DXL_ID_2};
  for (int i = 0; i < 2; ++i){
    uint8_t id = ids[i];
    if (!dxl.ping(id)){
      CMD_PORT.print("[ERR] ping failed id="); CMD_PORT.println(id);
      while (1) { delay(1000); }
    }
    dxl.torqueOff(id);
    dxl.writeControlTableItem((uint8_t)OPERATING_MODE, id, (int32_t)16, (uint32_t)10); // 16 = PWM(mode)
    
    // [Critical modification]
    // 1 = Reply only to READ instructions
    // 2 = Reply to both READ and WRITE instructions
    // Your writeControlTableItem function waits for WRITE instruction response packets,
    // so this must be set to 2, otherwise writeControlTableItem will timeout and return false.
    dxl.writeControlTableItem((uint8_t)STATUS_RETURN_LEVEL, id, (int32_t)2, (uint32_t)10);

    dxl.writeControlTableItem((uint8_t)RETURN_DELAY_TIME,   id, (int32_t)0, (uint32_t)10);
    dxl.torqueOn(id);
  }
}

// ======================= Group Read Configuration =========================
void setup_sync_read(){
  sync_read_param.addr      = ADDR_PRESENT_POSITION;
  sync_read_param.length    = LEN_PRESENT_POSITION;
  sync_read_param.xel[0].id = DXL_ID_1;
  sync_read_param.xel[1].id = DXL_ID_2;
  sync_read_param.id_count  = 2;
}

#if USE_BULK
// ======================= Group Write Configuration (only when USE_BULK=1) =========================
void setup_bulk_write(){
  bulk_write_param.id_count      = 2;
  bulk_write_param.xel[0].id     = DXL_ID_1;
  bulk_write_param.xel[0].addr   = ADDR_GOAL_PWM;
  bulk_write_param.xel[0].length = LEN_GOAL_PWM;
  bulk_write_param.xel[1].id     = DXL_ID_2;
  bulk_write_param.xel[1].addr   = ADDR_GOAL_PWM;
  bulk_write_param.xel[1].length = LEN_GOAL_PWM;
}
#endif

// ======================= Read Position =============================
bool read_positions_dual(){
#if USE_SYNC_READ
  if (!dxl.syncRead(sync_read_param, sync_read_result)) { CMD_PORT.println("[syncRead][FAIL]"); return false; }
  int32_t t1 = 0, t2 = 0; bool got1 = false, got2 = false;
  for (uint8_t i = 0; i < sync_read_result.id_count; ++i) {
    const uint8_t id = sync_read_result.xel[i].id;
    const uint16_t len = sync_read_result.xel[i].length;
    const uint8_t* d = sync_read_result.xel[i].data;
    if (len >= 4) {
      int32_t v =  (int32_t)((uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24));
      if (id == DXL_ID_1) { t1 = v; got1 = true; }
      if (id == DXL_ID_2) { t2 = v; got2 = true; }
    }
  }
  if (!got1 || !got2) {
    CMD_PORT.print("[syncRead][PARTIAL] got1="); CMD_PORT.print(got1);
    CMD_PORT.print(" got2="); CMD_PORT.println(got2);
    if (got1) g_pos1 = t1; if (got2) g_pos2 = t2; return false;
  }
  g_pos1 = t1; g_pos2 = t2; return true;
#else
  int32_t t1 = dxl.readControlTableItem((uint8_t)PRESENT_POSITION, (uint8_t)DXL_ID_1);
  int32_t t2 = dxl.readControlTableItem((uint8_t)PRESENT_POSITION, (uint8_t)DXL_ID_2);
  bool ok1 = (t1 != -1), ok2 = (t2 != -1);
  if (!ok1 || !ok2) { CMD_PORT.print("[read][FAIL] ok1="); CMD_PORT.print(ok1); CMD_PORT.print(" ok2="); CMD_PORT.println(ok2); }
  g_pos1 = t1; g_pos2 = t2; return ok1 && ok2;
#endif
}

// ======================= Write PWM (Dual Channel, Single Write Version) =====================
bool write_pwm_dual(int16_t pwm1, int16_t pwm2){
  pwm1 = clamp_pwm(pwm1);
  pwm2 = clamp_pwm(pwm2);
#if USE_BULK
  // ---- bulkWrite path (only enabled when USE_BULK=1) ----
  bulk_write_param.xel[0].data[0] = (uint8_t)(pwm1 & 0xFF);
  bulk_write_param.xel[0].data[1] = (uint8_t)((pwm1 >> 8) & 0xFF);
  bulk_write_param.xel[1].data[0] = (uint8_t)(pwm2 & 0xFF);
  bulk_write_param.xel[1].data[1] = (uint8_t)((pwm2 >> 8) & 0xFF);
  bool ok = dxl.bulkWrite(bulk_write_param);
  if (!ok){
    CMD_PORT.print("[bulkWrite][FAIL] pwm=("); CMD_PORT.print(pwm1); CMD_PORT.print(","); CMD_PORT.print(pwm2); CMD_PORT.println(")");
    return false;
  }
  return true;
#else
  // ---- Default path: two single writes (keep your existing logic unchanged) ----
  // [Modified] Increased timeout from 5ms to 10ms, 5ms may be too short for write operation + waiting for response packet
  bool ok1 = dxl.writeControlTableItem((uint8_t)GOAL_PWM, (uint8_t)DXL_ID_1, (int32_t)pwm1, (uint32_t)10);
  bool ok2 = dxl.writeControlTableItem((uint8_t)GOAL_PWM, (uint8_t)DXL_ID_2, (int32_t)pwm2, (uint32_t)10);
  if(!(ok1 && ok2)){
    CMD_PORT.print("[write][FAIL] pwm=("); CMD_PORT.print(pwm1); CMD_PORT.print(","); CMD_PORT.print(pwm2); CMD_PORT.println(")");
    return false;
  }
  return true;
#endif
}

// ======================= Stability Judgment Tools ==========================
bool is_stable(const int32_t* hist, uint16_t n, int32_t eps){
  if (n < 2) return false; int32_t minv = hist[0], maxv = hist[0];
  for (uint16_t i = 1; i < n; ++i){ if (hist[i] < minv) minv = hist[i]; if (hist[i] > maxv) maxv = hist[i]; }
  return (maxv - minv) <= eps;
}

// ======================= Initialization Process ========================
void run_initialization(){
  CMD_PORT.println("[init] start");
  write_pwm_dual(0, 0); delay(50);

  // Step1: Find open side
  {
    int32_t hist1[STABLE_WINDOW] = {0};
    int32_t hist2[STABLE_WINDOW] = {0};
    uint16_t idx = 0; uint32_t t0 = millis();
    while (true){
      CMD_PORT.println("[init] searching open-limit...");
      write_pwm_dual(INIT_PWM_OPEN, INIT_PWM_OPEN);
      delay(50);
      (void)read_positions_dual();
      hist1[idx % STABLE_WINDOW] = g_pos1; hist2[idx % STABLE_WINDOW] = g_pos2; idx++;
      bool s1 = (idx >= STABLE_WINDOW) && is_stable(hist1, STABLE_WINDOW, STABLE_EPS);
      bool s2 = (idx >= STABLE_WINDOW) && is_stable(hist2, STABLE_WINDOW, STABLE_EPS);
      if (s1 && s2) break; if (millis() - t0 > INIT_TIMEOUT_MS){ CMD_PORT.println("[init] open-limit timeout"); break; }
    }
    write_pwm_dual(0, 0); delay(100); (void)read_positions_dual();
    open_limit[0] = g_pos1; open_limit[1] = g_pos2;
    CMD_PORT.print("[init] open_limit = "); CMD_PORT.print(open_limit[0]); CMD_PORT.print(", "); CMD_PORT.println(open_limit[1]);
  }

  // Step2a: M1 -> close side, then return to open side limit, avoid pressing against M2
  {
    int32_t hist[STABLE_WINDOW] = {0}; uint16_t idx = 0; uint32_t t0 = millis();
    CMD_PORT.println("[init] step2a: M1 -> close-limit");
    while (true){
      if(!write_pwm_dual(INIT_PWM_CLOSE, 0)) { CMD_PORT.println("[init] step2a: write fail"); return; }
      delay(LOOP_DT_MS);
      if(!read_positions_dual()){ CMD_PORT.println("[init] step2a: read fail"); return; }
      hist[idx % STABLE_WINDOW] = g_pos1; idx++;
      if ((idx >= STABLE_WINDOW) && is_stable(hist, STABLE_WINDOW, STABLE_EPS)) break;
      if (millis() - t0 > INIT_TIMEOUT_MS){ CMD_PORT.println("[init] step2a: timeout"); break; }
    }
    write_pwm_dual(0, 0); delay(100); (void)read_positions_dual();
    close_limit[0] = g_pos1; CMD_PORT.print("[init] close_limit.M1="); CMD_PORT.println(close_limit[0]);

    // Return to open side limit
    CMD_PORT.print("[init] step2a-return: M1 -> open_limit[0] = "); CMD_PORT.println(open_limit[0]);
    uint32_t t1 = millis();
    while (true){
      if(!write_pwm_dual(INIT_PWM_OPEN, 0)) { CMD_PORT.println("[init] step2a-return: write fail"); return; }
      delay(LOOP_DT_MS);
      if(!read_positions_dual()){ CMD_PORT.println("[init] step2a-return: read fail"); return; }
      if (abs(g_pos1 - open_limit[0]) <= RETURN_OPEN_EPS) break;
      if (millis() - t1 > RETURN_OPEN_TIMEOUT){ CMD_PORT.println("[init] step2a-return: timeout"); break; }
    }
    write_pwm_dual(0, 0); delay(50);
  }

  // Step2b: M2 -> close side
  {
    int32_t hist[STABLE_WINDOW] = {0}; uint16_t idx = 0; uint32_t t0 = millis();
    CMD_PORT.println("[init] step2b: M2 -> close-limit");
    while (true){
      if(!write_pwm_dual(0, INIT_PWM_CLOSE)) { CMD_PORT.println("[init] step2b: write fail"); return; }
      delay(LOOP_DT_MS);
      if(!read_positions_dual()){ CMD_PORT.println("[init] step2b: read fail"); return; }
      hist[idx % STABLE_WINDOW] = g_pos2; idx++;
      if ((idx >= STABLE_WINDOW) && is_stable(hist, STABLE_WINDOW, STABLE_EPS)) break;
      if (millis() - t0 > INIT_TIMEOUT_MS){ CMD_PORT.println("[init] step2b: timeout"); break; }
    }
    write_pwm_dual(0, 0); delay(100); (void)read_positions_dual();
    close_limit[1] = g_pos2; CMD_PORT.print("[init] close_limit.M2="); CMD_PORT.println(close_limit[1]);
  }

  limits_valid = true;

  // Return to open + offset initial position (P control for several steps)
  int32_t init1 = open_limit[0] + OFFSET_COUNTS;
  int32_t init2 = open_limit[1] + OFFSET_COUNTS;
  for (int i = 0; i < 60; ++i){
    (void)read_positions_dual();
    int16_t u1 = clamp_pwm((int32_t)(KP * (init1 - g_pos1)));
    int16_t u2 = clamp_pwm((int32_t)(KP * (init2 - g_pos2)));
    write_pwm_dual(u1, u2);
    delay(LOOP_DT_MS);
  }
  write_pwm_dual(0, 0);
  CMD_PORT.println("[init] done");
}

// ======================= Command Parsing ==========================
void handle_command_line(String &line){
  line.trim(); if (line.length() == 0) return;
  int lt = line.indexOf('<'); int gt = line.lastIndexOf('>'); if (lt == -1 || gt == -1 || gt <= lt) return; 
  String inside = line.substring(lt + 1, gt); inside.trim();

  int comma = inside.indexOf(',');
  String cmd = (comma == -1) ? inside : inside.substring(0, comma); cmd.trim();
  if (cmd.length() >= 2 && cmd.charAt(0) == '"' && cmd.charAt(cmd.length()-1) == '"') cmd = cmd.substring(1, cmd.length()-1);
  CMD_PORT.println("[cmd] received: " + line);

  if (cmd.equalsIgnoreCase("initialization")){
    CMD_PORT.println("[cmd] run init");
    run_initialization();
    return;
  }
  else if (cmd.equalsIgnoreCase("open")){
    // Set target to 0 (open end), and switch back to position mode
    g_mode = MODE_POS;
    goal_norm_1 = 0.f; goal_norm_2 = 0.f; feed_pwm_1 = 0; feed_pwm_2 = 0;
    CMD_PORT.println("[cmd] open -> MODE_POS");
    return;
  }
  else if (cmd.equalsIgnoreCase("pos")){
    // [New] Simplified position target: <"pos", m1_goal_pos_norm, m2_goal_pos_norm>
    String rest = (comma == -1) ? "" : inside.substring(comma + 1); rest.trim();
    int c1 = rest.indexOf(','); if (c1 == -1) return; String s1 = rest.substring(0, c1); String s2 = rest.substring(c1+1);
    s1.trim(); s2.trim();
    float g1 = s1.toFloat(); float g2 = s2.toFloat();
    
    // Set targets and zero out feedforward
    goal_norm_1 = g1; goal_norm_2 = g2; 
    feed_pwm_1 = 0; feed_pwm_2 = 0;
    
    // Switch to position mode
    g_mode = MODE_POS;
    
    CMD_PORT.print("[cmd] MODE_POS (simple) g=("); CMD_PORT.print(goal_norm_1); CMD_PORT.print(", "); CMD_PORT.print(goal_norm_2); CMD_PORT.println(")");
    return;
  }
  else if (cmd.equalsIgnoreCase("motion")){
    // [Retained] Full position target: <"motion", m1_goal_pos_norm, m1_feed_pwm_norm, m2_goal_pos_norm, m2_feed_pwm_norm>
    String rest = (comma == -1) ? "" : inside.substring(comma + 1); rest.trim();
    int c1 = rest.indexOf(','); if (c1 == -1) return; String s1 = rest.substring(0, c1); rest = rest.substring(c1+1);
    int c2 = rest.indexOf(','); if (c2 == -1) return; String s2 = rest.substring(0, c2); rest = rest.substring(c2+1);
    int c3 = rest.indexOf(','); if (c3 == -1) return; String s3 = rest.substring(0, c3); String s4 = rest.substring(c3+1);
    s1.trim(); s2.trim(); s3.trim(); s4.trim();
    
    // [Modified] Parse normalized position(0..1) and normalized feedforward PWM(-1..1)
    float g1 = s1.toFloat(); 
    float ff_norm_1 = s2.toFloat(); // Previously s2.toInt()
    float g2 = s3.toFloat();
    float ff_norm_2 = s4.toFloat(); // Previously s4.toInt()
    
    // Set targets and feedforward
    goal_norm_1 = g1;
    goal_norm_2 = g2;
    // [Modified] Denormalize PWM
    feed_pwm_1 = clamp_pwm((long)(ff_norm_1 * PWM_MAX));
    feed_pwm_2 = clamp_pwm((long)(ff_norm_2 * PWM_MAX));
    
    // Switch to position mode
    g_mode = MODE_POS;
    CMD_PORT.print("[cmd] MODE_POS (full) g=("); CMD_PORT.print(goal_norm_1); CMD_PORT.print(", "); CMD_PORT.print(goal_norm_2);
    // [Modified] Print normalized feedforward
    CMD_PORT.print(") ff_norm=("); CMD_PORT.print(ff_norm_1); CMD_PORT.print(", "); CMD_PORT.print(ff_norm_2); CMD_PORT.println(")");
    return;
  }
  else if (cmd.equalsIgnoreCase("pwm")){
    // PWM target: <"pwm", pwm1_norm, pwm2_norm>
    String rest = (comma == -1) ? "" : inside.substring(comma + 1); rest.trim();
    int c1 = rest.indexOf(','); if (c1 == -1) return; String s1 = rest.substring(0, c1); String s2 = rest.substring(c1+1);
    s1.trim(); s2.trim();
    
    // [Modified] Parse normalized PWM(-1..1)
    float pwm_norm_1 = s1.toFloat(); // Previously s1.toInt()
    float pwm_norm_2 = s2.toFloat(); // Previously s2.toInt()
    
    // [Modified] Denormalize PWM
    pwm_goal_1 = clamp_pwm((long)(pwm_norm_1 * PWM_MAX));
    pwm_goal_2 = clamp_pwm((long)(pwm_norm_2 * PWM_MAX));
    
    // Switch to PWM mode
    g_mode = MODE_PWM;
    // [Modified] Print normalized PWM
    CMD_PORT.print("[cmd] MODE_PWM pwm_norm=("); CMD_PORT.print(pwm_norm_1); CMD_PORT.print(", "); CMD_PORT.print(pwm_norm_2); CMD_PORT.println(")");
    return;
  }
  else if (cmd.equalsIgnoreCase("pwm")){
    // PWM target: <"pwm", pwm1, pwm2>
    String rest = (comma == -1) ? "" : inside.substring(comma + 1); rest.trim();
    int c1 = rest.indexOf(','); if (c1 == -1) return; String s1 = rest.substring(0, c1); String s2 = rest.substring(c1+1);
    s1.trim(); s2.trim();
    
    // Set target PWM
    pwm_goal_1 = clamp_pwm(s1.toInt()); pwm_goal_2 = clamp_pwm(s2.toInt());
    
    // Switch to PWM mode
    g_mode = MODE_PWM;
    CMD_PORT.print("[cmd] MODE_PWM pwm=("); CMD_PORT.print(pwm_goal_1); CMD_PORT.print(", "); CMD_PORT.print(pwm_goal_2); CMD_PORT.println(")");
    return;
  }
  else{
    CMD_PORT.print("[cmd] unknown: "); CMD_PORT.println(cmd);
  }
}

// ======================= Arduino Entry Point =======================
void setup(){
  CMD_PORT.begin(CMD_BAUD); STAT_PORT.begin(STAT_BAUD); delay(200);
  dxl.begin(DXL_BAUD); dxl.setPortProtocolVersion(2.0);
  setup_motors_to_pwm_mode(); setup_sync_read();
#if USE_BULK
  setup_bulk_write();
#endif
  CMD_PORT.println("=== Gripper Controller (syncRead + dual write) @50Hz ===");
  CMD_PORT.print("DXL baud="); CMD_PORT.println(DXL_BAUD);
}

void loop(){
  static uint32_t last_ms = 0; uint32_t now = millis();

  // Receive commands
  static String buf;
  while (CMD_PORT.available()){
    char c = (char)CMD_PORT.read();
    if (c == '\r' || c == '\n') { handle_command_line(buf); buf = ""; }
    else { buf += c; if (buf.length() > 128) buf = ""; }
  }

  // 50Hz control + feedback
  if (now - last_ms >= LOOP_DT_MS){
    last_ms = now;

    bool ok = read_positions_dual();
    if (!ok){ CMD_PORT.println("[syncRead] fail"); }
    else{
      // =================================================================
      // [Modified] Normalize state variables
      // =================================================================
      int16_t u1 = 0, u2 = 0;
      float pos_norm_1 = 0.0f, pos_norm_2 = 0.0f;
      float tgt_norm_1 = 0.0f, tgt_norm_2 = 0.0f;

      // 1. Calculate current normalized position (0.0 ~ 1.0)
      if (limits_valid) {
        long range1 = (long)close_limit[0] - (long)open_limit[0];
        long range2 = (long)close_limit[1] - (long)open_limit[1];
        if (range1 != 0) pos_norm_1 = (float)((long)g_pos1 - (long)open_limit[0]) / (float)range1;
        if (range2 != 0) pos_norm_2 = (float)((long)g_pos2 - (long)open_limit[1]) / (float)range2;
        
        // Safety clamp to prevent overshoot causing > 1.0 or < 0.0
        if (pos_norm_1 < 0.0f) pos_norm_1 = 0.0f; if (pos_norm_1 > 1.0f) pos_norm_1 = 1.0f;
        if (pos_norm_2 < 0.0f) pos_norm_2 = 0.0f; if (pos_norm_2 > 1.0f) pos_norm_2 = 1.0f;
      }

      // 2. Calculate raw PWM and normalized target position
      if (g_mode == MODE_POS && limits_valid){
        // Position mode: P control + feedforward
        int32_t tgt1 = lerp_counts(open_limit[0], close_limit[0], goal_norm_1);
        int32_t tgt2 = lerp_counts(open_limit[1], close_limit[1], goal_norm_2);
        u1 = clamp_pwm((int32_t)(KP * (tgt1 - g_pos1)) + feed_pwm_1);
        u2 = clamp_pwm((int32_t)(KP * (tgt2 - g_pos2)) + feed_pwm_2);
        
        // Target position = received normalized target
        tgt_norm_1 = goal_norm_1;
        tgt_norm_2 = goal_norm_2;

      } else if (g_mode == MODE_PWM){
        // PWM mode: Send target PWM directly
        u1 = pwm_goal_1; 
        u2 = pwm_goal_2; 
        
        // Target position = current normalized position (because no position target)
        tgt_norm_1 = pos_norm_1;
        tgt_norm_2 = pos_norm_2;

      } else {
        // Not initialized or invalid mode: stay still
        u1 = 0; u2 = 0; 
        
        // Target position = current normalized position
        tgt_norm_1 = pos_norm_1;
        tgt_norm_2 = pos_norm_2;
      }

      write_pwm_dual(u1, u2);

      // 3. Calculate normalized PWM (-1.0 ~ 1.0)
      float pwm_norm_1 = (float)u1 / (float)PWM_MAX;
      float pwm_norm_2 = (float)u2 / (float)PWM_MAX;

      // Safety clamp
      if (pwm_norm_1 < -1.0f) pwm_norm_1 = -1.0f; if (pwm_norm_1 > 1.0f) pwm_norm_1 = 1.0f;
      if (pwm_norm_2 < -1.0f) pwm_norm_2 = -1.0f; if (pwm_norm_2 > 1.0f) pwm_norm_2 = 1.0f;


      // 4. Status output (all use normalized values, keep 4 decimal places)
      STAT_PORT.print("<");
      STAT_PORT.print(DXL_ID_1); STAT_PORT.print(",");
      STAT_PORT.print(pos_norm_1, 4);   STAT_PORT.print(","); // Normalized current position
      STAT_PORT.print(tgt_norm_1, 4);     STAT_PORT.print(","); // Normalized target position
      STAT_PORT.print(pwm_norm_1, 4);       STAT_PORT.print(","); // Normalized PWM
      STAT_PORT.print(DXL_ID_2); STAT_PORT.print(",");
      STAT_PORT.print(pos_norm_2, 4);   STAT_PORT.print(","); // Normalized current position
      STAT_PORT.print(tgt_norm_2, 4);     STAT_PORT.print(","); // Normalized target position
      STAT_PORT.print(pwm_norm_2, 4);       STAT_PORT.println(">"); // Normalized PWM
    }
  }
}