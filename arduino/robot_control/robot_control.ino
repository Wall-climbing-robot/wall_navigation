/*
 * 爬壁清洁机器人 Arduino Mega 控制程序
 *
 * 串口一：Serial (USB/CH340, Pin0/1)  ← 接上位机（Jetson/PC），接收导航速度指令
 * 串口二：Serial2 (Pin16/17)          ← 接 SBUS 转 UART 模块（遥控器，可选）
 *
 * ESC PWM：Pin9（左轮）/ Pin10（右轮）
 *
 * 模式切换：
 *   - 有遥控且通道5 > 1500 → 自动模式（执行导航指令）
 *   - 有遥控且通道5 < 1500 → 手动模式（执行遥控指令）
 *   - 无遥控               → 自动模式（执行导航指令）
 *
 * 通信协议（串口一，来自 serial_bridge.py）：
 *   帧格式：[0xAA][0x55][左速高][左速低][右速高][右速低][CRC8]
 *   速度单位：mm/s，int16 有符号（正=前进，负=后退）
 *   CRC8 多项式：0x07
 *
 * SBUS 转 UART 模块输出格式（35字节）：
 *   Byte 0    : 0x0F（起始标志）
 *   Byte 1-32 : 16通道 × 2字节，大端序，范围 0-2047
 *   Byte 33   : 标志字节（bit5=帧丢失）
 *   Byte 34   : XOR 校验（Byte1..Byte33）
 */

#include <Servo.h>

// ===== 根据实际硬件修改 =====
#define LEFT_ESC_PIN    9
#define RIGHT_ESC_PIN   10
#define JETSON_BAUD     115200   // 与 serial_bridge.py 的 BAUDRATE 一致
#define RC_BAUD         100000   // SBUS 转 UART 模块波特率（查模块手册）
#define MAX_SPEED_MMS   500      // 最大速度 mm/s（地面用 500，上墙用 200）
#define MODE_CHANNEL    4        // 遥控模式切换通道索引（0起，通道5=索引4）
#define CMD_TIMEOUT_MS  500      // 导航指令超时停车阈值（毫秒）
// ============================

Servo leftESC, rightESC;

// ── 串口一（导航指令）状态 ──
int16_t autoLeft  = 0;
int16_t autoRight = 0;
unsigned long lastCmdMs = 0;

// ── 串口二（遥控器）状态 ──
uint16_t rcChannels[16] = {0};
bool rcConnected = false;

// ────────────────────────────
void setup() {
  // 串口一：接上位机
  Serial.begin(JETSON_BAUD);

  // 串口二：接 SBUS 转 UART 模块
  Serial2.begin(RC_BAUD);

  // ESC 初始化：发中位信号，等待 ESC 上电完成（听到两声哔=成功）
  leftESC.attach(LEFT_ESC_PIN,  1000, 2000);
  rightESC.attach(RIGHT_ESC_PIN, 1000, 2000);
  leftESC.writeMicroseconds(1500);
  rightESC.writeMicroseconds(1500);
  delay(2000);

  Serial.println("[OK] robot_control started");
}

void loop() {
  readJetson();
  readRC();

  // 模式判断：无遥控默认自动；有遥控时由通道5决定
  bool autoMode = !rcConnected || (rcChannels[MODE_CHANNEL] > 1500);

  // 导航指令超时保护
  if (autoMode && (millis() - lastCmdMs > CMD_TIMEOUT_MS)) {
    autoLeft = autoRight = 0;
  }

  int16_t L, R;
  if (autoMode) {
    L = autoLeft;
    R = autoRight;
  } else {
    // 手动模式：通道1=前后，通道0=转向（根据实际通道调整）
    int vx = map(rcChannels[1], 1000, 2000, -MAX_SPEED_MMS, MAX_SPEED_MMS);
    int wz = map(rcChannels[0], 1000, 2000, -MAX_SPEED_MMS, MAX_SPEED_MMS);
    L = constrain(vx - wz, -MAX_SPEED_MMS, MAX_SPEED_MMS);
    R = constrain(vx + wz, -MAX_SPEED_MMS, MAX_SPEED_MMS);
  }

  leftESC.writeMicroseconds(map(L, -MAX_SPEED_MMS, MAX_SPEED_MMS, 1000, 2000));
  rightESC.writeMicroseconds(map(R, -MAX_SPEED_MMS, MAX_SPEED_MMS, 1000, 2000));

  delay(20);
}

// ────────────────────────────
// 串口一：解析来自上位机的导航指令帧
// 帧格式：[0xAA][0x55][L_H][L_L][R_H][R_L][CRC8]
// ────────────────────────────
void readJetson() {
  static uint8_t buf[7];
  static int idx = 0;

  while (Serial.available()) {
    uint8_t b = Serial.read();

    if      (idx == 0 && b == 0xAA) { buf[0] = b; idx = 1; }
    else if (idx == 1 && b == 0x55) { buf[1] = b; idx = 2; }
    else if (idx == 1)               { idx = 0; }           // 帧头错误，重新同步
    else if (idx >= 2) {
      buf[idx++] = b;
      if (idx == 7) {
        idx = 0;
        if (crc8(buf + 2, 4) == buf[6]) {
          autoLeft  = constrain((int16_t)((buf[2] << 8) | buf[3]),
                                -MAX_SPEED_MMS, MAX_SPEED_MMS);
          autoRight = constrain((int16_t)((buf[4] << 8) | buf[5]),
                                -MAX_SPEED_MMS, MAX_SPEED_MMS);
          lastCmdMs = millis();
        } else {
          Serial.println("[ERR] CRC mismatch");
        }
      }
    }
  }
}

// ────────────────────────────
// 串口二：解析 SBUS 转 UART 模块输出（35字节）
// Byte0=0x0F  Byte1-32=16ch×2B大端  Byte33=标志  Byte34=XOR校验
// ────────────────────────────
void readRC() {
  static uint8_t rcBuf[35];
  static int rcIdx = 0;

  while (Serial2.available()) {
    uint8_t b = Serial2.read();

    if (rcIdx == 0 && b != 0x0F) continue;   // 等帧头
    rcBuf[rcIdx++] = b;
    if (rcIdx < 35) continue;                 // 帧未完整
    rcIdx = 0;

    // XOR 校验（Byte1..Byte33）
    uint8_t xorVal = 0;
    for (int i = 1; i <= 33; i++) xorVal ^= rcBuf[i];
    if (xorVal != rcBuf[34]) continue;

    // 帧丢失标志（bit5 of Byte33）
    if (rcBuf[33] & 0x20) {
      rcConnected = false;
      return;
    }

    // 解析16通道（每通道2字节，大端序，范围 0-2047）
    for (int i = 0; i < 16; i++) {
      rcChannels[i] = ((uint16_t)rcBuf[1 + i * 2] << 8) | rcBuf[2 + i * 2];
    }
    rcConnected = true;
  }
}

// CRC8，多项式 0x07（与 serial_bridge.py 一致）
uint8_t crc8(uint8_t* data, int len) {
  uint8_t crc = 0;
  for (int i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++)
      crc = (crc & 0x80) ? ((crc << 1) ^ 0x07) : (crc << 1);
    crc &= 0xFF;
  }
  return crc;
}
