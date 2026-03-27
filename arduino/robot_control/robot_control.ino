*
 * 爬壁清洁机器人 Arduino Mega 控制程序
 * Serial1 (Pin18/19): 接Jetson（自动导航速度命令）
 * Serial2 (Pin16/17): 接SBUS转UART模块（遥控器）
 * Pin9/10: 左右电机ESC PWM输出
 * 遥控通道5 > 1500（0-2047范围，中位约1500）= 自动模式，< 1500 = 手动模式
 */
#include <Servo.h>

// ===== 改成你的实际值 =====
#define LEFT_ESC_PIN   9
#define RIGHT_ESC_PIN  10
#define JETSON_BAUD    115200
#define RC_BAUD        100000  // SBUS转UART模块波特率，查你模块手册（常见100000或115200）
#define MAX_SPEED_MMS  500     // 最大速度mm/s，地面500，上墙改200
#define MODE_CHANNEL   4       // 通道索引（0开始），通道5=索引4
// ==========================

Servo leftESC, rightESC;
uint16_t rcChannels[16] = {0};
bool rcConnected = false;
int16_t autoLeft = 0, autoRight = 0;
unsigned long lastCmd = 0;

void setup() {
  Serial.begin(115200);
  Serial1.begin(JETSON_BAUD);
  Serial2.begin(RC_BAUD);
  leftESC.attach(LEFT_ESC_PIN, 1000, 2000);
  rightESC.attach(RIGHT_ESC_PIN, 1000, 2000);
  // ESC初始化：先发停止信号，等2秒让ESC上电完成（听到两声哔=成功）
  leftESC.writeMicroseconds(1500);
  rightESC.writeMicroseconds(1500);
  delay(2000);
  Serial.println("启动完成");
}

void loop() {
  readJetson();
  readRC();

  bool autoMode = rcConnected && (rcChannels[MODE_CHANNEL] > 1500);
  if (autoMode && millis() - lastCmd > 500) {  // 超时停车
    autoLeft = autoRight = 0;
    Serial.println("Jetson超时，停车");
  }

  int16_t L, R;
  if (autoMode) {
    L = autoLeft; R = autoRight;
  } else {
    // 手动：遥控器通道1=前后，通道0=左右（根据你实际通道调整）
    int vx = map(rcChannels[1], 1000, 2000, -MAX_SPEED_MMS, MAX_SPEED_MMS);
    int wz = map(rcChannels[0], 1000, 2000, -MAX_SPEED_MMS, MAX_SPEED_MMS);
    L = constrain(vx - wz, -MAX_SPEED_MMS, MAX_SPEED_MMS);
    R = constrain(vx + wz, -MAX_SPEED_MMS, MAX_SPEED_MMS);
  }

  leftESC.writeMicroseconds(map(L, -MAX_SPEED_MMS, MAX_SPEED_MMS, 1000, 2000));
  rightESC.writeMicroseconds(map(R, -MAX_SPEED_MMS, MAX_SPEED_MMS, 1000, 2000));
  delay(20);
}

void readJetson() {
  static uint8_t buf[7]; static int idx = 0;
  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    if      (idx == 0 && b == 0xAA) { buf[0]=b; idx=1; }
    else if (idx == 1 && b == 0x55) { buf[1]=b; idx=2; }
    else if (idx == 1)               { idx=0; }
    else if (idx >= 2) {
      buf[idx++] = b;
      if (idx == 7) {
        idx = 0;
        if (crc8(buf+2, 4) == buf[6]) {
          autoLeft  = constrain((int16_t)((buf[2]<<8)|buf[3]), -MAX_SPEED_MMS, MAX_SPEED_MMS);
          autoRight = constrain((int16_t)((buf[4]<<8)|buf[5]), -MAX_SPEED_MMS, MAX_SPEED_MMS);
          lastCmd = millis();
        } else { Serial.println("CRC错误"); }
      }
    }
  }
}

// ──────────────────────────────────────────────────────────────
// SBUS帧格式（官方协议，35字节）：
//   Byte 0    : 0x0F（起始标志）
//   Byte 1-32 : 16个通道 × 2字节，大端序，每通道值范围 0-2047
//               Byte1-2=CH1, Byte3-4=CH2, ..., Byte31-32=CH16
//   Byte 33   : 标志字节（bit7=CH17, bit6=CH18, bit5=帧丢失, bit4=故障保护）
//   Byte 34   : XOR校验（Byte1 XOR Byte2 XOR ... XOR Byte33，不含起始标志）
//
// 示例帧：
//   0F 03 E8 03 E8 00 00 03 E8 03 E8 03 E8 03 E8 03 E8
//   04 2C 04 2C 04 2C 04 2C 04 00 04 00 04 00 04 00 0C E7
//
// 通道值说明（0-2047范围）：
//   1000 ≈ 摇杆最小（对应1000μs PWM）
//   1500 ≈ 中位（对应1500μs PWM）
//   2000 ≈ 摇杆最大（对应2000μs PWM）
//   → 模式切换阈值用 1500（通道5 > 1500 = 自动模式）
//
// SBUS转UART模块波特率：查看你模块手册，通常100000或115200
// ──────────────────────────────────────────────────────────────
void readRC() {
  static uint8_t rcBuf[35];
  static int rcIdx = 0;
  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    if (rcIdx == 0 && b != 0x0F) continue;  // 等起始标志
    rcBuf[rcIdx++] = b;
    if (rcIdx < 35) continue;               // 帧未收完
    rcIdx = 0;

    // 验证XOR校验（Byte1..Byte33的异或 == Byte34）
    uint8_t xorVal = 0;
    for (int i = 1; i <= 33; i++) xorVal ^= rcBuf[i];
    if (xorVal != rcBuf[34]) continue;       // 校验失败，丢弃

    // 检查帧丢失标志（bit5 of Byte33）
    if (rcBuf[33] & 0x20) { rcConnected = false; continue; }

    // 解析16个通道（每通道2字节大端序，范围0-2047）
    for (int i = 0; i < 16; i++) {
      rcChannels[i] = ((uint16_t)rcBuf[1 + i*2] << 8) | rcBuf[2 + i*2];
    }
    rcConnected = true;
  }
}

uint8_t crc8(uint8_t* data, int len) {
  uint8_t crc = 0;
  for (int i=0; i<len; i++) {
    crc ^= data[i];
    for (int j=0; j<8; j++)
      crc = (crc & 0x80) ? ((crc<<1)^0x07) : (crc<<1);
    crc &= 0xFF;
  }
  return crc;
}