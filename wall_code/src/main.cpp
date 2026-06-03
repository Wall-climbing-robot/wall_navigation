#include <Arduino.h>
#include <Servo.h>
#include <Arduino_FreeRTOS.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <avr/wdt.h>

// 串口定义
HardwareSerial& receiverSerial = Serial;   // SBUS 遥控接收机 Pin0/1
HardwareSerial& imuSerial      = Serial1;  // IMU Pin18/19
HardwareSerial& jetsonSerial   = Serial2;  // 上位机 (Jetson) Pin16/17（接 DAPLink 无线串口）
HardwareSerial& debugSerial    = Serial3;  // 调试输出 Pin14/15

// IMU 数据
volatile float imuAcc[3]   = {0};  // 加速度 X/Y/Z (m/s²)
volatile float imuGyro[3]  = {0};  // 角速度 X/Y/Z (°/s)
volatile float imuAngle[3] = {0};  // 角度 Roll/Pitch/Yaw (°)

// 引脚定义
#define LEFT_MOTOR_PWM  3
#define LEFT_MOTOR_DIR  43
#define RIGHT_MOTOR_PWM 2
#define RIGHT_MOTOR_DIR 42
#define RELAY_PIN  46
#define BUZZER_PIN 12

// SBUS 通道范围
#define CHANNEL_MIN 272
#define CHANNEL_MID 992
#define CHANNEL_MAX 1712

// 电机 PWM 范围 (0–100)
#define MOTOR_MIN 0
#define MOTOR_MAX 100

// 上位机协议：纯文本，每行格式 "L<左速> R<右速>\n"
// 例：L500 R-200\n   速度单位 mm/s，范围 -500~+500
#define JETSON_BAUD    115200
#define JETSON_MAX_MMS 500

// SBUS 通道数据
volatile int channelValues[16] = {0};

// 上位机速度指令 (mm/s)，int16 有符号，正=前进，负=后退
volatile int16_t jetsonLeftMms  = 0;
volatile int16_t jetsonRightMms = 0;

// 信号状态与超时
volatile unsigned long lastSignalTime  = 0;
volatile unsigned long lastJetsonTime  = 0;  // 0 表示从未收到 Jetson 帧
const unsigned long signalTimeout  = 1000;   // SBUS 超时 ms
const unsigned long jetsonTimeout  = 500;    // Jetson 超时 ms
volatile int signal_connected = 0;

// 模式切换：通道 5（索引 4）拨杆
// 拨杆高位 (> CHANNEL_MID) → 上位机模式；低位 → SBUS 遥控模式
// SBUS 丢失时 channelValues 清零，自动回遥控模式（值=0 < CHANNEL_MID）
#define MODE_CHANNEL_IDX 4

// 任务声明
void SBUS_LISTEN_TASK(void *pvParameters);
void JETSON_LISTEN_TASK(void *pvParameters);
void SERVO_CONTROL_TASK(void *pvParameters);
void LED_BUZZER_TASK(void *pvParameters);
void SAFETY_MONITOR_TASK(void *pvParameters);
void IMU_LISTEN_TASK(void *pvParameters);


void setup() {
    receiverSerial.begin(115200);
    imuSerial.begin(115200);
    jetsonSerial.begin(JETSON_BAUD);
    debugSerial.begin(115200);

    pinMode(LEFT_MOTOR_PWM,  OUTPUT);
    pinMode(LEFT_MOTOR_DIR,  OUTPUT);
    pinMode(RIGHT_MOTOR_PWM, OUTPUT);
    pinMode(RIGHT_MOTOR_DIR, OUTPUT);
    pinMode(RELAY_PIN,  OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);

    analogWrite(LEFT_MOTOR_PWM,  0);
    analogWrite(RIGHT_MOTOR_PWM, 0);
    digitalWrite(LEFT_MOTOR_DIR,  LOW);
    digitalWrite(RIGHT_MOTOR_DIR, LOW);

    wdt_enable(WDTO_2S);

    xTaskCreate(SBUS_LISTEN_TASK,    "SBUS_LISTEN",    256, NULL, 1, NULL);
    xTaskCreate(JETSON_LISTEN_TASK,  "JETSON_LISTEN",  256, NULL, 1, NULL);
    xTaskCreate(SERVO_CONTROL_TASK,  "SERVO_CONTROL",  256, NULL, 1, NULL);
    xTaskCreate(LED_BUZZER_TASK,     "LED_BUZZER",     128, NULL, 1, NULL);
    xTaskCreate(SAFETY_MONITOR_TASK, "SAFETY_MONITOR", 256, NULL, 1, NULL);
    xTaskCreate(IMU_LISTEN_TASK,     "IMU_LISTEN",     256, NULL, 1, NULL);
}

void loop() {
    // 空循环，所有操作在 FreeRTOS 任务中处理
}

/* 接收机串口监听任务 */
void SBUS_LISTEN_TASK(void *pvParameters) {
    while (1) {
        receiverSerial.flush();
        // 等待有数据，最多 200ms，期间让出 CPU 给其他任务
        {
            unsigned long t = millis();
            while (!receiverSerial.available()) {
                if (millis() - t > 200) break;
                vTaskDelay(pdMS_TO_TICKS(5));
            }
            if (!receiverSerial.available()) continue;
        }
        int incomingByte = receiverSerial.read();
        if (incomingByte == 0x0F) {
            byte data[34];
            int i = 0;
            while (i < 34) {
                if (receiverSerial.available()) {
                    data[i++] = receiverSerial.read();
                }
            }
            if (i < 34) continue;  // 帧不完整，丢弃
            if (data[32] == 0x00) {
                lastSignalTime = millis();
                signal_connected = 1;
                for (int j = 0; j < 16; j++) {
                    channelValues[j] = data[j * 2] << 8 | data[j * 2 + 1];
                }
            } else {
                debugSerial.println("Frame error");
            }
        }
        if (!receiverSerial) {
            receiverSerial.end();
            vTaskDelay(pdMS_TO_TICKS(100));
            receiverSerial.begin(115200);
        }
    }
}

/* 上位机 (Jetson) 串口接收任务
 * 协议：纯文本，每行 "L<左速> R<右速>\n"，速度单位 mm/s
 * 例：L500 R500\n   L-300 R300\n
 */
void JETSON_LISTEN_TASK(void *pvParameters) {
    while (1) {
        if (!jetsonSerial.available()) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        // 读一行（以 '\n' 结束，最多等 100ms）
        char buf[32] = {0};
        uint8_t idx = 0;
        unsigned long t0 = millis();
        while (millis() - t0 < 100) {
            if (jetsonSerial.available()) {
                char c = jetsonSerial.read();
                if (c == '\n') break;
                if (c != '\r' && idx < sizeof(buf) - 1) buf[idx++] = c;
            } else {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }
        if (idx == 0) continue;

        // 解析 "L<num> R<num>"
        int16_t lMms = 0, rMms = 0;
        char *pL = strchr(buf, 'L');
        char *pR = strchr(buf, 'R');
        if (pL == NULL || pR == NULL) continue;
        lMms = (int16_t)atoi(pL + 1);
        rMms = (int16_t)atoi(pR + 1);

        jetsonLeftMms  = constrain(lMms, -JETSON_MAX_MMS, JETSON_MAX_MMS);
        jetsonRightMms = constrain(rMms, -JETSON_MAX_MMS, JETSON_MAX_MMS);
        lastJetsonTime = millis();

        // 视觉反馈：收到 Jetson 指令时 LED 闪一下
        // digitalWrite(LED_BUILTIN, HIGH);
        // vTaskDelay(pdMS_TO_TICKS(50));
        // digitalWrite(LED_BUILTIN, LOW);

        // 听觉反馈：收到 Jetson 指令时蜂鸣器响 30ms
        digitalWrite(BUZZER_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(30));
        digitalWrite(BUZZER_PIN, LOW);

        // 通过 jetsonSerial (TX2) 回传给电脑，确认数据收到
        jetsonSerial.print("[Jetson OK] L=");
        jetsonSerial.print(jetsonLeftMms);
        jetsonSerial.print(" R=");
        jetsonSerial.print(jetsonRightMms);
        jetsonSerial.print(" CH5=");
        jetsonSerial.print(channelValues[MODE_CHANNEL_IDX]);
        jetsonSerial.print(" mode=");
        jetsonSerial.println(channelValues[MODE_CHANNEL_IDX] > CHANNEL_MID ? "JETSON" : "RC");

        debugSerial.print("[Jetson] L=");
        debugSerial.print(jetsonLeftMms);
        debugSerial.print(" R=");
        debugSerial.println(jetsonRightMms);
    }
}

/* 电机控制任务
 * 模式由遥控器通道 5（索引 4）拨杆决定：
 *   拨杆低位 (<= CHANNEL_MID) → SBUS 遥控模式
 *     通道索引 2（左摇杆）→ 右电机
 *     通道索引 1（右摇杆）→ 左电机
 *   拨杆高位 (> CHANNEL_MID) → 上位机模式
 *     jetsonLeftMms → 左电机，jetsonRightMms → 右电机
 *     Jetson 超时 500ms 则停车等待（不回退遥控，避免意外动作）
 * SBUS 丢失后 channelValues 清零 → 自动回遥控模式
 */
void SERVO_CONTROL_TASK(void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(2000));
    while (1) {
        bool jetsonMode = (channelValues[MODE_CHANNEL_IDX] > CHANNEL_MID);

        if (jetsonMode) {
            /******* 上位机模式 *******/
            bool jetsonActive = (lastJetsonTime > 0) &&
                                ((millis() - lastJetsonTime) < jetsonTimeout);
            if (!jetsonActive) {
                // Jetson 超时停车
                digitalWrite(LEFT_MOTOR_DIR,  LOW); analogWrite(LEFT_MOTOR_PWM,  0);
                digitalWrite(RIGHT_MOTOR_DIR, LOW); analogWrite(RIGHT_MOTOR_PWM, 0);
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }
            // 左电机
            int lPwm = map(abs(jetsonLeftMms), 0, JETSON_MAX_MMS, MOTOR_MIN, MOTOR_MAX);
            lPwm = constrain(lPwm, MOTOR_MIN, MOTOR_MAX);
            if (jetsonLeftMms > 0) {
                digitalWrite(LEFT_MOTOR_DIR, LOW);
                analogWrite(LEFT_MOTOR_PWM, lPwm);
            } else if (jetsonLeftMms < 0) {
                digitalWrite(LEFT_MOTOR_DIR, HIGH);
                analogWrite(LEFT_MOTOR_PWM, lPwm);
            } else {
                digitalWrite(LEFT_MOTOR_DIR, LOW);
                analogWrite(LEFT_MOTOR_PWM, 0);
            }
            // 右电机
            int rPwm = map(abs(jetsonRightMms), 0, JETSON_MAX_MMS, MOTOR_MIN, MOTOR_MAX);
            rPwm = constrain(rPwm, MOTOR_MIN, MOTOR_MAX);
            if (jetsonRightMms > 0) {
                digitalWrite(RIGHT_MOTOR_DIR, LOW);
                analogWrite(RIGHT_MOTOR_PWM, rPwm);
            } else if (jetsonRightMms < 0) {
                digitalWrite(RIGHT_MOTOR_DIR, HIGH);
                analogWrite(RIGHT_MOTOR_PWM, rPwm);
            } else {
                digitalWrite(RIGHT_MOTOR_DIR, LOW);
                analogWrite(RIGHT_MOTOR_PWM, 0);
            }
        } else {
            /******* SBUS 遥控模式 *******/
            // 右电机由通道索引 2（左摇杆）控制
            if (channelValues[2] < CHANNEL_MID && channelValues[2] >= CHANNEL_MIN) {
                int speed = map(channelValues[2], CHANNEL_MIN, CHANNEL_MID, MOTOR_MAX, MOTOR_MIN);
                digitalWrite(RIGHT_MOTOR_DIR, HIGH);
                analogWrite(RIGHT_MOTOR_PWM, speed);
            } else if (channelValues[2] > CHANNEL_MID && channelValues[2] <= CHANNEL_MAX) {
                int speed = map(channelValues[2], CHANNEL_MID, CHANNEL_MAX, MOTOR_MIN, MOTOR_MAX);
                digitalWrite(RIGHT_MOTOR_DIR, LOW);
                analogWrite(RIGHT_MOTOR_PWM, speed);
            } else {
                digitalWrite(RIGHT_MOTOR_DIR, LOW);
                analogWrite(RIGHT_MOTOR_PWM, 0);
            }
            // 左电机由通道索引 1（右摇杆）控制
            if (channelValues[1] < CHANNEL_MID && channelValues[1] >= CHANNEL_MIN) {
                int speed = map(channelValues[1], CHANNEL_MIN, CHANNEL_MID, MOTOR_MAX, MOTOR_MIN);
                digitalWrite(LEFT_MOTOR_DIR, HIGH);
                analogWrite(LEFT_MOTOR_PWM, speed);
            } else if (channelValues[1] > CHANNEL_MID && channelValues[1] <= CHANNEL_MAX) {
                int speed = map(channelValues[1], CHANNEL_MID, CHANNEL_MAX, MOTOR_MIN, MOTOR_MAX);
                digitalWrite(LEFT_MOTOR_DIR, LOW);
                analogWrite(LEFT_MOTOR_PWM, speed);
            } else {
                digitalWrite(LEFT_MOTOR_DIR, LOW);
                analogWrite(LEFT_MOTOR_PWM, 0);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* 安全保障任务 */
void SAFETY_MONITOR_TASK(void *pvParameters) {
    while (1) {
        wdt_reset();
        if (millis() - lastSignalTime > signalTimeout) {
            signal_connected = 0;
            for (int i = 0; i < 16; i++) {
                channelValues[i] = CHANNEL_MID;
            }
            // 两路信号都不活跃时停车
            bool jetsonActive = (lastJetsonTime > 0) &&
                                ((millis() - lastJetsonTime) < jetsonTimeout);
            if (!jetsonActive) {
                digitalWrite(LEFT_MOTOR_DIR,  LOW);
                analogWrite(LEFT_MOTOR_PWM,   0);
                digitalWrite(RIGHT_MOTOR_DIR, LOW);
                analogWrite(RIGHT_MOTOR_PWM,  0);
                digitalWrite(BUZZER_PIN, HIGH);
                vTaskDelay(pdMS_TO_TICKS(100));
                digitalWrite(BUZZER_PIN, LOW);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* IMU 串口监听任务 */
void IMU_LISTEN_TASK(void *pvParameters) {
    while (1) {
        if (imuSerial.available()) {
            byte header = imuSerial.read();
            if (header == 0x55) {
                unsigned long startWait = millis();
                while (imuSerial.available() < 10) {
                    if (millis() - startWait > 50) break;
                    vTaskDelay(pdMS_TO_TICKS(1));
                }
                if (imuSerial.available() >= 10) {
                    byte buf[10];
                    for (int i = 0; i < 10; i++) buf[i] = imuSerial.read();
                    byte sum = 0x55;
                    for (int i = 0; i < 9; i++) sum += buf[i];
                    if (sum != buf[9]) continue;

                    byte type = buf[0];
                    short d1 = (short)((short)buf[2] << 8 | buf[1]);
                    short d2 = (short)((short)buf[4] << 8 | buf[3]);
                    short d3 = (short)((short)buf[6] << 8 | buf[5]);

                    if (type == 0x51) {
                        imuAcc[0] = d1 / 32768.0 * 16 * 9.8;
                        imuAcc[1] = d2 / 32768.0 * 16 * 9.8;
                        imuAcc[2] = d3 / 32768.0 * 16 * 9.8;
                        debugSerial.print("Acc: ");
                        debugSerial.print(imuAcc[0], 2); debugSerial.print(" ");
                        debugSerial.print(imuAcc[1], 2); debugSerial.print(" ");
                        debugSerial.println(imuAcc[2], 2);
                    } else if (type == 0x52) {
                        imuGyro[0] = d1 / 32768.0 * 2000;
                        imuGyro[1] = d2 / 32768.0 * 2000;
                        imuGyro[2] = d3 / 32768.0 * 2000;
                        debugSerial.print("Gyro: ");
                        debugSerial.print(imuGyro[0], 2); debugSerial.print(" ");
                        debugSerial.print(imuGyro[1], 2); debugSerial.print(" ");
                        debugSerial.println(imuGyro[2], 2);
                    } else if (type == 0x53) {
                        imuAngle[0] = d1 / 32768.0 * 180;
                        imuAngle[1] = d2 / 32768.0 * 180;
                        imuAngle[2] = d3 / 32768.0 * 180;
                        debugSerial.print("Angle: ");
                        debugSerial.print(imuAngle[0], 2); debugSerial.print(" ");
                        debugSerial.print(imuAngle[1], 2); debugSerial.print(" ");
                        debugSerial.println(imuAngle[2], 2);
                    }
                }
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
}

/* LED 和蜂鸣器状态控制任务 */
void LED_BUZZER_TASK(void *pvParameters) {
    pinMode(LED_BUILTIN, OUTPUT);
    bool hasBuzzed = false;
    while (1) {
        digitalWrite(LED_BUILTIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(500));
        digitalWrite(LED_BUILTIN, LOW);
        vTaskDelay(pdMS_TO_TICKS(500));

        bool isValid = false;
        for (int i = 0; i < 16; i++) {
            if (channelValues[i] != 0) {
                isValid = true;
                break;
            }
        }
        if (isValid && !hasBuzzed) {
            hasBuzzed = true;
            debugSerial.println("Buzzer triggered");
            for (int i = 0; i < 3; i++) {
                digitalWrite(BUZZER_PIN, HIGH);
                vTaskDelay(pdMS_TO_TICKS(200));
                digitalWrite(BUZZER_PIN, LOW);
                vTaskDelay(pdMS_TO_TICKS(200));
            }
        } else if (!isValid) {
            hasBuzzed = false;
        }
    }
}
