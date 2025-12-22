#include <Arduino.h>
#include <Wire.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// --- Cấu hình WiFi & Firebase (Giữ nguyên) ---
const char* ssid ="huu nam";
const char* password = "matkhau987";
#define API_KEY "AIzaSyC7kU48070xVL3W10xmXDWropJBNlgLBlA"
#define DATABASE_URL "https://embedded-34716-default-rtdb.asia-southeast1.firebasedatabase.app"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
bool signupOK = false;

WiFiUDP udp;
NTPClient timeClient(udp, "pool.ntp.org", 25200, 60000);

MPU6050 mpu;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
bool dmpReady = false;
uint8_t packetSize;
double setpoint = 178.64;
double input, output;
double Kp = 30, Ki = 230, Kd = 1.5;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

LMotorController motorController(32, 25, 26, 33, 27, 14, 0.6, 0.6);
#define MIN_ABS_SPEED 30

// --- Cấu trúc dữ liệu nâng cấp cho Reinforcement Learning ---
struct RobotDataRL {
    float s_ang;   // State: Angle error
    float s_gy;    // State: Gyro Y
    float s_az;    // State: Accel Z
    double a_pwm;  // Action: PWM output
    float r;       // Reward: Điểm số hành động
    bool done;     // Terminal: Robot có bị ngã không?
};
volatile RobotDataRL sharedData;

// --- Hàm tính toán Phần thưởng (Reward Function) ---
float calculateReward(float angle, float pwm, float last_pwm) {
    float angle_err = abs(angle - 178.64);
    
    // 1. Thưởng đứng thẳng: Góc càng nhỏ thưởng càng lớn (max 1.0)
    float reward = 1.0 - (angle_err / 20.0); 
    
    // 2. Phạt độ rung (Smoothness penalty): Tránh motor giật cục
    float smoothness_penalty = abs(pwm - last_pwm) / 255.0;
    reward -= (smoothness_penalty * 0.1);

    // 3. Phạt nặng nếu ngã
    if (angle_err > 25.0) reward = -10.0; 
    
    return reward;
}

// ----------------------------------------------------------------
// TASK 1: PID & REWARD CALCULATION (Core 1)
// ----------------------------------------------------------------
void TaskPID(void *pvParameters) {
    int16_t ax, ay, az, gx, gy, gz; 
    float last_pwm = 0;

    for (;;) {
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            input = ypr[1] * 180 / M_PI + 180;

            pid.Compute();
            motorController.move(output, MIN_ABS_SPEED);

            mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

            // Cập nhật dữ liệu RL
            sharedData.s_ang = (float)(input - 178.64); // Lưu độ lệch góc
            sharedData.s_gy = (float)gy / 131.0;
            sharedData.s_az = (float)az;
            sharedData.a_pwm = output;
            sharedData.r = calculateReward(input, output, last_pwm);
            sharedData.done = (abs(input - 178.64) > 30.0);

            last_pwm = output;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); 
    }
}

// ----------------------------------------------------------------
// TASK 2: FIREBASE RL DATA LOGGING (Core 0)
// ----------------------------------------------------------------
void TaskFirebase(void *pvParameters) {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { delay(500); }
    
    config.api_key = API_KEY;
    config.database_url = DATABASE_URL;
    if (Firebase.signUp(&config, &auth, "", "")) { signupOK = true; }
    Firebase.begin(&config, &auth);

    for (;;) {
        if (Firebase.ready() && signupOK) {
            // Sử dụng "push" để tạo ID tự động theo thời gian, giúp việc lấy mẫu mượt hơn
            String path = "/RL_Training_Data"; 

            FirebaseJson json;
            json.set("s_ang", sharedData.s_ang);
            json.set("s_gy",  sharedData.s_gy);
            json.set("s_az",  sharedData.s_az);
            json.set("a_pwm", sharedData.a_pwm);
            json.set("reward", sharedData.r);
            json.set("done", sharedData.done);

            // pushJSON thay vì setJSON để tạo danh sách dữ liệu liên tục
            if (!Firebase.RTDB.pushJSON(&fbdo, path, &json)) {
                Serial.println("Lỗi gửi: " + fbdo.errorReason());
            }
        }
        vTaskDelay(20 / portTICK_PERIOD_MS); // ~25Hz là đủ cho việc huấn luyện RL
    }
}

void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22, 400000);

    mpu.initialize();
    if (mpu.dmpInitialize() == 0) {
        mpu.setXGyroOffset(-73);
        mpu.setYGyroOffset(-95);
        mpu.setZGyroOffset(-20);
        mpu.setZAccelOffset(1350);
        mpu.setDMPEnabled(true);
        packetSize = mpu.dmpGetFIFOPacketSize();
        dmpReady = true;
    }

    // 3. Khởi tạo PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);
    xTaskCreatePinnedToCore(TaskPID, "PID_Task", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(TaskFirebase, "FB_Task", 8192, NULL, 1, NULL, 0);
}

void loop() {}