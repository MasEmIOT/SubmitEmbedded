#include <Arduino.h>
#include <Wire.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#include <WiFi.h>
#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// ================================================================
// 1. CẤU HÌNH WIFI & FIREBASE
// ================================================================
const char* ssid = "ntd";
const char* password = "matkhau987";
#define API_KEY "AIzaSyC7kU48070xVL3W10xmXDWropJBNlgLBlA"
#define DATABASE_URL "https://embedded-34716-default-rtdb.asia-southeast1.firebasedatabase.app"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
bool signupOK = false;

// ================================================================
// 2. CẤU HÌNH MPU6050 (INTERRUPT MODE)
// ================================================================
MPU6050 mpu;
#define MPU_INT_PIN 4  // --- QUAN TRỌNG: Nối chân INT vào D4 ESP32 ---

// MPU control/status vars
bool dmpReady = false;  
uint8_t mpuIntStatus;   
uint8_t devStatus;      
uint16_t packetSize;    
uint16_t fifoCount;     
uint8_t fifoBuffer[64]; 

// Orientation/motion vars
Quaternion q;           
VectorFloat gravity;    
float ypr[3];           

// ================================================================
// 3. CẤU HÌNH PID & MOTOR
// ================================================================
double setpoint = 174.2; // Góc cân bằng
double input, output;

// --- THÔNG SỐ PID ---
double Kp = 30.0;   
double Ki = 230.0;
double Kd = 1.5;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

LMotorController motorController(32, 25, 26, 33, 27, 14, 0.6, 0.6);
#define MIN_ABS_SPEED 30

// ================================================================
// 4. DATA STRUCT (GIỮ NGUYÊN 5 THÔNG SỐ NHƯ RL)
// ================================================================
struct RobotData {
    float s_ang;   // 1. Góc lệch (Angle Error)
    float s_gy;    // 2. Vận tốc góc (Gyro Y)
    float s_az;    // 3. Gia tốc trục Z (Accel Z)
    double a_pwm;  // 4. Action (PWM)
    float r;       // 5. Reward (Dùng để giám sát độ ổn định)
    bool done;     // Trạng thái ngã
};
volatile RobotData sharedData;

// Hàm tính Reward (Chỉ dùng để hiển thị mức độ hiệu quả của PID)
float calculateMonitorReward(float angle) {
    float angle_err = abs(angle - 180.0);
    if (angle_err > 30.0) return -100.0; // Ngã
    return 1.0 - (angle_err / 20.0);     // Càng đứng thẳng điểm càng cao
}

// ================================================================
// 5. HÀM XỬ LÝ NGẮT
// ================================================================
volatile bool mpuInterrupt = false; 
void IRAM_ATTR dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// TASK 1: PID & SENSOR (CORE 1)
// ================================================================
void TaskPID(void *pvParameters) {
    int16_t ax, ay, az, gx, gy, gz; // Biến để lấy dữ liệu thô

    for (;;) {
        if (!dmpReady) { vTaskDelay(10); continue; }

        // Chờ tín hiệu ngắt (Chống treo)
        while (!mpuInterrupt && fifoCount < packetSize) {
             vTaskDelay(1); 
        }

        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();
        fifoCount = mpu.getFIFOCount();

        if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
            mpu.resetFIFO();
        } 
        else if (mpuIntStatus & 0x02) {
            while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

            mpu.getFIFOBytes(fifoBuffer, packetSize);
            fifoCount -= packetSize;

            // 1. Tính Góc (Input cho PID)
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            input = ypr[1] * 180 / M_PI + 180;

            // 2. Tính PID & Điều khiển Motor
            pid.Compute();
            
            // Bảo vệ: Nếu nghiêng quá 45 độ thì ngắt motor
            if (abs(input - setpoint) > 45) {
                motorController.stopMoving();
            } else {
                motorController.move(output, MIN_ABS_SPEED);
            }

            // 3. Lấy thêm dữ liệu thô (Gyro/Accel) để gửi cho đủ 5 thông số
            mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

            // 4. Đóng gói dữ liệu vào Shared Struct
            sharedData.s_ang = (float)(input - setpoint); // Góc lệch
            sharedData.s_gy  = (float)gy / 131.0;         // Gyro Y
            sharedData.s_az  = (float)az;                 // Accel Z
            sharedData.a_pwm = output;                    // PWM
            sharedData.r     = calculateMonitorReward(input); // Điểm hiệu quả
            sharedData.done  = (abs(input - setpoint) > 30.0); // Trạng thái ngã
        }
    }
}

// ================================================================
// TASK 2: FIREBASE LOGGING (CORE 0)
// ================================================================
void TaskFirebase(void *pvParameters) {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { vTaskDelay(500); Serial.print("."); }
    Serial.println("\nWiFi Connected!");

    config.api_key = API_KEY;
    config.database_url = DATABASE_URL;
    if (Firebase.signUp(&config, &auth, "", "")) { signupOK = true; }
    Firebase.begin(&config, &auth);
    
    // Sửa lỗi chính tả chữ 'w' thường
    Firebase.RTDB.setwriteSizeLimit(&fbdo, "tiny"); 

    for (;;) {
        // Gửi 5Hz (200ms) để giám sát
        if (Firebase.ready() && signupOK) {
            FirebaseJson json;
            
            // --- GỬI ĐỦ 5 THÔNG SỐ NHƯ YÊU CẦU ---
            json.set("s_ang", sharedData.s_ang);
            json.set("s_gy",  sharedData.s_gy);
            json.set("s_az",  sharedData.s_az);
            json.set("a_pwm", sharedData.a_pwm);
            json.set("reward", sharedData.r);
            json.set("done",  sharedData.done);
            
            // Gửi lên nhánh cũ hoặc nhánh mới tùy bạn (ở đây dùng RL_Training_Data cho giống format cũ)
            if (Firebase.RTDB.pushJSON(&fbdo, "/RL_Training_Data", &json)) {
                 // Serial.println("Sent OK");
            }
        }
        vTaskDelay(200 / portTICK_PERIOD_MS); 
    }
}

// ================================================================
// SETUP
// ================================================================
void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22, 400000);

    Serial.println(F("Initializing MPU..."));
    mpu.initialize();
    pinMode(MPU_INT_PIN, INPUT); 
    devStatus = mpu.dmpInitialize();

    // CALIBRATION (Của bạn)
    mpu.setXGyroOffset(-78);
    mpu.setYGyroOffset(-99);
    mpu.setZGyroOffset(-25);
    mpu.setZAccelOffset(1342); 

    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();

        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255);

        Serial.println(F("PID Running (Logging 5 params)..."));
    } else {
        Serial.print(F("DMP Failed: "));
        Serial.println(devStatus);
    }

    xTaskCreatePinnedToCore(TaskPID, "PID_Task", 8192, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(TaskFirebase, "FB_Task", 8192, NULL, 1, NULL, 0);
}

void loop() {
    vTaskDelete(NULL);
}
