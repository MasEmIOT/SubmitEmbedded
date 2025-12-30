<<<<<<< HEAD
#include <Arduino.h>
#include <Wire.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#include <WiFi.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Preferences.h>
#include <PID_v1.h> // <--- Thêm thư viện PID

// ================================================================
// ===               1. CẤU HÌNH NGƯỜI DÙNG                     ===
// ================================================================

// --- WiFi & Firebase ---
const char* ssid = "huu nam";
const char* password = "matkhau987";
#define API_KEY "AIzaSyC7kU48070xVL3W10xmXDWropJBNlgLBlA"
#define DATABASE_URL "https://embedded-34716-default-rtdb.asia-southeast1.firebasedatabase.app"

// --- Cấu hình Robot ---
double setpoint = 178.64; // Điểm cân bằng tuyệt đối
#define BAR_LIMIT_ANGLE 8.5 // Góc chạm thanh chắn (Nếu vượt quá -> Bật PID cứu hộ)
#define RL_START_ANGLE 1.5  // Khi PID đưa về được góc này -> Chuyển sang RL

// --- Chế độ chạy ---
// true = Robot HỌC (Q-Learning)
// false = Robot CHẠY (Chỉ dùng PID hoặc RL đã học xong)
#define TRAINING_MODE true 

// --- Động cơ L298N ---
LMotorController motorController(32, 25, 26, 33, 27, 14, 0.6, 0.6);
#define MIN_ABS_SPEED 30

// ================================================================
// ===               2. CẤU HÌNH PID (CHẾ ĐỘ CỨU HỘ)            ===
// ================================================================

// Thông số PID cũ của bạn (Dùng để dựng xe dậy)
double Kp = 30, Ki = 230, Kd = 1.5;
double pid_input, pid_output, pid_setpoint;
PID pid(&pid_input, &pid_output, &pid_setpoint, Kp, Ki, Kd, DIRECT);

// ================================================================
// ===               3. CẤU HÌNH Q-LEARNING (CHẾ ĐỘ HỌC)        ===
// ================================================================

#define NUM_ANGLE_BINS 7  
#define NUM_GYRO_BINS 5   
#define NUM_ACTIONS 5     

float alpha = 0.2;           
float discountFactor = 0.95; 
float epsilon = 0.2;         

#if !TRAINING_MODE
    float current_epsilon = 0.0;
#else
    float current_epsilon = epsilon;
#endif

float Q_Table[NUM_ANGLE_BINS][NUM_GYRO_BINS][NUM_ACTIONS];
int actionPWMs[NUM_ACTIONS] = {-255, -180, 0, 180, 255};

// ================================================================
// ===               4. BIẾN TOÀN CỤC                           ===
// ================================================================

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
bool signupOK = false;

MPU6050 mpu;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
bool dmpReady = false;
uint8_t packetSize;

Preferences preferences; 

// Biến trạng thái hệ thống
bool isRecoveringWithPID = false; // True = Đang dùng PID, False = Đang dùng RL
int prevAngleState = 0;
int prevGyroState = 0;
int prevActionIdx = 2;

// Struct chia sẻ dữ liệu
struct RobotDataRL {
    float s_ang;
    double a_pwm;
    float r;
    bool using_pid; // Báo lên Firebase biết đang chạy chế độ nào
};
volatile RobotDataRL sharedData;

// ================================================================
// ===               5. CÁC HÀM HỖ TRỢ                          ===
// ================================================================

int getAngleState(float angle_error) {
    if (angle_error < -6.0) return 0;
    if (angle_error < -3.5) return 1;
    if (angle_error < -1.5) return 2;
    if (angle_error < 1.5)  return 3; 
    if (angle_error < 3.5)  return 4;
    if (angle_error < 6.0)  return 5;
    return 6;
}

int getGyroState(float gyro) {
    if (gyro < -150) return 0;
    if (gyro < -50)  return 1;
    if (gyro < 50)   return 2;
    if (gyro < 150)  return 3;
    return 4;
}

float calculateReward(float angle_error) {
    float absError = abs(angle_error);
    if (absError < 1.5) return 10.0; 
    if (absError < 4.0) return 2.0;
    if (absError > 6.0) return -5.0; 
    return 0.0;
}

void loadBrain() {
    pinMode(0, INPUT_PULLUP); 
    if (digitalRead(0) == LOW) { 
        preferences.begin("robot_brain", false);
        preferences.clear(); 
        preferences.end();
        memset(Q_Table, 0, sizeof(Q_Table));
        Serial.println(">>> ĐÃ XÓA NÃO! PID SẼ DỰNG XE DẬY ĐỂ HỌC LẠI.");
        delay(2000);
    } else {
        preferences.begin("robot_brain", true);
        if (preferences.isKey("q_table")) {
            preferences.getBytes("q_table", (byte*)Q_Table, sizeof(Q_Table));
            Serial.println(">> ĐÃ TẢI KINH NGHIỆM TỪ FLASH!");
        }
        preferences.end();
    }
}

void saveBrain() {
    preferences.begin("robot_brain", false);
    preferences.putBytes("q_table", (byte*)Q_Table, sizeof(Q_Table));
    preferences.end();
}

int selectAction(int angleState, int gyroState) {
    if (random(0, 100) < current_epsilon * 100) {
        return random(0, NUM_ACTIONS);
    }
    int bestAction = 2;
    float maxQ = -999999.0;
    for (int i = 0; i < NUM_ACTIONS; i++) {
        if (Q_Table[angleState][gyroState][i] > maxQ) {
            maxQ = Q_Table[angleState][gyroState][i];
            bestAction = i;
        }
    }
    return bestAction;
}

// ================================================================
// ===               TASK 1: LOGIC ĐIỀU KHIỂN (HYBRID)          ===
// ================================================================

void TaskControl(void *pvParameters) {
    int16_t gx, gy, gz;
    
    for (;;) {
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            float input_angle = ypr[1] * 180 / M_PI + 180; 
            float angle_error = input_angle - setpoint;    
            
            mpu.getRotation(&gx, &gy, &gz);
            float current_gyro = (float)gy / 131.0; 

            // --- LOGIC CHUYỂN ĐỔI CHẾ ĐỘ (PID <-> RL) ---

            // 1. Nếu nghiêng quá nhiều (dựa vào thanh chắn) -> KÍCH HOẠT PID CỨU HỘ
            if (abs(angle_error) > BAR_LIMIT_ANGLE) {
                if (!isRecoveringWithPID) {
                    isRecoveringWithPID = true; // Chuyển sang chế độ PID
                    // Nếu đang học mà ngã -> Phạt nặng vào RL trước khi chuyển sang PID
                    if (TRAINING_MODE) {
                        Q_Table[prevAngleState][prevGyroState][prevActionIdx] -= 50.0;
                        saveBrain();
                    }
                }
            }
            
            // 2. Nếu PID đã dựng xe dậy ngon lành (góc nhỏ) -> TRẢ VỀ CHO RL
            else if (isRecoveringWithPID && abs(angle_error) < RL_START_ANGLE) {
                isRecoveringWithPID = false; // Chuyển quyền lại cho RL
                
                // Reset trạng thái RL để bắt đầu episode mới mượt mà
                prevAngleState = getAngleState(angle_error);
                prevGyroState = getGyroState(current_gyro);
                prevActionIdx = 2; 
            }

            // --- THỰC THI ĐIỀU KHIỂN ---

            if (isRecoveringWithPID) {
                // *** CHẾ ĐỘ PID (DỰNG XE DẬY) ***
                pid_input = input_angle;
                pid_setpoint = setpoint;
                pid.Compute(); // Tính toán PID
                
                // Điều khiển động cơ bằng PID output
                motorController.move(pid_output, MIN_ABS_SPEED);
                
                // Cập nhật data gửi đi
                sharedData.a_pwm = pid_output;
                sharedData.using_pid = true;
            } 
            else {
                // *** CHẾ ĐỘ RL (HỌC & GIỮ THĂNG BẰNG) ***
                int currAngleState = getAngleState(angle_error);
                int currGyroState = getGyroState(current_gyro);
                float reward = calculateReward(angle_error);

                if (TRAINING_MODE) {
                    float maxQ_next = -9999.0;
                    for (int i = 0; i < NUM_ACTIONS; i++) {
                        if (Q_Table[currAngleState][currGyroState][i] > maxQ_next)
                            maxQ_next = Q_Table[currAngleState][currGyroState][i];
                    }
                    Q_Table[prevAngleState][prevGyroState][prevActionIdx] += 
                        alpha * (reward + discountFactor * maxQ_next - Q_Table[prevAngleState][prevGyroState][prevActionIdx]);
                }

                int nextActionIdx = selectAction(currAngleState, currGyroState);
                motorController.move(actionPWMs[nextActionIdx], MIN_ABS_SPEED);

                prevAngleState = currAngleState;
                prevGyroState = currGyroState;
                prevActionIdx = nextActionIdx;

                sharedData.a_pwm = actionPWMs[nextActionIdx];
                sharedData.using_pid = false;
            }

            sharedData.s_ang = angle_error;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); 
    }
}

// ================================================================
// ===               TASK 2: FIREBASE (Core 0)                  ===
// ================================================================

void TaskFirebase(void *pvParameters) {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { delay(500); }
    
    config.api_key = API_KEY;
    config.database_url = DATABASE_URL;
    fbdo.setResponseSize(4096);
    if (Firebase.signUp(&config, &auth, "", "")) { signupOK = true; }
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);

    for (;;) {
        if (Firebase.ready() && signupOK) {
            String path = "/RL_Status"; 
            FirebaseJson json;
            json.set("Angle_Error", sharedData.s_ang);
            json.set("PWM", sharedData.a_pwm);
            json.set("Mode", sharedData.using_pid ? "PID_RECOVERY" : "RL_TRAINING");

            // Chỉ dùng setJSON để update trạng thái (tiết kiệm băng thông hơn push)
            Firebase.RTDB.setJSON(&fbdo, path, &json);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS); 
    }
}

// ================================================================
// ===                       SETUP                              ===
// ================================================================

void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22, 400000);

    loadBrain(); 

    Serial.println("Init MPU...");
    mpu.initialize();
    if (mpu.dmpInitialize() == 0) {
        mpu.setXGyroOffset(-73);
        mpu.setYGyroOffset(-95);
        mpu.setZGyroOffset(-20);
        mpu.setZAccelOffset(1350); 
        mpu.setDMPEnabled(true);
        packetSize = mpu.dmpGetFIFOPacketSize();
        dmpReady = true;
        Serial.println("MPU OK!");
    } else {
        Serial.println("MPU Failed!"); while(1);
    }

    // Cấu hình PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);

    // Chạy Task
    xTaskCreatePinnedToCore(TaskControl, "Control_Task", 8192, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(TaskFirebase, "FB_Task", 8192, NULL, 1, NULL, 0);

    Serial.println("SYSTEM STARTED. PID + RL HYBRID MODE.");
}

void loop() {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
=======
#include <Arduino.h>
#include <Wire.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#include <WiFi.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Preferences.h>
#include <PID_v1.h> // <--- Thêm thư viện PID

// ================================================================
// ===               1. CẤU HÌNH NGƯỜI DÙNG                     ===
// ================================================================

// --- WiFi & Firebase ---
const char* ssid = "huu nam";
const char* password = "matkhau987";
#define API_KEY "AIzaSyC7kU48070xVL3W10xmXDWropJBNlgLBlA"
#define DATABASE_URL "https://embedded-34716-default-rtdb.asia-southeast1.firebasedatabase.app"

// --- Cấu hình Robot ---
double setpoint = 178.64; // Điểm cân bằng tuyệt đối
#define BAR_LIMIT_ANGLE 8.5 // Góc chạm thanh chắn (Nếu vượt quá -> Bật PID cứu hộ)
#define RL_START_ANGLE 1.5  // Khi PID đưa về được góc này -> Chuyển sang RL

// --- Chế độ chạy ---
// true = Robot HỌC (Q-Learning)
// false = Robot CHẠY (Chỉ dùng PID hoặc RL đã học xong)
#define TRAINING_MODE true 

// --- Động cơ L298N ---
LMotorController motorController(32, 25, 26, 33, 27, 14, 0.6, 0.6);
#define MIN_ABS_SPEED 30

// ================================================================
// ===               2. CẤU HÌNH PID (CHẾ ĐỘ CỨU HỘ)            ===
// ================================================================

// Thông số PID cũ của bạn (Dùng để dựng xe dậy)
double Kp = 30, Ki = 230, Kd = 1.5;
double pid_input, pid_output, pid_setpoint;
PID pid(&pid_input, &pid_output, &pid_setpoint, Kp, Ki, Kd, DIRECT);

// ================================================================
// ===               3. CẤU HÌNH Q-LEARNING (CHẾ ĐỘ HỌC)        ===
// ================================================================

#define NUM_ANGLE_BINS 7  
#define NUM_GYRO_BINS 5   
#define NUM_ACTIONS 5     

float alpha = 0.2;           
float discountFactor = 0.95; 
float epsilon = 0.2;         

#if !TRAINING_MODE
    float current_epsilon = 0.0;
#else
    float current_epsilon = epsilon;
#endif

float Q_Table[NUM_ANGLE_BINS][NUM_GYRO_BINS][NUM_ACTIONS];
int actionPWMs[NUM_ACTIONS] = {-255, -180, 0, 180, 255};

// ================================================================
// ===               4. BIẾN TOÀN CỤC                           ===
// ================================================================

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
bool signupOK = false;

MPU6050 mpu;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
bool dmpReady = false;
uint8_t packetSize;

Preferences preferences; 

// Biến trạng thái hệ thống
bool isRecoveringWithPID = false; // True = Đang dùng PID, False = Đang dùng RL
int prevAngleState = 0;
int prevGyroState = 0;
int prevActionIdx = 2;

// Struct chia sẻ dữ liệu
struct RobotDataRL {
    float s_ang;
    double a_pwm;
    float r;
    bool using_pid; // Báo lên Firebase biết đang chạy chế độ nào
};
volatile RobotDataRL sharedData;

// ================================================================
// ===               5. CÁC HÀM HỖ TRỢ                          ===
// ================================================================

int getAngleState(float angle_error) {
    if (angle_error < -6.0) return 0;
    if (angle_error < -3.5) return 1;
    if (angle_error < -1.5) return 2;
    if (angle_error < 1.5)  return 3; 
    if (angle_error < 3.5)  return 4;
    if (angle_error < 6.0)  return 5;
    return 6;
}

int getGyroState(float gyro) {
    if (gyro < -150) return 0;
    if (gyro < -50)  return 1;
    if (gyro < 50)   return 2;
    if (gyro < 150)  return 3;
    return 4;
}

float calculateReward(float angle_error) {
    float absError = abs(angle_error);
    if (absError < 1.5) return 10.0; 
    if (absError < 4.0) return 2.0;
    if (absError > 6.0) return -5.0; 
    return 0.0;
}

void loadBrain() {
    pinMode(0, INPUT_PULLUP); 
    if (digitalRead(0) == LOW) { 
        preferences.begin("robot_brain", false);
        preferences.clear(); 
        preferences.end();
        memset(Q_Table, 0, sizeof(Q_Table));
        Serial.println(">>> ĐÃ XÓA NÃO! PID SẼ DỰNG XE DẬY ĐỂ HỌC LẠI.");
        delay(2000);
    } else {
        preferences.begin("robot_brain", true);
        if (preferences.isKey("q_table")) {
            preferences.getBytes("q_table", (byte*)Q_Table, sizeof(Q_Table));
            Serial.println(">> ĐÃ TẢI KINH NGHIỆM TỪ FLASH!");
        }
        preferences.end();
    }
}

void saveBrain() {
    preferences.begin("robot_brain", false);
    preferences.putBytes("q_table", (byte*)Q_Table, sizeof(Q_Table));
    preferences.end();
}

int selectAction(int angleState, int gyroState) {
    if (random(0, 100) < current_epsilon * 100) {
        return random(0, NUM_ACTIONS);
    }
    int bestAction = 2;
    float maxQ = -999999.0;
    for (int i = 0; i < NUM_ACTIONS; i++) {
        if (Q_Table[angleState][gyroState][i] > maxQ) {
            maxQ = Q_Table[angleState][gyroState][i];
            bestAction = i;
        }
    }
    return bestAction;
}

// ================================================================
// ===               TASK 1: LOGIC ĐIỀU KHIỂN (HYBRID)          ===
// ================================================================

void TaskControl(void *pvParameters) {
    int16_t gx, gy, gz;
    
    for (;;) {
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            float input_angle = ypr[1] * 180 / M_PI + 180; 
            float angle_error = input_angle - setpoint;    
            
            mpu.getRotation(&gx, &gy, &gz);
            float current_gyro = (float)gy / 131.0; 

            // --- LOGIC CHUYỂN ĐỔI CHẾ ĐỘ (PID <-> RL) ---

            // 1. Nếu nghiêng quá nhiều (dựa vào thanh chắn) -> KÍCH HOẠT PID CỨU HỘ
            if (abs(angle_error) > BAR_LIMIT_ANGLE) {
                if (!isRecoveringWithPID) {
                    isRecoveringWithPID = true; // Chuyển sang chế độ PID
                    // Nếu đang học mà ngã -> Phạt nặng vào RL trước khi chuyển sang PID
                    if (TRAINING_MODE) {
                        Q_Table[prevAngleState][prevGyroState][prevActionIdx] -= 50.0;
                        saveBrain();
                    }
                }
            }
            
            // 2. Nếu PID đã dựng xe dậy ngon lành (góc nhỏ) -> TRẢ VỀ CHO RL
            else if (isRecoveringWithPID && abs(angle_error) < RL_START_ANGLE) {
                isRecoveringWithPID = false; // Chuyển quyền lại cho RL
                
                // Reset trạng thái RL để bắt đầu episode mới mượt mà
                prevAngleState = getAngleState(angle_error);
                prevGyroState = getGyroState(current_gyro);
                prevActionIdx = 2; 
            }

            // --- THỰC THI ĐIỀU KHIỂN ---

            if (isRecoveringWithPID) {
                // *** CHẾ ĐỘ PID (DỰNG XE DẬY) ***
                pid_input = input_angle;
                pid_setpoint = setpoint;
                pid.Compute(); // Tính toán PID
                
                // Điều khiển động cơ bằng PID output
                motorController.move(pid_output, MIN_ABS_SPEED);
                
                // Cập nhật data gửi đi
                sharedData.a_pwm = pid_output;
                sharedData.using_pid = true;
            } 
            else {
                // *** CHẾ ĐỘ RL (HỌC & GIỮ THĂNG BẰNG) ***
                int currAngleState = getAngleState(angle_error);
                int currGyroState = getGyroState(current_gyro);
                float reward = calculateReward(angle_error);

                if (TRAINING_MODE) {
                    float maxQ_next = -9999.0;
                    for (int i = 0; i < NUM_ACTIONS; i++) {
                        if (Q_Table[currAngleState][currGyroState][i] > maxQ_next)
                            maxQ_next = Q_Table[currAngleState][currGyroState][i];
                    }
                    Q_Table[prevAngleState][prevGyroState][prevActionIdx] += 
                        alpha * (reward + discountFactor * maxQ_next - Q_Table[prevAngleState][prevGyroState][prevActionIdx]);
                }

                int nextActionIdx = selectAction(currAngleState, currGyroState);
                motorController.move(actionPWMs[nextActionIdx], MIN_ABS_SPEED);

                prevAngleState = currAngleState;
                prevGyroState = currGyroState;
                prevActionIdx = nextActionIdx;

                sharedData.a_pwm = actionPWMs[nextActionIdx];
                sharedData.using_pid = false;
            }

            sharedData.s_ang = angle_error;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); 
    }
}

// ================================================================
// ===               TASK 2: FIREBASE (Core 0)                  ===
// ================================================================

void TaskFirebase(void *pvParameters) {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { delay(500); }
    
    config.api_key = API_KEY;
    config.database_url = DATABASE_URL;
    fbdo.setResponseSize(4096);
    if (Firebase.signUp(&config, &auth, "", "")) { signupOK = true; }
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);

    for (;;) {
        if (Firebase.ready() && signupOK) {
            String path = "/RL_Status"; 
            FirebaseJson json;
            json.set("Angle_Error", sharedData.s_ang);
            json.set("PWM", sharedData.a_pwm);
            json.set("Mode", sharedData.using_pid ? "PID_RECOVERY" : "RL_TRAINING");

            // Chỉ dùng setJSON để update trạng thái (tiết kiệm băng thông hơn push)
            Firebase.RTDB.setJSON(&fbdo, path, &json);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS); 
    }
}

// ================================================================
// ===                       SETUP                              ===
// ================================================================

void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22, 400000);

    loadBrain(); 

    Serial.println("Init MPU...");
    mpu.initialize();
    if (mpu.dmpInitialize() == 0) {
        mpu.setXGyroOffset(-73);
        mpu.setYGyroOffset(-95);
        mpu.setZGyroOffset(-20);
        mpu.setZAccelOffset(1350); 
        mpu.setDMPEnabled(true);
        packetSize = mpu.dmpGetFIFOPacketSize();
        dmpReady = true;
        Serial.println("MPU OK!");
    } else {
        Serial.println("MPU Failed!"); while(1);
    }

    // Cấu hình PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);

    // Chạy Task
    xTaskCreatePinnedToCore(TaskControl, "Control_Task", 8192, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(TaskFirebase, "FB_Task", 8192, NULL, 1, NULL, 0);

    Serial.println("SYSTEM STARTED. PID + RL HYBRID MODE.");
}

void loop() {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
>>>>>>> b974c8d5686fb29fbbb400af23a843378e66833f
}