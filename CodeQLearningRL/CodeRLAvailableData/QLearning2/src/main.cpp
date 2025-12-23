/**
 * ROBOT CÂN BẰNG TỰ HỌC - PHIÊN BẢN HYBRID (PID + Q-LEARNING)
 * - Tự động đứng dậy bằng PID.
 * - Tự động học cân bằng bằng Q-Learning.
 * - Đã nạp sẵn kiến thức cơ bản từ dữ liệu cũ.
 */

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
#include <PID_v1.h> 

// ================================================================
// ===               1. CẤU HÌNH NGƯỜI DÙNG                     ===
// ================================================================

// --- WiFi & Firebase --- 
const char* ssid = "huu nam";
const char* password = "matkhau987";
#define API_KEY "AIzaSyC7kU48070xVL3W10xmXDWropJBNlgLBlA"
#define DATABASE_URL "https://embedded-34716-default-rtdb.asia-southeast1.firebasedatabase.app"

// --- Cấu hình Robot ---
double setpoint = 178.64; // Góc đứng thẳng tuyệt đối
#define BAR_LIMIT_ANGLE 8.5 // Góc chạm thanh chắn (9 độ) -> Kích hoạt PID cứu hộ
#define RL_START_ANGLE 1.5  // Khi PID đưa về góc này -> Chuyển sang cho RL học

// --- Chế độ chạy ---
// true = Cho phép học cập nhật thêm vào não
// false = Chỉ chạy, không sửa não nữa
#define TRAINING_MODE false

// --- Động cơ L298N ---
// (ENA, IN1, IN2, ENB, IN3, IN4) - Sửa lại nếu bánh quay ngược
LMotorController motorController(32, 25, 26, 16, 27, 14, 0.6, 0.6);
#define MIN_ABS_SPEED 30

// ================================================================
// ===               2. CẤU HÌNH Q-LEARNING                     ===
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

// BẢNG Q-TABLE ĐÃ PRE-TRAIN TỪ DỮ LIỆU PID CỦA BẠN
// (Robot đã có kiến thức nền, không cần học từ con số 0)
float Q_Table[NUM_ANGLE_BINS][NUM_GYRO_BINS][NUM_ACTIONS] = {
  { // Angle Bin 0 (Ngã Lùi Nặng)
    { 0.00, 0.00, 0.00, 0.00, 0.00 },
    { 0.00, 0.00, 0.00, 0.00, 0.00 },
    { 0.00, 0.00, 6.87, 15.89, 15.84 }, // PID khuyên: Hãy chạy tới!
    { 0.00, 0.00, 0.00, 0.00, 0.00 },
    { 0.00, 0.00, 0.00, 0.00, 0.00 }
  },
  { // Angle Bin 1 (Nghiêng Lùi)
    { 0.00, 0.00, 0.00, 0.00, 0.00 },
    { 0.00, 0.00, 0.00, 0.00, 0.00 },
    { 0.00, 5.40, 16.42, 16.96, 16.72 }, // PID khuyên: Chạy tới mạnh
    { 0.00, 0.00, 0.00, 0.00, 0.00 },
    { 0.00, 0.00, 0.00, 0.00, 0.00 }
  },
  { // Angle Bin 2 (Hơi Lùi)
    { 0.00, 0.00, 0.00, 0.00, 0.00 },
    { 0.00, 0.00, 0.00, 0.00, 0.00 },
    { 5.43, 15.84, 17.04, 16.91, 15.30 }, // PID khuyên: Chạy tới vừa
    { 0.00, 0.00, 0.00, 0.00, 0.00 },
    { 0.00, 0.00, 0.00, 0.00, 0.00 }
  },
  { // Angle Bin 3 (CÂN BẰNG - Vùng Ngon Nhất)
    { 0.00, 0.00, 0.00, 0.00, 0.00 },
    { 0.00, 0.00, 0.00, 0.00, 0.00 },
    { 16.17, 17.20, 17.06, 17.22, 10.85 }, // PID khuyên: Giữ nguyên hoặc nhích nhẹ
    { 0.00, 0.00, 0.00, 0.00, 0.00 },
    { 0.00, 0.00, 0.00, 0.00, 0.00 }
  },
  { // Angle Bin 4 (Hơi Tiến)
    { 0.00, 0.00, 0.00, 0.00, 0.00 },
    { 0.00, 0.00, 0.00, 0.00, 0.00 },
    { 15.70, 16.94, 16.79, 16.35, 2.92 }, // PID khuyên: Lùi lại
    { 0.00, 0.00, 0.00, 0.00, 0.00 },
    { 0.00, 0.00, 0.00, 0.00, 0.00 }
  },
  { // Angle Bin 5 (Nghiêng Tiến)
    { 0.00, 0.00, 0.00, 0.00, 0.00 },
    { 0.00, 0.00, 0.00, 0.00, 0.00 },
    { 16.76, 16.81, 16.65, 7.93, 0.00 }, // PID khuyên: Lùi mạnh
    { 0.00, 0.00, 0.00, 0.00, 0.00 },
    { 0.00, 0.00, 0.00, 0.00, 0.00 }
  },
  { // Angle Bin 6 (Ngã Tiến Nặng)
    { 0.00, 0.00, 0.00, 0.00, 0.00 },
    { 0.00, 0.00, 0.00, 0.00, 0.00 },
    { 16.40, 14.93, 4.14, 0.00, 0.00 }, // PID khuyên: Lùi hết tốc lực
    { 0.00, 0.00, 0.00, 0.00, 0.00 },
    { 0.00, 0.00, 0.00, 0.00, 0.00 }
  }
};

// Các mức PWM hành động
int actionPWMs[NUM_ACTIONS] = {-255, -180, 0, 180, 255};

// ================================================================
// ===               3. CẤU HÌNH PID (CHẾ ĐỘ CỨU HỘ)            ===
// ================================================================

double Kp = 30, Ki = 230, Kd = 1.5;
double pid_input, pid_output, pid_setpoint;
PID pid(&pid_input, &pid_output, &pid_setpoint, Kp, Ki, Kd, DIRECT);

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

bool isRecoveringWithPID = false; 
int prevAngleState = 0;
int prevGyroState = 0;
int prevActionIdx = 2;

struct RobotDataRL {
    float s_ang;
    double a_pwm;
    bool using_pid; 
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

// Hàm tải não: Ưu tiên Flash, nếu không có thì dùng mảng Pre-trained ở trên
void loadBrain() {
    pinMode(0, INPUT_PULLUP); 
    
    // NẾU NHẤN NÚT BOOT -> XÓA FLASH ĐỂ QUAY VỀ MẶC ĐỊNH (PRE-TRAINED)
    if (digitalRead(0) == LOW) { 
        preferences.begin("robot_brain", false);
        preferences.clear(); 
        preferences.end();
        Serial.println(">>> ĐÃ RESET FLASH! SẼ DÙNG BẢNG PRE-TRAINED MẶC ĐỊNH.");
        for(int i=0; i<10; i++) { // Nháy đèn báo hiệu
             digitalWrite(2, !digitalRead(2)); delay(100);
        }
    } 
    else {
        preferences.begin("robot_brain", true);
        if (preferences.isKey("q_table")) {
            preferences.getBytes("q_table", (byte*)Q_Table, sizeof(Q_Table));
            Serial.println(">> ĐÃ TẢI KINH NGHIỆM TỪ FLASH!");
        } else {
            Serial.println(">> DÙNG BẢNG PRE-TRAINED TỪ CODE!");
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
    // Epsilon-Greedy: Đôi khi thử nghiệm cái mới
    if (random(0, 100) < current_epsilon * 100) {
        return random(0, NUM_ACTIONS);
    }
    // Chọn hành động tốt nhất hiện tại
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
// ===               TASK 1: ĐIỀU KHIỂN CHÍNH (Core 1)          ===
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

            // --- 1. KIỂM TRA CHUYỂN CHẾ ĐỘ ---
            
            // Nếu nghiêng quá 8.5 độ (dựa vào thanh chắn) -> Dùng PID
            if (abs(angle_error) > BAR_LIMIT_ANGLE) {
                if (!isRecoveringWithPID) {
                    isRecoveringWithPID = true; 
                    // Phạt cú chót vào RL trước khi chuyển giao
                    if (TRAINING_MODE) {
                        Q_Table[prevAngleState][prevGyroState][prevActionIdx] -= 20.0;
                        saveBrain();
                    }
                }
            }
            // Nếu PID đã dựng xe dậy ngon lành (< 1.5 độ) -> Trả về RL
            else if (isRecoveringWithPID && abs(angle_error) < RL_START_ANGLE) {
                isRecoveringWithPID = false; 
                prevAngleState = getAngleState(angle_error);
                prevGyroState = getGyroState(current_gyro);
                prevActionIdx = 2; 
            }

            // --- 2. THỰC THI ---

            if (isRecoveringWithPID) {
                // *** CHẾ ĐỘ PID (CỨU HỘ) ***
                pid_input = input_angle;
                pid_setpoint = setpoint;
                pid.Compute(); 
                motorController.move(pid_output, MIN_ABS_SPEED);
                
                sharedData.a_pwm = pid_output;
                sharedData.using_pid = true;
            } 
            else {
                // *** CHẾ ĐỘ RL (HỌC & GIỮ THĂNG BẰNG) ***
                int currAngleState = getAngleState(angle_error);
                int currGyroState = getGyroState(current_gyro);
                
                // Tính thưởng
                float reward = calculateReward(angle_error);

                // Học (Cập nhật Q-Table)
                if (TRAINING_MODE) {
                    float maxQ_next = -9999.0;
                    for (int i = 0; i < NUM_ACTIONS; i++) {
                        if (Q_Table[currAngleState][currGyroState][i] > maxQ_next)
                            maxQ_next = Q_Table[currAngleState][currGyroState][i];
                    }
                    Q_Table[prevAngleState][prevGyroState][prevActionIdx] += 
                        alpha * (reward + discountFactor * maxQ_next - Q_Table[prevAngleState][prevGyroState][prevActionIdx]);
                }

                // Chọn hành động từ bảng (đã pre-train)
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
// ===               TASK 2: GỬI FIREBASE (Core 0)              ===
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
            
            // Dùng setJSON để cập nhật trạng thái realtime
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
    pinMode(2, OUTPUT); // Đèn LED báo hiệu

    // 1. Tải Não (Nếu chưa có Flash thì dùng Pre-trained)
    loadBrain(); 

    // 2. Khởi tạo MPU
    Serial.println("Init MPU...");
    mpu.initialize();
    if (mpu.dmpInitialize() == 0) {
        // Offset của bạn
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

    // 3. Cấu hình PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);

    // 4. Chạy Task
    xTaskCreatePinnedToCore(TaskControl, "Control_Task", 8192, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(TaskFirebase, "FB_Task", 8192, NULL, 1, NULL, 0);

    Serial.println("=== SYSTEM READY: HYBRID MODE ===");
    Serial.println("1. RL giữ cân bằng trong vùng +/- 8.5 độ");
    Serial.println("2. PID tự dựng xe dậy khi chạm thanh chắn");
}

void loop() {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}