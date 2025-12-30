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

// --- Cấu hình WiFi & Firebase ---
const char* ssid = "huu nam";
const char* password = "matkhau987";
#define API_KEY "AIzaSyC7kU48070xVL3W10xmXDWropJBNlgLBlA"
#define DATABASE_URL "https://embedded-34716-default-rtdb.asia-southeast1.firebasedatabase.app"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
bool signupOK = false;

// --- Cấu hình MPU6050 & Motor ---
MPU6050 mpu;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
bool dmpReady = false;
uint8_t packetSize;

// --- Cấu hình PID & Motor ---
double setpoint = 180.0;
double input, output;
// Giá trị khởi tạo (sẽ được RL chỉnh lại trong lúc chạy)
double Kp = 20.0, Ki = 200.0, Kd = 1.5; 

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
LMotorController motorController(32, 25, 26, 33, 27, 14, 0.6, 0.6);
#define MIN_ABS_SPEED 60 // Tăng lên 60 cho động cơ vàng dễ khởi động

// --- Q-LEARNING CONFIGURATION ---
#define NUM_STATES 3   // 0: Ổn định, 1: Rung nhẹ, 2: Rung mạnh/Sắp đổ
#define NUM_ACTIONS 5  // 0: Giữ nguyên, 1: Tăng Kp, 2: Giảm Kp, 3: Tăng Kd, 4: Giảm Kd
float Q_Table[NUM_STATES][NUM_ACTIONS]; 

// Tham số RL
float alpha = 0.1;   // Learning rate
float gamma_rl = 0.9;   // Discount factor
float epsilon = 0.2; // Exploration rate (20% thử ngẫu nhiên)

// Biến trạng thái RL
int currentState = 0;
int lastState = 0;
int currentAction = 0;
float currentReward = 0;
unsigned long lastRLTime = 0;

// Shared Data giữa 2 Core
volatile float shared_angle_error = 0;
volatile float shared_pwm = 0;

// --- HÀM HỖ TRỢ RL ---

// 1. Xác định trạng thái dựa trên sai số góc
int getRLState(float error) {
    float absErr = abs(error);
    if (absErr < 2.0) return 0;       // Rất ổn định
    else if (absErr < 8.0) return 1;  // Hơi lắc
    else return 2;                    // Nguy hiểm
}

// 2. Tính điểm thưởng (Reward Function)
float calculateReward(float error) {
    float absErr = abs(error);
    // Thưởng cao nếu đứng thẳng, phạt nặng nếu nghiêng
    if (absErr < 2.0) return 2.0; 
    if (absErr < 5.0) return 1.0;
    if (absErr > 30.0) return -100.0; // Xe đổ
    return -0.1 * absErr; // Phạt nhẹ theo độ nghiêng
}

// 3. Chọn hành động (Epsilon-Greedy)
int chooseAction(int state) {
    if (random(0, 100) < epsilon * 100) {
        return random(0, NUM_ACTIONS); // Khám phá
    } else {
        // Chọn tốt nhất (Exploit)
        float maxQ = -9999;
        int bestAction = 0;
        for (int i = 0; i < NUM_ACTIONS; i++) {
            if (Q_Table[state][i] > maxQ) {
                maxQ = Q_Table[state][i];
                bestAction = i;
            }
        }
        return bestAction;
    }
}

// 4. Thực thi hành động (Điều chỉnh PID)
void executeAction(int action) {
    switch (action) {
        case 0: break; // Do nothing
        case 1: Kp += 1.0; break;
        case 2: Kp -= 1.0; break;
        case 3: Kd += 0.2; break;
        case 4: Kd -= 0.2; break;
    }
    
    // Giới hạn để không cháy động cơ hoặc mất kiểm soát
    Kp = constrain(Kp, 10.0, 60.0);
    Kd = constrain(Kd, 0.5, 5.0);
    
    // Cập nhật vào bộ điều khiển PID ngay lập tức
    pid.SetTunings(Kp, Ki, Kd);
}

// ----------------------------------------------------------------
// TASK 1: PID DRIVER (Core 1) - Chạy nhanh nhất có thể
// ----------------------------------------------------------------
void TaskPID(void *pvParameters) {
    for (;;) {
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            
            // Góc thực tế
            input = ypr[1] * 180 / M_PI + 180;
            
            // Chia sẻ dữ liệu sang Core 0
            shared_angle_error = input - setpoint;

            // Nếu xe đổ quá 45 độ, tắt động cơ để bảo vệ
            if (abs(input - setpoint) > 45) {
                motorController.stopMoving();
            } else {
                pid.Compute();
                motorController.move(output, MIN_ABS_SPEED);
                shared_pwm = output;
            }
        }
        vTaskDelay(5 / portTICK_PERIOD_MS); // Chu kỳ PID ~5-10ms
    }
}

// ----------------------------------------------------------------
// TASK 2: RL BRAIN & FIREBASE (Core 0) - Chạy chậm hơn (100ms)
// ----------------------------------------------------------------
void TaskRL(void *pvParameters) {
    // Setup WiFi & Firebase
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { vTaskDelay(100); }
    config.api_key = API_KEY;
    config.database_url = DATABASE_URL;
    if (Firebase.signUp(&config, &auth, "", "")) { signupOK = true; }
    Firebase.begin(&config, &auth);

    for (;;) {
        unsigned long now = millis();
        
        // --- CHẠY THUẬT TOÁN Q-LEARNING (Mỗi 100ms) ---
        if (now - lastRLTime > 100) {
            // 1. Lấy trạng thái hiện tại
            currentState = getRLState(shared_angle_error);
            
            // 2. Tính phần thưởng cho hành động TRƯỚC ĐÓ
            currentReward = calculateReward(shared_angle_error);
            
            // 3. Cập nhật Q-Table (Bellman Equation)
            float maxQ_next = -9999;
            for (int i = 0; i < NUM_ACTIONS; i++) {
                if (Q_Table[currentState][i] > maxQ_next) maxQ_next = Q_Table[currentState][i];
            }
            // Q(s,a) = Q(s,a) + alpha * [R + gamma_rl * maxQ(s') - Q(s,a)]
            Q_Table[lastState][currentAction] += alpha * (currentReward + gamma_rl * maxQ_next - Q_Table[lastState][currentAction]);
            
            // 4. Chọn và Thực thi hành động MỚI
            currentAction = chooseAction(currentState);
            executeAction(currentAction);
            
            // 5. Lưu trạng thái
            lastState = currentState;
            lastRLTime = now;

            // Debug ra Serial
            Serial.print("Err:"); Serial.print(shared_angle_error);
            Serial.print(" | Kp:"); Serial.print(Kp);
            Serial.print(" | Kd:"); Serial.print(Kd);
            Serial.print(" | Rew:"); Serial.println(currentReward);
        }

        // --- GỬI FIREBASE (Thỉnh thoảng gửi để đỡ lag) ---
        static unsigned long lastFBTime = 0;
        if (now - lastFBTime > 500 && Firebase.ready() && signupOK) {
            FirebaseJson json;
            json.set("angle_error", shared_angle_error);
            json.set("Kp", Kp);
            json.set("Kd", Kd);
            json.set("reward", currentReward);
            Firebase.RTDB.pushJSON(&fbdo, "/RL_Log", &json);
            lastFBTime = now;
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22, 400000);

    // Khởi tạo MPU6050
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

    // Khởi tạo PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10); // PID chạy mỗi 10ms
    pid.SetOutputLimits(-255, 255);

    // Khởi tạo Q-Table về 0
    for(int i=0; i<NUM_STATES; i++)
        for(int j=0; j<NUM_ACTIONS; j++)
            Q_Table[i][j] = 0.0;

    // Chạy Multithreading
    // Core 1: PID (Ưu tiên cao nhất)
    xTaskCreatePinnedToCore(TaskPID, "PID_Task", 4096, NULL, 5, NULL, 1);
    // Core 0: RL & Firebase (Ưu tiên thấp hơn)
    xTaskCreatePinnedToCore(TaskRL, "RL_Task", 8192, NULL, 1, NULL, 0);
}

void loop() {
    // Loop để trống vì mọi thứ đã chạy trong Task
}