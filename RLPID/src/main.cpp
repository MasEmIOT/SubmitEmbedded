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
const char* ssid = "huu nam";
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
#define MPU_INT_PIN 4  // --- CHÂN NGẮT: NỐI INT CỦA MPU VÀO D4 CỦA ESP32 ---

// MPU Status Vars
bool dmpReady = false;  
uint8_t mpuIntStatus;   
uint8_t devStatus;      
uint16_t packetSize;    
uint16_t fifoCount;     
uint8_t fifoBuffer[64]; 

// Orientation Vars
Quaternion q;           
VectorFloat gravity;    
float ypr[3];           

// ================================================================
// 3. CẤU HÌNH PID & MOTOR
// ================================================================
double setpoint = 174.2; // Góc cân bằng
double input, output;
double Kp = 30.0, Ki = 0.0, Kd = 0.5; 

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
LMotorController motorController(32, 25, 26, 33, 27, 14, 0.6, 0.6);
#define MIN_ABS_SPEED 40

// ================================================================
// 4. BIẾN CHIA SẺ (SHARED DATA) CHO RL - ĐỦ 5 THÔNG SỐ
// ================================================================
struct RobotDataRL {
    float s_ang;   // 1. Góc lệch
    float s_gy;    // 2. Vận tốc góc (Gyro Y)
    float s_az;    // 3. Gia tốc trục Z (Accel Z)
    double a_pwm;  // 4. PWM hiện tại
    float r;       // 5. Phần thưởng
    bool done;     // Trạng thái kết thúc
};
volatile RobotDataRL sharedData;

// ================================================================
// 5. CẤU HÌNH Q-LEARNING (AI)
// ================================================================
#define NUM_STATES 3   
#define NUM_ACTIONS 5  
float Q_Table[NUM_STATES][NUM_ACTIONS]; 

float alpha = 0.1;    
float gamma_rl = 0.9; 
float epsilon = 0.2;  

int currentState = 0;
int lastState = 0;
int currentAction = 0;
float currentReward = 0;
unsigned long lastRLTime = 0;

// --- HÀM HỖ TRỢ RL ---
int getRLState(float error) {
    float absErr = abs(error);
    if (absErr < 2.0) return 0;       
    else if (absErr < 8.0) return 1;  
    else return 2;                    
}

float calculateReward(float error) {
    float absErr = abs(error);
    if (absErr < 2.0) return 5.0;     
    if (absErr < 5.0) return 1.0;     
    if (absErr > 30.0) return -100.0; 
    return -0.5 * absErr;             
}

int chooseAction(int state) {
    if (random(0, 100) < epsilon * 100) {
        return random(0, NUM_ACTIONS); 
    } else {
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

void executeAction(int action) {
    switch (action) {
        case 0: break; 
        case 1: Kp += 0.5; break;
        case 2: Kp -= 0.5; break;
        case 3: Kd += 0.1; break;
        case 4: Kd -= 0.1; break;
    }
    Kp = constrain(Kp, 10.0, 60.0);
    Kd = constrain(Kd, 0.0, 10.0);
    pid.SetTunings(Kp, Ki, Kd); 
}

// ================================================================
// 6. HÀM XỬ LÝ NGẮT
// ================================================================
volatile bool mpuInterrupt = false; 
void IRAM_ATTR dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// TASK 1: PID & MOTOR CONTROL (CORE 1)
// ================================================================
void TaskPID(void *pvParameters) {
    // Biến tạm để lấy dữ liệu thô (Raw Data)
    int16_t ax, ay, az, gx, gy, gz; 

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
            // Serial.println(F("FIFO Overflow!")); 
        } 
        else if (mpuIntStatus & 0x02) {
            while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

            mpu.getFIFOBytes(fifoBuffer, packetSize);
            fifoCount -= packetSize;

            // 1. Tính góc nghiêng (Input cho PID)
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            input = ypr[1] * 180 / M_PI + 180;

            // 2. Lấy dữ liệu thô (Input cho RL - s_gy, s_az)
            mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

            // 3. Cập nhật struct chia sẻ
            sharedData.s_ang = input - setpoint;
            sharedData.s_gy  = (float)gy / 131.0; // Đổi sang độ/giây
            sharedData.s_az  = (float)az;
            sharedData.a_pwm = output;
            sharedData.done = (abs(input - setpoint) > 35.0);

            // 4. Điều khiển Motor
            if (sharedData.done) {
                motorController.stopMoving(); 
            } else {
                pid.Compute();
                motorController.move(output, MIN_ABS_SPEED);
            }
        }
    }
}

// ================================================================
// TASK 2: RL BRAIN & FIREBASE (CORE 0)
// ================================================================
void TaskRL(void *pvParameters) {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { vTaskDelay(500); Serial.print("."); }
    Serial.println("\nWiFi Connected!");

    config.api_key = API_KEY;
    config.database_url = DATABASE_URL;
    if (Firebase.signUp(&config, &auth, "", "")) { signupOK = true; }
    Firebase.begin(&config, &auth);
    
    // Đã sửa lỗi chính tả 'W' -> 'w'
    Firebase.RTDB.setwriteSizeLimit(&fbdo, "tiny"); 

    for (;;) {
        unsigned long now = millis();

        // --- A. RL Logic ---
        if (now - lastRLTime > 100) {
            currentState = getRLState(sharedData.s_ang);
            currentReward = calculateReward(sharedData.s_ang);
            sharedData.r = currentReward;
            
            float maxQ_next = -9999;
            for (int i = 0; i < NUM_ACTIONS; i++) {
                if (Q_Table[currentState][i] > maxQ_next) maxQ_next = Q_Table[currentState][i];
            }
            Q_Table[lastState][currentAction] += alpha * (currentReward + gamma_rl * maxQ_next - Q_Table[lastState][currentAction]);
            
            currentAction = chooseAction(currentState);
            executeAction(currentAction);
            
            lastState = currentState;
            lastRLTime = now;
        }

        // --- B. Firebase Logic (Đầy đủ thông số) ---
        static unsigned long lastFBTime = 0;
        if (now - lastFBTime > 200 && Firebase.ready() && signupOK) {
            FirebaseJson json;
            // 5 thông số bạn cần + done
            json.set("s_ang", sharedData.s_ang);
            json.set("s_gy",  sharedData.s_gy);  // Đã thêm lại
            json.set("s_az",  sharedData.s_az);  // Đã thêm lại
            json.set("a_pwm", sharedData.a_pwm);
            json.set("reward", sharedData.r);
            json.set("done", sharedData.done);
            
            // Tùy chọn: Gửi thêm Kp, Kd để monitor (nếu không cần thì xóa)
            json.set("Kp", Kp); 
            json.set("Kd", Kd);
            
            if (Firebase.RTDB.pushJSON(&fbdo, "/RL_Training_Data", &json)) {
                // Success
            }
            lastFBTime = now;
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// ================================================================
// SETUP
// ================================================================
void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22, 400000);

    // MPU Init
    Serial.println(F("Initializing MPU..."));
    mpu.initialize();
    pinMode(MPU_INT_PIN, INPUT); 
    devStatus = mpu.dmpInitialize();

    // Calibration
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

        for(int i=0; i<NUM_STATES; i++)
            for(int j=0; j<NUM_ACTIONS; j++)
                Q_Table[i][j] = 0.0;

        Serial.println(F("System Ready!"));
    } else {
        Serial.print(F("DMP Init Failed: "));
        Serial.println(devStatus);
    }

    xTaskCreatePinnedToCore(TaskPID, "PID_Task", 8192, NULL, 2, NULL, 1); 
    xTaskCreatePinnedToCore(TaskRL,  "RL_Task",  8192, NULL, 1, NULL, 0); 
}

void loop() {
    vTaskDelete(NULL); 
}
