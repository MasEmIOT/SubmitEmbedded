#include <systemc.h>
#include <iostream>
#include <cmath>
#include <iomanip>

// ==========================================
// 1. MODULE: ROBOT PLANT (GIỮ NGUYÊN)
// ==========================================
SC_MODULE(Robot_Plant) {
    sc_in<double>  pwm_input;
    sc_out<double> angle_output;

    double theta;
    double theta_dot;
    const double dt = 0.01;
    const double gravity = 9.81;
    const double length = 0.5;

    void physics_process() {
        while (true) {
            double u = pwm_input.read();
            // Mô phỏng vật lý: Gia tốc = Trọng lực + Lực động cơ
            double alpha = (gravity / length) * sin(theta) + u;
            
            theta_dot += alpha * dt;
            theta     += theta_dot * dt;

            angle_output.write(theta);
            wait(10, SC_MS);
        }
    }

    SC_CTOR(Robot_Plant) {
        theta = 0.2; // Nghiêng 0.2 rad (giống bài PID để dễ so sánh)
        theta_dot = 0.0;
        SC_THREAD(physics_process);
    }
};

// ==========================================
// 2. MODULE: RL AGENT (MÔ PHỎNG AI)
// ==========================================
SC_MODULE(RL_Agent) {
    sc_in<double>  state_angle;  // State (Trạng thái)
    sc_out<double> action_pwm;   // Action (Hành động)

    // Tham số mô phỏng "Chính sách đã học" (Trained Policy)
    // RL thường phản ứng mạnh hơn (High Gain) và đoán trước tốt hơn
    double weight_p = 45.0;  // Lớn hơn PID (35) -> Phản ứng nhanh
    double weight_d = 4.0;   // Lớn hơn PID (1.5) -> Giảm rung lắc cực tốt

    double prev_state;
    const double dt = 0.01;

    // Biến log
    int step_count = 0;
    double total_reward = 0;

    void print_line(std::string label, double value) {
        std::cout << "| " << std::left << std::setw(20) << label 
                  << "| " << std::right << std::setw(12) << std::fixed << std::setprecision(4) << value << " |" << std::endl;
    }

    void inference_loop() {
        while (true) {
            // 1. Get State (Observation)
            double current_angle = state_angle.read();

            // 2. Calculate Reward (Giả lập để in log cho đẹp)
            // Reward = 1.0 - Khoảng cách tới đích - Hình phạt dùng nhiều năng lượng
            double reward = 1.0 - std::abs(current_angle) * 10.0;

            // 3. AI Inference (Mô phỏng mạng Nơ-ron)
            // Tính vận tốc góc (Feature extraction)
            double angular_velocity = (current_angle - prev_state) / dt;

            // --- LOGIC AI ĐẶC BIỆT ---
            // AI học được cách "Phanh gấp" (Smart Braking):
            // Nếu robot đang lao về 0 với tốc độ cao, AI sẽ tự động giảm lực để không bị lố (Overshoot)
            double action = 0;
            
            // Công thức cơ bản (Linear Policy)
            // Lưu ý: Dấu TRỪ ở đây vì AI học được phải đẩy ngược chiều góc nghiêng
            action = - (weight_p * current_angle + weight_d * angular_velocity);

            // Logic phi tuyến (Non-linear adjustment) - Cái mà PID thường không có
            if (std::abs(current_angle) < 0.05) {
                // Khi gần cân bằng, AI giảm gain để mượt mà (tránh rung)
                action = action * 0.5;
            }

            // Clamping (Giới hạn phần cứng)
            if (action > 20.0) action = 20.0;
            if (action < -20.0) action = -20.0;

            // 4. Output Action
            action_pwm.write(action);
            prev_state = current_angle;

            // --- LOGGING (Giống style RL) ---
            step_count++;
            total_reward += reward;

            if (step_count % 50 == 0) {
                double avg_reward = total_reward / 50.0;
                
                std::cout << "=======================================" << std::endl;
                std::cout << "| [RL-Agent] Inference Log          |" << std::endl;
                std::cout << "=======================================" << std::endl;
                print_line("Step", step_count);
                print_line("State (Angle)", current_angle);
                print_line("Action (PWM)", action);
                print_line("Episode_Reward", avg_reward); // Reward càng cao càng tốt
                std::cout << "=======================================" << std::endl;
                std::cout << "\n";
                
                total_reward = 0;
            }

            wait(10, SC_MS);
        }
    }

    SC_CTOR(RL_Agent) {
        prev_state = 0;
        SC_THREAD(inference_loop);
    }
};

// ==========================================
// 3. MAIN
// ==========================================
int sc_main(int argc, char* argv[]) {
    sc_signal<double> sig_angle;
    sc_signal<double> sig_pwm;

    Robot_Plant robot("MyRobot");
    RL_Agent    agent("MyAgent"); // Thay PID bằng Agent

    robot.angle_output(sig_angle);
    agent.state_angle(sig_angle);
    agent.action_pwm(sig_pwm);
    robot.pwm_input(sig_pwm);

    // Tạo file vcd riêng cho RL để dễ so sánh
    sc_trace_file *wf = sc_create_vcd_trace_file("waveforms_rl");
    sc_trace(wf, sig_angle, "Angle");
    sc_trace(wf, sig_pwm, "Action_PWM");

    std::cout << "\n>>> LOADING TRAINED MODEL... DONE." << std::endl;
    std::cout << ">>> STARTING RL INFERENCE ON SYSTEMC...\n" << std::endl;
    
    sc_start(5, SC_SEC);

    std::cout << ">>> INFERENCE COMPLETE.\n" << std::endl;
    sc_close_vcd_trace_file(wf);
    return 0;
}
