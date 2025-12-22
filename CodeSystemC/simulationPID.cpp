#include <systemc.h>
#include <iostream>
#include <cmath>
#include <iomanip> // Thư viện căn chỉnh bảng in

// ==========================================
// 1. MODULE: ROBOT PLANT (MÔ PHỎNG VẬT LÝ)
// ==========================================
SC_MODULE(Robot_Plant) {
    sc_in<double>  pwm_input;      // Lực đẩy từ động cơ
    sc_out<double> angle_output;   // Góc nghiêng thực tế

    double theta;       // Góc hiện tại (rad)
    double theta_dot;   // Vận tốc góc
    
    // Hằng số vật lý
    const double dt = 0.01;      // 10ms
    const double gravity = 9.81;
    const double length = 0.5;   // Chiều dài con lắc

    void physics_process() {
        while (true) {
            double u = pwm_input.read();

            // Phương trình động lực học: Gia tốc = Trọng lực + Lực động cơ
            // Nếu u cùng dấu với theta -> Robot ngã nhanh hơn (Phản hồi dương - Sai)
            // Nếu u ngược dấu với theta -> Robot được dựng lại (Phản hồi âm - Đúng)
            double alpha = (gravity / length) * sin(theta) + u;

            // Tích phân Euler
            theta_dot += alpha * dt;
            theta     += theta_dot * dt;

            angle_output.write(theta);
            wait(10, SC_MS);
        }
    }

    SC_CTOR(Robot_Plant) {
        theta = 0.2; // Khởi tạo robot bị nghiêng 0.2 rad (~11 độ)
        theta_dot = 0.0;
        SC_THREAD(physics_process);
    }
};

// ==========================================
// 2. MODULE: PID CONTROLLER (ĐÃ SỬA DẤU)
// ==========================================
SC_MODULE(PID_Controller) {
    sc_in<double>  angle_input;
    sc_out<double> pwm_output;

    // --- SỬA LẠI THAM SỐ (DƯƠNG) ---
    double Kp = 35.0; 
    double Ki = 2.5;
    double Kd = 1.5;

    double prev_error;
    double integral;
    const double dt = 0.01;

    // Biến hỗ trợ in Log đẹp
    int step_count = 0;
    double total_error_accum = 0;

    void print_line(std::string label, double value) {
        std::cout << "| " << std::left << std::setw(20) << label 
                  << "| " << std::right << std::setw(12) << std::fixed << std::setprecision(4) << value << " |" << std::endl;
    }

    void control_loop() {
        while (true) {
            double current_angle = angle_input.read();

            // Tính sai số: Muốn về 0, nên Error = 0 - Hiện tại
            // Ví dụ: Góc đang là 0.2 (Dương) -> Error = -0.2 (Âm)
            double error = 0.0 - current_angle;

            integral += error * dt;
            double derivative = (error - prev_error) / dt;
            
            // Công thức PID: Output = Kp*E + Ki*I + Kd*D
            // Vì Error là Âm, Kp là Dương -> Output sẽ là Âm (Đẩy ngược lại) -> ĐÚNG
            double output = (Kp * error) + (Ki * integral) + (Kd * derivative);

            // Giới hạn bão hòa (Saturation)
            if (output > 20.0) output = 20.0;
            if (output < -20.0) output = -20.0;

            pwm_output.write(output);
            prev_error = error;

            // --- PHẦN IN LOG RA TERMINAL ---
            step_count++;
            total_error_accum += std::abs(error); // Cộng dồn trị tuyệt đối sai số

            // In log mỗi 0.5 giây (50 bước)
            if (step_count % 50 == 0) {
                double avg_loss = total_error_accum / 50.0;
                
                std::cout << "---------------------------------------" << std::endl;
                std::cout << "| [PID] Simulation Log              |" << std::endl;
                std::cout << "---------------------------------------" << std::endl;
                print_line("time_step", step_count);
                print_line("current_angle (rad)", current_angle);
                print_line("pwm_output", output);
                print_line("error_loss", avg_loss);
                print_line("integral_term", integral);
                std::cout << "---------------------------------------" << std::endl;
                std::cout << "\n";

                total_error_accum = 0;
            }

            wait(10, SC_MS);
        }
    }

    SC_CTOR(PID_Controller) {
        prev_error = 0;
        integral = 0;
        SC_THREAD(control_loop);
    }
};

// ==========================================
// 3. MAIN
// ==========================================
int sc_main(int argc, char* argv[]) {
    sc_signal<double> sig_angle;
    sc_signal<double> sig_pwm;

    Robot_Plant    robot("MyRobot");
    PID_Controller controller("MyPID");

    robot.angle_output(sig_angle);
    controller.angle_input(sig_angle);
    controller.pwm_output(sig_pwm);
    robot.pwm_input(sig_pwm);

    sc_trace_file *wf = sc_create_vcd_trace_file("waveforms");
    sc_trace(wf, sig_angle, "Angle");
    sc_trace(wf, sig_pwm, "PWM_Control");

    std::cout << "\n=== BAT DAU MO PHONG SYSTEMC (FIXED PID) ===\n" << std::endl;
    
    sc_start(5, SC_SEC); // Chạy 5 giây

    std::cout << "=== KET THUC MO PHONG ===\n" << std::endl;
    sc_close_vcd_trace_file(wf);
    return 0;
}
