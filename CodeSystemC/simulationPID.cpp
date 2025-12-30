#include <systemc.h>
#include <iostream>
#include <cmath>
#include <iomanip>

// ==========================================
<<<<<<< HEAD
// 1. MODULE: ROBOT PLANT (CÓ NHIỄU & NGOẠI LỰC)
// ==========================================
SC_MODULE(Robot_Plant) {
    sc_in<double> pwm_input;      // Lực từ động cơ
    sc_in<double> dist_input;     // Lực đẩy từ bên ngoài (Cú đá/Gió)
=======
// 1. MODULE: ROBOT PLANT
// ==========================================
SC_MODULE(Robot_Plant) {
    sc_in<double> pwm_input;      // Lực từ động cơ
    sc_in<double> dist_input;     // Lực đẩy từ bên ngoài 
>>>>>>> b974c8d5686fb29fbbb400af23a843378e66833f
    sc_out<double> angle_output;  // Góc nghiêng

    double theta;       
    double theta_dot;   
    
    // Hằng số vật lý
    const double dt = 0.01;      
    const double gravity = 9.81;
    const double length = 0.5;   

    void physics_process() {
        while (true) {
            double u = pwm_input.read();
            double d = dist_input.read(); // Đọc lực đẩy bên ngoài

            // Phương trình động lực học có thêm thành phần 'd'
            // alpha = Torque_Gravity + Torque_Motor + Torque_Disturbance
            double alpha = (gravity / length) * sin(theta) + u + d;

            // Tích phân Euler
            theta_dot += alpha * dt;
            
<<<<<<< HEAD
            // Giả lập ma sát (Damping) để thực tế hơn (mất 1% năng lượng mỗi tick)
=======
            // Giả lập ma sát
>>>>>>> b974c8d5686fb29fbbb400af23a843378e66833f
            theta_dot *= 0.99; 

            theta += theta_dot * dt;

            angle_output.write(theta);
            wait(10, SC_MS);
        }
    }

    SC_CTOR(Robot_Plant) {
        theta = 0.1; // Khởi tạo nghiêng nhẹ
        theta_dot = 0.0;
        SC_THREAD(physics_process);
    }
};

// ==========================================
// 2. MODULE: PID CONTROLLER (CÓ SETPOINT)
// ==========================================
SC_MODULE(PID_Controller) {
    sc_in<double> angle_input;
    sc_in<double> setpoint_input; // Nhận lệnh điểm đặt (0=đứng, !=0 là đi)
    sc_out<double> pwm_output;

<<<<<<< HEAD
    // Tham số PID (Đã tinh chỉnh cho kịch bản này)
=======
    // Tham số PID 
>>>>>>> b974c8d5686fb29fbbb400af23a843378e66833f
    double Kp = 45.0; 
    double Ki = 2.0;
    double Kd = 2.5;

    double prev_error;
    double integral;
    const double dt = 0.01;

    void control_loop() {
        while (true) {
            double current_angle = angle_input.read();
            double target = setpoint_input.read();

            // Tính sai số dựa trên Target thay vì cứng nhắc là 0
            double error = target - current_angle;

            // Tích phân (Có Anti-windup nhẹ)
            integral += error * dt;
            if (integral > 5.0) integral = 5.0;
            if (integral < -5.0) integral = -5.0;

            // Vi phân
            double derivative = (error - prev_error) / dt;
            
            // PID Output
            double output = (Kp * error) + (Ki * integral) + (Kd * derivative);

            // Bão hòa động cơ
            if (output > 25.0) output = 25.0;
            if (output < -25.0) output = -25.0;

            pwm_output.write(output);
            prev_error = error;

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
// 3. MODULE: SCENARIO DRIVER (ĐẠO DIỄN KỊCH BẢN)
// ==========================================
// Module này chịu trách nhiệm tạo ra các tình huống khó cho Robot
SC_MODULE(Scenario_Driver) {
    sc_out<double> setpoint_out; // Điều khiển robot đi đâu
    sc_out<double> dist_out;     // Tạo lực đẩy (phá đám)

    void run_scenario() {
        // --- GIAI ĐOẠN 1: ỔN ĐỊNH (0s - 2s) ---
        std::cout << "\n[0.0s] >>> KHOI DONG: Robot tu can bang..." << std::endl;
        setpoint_out.write(0.0);
        dist_out.write(0.0);
        wait(2, SC_SEC);

        // --- GIAI ĐOẠN 2: BỊ ĐẨY MẠNH (2s - 2.2s) ---
        std::cout << "\n[2.0s] >>> SỰ CỐ: CO NGUOI DAY ROBOT (DISTURBANCE) <<<" << std::endl;
        dist_out.write(5.0); // Tác động lực lớn làm xe ngã
        wait(0.2, SC_SEC);
        dist_out.write(0.0); // Thả tay ra
        std::cout << "[2.2s] >>> DA THA TAY: Robot dang tu khoi phuc..." << std::endl;
        wait(2.8, SC_SEC); // Chờ xem nó đứng dậy thế nào

        // --- GIAI ĐOẠN 3: DI CHUYỂN (5s - 7s) ---
        std::cout << "\n[5.0s] >>> LENH: DI CHUYEN TOI (Forward) <<<" << std::endl;
        setpoint_out.write(0.15); // Nghiêng người 0.15 rad để đi tới
        wait(2, SC_SEC);

        // --- GIAI ĐOẠN 4: DỪNG LẠI (7s - 9s) ---
        std::cout << "\n[7.0s] >>> LENH: DUNG LAI (Stop) <<<" << std::endl;
        setpoint_out.write(0.0);
        wait(2, SC_SEC);

        std::cout << "\n[9.0s] >>> KET THUC KICH BAN." << std::endl;
        sc_stop(); // Dừng mô phỏng
    }

    SC_CTOR(Scenario_Driver) {
        SC_THREAD(run_scenario);
    }
};

// ==========================================
// 4. MODULE: MONITOR (MÀN HÌNH THEO DÕI)
// ==========================================
SC_MODULE(Monitor) {
    sc_in<double> angle;
    sc_in<double> pwm;
    sc_in<double> setpoint;

    void display() {
        std::cout << " Time(s) |  Setpoint |   Angle   |    PWM    | Status" << std::endl;
        std::cout << "---------|-----------|-----------|-----------|--------" << std::endl;
        
        while(true) {
            double ang = angle.read();
            double sp = setpoint.read();
            double err = std::abs(sp - ang);
            
            std::string status = "OK";
            if (err > 0.1) status = "UNSTABLE!";
            if (err < 0.01) status = "STABLE";

            std::cout << std::fixed << std::setprecision(2)
                      << std::setw(7) << sc_time_stamp().to_seconds() << " | "
                      << std::setw(9) << sp << " | "
                      << std::setw(9) << ang << " | "
                      << std::setw(9) << pwm.read() << " | "
                      << status << std::endl;

            wait(0.25, SC_SEC); // Cập nhật màn hình 4 lần/giây (cho đỡ loạn)
        }
    }

    SC_CTOR(Monitor) {
        SC_THREAD(display);
    }
};

// ==========================================
// 5. MAIN
// ==========================================
int sc_main(int argc, char* argv[]) {
    // Tín hiệu kết nối
    sc_signal<double> sig_angle;
    sc_signal<double> sig_pwm;
    sc_signal<double> sig_setpoint;
    sc_signal<double> sig_dist;

    // Khởi tạo Modules
    Robot_Plant     robot("Robot");
    PID_Controller  pid("Controller");
    Scenario_Driver director("Director");
    Monitor         screen("Screen");

    // Kết nối Robot
    robot.pwm_input(sig_pwm);
    robot.dist_input(sig_dist);
    robot.angle_output(sig_angle);

    // Kết nối PID
    pid.angle_input(sig_angle);
    pid.setpoint_input(sig_setpoint);
    pid.pwm_output(sig_pwm);

    // Kết nối Scenario Driver
    director.setpoint_out(sig_setpoint);
    director.dist_out(sig_dist);

    // Kết nối Monitor
    screen.angle(sig_angle);
    screen.pwm(sig_pwm);
    screen.setpoint(sig_setpoint);

<<<<<<< HEAD
    // Trace file (Vẽ đồ thị)
=======
    // Trace file 
>>>>>>> b974c8d5686fb29fbbb400af23a843378e66833f
    sc_trace_file *wf = sc_create_vcd_trace_file("waveforms_diverse");
    sc_trace(wf, sig_angle, "Angle");
    sc_trace(wf, sig_pwm, "PWM");
    sc_trace(wf, sig_setpoint, "Setpoint");
    sc_trace(wf, sig_dist, "Disturbance");

<<<<<<< HEAD
    // Chạy mô phỏng (Scenario Driver sẽ quyết định khi nào dừng)
=======
   
>>>>>>> b974c8d5686fb29fbbb400af23a843378e66833f
    sc_start();

    sc_close_vcd_trace_file(wf);
    return 0;
<<<<<<< HEAD
}
=======
}
>>>>>>> b974c8d5686fb29fbbb400af23a843378e66833f
