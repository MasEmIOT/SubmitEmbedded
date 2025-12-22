class RobotData {
  final double angle;  // s_ang
  final double gyro;   // s_gy
  final double pwm;    // a_pwm
  final double accelZ; // s_az
  final double reward; // reward
  final DateTime timestamp;

  RobotData({
    required this.angle,
    required this.gyro,
    required this.pwm,
    required this.accelZ,
    required this.reward,
    required this.timestamp,
  });

  // Hàm chuyển từ dữ liệu Firebase (Map) sang đối tượng Dart
  factory RobotData.fromMap(Map<dynamic, dynamic> map) {
    return RobotData(
      // Dùng 'as num? ?? 0' để tránh lỗi nếu dữ liệu null hoặc là int
      angle: (map['s_ang'] as num? ?? 0).toDouble(),
      gyro: (map['s_gy'] as num? ?? 0).toDouble(),
      pwm: (map['a_pwm'] as num? ?? 0).toDouble(),
      accelZ: (map['s_az'] as num? ?? 0).toDouble(),
      reward: (map['reward'] as num? ?? 0).toDouble(),
      timestamp: DateTime.now(),
    );
  }
}