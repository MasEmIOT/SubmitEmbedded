import 'dart:async';
import 'dart:io';
import 'dart:math';
import 'package:flutter/material.dart';
import 'package:firebase_database/firebase_database.dart';
import 'package:fl_chart/fl_chart.dart';
import 'package:csv/csv.dart';
import 'package:path_provider/path_provider.dart'; // Tìm thư mục tạm
import 'package:share_plus/share_plus.dart';     // Chia sẻ file an toàn
import 'package:intl/intl.dart';                  // Format ngày giờ
import '../models/robot_data.dart';

class DashboardScreen extends StatefulWidget {
  const DashboardScreen({Key? key}) : super(key: key);

  @override
  State<DashboardScreen> createState() => _DashboardScreenState();
}

class _DashboardScreenState extends State<DashboardScreen> {
  final DatabaseReference _dbRef = FirebaseDatabase.instance.ref('RL_Training_Data');
  
  // --- DỮ LIỆU ---
  List<RobotData> _allData = []; 
  bool _isPaused = false;
  StreamSubscription? _subscription;
  final ScrollController _scrollController = ScrollController();

  // --- CẤU HÌNH HIỂN THỊ ---
  final Map<String, bool> _visibility = {
    'Angle': true, 'Gyro': false, 'PWM': false, 'AccelZ': false, 'Reward': false,
  };

  final Map<String, Color> _colors = {
    'Angle': Colors.orange, 'Gyro': Colors.purpleAccent, 'PWM': Colors.green, 'AccelZ': Colors.redAccent, 'Reward': Colors.blue,
  };

  @override
  void initState() {
    super.initState();
    // Không cần gọi _requestPermission() nữa
    _startListening();
  }

  void _startListening() {
    _subscription = _dbRef.limitToLast(1).onChildAdded.listen((event) {
      if (_isPaused) return;

      final value = event.snapshot.value;
      if (value != null && value is Map) {
        final newData = RobotData.fromMap(value);
        
        if (mounted) {
          setState(() {
            _allData.add(newData);
          });
          
          // Tự động cuộn sang phải
          WidgetsBinding.instance.addPostFrameCallback((_) {
            if (_scrollController.hasClients) {
              _scrollController.jumpTo(_scrollController.position.maxScrollExtent);
            }
          });
        }
      }
    });
  }

  // --- XỬ LÝ CSV (Dùng Share Plus - Không cần quyền) ---
  Future<void> _exportCSV() async {
    if (_allData.isEmpty) {
      ScaffoldMessenger.of(context).showSnackBar(const SnackBar(content: Text("Chưa có dữ liệu để xuất!")));
      return;
    }

    try {
      // 1. Tạo nội dung CSV
      List<List<dynamic>> rows = [];
      // Header: Cột đầu tiên là Time
      rows.add(["Time", "Angle", "Gyro", "PWM", "AccelZ", "Reward"]);
      
      // Data
      for (var d in _allData) {
        // Format thời gian đầy đủ cho file Excel/CSV
        String timeStr = DateFormat('yyyy-MM-dd HH:mm:ss.SSS').format(d.timestamp);
        rows.add([timeStr, d.angle, d.gyro, d.pwm, d.accelZ, d.reward]);
      }

      String csvData = const ListToCsvConverter().convert(rows);

      // 2. Lưu file vào thư mục tạm (Cache) của ứng dụng
      // Thư mục này app được quyền ghi thoải mái mà không cần xin phép OS
      final directory = await getTemporaryDirectory();
      final fileName = "RobotData_${DateFormat('yyyyMMdd_HHmmss').format(DateTime.now())}.csv";
      final path = "${directory.path}/$fileName";
      final file = File(path);
      await file.writeAsString(csvData);

      // 3. Mở popup Chia sẻ
      // Android sẽ tự hiện options: Save to Drive, Zalo, Gmail, hoặc Copy to...
      await Share.shareXFiles([XFile(path)], text: 'Dữ liệu Robot Export');
      
    } catch (e) {
      print("Lỗi export: $e");
      ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text("Lỗi: $e")));
    }
  }

  @override
  void dispose() {
    _subscription?.cancel();
    _scrollController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.black87,
      appBar: AppBar(
        title: const Text("Robot Monitor Pro"),
        backgroundColor: Colors.black54,
        actions: [
          IconButton(
            icon: const Icon(Icons.share), // Đổi icon thành Share cho đúng ngữ nghĩa
            onPressed: _exportCSV,
            tooltip: "Xuất CSV",
          ),
          IconButton(
            icon: Icon(
              _isPaused ? Icons.play_arrow : Icons.pause,
              color: _isPaused ? Colors.green : Colors.red,
            ),
            onPressed: () => setState(() => _isPaused = !_isPaused),
          )
        ],
      ),
      body: Column(
        children: [
          // --- PHẦN 1: BỘ LỌC (FILTER CHIPS) ---
          _buildFilterChips(),

          // --- PHẦN 2: BIỂU ĐỒ (Có hiển thị thời gian) ---
          Expanded(
            flex: 3,
            child: _buildChart(),
          ),
          
          const Divider(color: Colors.white24, height: 1),

          // --- PHẦN 3: BẢNG SỐ LIỆU ---
          Expanded(
            flex: 2,
            child: _allData.isEmpty
                ? const SizedBox()
                : _buildDataGrid(_allData.last),
          ),
        ],
      ),
    );
  }

  Widget _buildFilterChips() {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
      color: Colors.white10,
      child: Row(
        children: [
          TextButton(
            onPressed: () {
              setState(() {
                _visibility.updateAll((key, value) => true);
              });
            },
            child: const Text("All", style: TextStyle(color: Colors.blue)),
          ),
          Expanded(
            child: SingleChildScrollView(
              scrollDirection: Axis.horizontal,
              child: Row(
                children: _visibility.keys.map((key) {
                  return Padding(
                    padding: const EdgeInsets.symmetric(horizontal: 4),
                    child: FilterChip(
                      label: Text(key),
                      selected: _visibility[key]!,
                      onSelected: (bool value) => setState(() => _visibility[key] = value),
                      backgroundColor: Colors.black,
                      selectedColor: _colors[key]!.withOpacity(0.3),
                      checkmarkColor: _colors[key],
                      labelStyle: TextStyle(
                        color: _visibility[key]! ? _colors[key] : Colors.grey,
                      ),
                      side: BorderSide(
                        color: _visibility[key]! ? _colors[key]! : Colors.grey,
                      ),
                    ),
                  );
                }).toList(),
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildChart() {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 10),
      child: _allData.isEmpty
          ? const Center(child: Text("Đang chờ dữ liệu...", style: TextStyle(color: Colors.white54)))
          : SingleChildScrollView(
              controller: _scrollController,
              scrollDirection: Axis.horizontal,
              child: Container(
                // Mỗi điểm dữ liệu chiếm 40 pixel để đủ chỗ hiển thị giờ
                width: max(MediaQuery.of(context).size.width, _allData.length * 40.0),
                padding: const EdgeInsets.only(right: 30, bottom: 10),
                child: LineChart(
                  LineChartData(
                    lineTouchData: LineTouchData(
                      touchTooltipData: LineTouchTooltipData(
                        getTooltipItems: (touchedSpots) {
                          return touchedSpots.map((LineBarSpot touchedSpot) {
                            // Lấy timestamp từ index trục X
                            final date = _allData[touchedSpot.x.toInt()].timestamp;
                            final timeStr = DateFormat('HH:mm:ss').format(date);
                            
                            // Xác định tên thông số
                            String paramName = "Val";
                            _colors.forEach((key, color) {
                              if (touchedSpot.bar.color == color) paramName = key;
                            });

                            return LineTooltipItem(
                              '$timeStr\n$paramName: ${touchedSpot.y.toStringAsFixed(2)}',
                              TextStyle(color: touchedSpot.bar.color, fontWeight: FontWeight.bold),
                            );
                          }).toList();
                        },
                      ),
                    ),
                    gridData: FlGridData(
                      show: true, 
                      drawVerticalLine: true, 
                      getDrawingVerticalLine: (value) => FlLine(color: Colors.white10, strokeWidth: 1)
                    ),
                    titlesData: FlTitlesData(
                      topTitles: AxisTitles(sideTitles: SideTitles(showTitles: false)),
                      rightTitles: AxisTitles(sideTitles: SideTitles(showTitles: false)),
                      leftTitles: AxisTitles(sideTitles: SideTitles(showTitles: false)),
                      bottomTitles: AxisTitles(
                        sideTitles: SideTitles(
                          showTitles: true,
                          reservedSize: 30,
                          interval: 5, // Cách 5 điểm hiển thị 1 mốc thời gian
                          getTitlesWidget: (value, meta) {
                            final index = value.toInt();
                            if (index >= 0 && index < _allData.length) {
                              final date = _allData[index].timestamp;
                              return Padding(
                                padding: const EdgeInsets.only(top: 8.0),
                                child: Text(
                                  DateFormat('mm:ss').format(date), // Hiển thị Phút:Giây
                                  style: const TextStyle(color: Colors.white54, fontSize: 10),
                                ),
                              );
                            }
                            return const SizedBox();
                          },
                        ),
                      ),
                    ),
                    borderData: FlBorderData(show: true, border: Border.all(color: Colors.white12)),
                    lineBarsData: _generateLineBars(),
                  ),
                ),
              ),
            ),
    );
  }

  List<LineChartBarData> _generateLineBars() {
    List<LineChartBarData> lines = [];
    if (_visibility['Angle']!) lines.add(_makeLine((d) => d.angle, _colors['Angle']!));
    if (_visibility['Gyro']!) lines.add(_makeLine((d) => d.gyro, _colors['Gyro']!));
    if (_visibility['PWM']!) lines.add(_makeLine((d) => d.pwm, _colors['PWM']!));
    if (_visibility['AccelZ']!) lines.add(_makeLine((d) => d.accelZ, _colors['AccelZ']!));
    if (_visibility['Reward']!) lines.add(_makeLine((d) => d.reward, _colors['Reward']!));
    return lines;
  }

  LineChartBarData _makeLine(double Function(RobotData) getter, Color color) {
    return LineChartBarData(
      spots: _allData.asMap().entries.map((e) {
        return FlSpot(e.key.toDouble(), getter(e.value));
      }).toList(),
      isCurved: false,
      color: color,
      barWidth: 2,
      dotData: FlDotData(show: false),
    );
  }

  Widget _buildDataGrid(RobotData data) {
    return Column(
      children: [
        Padding(
          padding: const EdgeInsets.all(4.0),
          child: Text(
            "Cập nhật lúc: ${DateFormat('dd/MM/yyyy HH:mm:ss').format(data.timestamp)}",
            style: const TextStyle(color: Colors.white54, fontSize: 12),
          ),
        ),
        Expanded(
          child: GridView.count(
            crossAxisCount: 3,
            childAspectRatio: 1.6,
            crossAxisSpacing: 8,
            mainAxisSpacing: 8,
            padding: const EdgeInsets.all(8),
            children: [
              _buildInfoCard("Angle", data.angle.toStringAsFixed(2), _colors['Angle']!),
              _buildInfoCard("Gyro", data.gyro.toStringAsFixed(2), _colors['Gyro']!),
              _buildInfoCard("PWM", data.pwm.toStringAsFixed(1), _colors['PWM']!),
              _buildInfoCard("AccelZ", data.accelZ.toStringAsFixed(0), _colors['AccelZ']!),
              _buildInfoCard("Reward", data.reward.toStringAsFixed(3), _colors['Reward']!),
            ],
          ),
        ),
      ],
    );
  }

  Widget _buildInfoCard(String title, String value, Color color) {
    return Container(
      decoration: BoxDecoration(
        color: Colors.white10,
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: color.withOpacity(0.5)),
      ),
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Text(title, style: TextStyle(color: Colors.grey[400], fontSize: 10)),
          const SizedBox(height: 2),
          Text(value, style: TextStyle(color: color, fontSize: 18, fontWeight: FontWeight.bold)),
        ],
      ),
    );
  }
}