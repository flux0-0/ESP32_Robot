#include <Arduino.h>
#include <RPLidar.h>
#include <micro_ros_platformio.h>
#include <WiFi.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <geometry_msgs/msg/twist.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <math.h>
#include <motor.h>

#include <std_msgs/msg/int32.h>
#include <nav_msgs/msg/odometry.h>
#include <tuple>
#include <rosidl_runtime_c/string_functions.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}

unsigned long long time_offset = 0;

// Hàm đồng bộ thời gian
bool syncTime() {
    const int timeout_ms = 2000;
    if (rmw_uros_epoch_synchronized()) return true; // Nếu đã đồng bộ trước đó

    RCCHECK(rmw_uros_sync_session(timeout_ms)); // Đồng bộ hóa với ROS2
    if (rmw_uros_epoch_synchronized()) {
        // Tính toán sự chênh lệch thời gian giữa ROS2 và microcontroller
        unsigned long long ros_time_ms = rmw_uros_epoch_millis();
        time_offset = ros_time_ms - millis();
        return true;
    }
    return false;
}

// Hàm lấy thời gian hiện tại sau khi đồng bộ
struct timespec getTime() {
    struct timespec tp = {0};
    unsigned long long now = millis() + time_offset; // Thời gian hiện tại điều chỉnh với time_offset
    tp.tv_sec = now / 1000;  // Chuyển  sang seconds
    tp.tv_nsec = (now % 1000) * 1000000;  // Chuyển sang nanoseconds
    return tp;
}


// Khai báo các biến toàn cục
RPLidar lidar;
HardwareSerial SerialLidar(2);
const int motorPin = 14;

rcl_publisher_t laserscan_publisher;
sensor_msgs__msg__LaserScan laserscan_msg;
rcl_publisher_t odom_publisher;

rcl_publisher_t encoder_publisher;
std_msgs__msg__Int32MultiArray encoder_data;
nav_msgs__msg__Odometry odom_msg;

rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;


unsigned long last_update_time = 0;  // Lưu thời gian của lần cập nhật trước
unsigned long update_interval = 700;  // Khoảng thời gian (ms) để kiểm tra (ví dụ: 1000ms = 1 giây)

void cmd_vel_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

    int linear_speed = msg->linear.x * 255;
    int angular_speed = msg->angular.z * 255;

    unsigned long current_time = millis();  // Lấy thời gian hiện tại

    // Kiểm tra xem thời gian đã trôi qua đủ lâu chưa
    if (current_time - last_update_time >= update_interval) {
        // Tiến hành hành động sau khi đủ thời gian
        if (linear_speed > 0) {
            forward(linear_speed);
        } else if (linear_speed < 0) {
            reverse(abs(linear_speed));
        } else if (angular_speed > 0) {
            left(angular_speed);
        } else if (angular_speed < 0) {
            right(abs(angular_speed));
        } else {
            stop();
        }

        // Cập nhật thời gian lần điều khiển
        last_update_time = current_time;
    }
}



// Hàm khởi tạo cấu hình cho LaserScan
void setup_laserscan() {
    laserscan_msg.angle_min = 0.0;
    laserscan_msg.angle_max = 2 * M_PI;
    laserscan_msg.angle_increment = 0.75 * (M_PI / 180.0);  // Tăng góc tăng thêm để giảm số lượng điểm
    laserscan_msg.range_min = 0.15;
    laserscan_msg.range_max = 12.0;

    int num_readings = (laserscan_msg.angle_max - laserscan_msg.angle_min) / laserscan_msg.angle_increment;

    laserscan_msg.ranges.data = (float*)malloc(num_readings * sizeof(float));
    laserscan_msg.ranges.size = num_readings;
    laserscan_msg.ranges.capacity = num_readings;

    laserscan_msg.header.frame_id.data = (char*)"base_link";
    laserscan_msg.header.frame_id.size = strlen("base_link");
    laserscan_msg.header.frame_id.capacity = laserscan_msg.header.frame_id.size + 1;
}

// Hàm khởi tạo cấu hình cho dữ liệu encoder
void setup_encoder_data() {
    encoder_data.data.size = 2;
    encoder_data.data.capacity = 2;
    encoder_data.data.data = (int32_t*)malloc(encoder_data.data.capacity * sizeof(int32_t));
}

void setup() {
    // Cài đặt phần cứng
    motor_setup();
    Serial.begin(115200);
    SerialLidar.begin(115200, SERIAL_8N1, 16, 17);
    pinMode(motorPin, OUTPUT);
    digitalWrite(motorPin, HIGH);

    IPAddress agent_ip(172, 20, 10, 3);
    size_t agent_port = 8888;
    char ssid[] = "Flux";
    char psk[] = "phungbao02";

    // Kết nối WiFi và thiết lập micro-ROS
    WiFi.begin(ssid, psk);
    set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);

    // Khởi tạo RPLidar
    lidar.begin(SerialLidar);
    if (!lidar.isOpen()) {
        Serial.println("Không thể kết nối với RPLidar!");
        while (1);
    }
    Serial.println("Kết nối với RPLidar thành công!");
    lidar.startScan();

    // Cấu hình ROS 2
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "esp32_lidar_node", "", &support);
    rclc_publisher_init_default(
        &laserscan_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
        "/scan"
    );
    rclc_publisher_init_default(
        &encoder_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "encoder_data"
    );
    rclc_subscription_init_default(
        &cmd_vel_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    );

    rclc_publisher_init_default(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/odom"
    );

    setup_laserscan();
    setup_encoder_data();

    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);

    // Đồng bộ thời gian với ROS2
    syncTime();
}


void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

    // Lấy thời gian đồng bộ từ ROS2
    struct timespec time_stamp = getTime();

    // Gửi dữ liệu encoder
    auto encoder_values = read_encoder();
    encoder_data.data.data[0] = std::get<0>(encoder_values);
    encoder_data.data.data[1] = std::get<1>(encoder_values);
    rcl_publish(&encoder_publisher, &encoder_data, NULL);
    delay(35);

    // Cập nhật dữ liệu LaserScan
    int num_readings = laserscan_msg.ranges.size;
    for (int i = 0; i < num_readings; i++) {
        laserscan_msg.ranges.data[i] = laserscan_msg.range_max;
    }


    for (int i = 0; i < num_readings; i++) {
        if (lidar.waitPoint(240) == RESULT_OK) {
            RPLidarMeasurement currentMeasurement = lidar.getCurrentPoint();
            float angle = currentMeasurement.angle * M_PI / 180;
            float distance = currentMeasurement.distance / 1000;

            if (distance >= laserscan_msg.range_min && distance <= laserscan_msg.range_max) {
                int index = (angle - laserscan_msg.angle_min) / laserscan_msg.angle_increment;
                if (index >= 0 && index < num_readings) {
                    laserscan_msg.ranges.data[num_readings - 1 - index] = distance;  // Đảo ngược mảng `ranges`
                }
            }
        }
    }

    // Gán thời gian cho laserscan
    laserscan_msg.header.stamp.sec = time_stamp.tv_sec;
    laserscan_msg.header.stamp.nanosec = time_stamp.tv_nsec;
    rcl_publish(&laserscan_publisher, &laserscan_msg, NULL);

    delay(70);  // Tăng độ trễ để giảm tải CPU
}