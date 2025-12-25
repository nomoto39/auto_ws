#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <thread>

// グローバル変数
int fd1 = -1;

// Arduinoからのシリアルデータを読み取るスレッド関数（変更なし）
void serial_read_thread_func(int serial_fd, rclcpp::Logger logger) {
    char buffer[256];
    RCLCPP_INFO(logger, "Serial read thread started for fd: %d", serial_fd);
    while (rclcpp::ok()) {
        if (serial_fd < 0) {
            RCLCPP_ERROR(logger, "Serial port fd is invalid in read thread.");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            continue;
        }
        ssize_t bytes_read = read(serial_fd, buffer, sizeof(buffer) - 1);
        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            RCLCPP_INFO(logger, "Arduino: %s", buffer);
        } else if (bytes_read < 0 && errno != EAGAIN) {
            RCLCPP_ERROR(logger, "Serial read error: %s (errno: %d)", strerror(errno), errno);
            break;
        }
    }
    RCLCPP_INFO(logger, "Serial read thread stopping.");
}

// ★★★ 手動操縦専用に簡略化したクラス ★★★
class ManualSerialNode : public rclcpp::Node {
public:
    ManualSerialNode() : Node("manual_serial_sender") {
        // 購読するトピックは手動操縦用の /cmd_vel のみ
        sub_teleop_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 
            10,
            std::bind(&ManualSerialNode::teleop_cmd_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "ManualSerialNode started. Waiting for /cmd_vel commands.");
    }

private:
    // /cmd_vel を受信したら、チェックなしで即座にシリアル送信する
    void teleop_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        send_serial_command(msg->linear.x, msg->angular.z);
    }

    // シリアル送信関数
    void send_serial_command(double linear_x, double angular_z) {
        if (fd1 < 0) {
            RCLCPP_ERROR_ONCE(this->get_logger(), "Serial port (fd1) not open. Cannot send command.");
            return;
        }
        char buf[64];
        // Arduinoに送る識別子は 't' (teleop) に固定
        int bytes_written = snprintf(buf, sizeof(buf), "t,%.3f,%.3f\n", linear_x, angular_z);
        
        if (bytes_written > 0) {
            ssize_t rec = write(fd1, buf, bytes_written);
            if (rec < 0) {
                RCLCPP_ERROR(this->get_logger(), "Serial write failed: %s", strerror(errno));
            }
        }
    }

    // メンバー変数
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_teleop_;
};

// シリアルポートを開き、正しく設定する関数（変更なし）
int open_serial(const char *device_name) {
    int fd = open(device_name, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("open_serial"), "Serial Fail: could not open %s", device_name);
        return -1;
    }
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1; // 0.1秒のタイムアウト
    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

// main関数
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto main_logger = rclcpp::get_logger("main_runner");

    char device_name[] = "/dev/ttyACM0";
    fd1 = open_serial(device_name); 

    if (fd1 < 0) {
        RCLCPP_ERROR(main_logger, "Serial port open failed. Shutting down.");
        return -1;
    }

    std::thread reader_thread(serial_read_thread_func, fd1, main_logger);
    
    // ★★★ 簡略化した ManualSerialNode を起動 ★★★
    auto manual_serial_node = std::make_shared<ManualSerialNode>();
    
    RCLCPP_INFO(main_logger, "Node spinning. Press Ctrl+C to exit.");
    rclcpp::spin(manual_serial_node);

    if (reader_thread.joinable()) {
        reader_thread.join();
    }
    if (fd1 >= 0) {
        close(fd1);
    }
    rclcpp::shutdown();
    return 0; 
}