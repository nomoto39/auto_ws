#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <thread>        // スレッド処理用
#include <atomic>        // アトミック操作用 (スレッド間の安全なフラグ共有)


using std::placeholders::_1;

int fd1 = -1;
int loop_ct = 0;

/**
 * @brief Arduinoからのシリアルデータを読み取り、ログに出力するスレッド関数
 * @param serial_fd シリアルポートのファイルディスクリプタ
 * @param logger ROS 2ロガーインスタンス
 */
void serial_read_thread_func(int serial_fd, rclcpp::Logger logger) {
    char buffer[256]; // 受信バッファ (適宜サイズを調整)
    RCLCPP_INFO(logger, "Serial read thread started for fd: %d", serial_fd);

    // メインループが rclcpp::ok() で制御されるため、スレッドもそれに追従
    while (rclcpp::ok()) {
        if (serial_fd < 0) { // fd1が何らかの理由で無効になった場合
            RCLCPP_ERROR(logger, "Serial port fd is invalid in read thread.");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 少し待って再試行を期待 (エラー処理による)
            continue;
        }

        ssize_t bytes_read = read(serial_fd, buffer, sizeof(buffer) - 1);

        if (bytes_read > 0) {
            buffer[bytes_read] = '\0'; // 受信データをC文字列として終端
            // Arduinoからの出力には改行が含まれることが多いので、そのままログに出力
            // 必要であれば、ここでデータを整形したり、特定の書式をパースしたりすることも可能
            RCLCPP_INFO(logger, "Arduino: %s", buffer);
        } else if (bytes_read == 0) {
            // ポートが閉じたか、EOF (通常ttyACMではあまり発生しない)
            RCLCPP_WARN(logger, "Read 0 bytes from serial port. It might be closed.");
            // ループを抜けるか、再接続ロジックを試みるかなどの対応が必要な場合がある
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // CPUの過度な使用を避ける
        } else { // bytes_read < 0 (エラー)
            // O_NONBLOCKが設定されていない場合、readはデータが来るまでブロックする。
            // エラーが発生した場合 (例: ポートが物理的に切断された)
            if (errno == EINTR) { // シグナルによる中断
                continue;
            }
            RCLCPP_ERROR(logger, "Serial read error: %s (errno: %d)", strerror(errno), errno);
            // エラー発生時はスレッドを終了することを検討
            break; 
        }
        // ポーリング間隔 (任意、readがブロッキングなら不要な場合も)
        // std::this_thread::sleep_for(std::chrono::milliseconds(10)); 
    }
    RCLCPP_INFO(logger, "Serial read thread stopping.");
}

class MySubscriber : public rclcpp::Node
{
public:
    MySubscriber()
        : Node("my_subscriber")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&MySubscriber::topic_callback, this, _1));
    }

private:
    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
    {
        char buf[64]; // メッセージのバッファーサイズを適切に制限
        unsigned int bytes_written;

        RCLCPP_INFO(this->get_logger(), "I heard: '%lf'", msg->linear.x);

        // メッセージをバッファに書き込む
        if(loop_ct == 0){
            buf[0] = 's';
            buf[1] = '\n';
        }

        bytes_written = snprintf(buf, sizeof(buf), "%7.3f,%7.3f\n", msg->linear.x, msg->angular.z);
        //bytes_written = snprintf(buf, sizeof(buf), "%7.3f,%7.3f\n", msg->linear.x, msg->angulrar.z);
        //bytes_written = snprintf(buf, sizeof(buf), "%7.3f,%7.3f,%7.3f,%7.3f\n", msg->linear.x, msg->linear.z, msg->angular.x, msg->angular.y);


        if (bytes_written > sizeof(buf)) {
            RCLCPP_ERROR(this->get_logger(), "Serial Fail: message formatting error"); 
            return;
        }
        else
        {
            printf("cmd_vel recv:%s\n", buf);

            int rec = write(fd1, buf, bytes_written);

            if (rec >= 0) {
                printf("Serial send:%s\n", buf);
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Serial Fail: could not write");
            }
        }

        loop_ct++;

    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
}; 

int open_serial(const char *device_name)
{
    int fd = open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    fcntl(fd, F_SETFL, 0);

    if (fd < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Serial Fail: could not open %s", device_name);
        return -1;
    }

    // 以下略...
    return fd;
}

/*
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("Serialport");

    char device_name[] = "/dev/ttyACM0"; 
    fd1 = open_serial(device_name);

    if (fd1 < 0) {
        printf("Serial Fail\n");
        rclcpp::shutdown();
        return -1;
    }

    rclcpp::spin(std::make_shared<MySubscriber>());
    rclcpp::shutdown();

    // === ここでシリアルポートを閉じる ===
    if (fd1 >= 0) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Closing serial port /dev/ttyACM0");
        if (close(fd1) < 0) {
            // エラーログには errno から詳細なエラーメッセージを含めると良いでしょう
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to close serial port: %s", strerror(errno));
        } else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Serial port closed successfully.");
        }
        fd1 = -1; // 閉じたことを示すために無効な値に設定 (任意)
    } 

    return 0;
}
*/


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // シリアルポート操作やログ出力のためのノード (または既存のノードのロガーを利用)
    auto serial_manager_node = std::make_shared<rclcpp::Node>("serial_manager_node");
    rclcpp::Logger logger = serial_manager_node->get_logger();

    char device_name[] = "/dev/ttyACM0";
    fd1 = open_serial(device_name); // open_serial は fd1 を設定すると仮定

    if (fd1 < 0) {
        RCLCPP_ERROR(logger, "Serial Fail in main: Could not open %s", device_name);
        rclcpp::shutdown();
        return -1;
    }

    // (オプション) ポート開直後に古い入力データをフラッシュする
    // tcflush(fd1, TCIFLUSH); 

    // 読み取りスレッドの起動
    std::thread reader_thread;
    RCLCPP_INFO(logger, "Starting serial read thread.");
    reader_thread = std::thread(serial_read_thread_func, fd1, logger);

    // 既存の Subscriber ノードの作成とスピン
    auto my_subscriber_node = std::make_shared<MySubscriber>();
    RCLCPP_INFO(logger, "MySubscriber node spinning. Press Ctrl+C to exit.");
    
    // rclcpp::spin() は通常、Ctrl+C などで rclcpp::ok() が false になるまでブロックします
    rclcpp::spin(my_subscriber_node);

    // シャットダウン処理
    RCLCPP_INFO(logger, "Shutdown initiated. Waiting for read thread to join...");
    // rclcpp::ok() が false になると、読み取りスレッドのループも終了するはず
    if (reader_thread.joinable()) {
        reader_thread.join(); // 読み取りスレッドの終了を待つ
    }
    RCLCPP_INFO(logger, "Read thread joined.");

    if (fd1 >= 0) {
        RCLCPP_INFO(logger, "Closing serial port fd: %d", fd1);
        close(fd1);
        fd1 = -1;
    }

    rclcpp::shutdown();
    RCLCPP_INFO(logger, "ROS 2 shutdown complete.");
    return 0;
}