#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

int openSerial(const char *device_name){
    int  fd1 = open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    
    fcntl(fd1, F_SETFL, 0);
    struct termios conf_tio;
    tcgetattr(fd1, &conf_tio);
    
    speed_t BAUDRATE = B115200;
    cfsetispeed(&conf_tio, BAUDRATE);
    cfsetospeed(&conf_tio, BAUDRATE);

    conf_tio.c_lflag &= ~(ECHO | ICANON);

    conf_tio.c_cc[VMIN] = 0;
    conf_tio.c_cc[VTIME] = 0;

    tcsetattr(fd1, TCSANOW, &conf_tio);
    return fd1;
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("serial_receive_node");
    auto serial_pub = node->create_publisher<std_msgs::msg::String>("serial_in", 1000);

    char device_name[] = "/dev/arduino-mega"; // /dev/ttyACM0
    char fd1 = openSerial(device_name);
    if(fd1 < 0){
        RCLCPP_ERROR(node->get_logger(), "Serial Failed: could not open %s", device_name);
        printf("Serial Failed \n");
        rclcpp::shutdown();
    }

    rclcpp::WallRate loop_rate(20);
    while(rclcpp::ok()){
        char buf[256] = {0};
        std::string data;
        int flag = 0;

        while(true){
            int recv_data = read(fd1, buf, sizeof(buf));
            if(recv_data > 0){
                flag = 1;
                data += std::string(buf, recv_data);

                auto serial_msg = std::make_unique<std_msgs::msg::String>();
                serial_msg->data = data;
                serial_pub->publish(std::move(serial_msg));
                
                std::cout << "recv: " << data << std::endl; 
                
                break;

            }else{
                if(flag == 0) break;
            }
        }
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}