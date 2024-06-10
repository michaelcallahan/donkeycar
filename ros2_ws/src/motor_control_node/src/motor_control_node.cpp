#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#ifdef __arm__
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <unistd.h>
#endif

class MotorControlNode : public rclcpp::Node
{
public:
    MotorControlNode()
    : Node("motor_control_node")
    {
        motor_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "motor_speed", 10, std::bind(&MotorControlNode::motor_callback, this, std::placeholders::_1));
        steer_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "steering_angle", 10, std::bind(&MotorControlNode::steer_callback, this, std::placeholders::_1));

#ifdef __arm__
        i2c_fd_ = wiringPiI2CSetup(0x40); // PCA9685 default I2C address
        if (i2c_fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to init I2C communication.");
            rclcpp::shutdown();
        }

        // Set PCA9685 frequency to 60Hz
        setPWMFreq(60);
#else
        RCLCPP_WARN(this->get_logger(), "Running without Raspberry Pi hardware");
#endif

        // Calibration values
        steer_left_ = 204;
        steer_center_ = 307;
        steer_right_ = 410;
        throttle_reverse_ = 204;
        throttle_zero_ = 307;
        throttle_forward_ = 410;
    }

private:
#ifdef __arm__
    void setPWMFreq(int freq)
    {
        // Calculate the prescale value
        float prescaleval = 25000000.0;
        prescaleval /= 4096.0;
        prescaleval /= float(freq);
        prescaleval -= 1.0;
        int prescale = int(prescaleval + 0.5);

        int oldmode = wiringPiI2CReadReg8(i2c_fd_, 0x00);
        int newmode = (oldmode & 0x7F) | 0x10; // Sleep
        wiringPiI2CWriteReg8(i2c_fd_, 0x00, newmode);
        wiringPiI2CWriteReg8(i2c_fd_, 0xFE, prescale);
        wiringPiI2CWriteReg8(i2c_fd_, 0x00, oldmode);
        usleep(5000);
        wiringPiI2CWriteReg8(i2c_fd_, 0x00, oldmode | 0x80);
    }

    void setPWM(int channel, int on, int off)
    {
        wiringPiI2CWriteReg8(i2c_fd_, 0x06 + 4 * channel, on & 0xFF);
        wiringPiI2CWriteReg8(i2c_fd_, 0x07 + 4 * channel, on >> 8);
        wiringPiI2CWriteReg8(i2c_fd_, 0x08 + 4 * channel, off & 0xFF);
        wiringPiI2CWriteReg8(i2c_fd_, 0x09 + 4 * channel, off >> 8);
    }
#endif

    void motor_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
#ifdef __arm__
        int pulse = convert_to_pwm(msg->data, throttle_reverse_, throttle_zero_, throttle_forward_);
        setPWM(0, 0, pulse);
#else
        RCLCPP_WARN(this->get_logger(), "Motor control not supported on non-ARM hardware");
#endif
    }

    void steer_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
#ifdef __arm__
        int pulse = convert_to_pwm(msg->data, steer_left_, steer_center_, steer_right_);
        setPWM(1, 0, pulse);
#else
        RCLCPP_WARN(this->get_logger(), "Steering control not supported on non-ARM hardware");
#endif
    }

    int convert_to_pwm(float value, int min_pwm, int zero_pwm, int max_pwm)
    {
        if (value < -1.0) value = -1.0;
        if (value > 1.0) value = 1.0;
        if (value < 0)
            return zero_pwm + value * (zero_pwm - min_pwm);
        else
            return zero_pwm + value * (max_pwm - zero_pwm);
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr motor_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steer_sub_;

    int steer_left_, steer_center_, steer_right_;
    int throttle_reverse_, throttle_zero_, throttle_forward_;

#ifdef __arm__
    int i2c_fd_;
#endif
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
