#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "std_msgs/msg/string.hpp"
#include <sstream> // 👈 thêm dòng này để dùng stringstream

class TurtleSpawner : public rclcpp::Node
{
public:
    TurtleSpawner();
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    int count_ = 1;
    rclcpp::TimerBase::SharedPtr timer_;
    void turtle_spawner();
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client_;
};

TurtleSpawner::TurtleSpawner() : Node("turtle")
{
    RCLCPP_INFO(this->get_logger(), "TurtleSpawner() START!");
    client_ = this->create_client<turtlesim::srv::Spawn>("spawn");
    publisher_ = this->create_publisher<std_msgs::msg::String>("spawn", 10);
    timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                     std::bind(&TurtleSpawner::turtle_spawner, this));
}

void TurtleSpawner::turtle_spawner()
{
    while (!client_->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_INFO(this->get_logger(), "Wait respond from server...");
    }

    auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
    request->x = static_cast<float>(rand() % 10 + 1);
    request->y = static_cast<float>(rand() % 10 + 1);
    request->theta = static_cast<float>((rand() % 360) * M_PI / 180.0);
    request->name = "turtle_" + std::to_string(count_++);

    auto respond = [this, request](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future)
    {
        auto ret = future.get();
        RCLCPP_INFO(this->get_logger(), "[TurtleSpawner] %s: (%.2f, %.2f, %.2f)", ret->name.c_str(), request->x, request->y, request->theta);

        std_msgs::msg::String msg;
        std::stringstream ss;
        ss << ret->name << "," << request->x << "," << request->y << "," << request->theta;
        msg.data = ss.str();

        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Turtle[%d] Published: %s", count_ - 1, msg.data.c_str());
    };

    client_->async_send_request(request, respond);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSpawner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
