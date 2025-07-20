#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/kill.hpp"
#include "turtlesim/msg/pose.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <queue>
#include <string>
#include <cmath>
#include <sstream>
#include <memory>

using namespace std::placeholders;

struct TurtlePosition
{
    std::string name_;
    float x_;
    float y_;
    float theta_;
};

class TurtleController : public rclcpp::Node
{
public:
    TurtleController();
private:
    turtlesim::msg::Pose current_position_;
    std::queue<TurtlePosition> list_turtle_;
    bool isTurtleExist_ = false;
    bool isKilling_ = false;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_velocity_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subcriber_current_postion_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subcriber_new_postion_;
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr client_killer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subcriber_;

    void turtle_controller();
    void current_turtle_position(const turtlesim::msg::Pose::SharedPtr msg);
    void new_turtle_position(const std_msgs::msg::String::SharedPtr msg);
};

TurtleController::TurtleController() : Node ("TurtleController")
{
    subcriber_current_postion_ = this->create_subscription<turtlesim::msg::Pose>(
                                        "/turtle1/pose", 10,
                                        std::bind(&TurtleController::current_turtle_position,this, _1)
                                        );
    subcriber_new_postion_ = this->create_subscription<std_msgs::msg::String>(
                                        "spawn",10,
                                        std::bind(&TurtleController::new_turtle_position,this,_1)
                                        );   
    client_killer_ = this->create_client<turtlesim::srv::Kill>("/kill");     
    
    publisher_velocity_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                    std::bind(&TurtleController::turtle_controller,this)
                                    );
}

void TurtleController::current_turtle_position(const turtlesim::msg::Pose::SharedPtr msg)
{
    current_position_ = *msg;
    isTurtleExist_ = true;
}

void TurtleController::new_turtle_position(const std_msgs::msg::String::SharedPtr msg)
{
    TurtlePosition tp;
    std::string line = msg->data;
    std::stringstream ss(line);
    std::string data_in_line;

    std::vector<std::string> data_save;
    while (std::getline(ss, data_in_line, ','))
    {
        data_save.push_back(data_in_line);
    }

    if (data_save.size() != 4)
    {
        RCLCPP_INFO(this->get_logger(),"Data format INCORRECT");
        return;
    }

    tp.name_ = data_save[0];
    tp.x_ = std::stof(data_save[1]);
    tp.y_ = std::stof(data_save[2]);
    tp.theta_ = std::stof(data_save[3]);

    RCLCPP_INFO(this->get_logger(), "[TurtleController][Queue] New_turtle %s: (%.2f, %.2f, %.2f)",
                tp.name_.c_str(), tp.x_, tp.y_, tp.theta_);

    list_turtle_.push(tp);

}   

void TurtleController::turtle_controller()
{
    if (!isTurtleExist_ || list_turtle_.empty())
    {
        geometry_msgs::msg::Twist msg_top;
        publisher_velocity_->publish(msg_top);
        return;
    }

    if (isKilling_)
    {
        return;
    }

    auto &tp = list_turtle_.front();

    // Computer distance.
    double dx = tp.x_ - current_position_.x;
    double dy = tp.y_ - current_position_.y;
    double distance = std::hypot(dx,dy);

    // Compute orientation.
    double theta_target = std::atan2(dy,dx);
    auto theta_current = current_position_.theta;
    
    // Normalize theta_current = [-pi, pi]
    if(theta_current > M_PI) theta_current -= 2*M_PI;
    if(theta_current < -M_PI) theta_current += 2*M_PI;
    double angle_orientation = theta_target - theta_current;

    // Normalize angle_orientation = [-pi, pi]
    if(angle_orientation > M_PI) angle_orientation -= 2*M_PI;
    if(angle_orientation < -M_PI) angle_orientation += 2*M_PI;

    geometry_msgs::msg::Twist move;

    if (distance > 0.5)
    {
        move.linear.x = 3 * distance;
        move.angular.z = 3.0 * angle_orientation;
        publisher_velocity_->publish(move);
    }

    else
    {
        move.linear.x = 0.0;
        move.angular.z = 0.0;
        publisher_velocity_->publish(move);

        if(client_killer_->wait_for_service(std::chrono::milliseconds(500)))
        {
            auto request = std::make_shared<turtlesim::srv::Kill::Request>();
            request->name = tp.name_;
            isKilling_ = true;

            auto respond = [this,request] (rclcpp::Client<turtlesim::srv::Kill>::SharedFuture future)
            {
                if (future.valid())
                {
                    RCLCPP_INFO(this->get_logger(), "[TurtleController][Kill] %s killed.",request->name.c_str());
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "[TurtleController][Kill] Service call failed.");
                }
            };
            client_killer_->async_send_request(request,respond);
            list_turtle_.pop();
            isKilling_ = false;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Skill service not available. ");
        }   
    }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}