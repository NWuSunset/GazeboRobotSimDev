#include <gz/msgs/twist.pb.h>
#include <gz/transport/Node.hh>

#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

//Subscriber
namespace
{
struct InputParameters
{
  double h1{1.0};
  double h2{2.0};
  double s{2.0};
  double v0{1.0};
  double g{9.81};
};

struct ThrowSolution
{
  double theta_rad{0.0};
  double theta_deg{0.0};
  double time_to_collision{0.0};
  double collision_height{0.0};
  double vx0{0.0};
  double vy0{0.0};
  double vz0{0.0};
};

ThrowSolution computeSolution(const InputParameters &inputs)
{
  const double delta_h = inputs.h1 - inputs.h2;
  const double theta_rad = std::atan2(delta_h, inputs.s);
  const double theta_deg = theta_rad * 180.0 / std::acos(-1.0);
  const double flight_distance = std::hypot(inputs.s, delta_h);
  const double time_to_collision = flight_distance / inputs.v0;
  const double collision_height = inputs.h1 - 0.5 * inputs.g * time_to_collision * time_to_collision;

  ThrowSolution solution;
  solution.theta_rad = theta_rad;
  solution.theta_deg = theta_deg;
  solution.time_to_collision = time_to_collision;
  solution.collision_height = collision_height;
  solution.vx0 = inputs.v0 * std::cos(theta_rad);
  solution.vy0 = 0.0;
  solution.vz0 = inputs.v0 * std::sin(theta_rad);
  return solution;
}
}  // namespace

class BallPailNode : public rclcpp::Node
{
public:
  BallPailNode()
  : Node("ball_pail_planner")
  {
    inputs_.h1 = declare_parameter<double>("h1", 1.0);
    inputs_.h2 = declare_parameter<double>("h2", 2.0);
    inputs_.s = declare_parameter<double>("s", 2.0);
    inputs_.v0 = declare_parameter<double>("v0", 1.0);
    inputs_.g = declare_parameter<double>("g", 9.81);

    input_topic_ = declare_parameter<std::string>("input_topic", "ball_pail/input");
    angle_topic_ = declare_parameter<std::string>("angle_topic", "ball_pail/launch_angle");
    velocity_topic_ = declare_parameter<std::string>("velocity_topic", "ball_pail/launch_velocity");

    angle_publisher_ = create_publisher<std_msgs::msg::Float64>(angle_topic_, 10);
    velocity_publisher_ = create_publisher<geometry_msgs::msg::Twist>(velocity_topic_, 10);

    input_subscription_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      input_topic_, 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr message) {
        updateInputs(*message);
        publishSolution("subscription");
      });

    publishSolution("startup");
  }

private:
  void updateInputs(const std_msgs::msg::Float64MultiArray &message)
  {
    if (message.data.size() > 0) {
      inputs_.h1 = message.data[0];
    }
    if (message.data.size() > 1) {
      inputs_.h2 = message.data[1];
    }
    if (message.data.size() > 2) {
      inputs_.s = message.data[2];
    }
    if (message.data.size() > 3) {
      inputs_.v0 = message.data[3];
    }
    if (message.data.size() > 4) {
      inputs_.g = message.data[4];
    }
  }

  void publishSolution(const std::string &source)
  {
    if (inputs_.s <= 0.0 || inputs_.v0 <= 0.0 || inputs_.g <= 0.0) {
      RCLCPP_ERROR(get_logger(), "Invalid inputs from %s: s, v0, and g must be positive.", source.c_str());
      return;
    }

    const ThrowSolution solution = computeSolution(inputs_);

    std_msgs::msg::Float64 angle_message;
    angle_message.data = solution.theta_rad;

    geometry_msgs::msg::Twist velocity_message;
    velocity_message.linear.x = solution.vx0;
    velocity_message.linear.y = solution.vy0;
    velocity_message.linear.z = solution.vz0;

    angle_publisher_->publish(angle_message);
    velocity_publisher_->publish(velocity_message);

    RCLCPP_INFO(
      get_logger(),
      "[%s] theta=%.6f rad (%.6f deg), time=%.6f s, collision height=%.6f m, velocity=(%.6f, %.6f, %.6f)",
      source.c_str(), solution.theta_rad, solution.theta_deg, solution.time_to_collision,
      solution.collision_height, solution.vx0, solution.vy0, solution.vz0);

    if (solution.collision_height < 0.0) {
      RCLCPP_WARN(get_logger(), "Predicted collision height is below ground for the current inputs.");
    }
  }

  InputParameters inputs_;
  std::string input_topic_;
  std::string angle_topic_;
  std::string velocity_topic_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr input_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallPailNode>());
  rclcpp::shutdown();
  return 0;
}