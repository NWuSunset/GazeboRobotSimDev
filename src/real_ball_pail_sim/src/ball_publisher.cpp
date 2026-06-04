#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <ros_gz_interfaces/msg/entity.hpp>
#include <ros_gz_interfaces/msg/entity_wrench.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

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

class BallPublisherNode : public rclcpp::Node
{
public:
  BallPublisherNode()
  : Node("ball_publisher_node")
  {
    inputs_.h1 = declare_parameter<double>("h1", 1.0);
    inputs_.h2 = declare_parameter<double>("h2", 2.0);
    inputs_.s = declare_parameter<double>("s", 2.0);
    inputs_.v0 = declare_parameter<double>("v0", 1.0);
    inputs_.g = declare_parameter<double>("g", 9.81);
    ball_mass_ = declare_parameter<double>("ball_mass", 0.5);
    impulse_duration_ = declare_parameter<double>("impulse_duration", 0.001);

    angle_topic_ = declare_parameter<std::string>("angle_topic", "ball_pail/launch_angle");
    wrench_topic_ = declare_parameter<std::string>("wrench_topic", "ball_pail/wrench");
    solution_topic_ = declare_parameter<std::string>("solution_topic", "ball_pail/solution");

    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    angle_publisher_ = create_publisher<std_msgs::msg::Float64>(angle_topic_, qos);
    wrench_publisher_ = create_publisher<ros_gz_interfaces::msg::EntityWrench>(wrench_topic_, qos);
    solution_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(solution_topic_, qos);

    publish_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      [this]() {
        if (published_) {
          return;
        }
        
        // Wait until the ros_gz_bridge has actually subscribed to the topic
        if (wrench_publisher_->get_subscription_count() == 0) {
          return;
        }
        
        // Wait an additional 1.5 seconds (3 ticks) for Gazebo physics & entities to settle
        if (ready_ticks_ < 3) {
          ready_ticks_++;
          return;
        }

        published_ = true;
        publishSolution();
        publish_timer_->cancel();
      });
  }
  

private:
  void publishSolution()
  {
    if (inputs_.s <= 0.0 || inputs_.v0 <= 0.0 || inputs_.g <= 0.0) {
      RCLCPP_ERROR(get_logger(), "Invalid inputs: s, v0, and g must be positive.");
      return;
    }

    const ThrowSolution solution = computeSolution(inputs_);

    std_msgs::msg::Float64 angle_message;
    angle_message.data = solution.theta_rad;

    ros_gz_interfaces::msg::EntityWrench wrench_message;
    wrench_message.entity.name = "ball";
    wrench_message.entity.type = ros_gz_interfaces::msg::Entity::MODEL;
    wrench_message.wrench.force.x = ball_mass_ * solution.vx0 / impulse_duration_;
    wrench_message.wrench.force.y = ball_mass_ * solution.vy0 / impulse_duration_;
    wrench_message.wrench.force.z = ball_mass_ * solution.vz0 / impulse_duration_;
    wrench_message.wrench.torque.x = 0.0;
    wrench_message.wrench.torque.y = 0.0;
    wrench_message.wrench.torque.z = 0.0;

    std_msgs::msg::Float64MultiArray solution_message;
    solution_message.data = {
      solution.theta_rad,
      solution.theta_deg,
      solution.time_to_collision,
      solution.collision_height,
      solution.vx0,
      solution.vy0,
      solution.vz0
    };

    angle_publisher_->publish(angle_message);
    wrench_publisher_->publish(wrench_message);
    solution_publisher_->publish(solution_message);

    RCLCPP_INFO(
      get_logger(),
      "theta=%.6f rad (%.6f deg), time=%.6f s, collision height=%.6f m, velocity=(%.6f, %.6f, %.6f), force=(%.6f, %.6f, %.6f)",
      solution.theta_rad, solution.theta_deg, solution.time_to_collision,
      solution.collision_height, solution.vx0, solution.vy0, solution.vz0,
      wrench_message.wrench.force.x, wrench_message.wrench.force.y,
      wrench_message.wrench.force.z);

    if (solution.collision_height < 0.0) {
      RCLCPP_WARN(get_logger(), "Predicted collision height is below ground for the current inputs.");
    }
  }
  //test 

  InputParameters inputs_;
  std::string angle_topic_;
  std::string wrench_topic_;
  std::string solution_topic_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_publisher_;
  rclcpp::Publisher<ros_gz_interfaces::msg::EntityWrench>::SharedPtr wrench_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr solution_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  bool published_{false};
  int ready_ticks_{0};
  double ball_mass_{0.5};
  double impulse_duration_{0.001};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallPublisherNode>());
  rclcpp::shutdown();
  return 0;
}