#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

struct InputParameters
{
  double h1{1.0};
  double h2{2.0};
  double s{2.0};
  double v0{1.0};
  double g{9.81};
};

//Publish values to rostopics
class BallPublisherNode : public rclcpp::Node {
    public:
        BallPublisherNode()
        : Node("ball_publisher_node") {
            inputs_.h1 = declare_parameter<double>("h1", 1.0);
            inputs_.h2 = declare_parameter<double>("h2", 2.0);
            inputs_.s = declare_parameter<double>("s", 2.0);
            inputs_.v0 = declare_parameter<double>("v0", 1.0);
            inputs_.g = declare_parameter<double>("g", 9.81);

        }

        private:
          InputParameters inputs_;

};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BallPublisherNode>());
    rclcpp::shutdown();
    return 0;
}