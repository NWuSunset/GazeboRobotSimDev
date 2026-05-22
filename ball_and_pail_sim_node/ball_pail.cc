#include <gz/msgs/twist.pb.h>
#include <gz/transport/Node.hh>

#include <cmath>

//Subscriber
namespace {
    struct InputParameters 
    {
        double h1{1.0};
        double h2{2.0};
        double s{2.0};
        double v0{1.0};
        double g{9.81};
    };

    struct SolutionResult {
double theta_rad{0.0};
  double theta_deg{0.0};
  double time_to_collision{0.0};
  double collision_height{0.0};
  double vx0{0.0};
  double vy0{0.0};
  double vz0{0.0};
    };
}


int main(int argc, char **argv) {
    //Get parameters: h1, h2, s, v0, g (default)
    // Get params from publisher 
    const double h1 = 1.0;
    const double h2 = 2.0;
    const double s = 2.0;
    const double v0 = 1.0;
    const double g = 9.81;
    
    //theata 
    const double deltaH = h1 - h2;
    const double theta0Rad = std::atan2(deltaH, s);
    const double theta0Deg = theta0Rad  * 180.0 / std::acos(-1.0); //arccos -1 same as pi

    const double flightDistance = std::hypot(s, deltaH); // sqrt(s^2 + (h1-h2)^2) same as finding the hypotenuse
    const double timeOfCollision = flightDistance / v0;
    
    const double collisionHeight = h1 - (0.5 * g * timeOfCollision * timeOfCollision); // y(t) = h1 - 1/2gt^2

    const double vx0 = v0 * std::cos(theta0Rad);
    const double vy0 = 0.0; // in 3d coordinate space this should be 0
    const double vz0 = v0 * std::sin(theta0Rad);

    //std::cout << std::fixed << std::setprecision(6);
    std::cout << "Predicted throw solution\n";
    std::cout << "theta0 = " << theta0Rad << " rad = " << theta0Deg << " deg\n";
    std::cout << "Does theta0 depend on v0? no\n";
    std::cout << "time of collision = " << timeOfCollision << " s\n";
    std::cout << "collision height = " << collisionHeight << " m\n";
    std::cout << "initial velocity vector = (" << vx0 << ", " << vy0 << ", " << vz0 << ") m/s\n";

    if (collisionHeight < 0.0)
    {
      std::cout << "Warning: collision height is below ground for this v0.\n";
    }

    // Get correct theata value, then publish it to the ball thrown (aiming)

    return 0;
}