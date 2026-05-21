#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace
{
double parseArg(const char *arg, const std::string &name)
{
  try
  {
    size_t consumed = 0;
    const double value = std::stod(arg, &consumed);
    if (consumed != std::string(arg).size())
    {
      throw std::invalid_argument("trailing characters");
    }
    return value;
  }
  catch (const std::exception &e)
  {
    throw std::runtime_error("Invalid value for " + name + ": " + arg + " (" + e.what() + ")");
  }
}

void printUsage(const char *program)
{
  std::cout << "Usage: " << program
            << " h1 h2 s v0 [g]\n"
            << "  h1 = pail release height above ground\n"
            << "  h2 = ball release height above ground\n"
            << "  s  = horizontal distance from thrower to pail\n"
            << "  v0 = ball launch speed\n"
            << "  g  = gravity (optional, default 9.81)\n";
}
}  // namespace

int main(int argc, char **argv)
{
  if (argc != 5 && argc != 6)
  {
    printUsage(argv[0]);
    return 1;
  }

  try
  {
    const double h1 = parseArg(argv[1], "h1");
    const double h2 = parseArg(argv[2], "h2");
    const double s = parseArg(argv[3], "s");
    const double v0 = parseArg(argv[4], "v0");
    const double g = argc == 6 ? parseArg(argv[5], "g") : 9.81;

    if (s <= 0.0 || v0 <= 0.0 || g <= 0.0)
    {
      std::cerr << "s, v0, and g must be positive.\n";
      return 1;
    }

    const double deltaH = h1 - h2;
    const double theta0Rad = std::atan2(deltaH, s);
    const double theta0Deg = theta0Rad * 180.0 / std::acos(-1.0);
    const double flightDistance = std::hypot(s, deltaH);
    const double timeToCollision = flightDistance / v0;
    const double collisionHeight = h1 - 0.5 * g * timeToCollision * timeToCollision;
    const double vx0 = v0 * std::cos(theta0Rad);
    const double vy0 = 0.0;
    const double vz0 = v0 * std::sin(theta0Rad);

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Predicted throw solution\n";
    std::cout << "theta0 = " << theta0Rad << " rad = " << theta0Deg << " deg\n";
    std::cout << "Does theta0 depend on v0? no\n";
    std::cout << "time of collision = " << timeToCollision << " s\n";
    std::cout << "collision height = " << collisionHeight << " m\n";
    std::cout << "initial velocity vector = (" << vx0 << ", " << vy0 << ", " << vz0 << ") m/s\n";

    if (collisionHeight < 0.0)
    {
      std::cout << "Warning: collision height is below ground for this v0.\n";
    }

    return 0;
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << '\n';
    return 1;
  }
}