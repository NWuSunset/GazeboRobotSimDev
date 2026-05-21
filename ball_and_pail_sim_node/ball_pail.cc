#include <gz/msgs/twist.pb.h>
#include <gz/transport/Node.hh>

#include <cmath>

double getTheta0(double h1, double h2, double s) { // function of h1, h2, and s
    double delatH = h1 - h2;
    double theata0Rad = std::atan2(delatH, s); // radians
    return theata0Rad;
}

int main(int argc, char **argv) {
    //Get parameters: h1, h2, s, v0, g (default)
    const double h1 = 1.0;
    const double h2 = 2.0;
    const double s = 2.0;
    const double v0 = 1.0;
    const double g = 9.81;

    double theata0 = getTheta0(h1, h2, s);
    std::cout << "theta0 = " << theata0 << " rad" << std::endl;
 
    return 0;
}