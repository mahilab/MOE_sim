#include <Mahi/Util.hpp>
// #include <MeiiModel.hpp>
// #include <Eqns.hpp>
#include <Eigen/Dense>

using namespace mahi::util;
using Eigen::Matrix4d;
using Eigen::Vector4d;

int main(){
    std::vector<double> qs = {0.2,-1.0,0.234,3.78,0.1,-0.16, 0.0, 0.15};

    Clock time_clock;
    // std::cout << get_M(qs).inverse() << "\n\n";
    // std::cout << time_clock.get_elapsed_time().as_microseconds() << "\n\n";
    // time_clock.restart();
    // std::cout << get_nathanisdumb(qs) << "\n\n";
    // std::cout << time_clock.get_elapsed_time().as_microseconds();

    return 0;
}