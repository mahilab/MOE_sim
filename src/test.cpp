#include <Mahi/Util.hpp>
// #include <MeiiModel.hpp>
#include <Eqns.hpp>
#include <Eigen/Dense>

using namespace mahi::util;
using Eigen::Matrix4d;
using Eigen::Vector4d;

int main(){
    std::vector<double> qs = {0.0,0.0,0.0,0.0,0.0,0.0};

    std::cout << get_M(qs);

    return 0;
}