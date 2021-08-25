#include <Mahi/Util.hpp>
// #include <MeiiModel.hpp>
#include <Eqns.hpp>
#include <Eigen/Dense>
#include <ctpl_stl.h>

using namespace mahi::util;
using Eigen::Matrix4d;
using Eigen::Vector4d;

int main(){
    std::vector<double> qs = {-45*DEG2RAD,0.0,0.1,0.1,0.1,1.02844,1.02844,1.02844, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    std::cout.precision(128);
    std::scientific;
    std::cout << std::isnan((get_V11(qs)));

    // Matrix4d V;

    // V(0,0) = get_V11(qs);
    // V(0,1) = get_V12(qs);
    // V(0,2) = get_V13(qs);
    // V(0,3) = get_V14(qs);
    // V(1,0) = get_V21(qs);
    // V(1,1) = get_V22(qs);
    // V(1,2) = get_V23(qs);
    // V(1,3) = get_V24(qs);
    // V(2,0) = get_V31(qs);
    // V(2,1) = get_V32(qs);
    // V(2,2) = get_V33(qs);
    // V(2,3) = get_V34(qs);
    // V(3,0) = get_V41(qs);
    // V(3,1) = get_V42(qs);
    // V(3,2) = get_V43(qs);
    // V(3,3) = get_V44(qs);

    // std::cout << V(0,0) << ", " << V(0,1) << ", " << V(0,2) << ", " << V(0,3) << std::endl;
    // std::cout << V(1,0) << ", " << V(1,1) << ", " << V(1,2) << ", " << V(1,3) << std::endl;
    // std::cout << V(2,0) << ", " << V(2,1) << ", " << V(2,2) << ", " << V(2,3) << std::endl;
    // std::cout << V(3,0) << ", " << V(3,1) << ", " << V(3,2) << ", " << V(3,3) << std::endl;

    return 0;
}