#include <Mahi/Util.hpp>
#include <MoeModel.hpp>
#include <Eqns.hpp>
#include <Eigen/Dense>

using namespace mahi::util;
using Eigen::Matrix4d;
using Eigen::Vector4d;

int main(){
    MoeModel g_model;
    std::cout << "Iczz0: " << Iczz0 << std::endl;
    std::cout << "Pcy0: " << Pcy0 << std::endl;

    g_model.set_params(0,4,3);

    std::cout << "Iczz0: " << Iczz0 << std::endl;
    std::cout << "Pcy0: " << Pcy0 << std::endl;

    g_model.set_params(0,4,2);

    std::cout << "Iczz0: " << Iczz0 << std::endl;
    std::cout << "Pcy0: " << Pcy0 << std::endl;

    return 0;
}