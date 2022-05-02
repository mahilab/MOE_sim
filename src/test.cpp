#include <Mahi/Util.hpp>
#include <MOE/MOE.hpp>
// #include <Eqns.hpp>
#include <Eigen/Dense>

using namespace mahi::util;
using Eigen::Matrix4d;
using Eigen::Vector4d;
moe::MoeDynamicModel model;
moe::MoeParameters moe_params;


std::vector<mahi::util::Integrator> qdd_qd;
std::vector<mahi::util::Integrator> qd_q;
std::vector<double> q(4,0);
std::vector<double> qd(4,0);

int main(){
    for (int i = 0; i < 4; i++) {
        qdd_qd.push_back(Integrator(0));
        qd_q.push_back(Integrator(0));
    }
    auto M = model.get_M();
    auto V = model.get_V();
    auto G = model.get_G();
    auto Tau = Eigen::VectorXd::Zero(4);
    auto Friction = Eigen::VectorXd::Zero(4);
    auto A = M;
    auto b = Tau - V - G - Friction;
    auto x = A.inverse()*b;
    std::cout << x << std::endl;

    for (size_t i = 0; i < 4; i++){
        qd[i] = qdd_qd[i].update(x(i), milliseconds(1));
        q[i]  = qd_q[i].update(qd[i], milliseconds(1));
    }
    for (size_t i = 0; i < 4; i++){
        qd[i] = qdd_qd[i].update(x(i), milliseconds(2));
        q[i]  = qd_q[i].update(qd[i], milliseconds(2));
    }
    std::cout << q << std::endl;
    std::cout << qd << std::endl;
    
    
    return 0;
}