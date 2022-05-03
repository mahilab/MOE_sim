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

void print_matrix(Eigen::MatrixXd mat){
    for (int i = 0; i < mat.rows(); i++){
        for (int j = 0; j < mat.cols(); j++){
            std::cout << mat(i,j) << " ";
        }
        std::cout << std::endl;
    }
}

int main(){
    model.set_user_params({3,4,0});
    // for (int i = 0; i < 4; i++) {
    //     qdd_qd.push_back(Integrator(0));
    //     qd_q.push_back(Integrator(0));
    // }
    model.update({-35*DEG2RAD,0,0,0},{0,0,0,0});
    auto M = model.get_M();
    auto V = model.get_V();
    auto G = model.get_G();
    auto Tau = model.get_G();
    auto Friction = model.get_Friction();
    auto A = M;
    auto b = Tau - V - G - Friction;
    auto x = A.inverse()*b;
    print_matrix(x);

    // for (size_t i = 0; i < 4; i++){
    //     qd[i] = qdd_qd[i].update(x(i), milliseconds(1));
    //     q[i]  = qd_q[i].update(qd[i], milliseconds(1));
    // }
    // for (size_t i = 0; i < 4; i++){
    //     qd[i] = qdd_qd[i].update(x(i), milliseconds(2));
    //     q[i]  = qd_q[i].update(qd[i], milliseconds(2));
    // }
    // std::cout << q << std::endl;
    // std::cout << qd << std::endl;

    // std::cout << model.get_G() << std::endl;
    // print_matrix(model.get_V());
    // print_matrix(model.get_M());
    // std::cout << model.get_Friction() << std::endl;
    
    return 0;
}