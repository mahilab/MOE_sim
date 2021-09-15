#include "MoeModel.hpp"
#include "Eqns.hpp"

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using namespace mahi::util;

inline double hardstop_torque(double q, double qd, double qmin, double qmax, double K, double B) {
    if (q < qmin)
        return K * (qmin - q) - B * qd;
    else if (q > qmax){
        return K * (qmax - q) - B * qd;
    }
    else
        return 0;
}

MoeModel::MoeModel() : 
    lim0(tau0_mot_cont * eta0, tau0_mot_max * eta0, seconds(2)),
    lim1(tau1_mot_cont * eta1, tau1_mot_max * eta1, seconds(2)),
    lim2(tau2_mot_cont * eta2, tau2_mot_max * eta2, seconds(2)),
    lim3(tau3_mot_cont * eta3, tau3_mot_max * eta3, seconds(2)),
    Tau(4),
    B(4),
    Fk(4),
    A(4,4),
    b(4),
    x(4)
{
    reset();
}

void MoeModel::update(Time t)
{
    // get list of qs and q_dots
    std::vector<double> qs = {q0, q1, q2, q3, q0d, q1d, q2d, q3d};

    VectorXd qdot(4);
    qdot(0) = q0d;
    qdot(1) = q1d;
    qdot(2) = q2d;
    qdot(3) = q3d;

    Clock CompTime;

    // MatrixXd M = get_M(qs);
    MatrixXd V = get_V(qs);
    VectorXd G = get_G(qs);

    Tau[0] = tau0 + hardstop_torque(q0,q0d,q0min,q0max,Khard0,Bhard0);
    Tau[1] = tau1 + hardstop_torque(q1,q1d,q1min,q1max,Khard1,Bhard1);
    Tau[2] = tau2 + hardstop_torque(q2,q2d,q2min,q2max,Khard2,Bhard2);
    Tau[3] = tau3 + hardstop_torque(q3,q3d,q3min,q3max,Khard3,Bhard3);

    constexpr double  B_coef[4] = {0.0252, 0.0019, 0.0029, 0.0019};
    constexpr double Fk_coef[4] = {   0.2, 0.1891, 0.0541, 0.1339};

    B[0] = B_coef[0]*q0d*1.0;
    B[1] = B_coef[1]*q1d*1.0;
    B[2] = B_coef[2]*q2d*1.0;
    B[3] = B_coef[3]*q3d*1.0;

    Fk[0] = Fk_coef[0]*std::tanh(q0d*10);
    Fk[1] = Fk_coef[1]*std::tanh(q1d*10);
    Fk[2] = Fk_coef[2]*std::tanh(q2d*10);
    Fk[3] = Fk_coef[3]*std::tanh(q3d*10);

    // A = M;
    b = Tau - V*qdot - G - B - Fk;
    x = get_nathanisdumb(qs)*b;

    q0dd = x[0];
    q1dd = x[1];
    q2dd = x[2];
    q3dd = x[3];

    // integrate acclerations to find velocities
    q0d = q0dd_q0d.update(q0dd, t);
    q1d = q1dd_q1d.update(q1dd, t);
    q2d = q2dd_q2d.update(q2dd, t);
    q3d = q3dd_q3d.update(q3dd, t);

    // integrate velocities to find positions
    q0 = q0d_q0.update(q0d, t);
    q1 = q1d_q1.update(q1d, t);
    q2 = q2d_q2.update(q2d, t);
    q3 = q3d_q3.update(q3d, t);
    
    comp_time = CompTime.get_elapsed_time().as_microseconds();

}

void MoeModel::set_torques(double tau0_, double tau1_, double tau2_, double tau3_) {
    tau0 = tau0_;
    tau1 = tau1_;
    tau2 = tau2_;
    tau3 = tau3_;
}

void MoeModel::set_positions(double q0_, double q1_, double q2_, double q3_) {
    q0 = q0_;
    q1 = q1_;
    q2 = q2_;
    q3 = q3_;
    q0d_q0 = Integrator(q0);
    q1d_q1 = Integrator(q1);
    q2d_q2 = Integrator(q2);
    q3d_q3 = Integrator(q3);
}

void MoeModel::set_velocities(double q0d_, double q1d_, double q2d_, double q3d_) {
    q0d = q0d_;
    q1d = q1d_;
    q2d = q2d_;
    q3d = q3d_;
    q0dd_q0d = Integrator(q0d);
    q1dd_q1d = Integrator(q1d);
    q2dd_q2d = Integrator(q2d);
    q3dd_q3d = Integrator(q3d);
}

void MoeModel::reset() {
    set_torques(0,0,0,0);
    set_positions(0,0,0,0);
    set_velocities(0,0,0,0);
    logged = false;
}