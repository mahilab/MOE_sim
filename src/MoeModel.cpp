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
    Jm(4,4),
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

    MatrixXd M = get_M(qs);
    MatrixXd V = get_V(qs);
    VectorXd G = get_G(qs);

    Tau[0] = tau0 + hardstop_torque(q0,q0d,q0min,q0max,Khard0,Bhard0);
    Tau[1] = tau1 + hardstop_torque(q1,q1d,q1min,q1max,Khard1,Bhard1);
    Tau[2] = tau2 + hardstop_torque(q2,q2d,q2min,q2max,Khard2,Bhard2);
    Tau[3] = tau3 + hardstop_torque(q3,q3d,q3min,q3max,Khard3,Bhard3);

    // constexpr double  B_coef[4] = {0.0840, 0.0252, 0.0019, 0.0029};
    // constexpr double Fk_coef[4] = {0.2186, 0.1891, 0.0541, 0.1339};
    
    // 
    constexpr double  B_coef[4] = {0.0393, 0.0691, 0.0068, 0.0025};
    constexpr double Fk_coef[4] = {0.1838, 0.1572, 0.0996, 0.1685};

    B[0] = B_coef[0]*q0d*1.0;
    B[1] = B_coef[1]*q1d*1.0;
    B[2] = B_coef[2]*q2d*1.0;
    B[3] = B_coef[3]*q3d*1.0;

    Fk[0] = Fk_coef[0]*std::tanh(q0d*10);
    Fk[1] = Fk_coef[1]*std::tanh(q1d*10);
    Fk[2] = Fk_coef[2]*std::tanh(q2d*10);
    Fk[3] = Fk_coef[3]*std::tanh(q3d*10);

    Jm = MatrixXd::Zero(4,4);

    Jm(0,0) = Jm0/eta0/eta0;
    Jm(1,1) = Jm1/eta1/eta1;
    Jm(2,2) = Jm2/eta2/eta2;
    Jm(3,3) = Jm3/eta3/eta3;

    A = M + Jm;
    b = Tau - V*qdot - G - B - Fk;
    x = A.inverse()*b;

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

void MoeModel::set_params(int shoulder_pos_, int counterweight_pos_, int forearm_pos_) {
    double x_cw = PcxCw;
    double y_cw = PcyCw + static_cast<double>(counterweight_pos_ - 4)*0.0127;
    // std::cout << "y_cw: " << y_cw << std::endl;
    // double z_cw = PczCw;
    double x_sl = PcxSl;
    double y_sl = PcySl + static_cast<double>(forearm_pos_-3)*0.005;
    // std::cout << "y_sl: " << y_sl << std::endl;
    // double z_sl = PczSl;
    double newPcy = (mCw*y_cw + mSl*y_sl + mEl*PcyEl)/(mCw + mSl + mEl);
    // Parallel Axis Thm
    double Iczz_cw = IczzCw + mCw*((x_cw)*(x_cw)+(y_cw)*(y_cw));
    double Iczz_sl = IczzSl + mSl*((x_sl)*(x_sl)+(y_sl)*(y_sl));
    double Iczz_el = IczzEl + mEl*((PcxEl)*(PcxEl)+(PcyEl)*(PcyEl));
    // moe_mass_props.all_props[0].Pcx = ;
    Pcy0 = newPcy;
    // moe_mass_props.all_props[0].Pcz = ;
    // moe_mass_props.all_props[0].Icxx = ;
    // moe_mass_props.all_props[0].Icyy = ;
    Iczz0 = Iczz_cw + Iczz_sl + Iczz_el;

    dist = 2.8024836E-1 - static_cast<double>(forearm_pos_ - 3)*0.005;
    q_s = static_cast<double>(shoulder_pos_)*15.0*DEG2RAD;
}