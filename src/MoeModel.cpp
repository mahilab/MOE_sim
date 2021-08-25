#include "MeiiModel.hpp"
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

MeiiModel::MeiiModel() : 
    lim1(tau1_mot_cont * eta1, tau1_mot_max * eta1, seconds(2)),
    lim2(tau2_mot_cont * eta2, tau2_mot_max * eta2, seconds(2)),
    lim3(tau3_mot_cont * eta3, tau3_mot_max * eta3, seconds(2)),
    lim4(tau3_mot_cont * eta3, tau3_mot_max * eta3, seconds(2)),
    lim5(tau3_mot_cont * eta3, tau3_mot_max * eta3, seconds(2)),
    Tau(5),
    B(5),
    Fk(5),
    A(5,5),
    b(5),
    x(5)
{
    reset();
}

void MeiiModel::update(Time t)
{
    // get list of qs and q_dots
    std::vector<double> qs = {q1, q2, q3, q4, q5, q6, q7, q8, q9, q10, q11, q12, q13, q14, q1d, q2d, q3d, q4d, q5d, q6d, q7d, q8d, q9d, q10d, q11d, q12d, q13d, q14d};

    VectorXd qdot(5);
    qdot(0) = q1d;
    qdot(1) = q2d;
    qdot(2) = q3d;
    qdot(3) = q4d;
    qdot(4) = q5d;    

    VectorXd psi = get_psi(qs);
    MatrixXd psi_dq = get_psi_dq(qs);
    MatrixXd psi_dq_dt = get_psi_dq_dt(qs);

    Clock CompTime;

    MatrixXd M = get_M(qs);
    MatrixXd V = get_V(qs);
    VectorXd G = get_G(qs);

    MatrixXd ident(14,5);
    ident << MatrixXd::Zero(9,5) , MatrixXd::Identity(5,5);

    MatrixXd psi_dq_inv = psi_dq.inverse();
    MatrixXd rho        = psi_dq_inv*ident;
    MatrixXd rho_dt     = -psi_dq_inv*psi_dq_dt*psi_dq_inv*ident;

    Tau[0] = tau1 + hardstop_torque(q1,q1d,q1min,q1max,Khard1,Bhard1);
    Tau[1] = tau2 + hardstop_torque(q2,q2d,q2min,q2max,Khard1,Bhard1);
    Tau[2] = tau3 + hardstop_torque(q3,q3d,q3min,q3max,Khard,Bhard);
    Tau[3] = tau4 + hardstop_torque(q4,q4d,q4min,q4max,Khard,Bhard);
    Tau[4] = tau5 + hardstop_torque(q5,q5d,q5min,q5max,Khard,Bhard);

    MatrixXd Mpar = rho.transpose()*M*rho;
    MatrixXd Vpar = rho.transpose()*V*rho + rho.transpose()*M*rho_dt;
    MatrixXd Gpar = rho.transpose()*G;


    /// Damping coefficients [Nm*s/rad]
    // constexpr double B_coef[5] = {0.1215, 0.1, 0.1, 0.1, 0.1};
    /// Kinetic friction [Nm]
    // constexpr double Fk_coef[5] = {0.01, 0.01, 0.05, 0.05, 0.05};

    constexpr double B_coef[5] = {0.5, 0.1, 0.3, 0.3, 0.3};
    constexpr double Fk_coef[5] = {0.1, 0.01, 0.2, 0.2, 0.2};

    B[0] = B_coef[0]*q1d*10.0;
    B[1] = B_coef[1]*q2d*10.0;
    B[2] = B_coef[2]*q3d*100.0;
    B[3] = B_coef[3]*q4d*100.0;
    B[4] = B_coef[4]*q5d*100.0;

    Fk[0] = Fk_coef[0]*std::tanh(q1d*10);
    Fk[1] = Fk_coef[1]*std::tanh(q2d*10);
    Fk[2] = Fk_coef[2]*std::tanh(q3d*100);
    Fk[3] = Fk_coef[3]*std::tanh(q4d*100);
    Fk[4] = Fk_coef[4]*std::tanh(q5d*100);

    A = Mpar;// + M_mot;
    b = Tau - Vpar*qdot - Gpar - B - Fk;
    x = A.inverse()*b;

    q1dd = x[0];
    q2dd = x[1];
    q3dd = x[2];
    q4dd = x[3];
    q5dd = x[4];

    // integrate acclerations to find velocities
    q1d = q1dd_q1d.update(q1dd, t);
    q2d = q2dd_q2d.update(q2dd, t);
    q3d = q3dd_q3d.update(q3dd, t);
    q4d = q4dd_q4d.update(q4dd, t);
    q5d = q5dd_q5d.update(q5dd, t);

    // integrate velocities to find positions
    q1 = q1d_q1.update(q1d, t);
    q2 = q2d_q2.update(q2d, t);
    q3 = q3d_q3.update(q3d, t);
    q4 = q4d_q4.update(q4d, t);
    q5 = q5d_q5.update(q5d, t);

    calc_dependent_joint_values();    
    
    comp_time = CompTime.get_elapsed_time().as_microseconds();

}

void MeiiModel::set_torques(double _tau1, double _tau2, double _tau3, double _tau4, double _tau5) {
    tau1 = _tau1;
    tau2 = _tau2;
    tau3 = _tau3;
    tau4 = _tau4;
    tau5 = _tau5;
}

void MeiiModel::set_positions(double _q1, double _q2, double _q3, double _q4, double _q5, double _q6, double _q7, double _q8, double _q9, double _q10, double _q11, double _q12, double _q13, double _q14) {
    q1 = _q1;
    q2 = _q2;
    q3 = _q3;
    q4 = _q4;
    q5 = _q5;
    q1d_q1 = Integrator(q1);
    q2d_q2 = Integrator(q2);
    q3d_q3 = Integrator(q3);
    q4d_q4 = Integrator(q4);
    q5d_q5 = Integrator(q5);

    q6 = _q6;
    q7 = _q7;
    q8 = _q8;
    q9 = _q9;
    q10 = _q10;
    q11 = _q11;
    q12 = _q12;
    q13 = _q13;
    q14 = _q14;
}

void MeiiModel::set_velocities(double _q1d, double _q2d, double _q3d, double _q4d, double _q5d, double _q6d, double _q7d, double _q8d, double _q9d, double _q10d, double _q11d, double _q12d, double _q13d, double _q14d) {
    q1d = _q1d;
    q2d = _q2d;
    q3d = _q3d;
    q4d = _q4d;
    q5d = _q5d;
    q1dd_q1d = Integrator(q1d);
    q2dd_q2d = Integrator(q2d);
    q3dd_q3d = Integrator(q3d);
    q4dd_q4d = Integrator(q4d);
    q5dd_q5d = Integrator(q5d);

    q6d = _q6d;
    q7d = _q7d;
    q8d = _q8d;
}

void MeiiModel::reset() {
    set_torques(0,0,0,0,0);
    set_positions(-45*DEG2RAD,0.0,0.1,0.1,0.1,1.02844,1.02844,1.02844,0.0856,0.0,0.0,0.0,0.0,0.0);
    set_velocities(0,0,0,0,0,0,0,0,0,0,0,0,0,0);
    logged = false;
}

void MeiiModel::calc_dependent_joint_values() {

    VectorXd q_guess(14);
    q_guess << q1, q2, q3, q4, q5, mahi::util::PI / 4, mahi::util::PI / 4, mahi::util::PI / 4, 0.1, 0, 0, 0, 0, 0;

    // initialize variables for keeping track of error
    double tol = 1e-10;
    double err = 2*tol;
    size_t max_it = 10;

    // run no more than max_it iterations of updating the solution for m_qp
    // exit loop once the error is below the input tolerance
    uint32 it = 0;
    while (it < max_it && err > tol) {
        std::vector<double> q_guess_double;
        q_guess_double.resize(q_guess.size());
        for (size_t i = 0; i < 14; i++){
            q_guess_double[i] = q_guess(i);
        }
        VectorXd psi = get_psi(q_guess_double);

        VectorXd to_psi_bar(14);
        to_psi_bar << 0, 0, 0, 0, 0, 0, 0, 0, 0, q1, q2, q3, q4, q5;

        VectorXd psi_bar = psi - to_psi_bar;

        q_guess -= get_psi_dq(q_guess_double).inverse()*psi_bar;

        // update the error. this ends up being sqrt of sum of squares
        err = 0;
        for (size_t i = 0; i < 14; ++i) {
            err += psi(i)*psi(i);
        }
        err = sqrt(err);

        // while iterator
        it++;
    }

    q6  = q_guess(5);
    q7  = q_guess(6);
    q8  = q_guess(7);
    q9  = q_guess(8);
    q10 = q_guess(9);
    q11 = q_guess(10);
    q12 = q_guess(11); 
    q13 = q_guess(12);
    q14 = q_guess(13);

    // Velocity calculation
    std::vector<double> qs = {q1, q2, q3, q4, q5, q6, q7, q8, q9, q10, q11, q12, q13, q14};
    VectorXd qds(5);
    qds(0) = q1d;
    qds(1) = q2d;
    qds(2) = q3d;
    qds(3) = q4d;
    qds(4) = q5d;

    MatrixXd psi_dq = get_psi_dq(qs);
    MatrixXd ident(14,5);
    ident << MatrixXd::Zero(9,5) , MatrixXd::Identity(5,5);

    MatrixXd psi_dq_inv = psi_dq.inverse();
    MatrixXd rho        = psi_dq_inv*ident;

    VectorXd qd = rho*qds;

    q6d = qd[5];
    q7d = qd[6];
    q8d = qd[7];
    q9d = qd[8];
    q10d = qd[9];
    q11d = qd[10];
    q12d = qd[11];
    q13d = qd[12];
    q14d = qd[13];

}