#pragma once
#include <Mahi/Util/Timing/Time.hpp>
#include <Mahi/Util/Math/Constants.hpp>
#include <Mahi/Util/Math/Integrator.hpp>
#include <Mahi/Robo/Control/Limiter.hpp>
#include <Eigen/Dense>
// #include <ctpl_stl.h>

/// Dynamic Model of the OpenWrist
class MeiiModel {
public:

    MeiiModel();

    void update(mahi::util::Time t);
    void set_torques(double tau1, double tau2, double tau3, double tau4, double tau5);
    void set_positions(double _q1, double _q2, double _q3, double _q4, double _q5, double _q6, double _q7, double _q8, double _q9, double _q10, double _q11, double _q12, double _q13, double _q14);
    void set_velocities(double _q1d, double _q2d, double _q3d, double _q4d, double _q5d, double _q6d, double _q7d, double _q8d, double _q9d, double _q10d, double _q11d, double _q12d, double _q13d, double _q14d);
    void reset();
    void calc_dependent_joint_values();

public:

    bool threadpool = true;

    // Joint torques [Nm]
    double tau1, tau2, tau3, tau4, tau5;
    // Joint Positions [m]
    double   q1,   q2,   q3,   q4,   q5;
    // Joint Velocities [m/s]
    double  q1d,  q2d,  q3d,  q4d,  q5d;
    // Joint Accelerations [m/s^2]
    double q1dd, q2dd, q3dd, q4dd, q5dd;

    // Dependent joint positions [rad]
    // q5 -> theta1
    // q6 -> theta2
    // q7 -> theta3
    // q8 -> Px
    // q9 -> Py
    // q10 -> Pz
    // q11 -> alpha (Rx)
    // q12 -> beta (Ry)
    // q13 -> gamma (Rz)
    double q6, q7, q8, q9, q10, q11, q12, q13, q14;
    // Dependent joint velocities [rad/s]
    double q6d, q7d, q8d, q9d, q10d, q11d, q12d, q13d, q14d;

    // Hardstops
    const double q1min = -91.5 * mahi::util::DEG2RAD;
    const double q1max = 3.0 * mahi::util::DEG2RAD;
    const double q2min = -99 * mahi::util::DEG2RAD;
    const double q2max = 108 * mahi::util::DEG2RAD;
    const double q3min = 0.050;
    const double q4min = 0.050;
    const double q5min = 0.050;
    const double q3max = 0.1305;
    const double q4max = 0.1305;
    const double q5max = 0.1305;

    //// Hardstops if you want to see the effect of gravity without instablities
    // const double q3min = 0.09;
    // const double q4min = 0.09;
    // const double q5min = 0.09;
    // const double q3max = 0.110;
    // const double q4max = 0.110;
    // const double q5max = 0.110;

    double Khard = 20000; // hardstop stiffness
    double Bhard = 100;  // hardstop damping
    double Khard1 = 2000; // hardstop stiffness
    double Bhard1 = 100;  // hardstop damping
    double mat_calc_time = 0;
    double setup_time = 0;
    double comp_time = 0;

    const double R = 0.1044956; // m
    const double r = 0.05288174521; // m
    const double a4 = 0.159385;
    const double a5 = 0.0268986; // m
    const double a6 = 0.0272820; // m
    const double a56 = -(a5-a6); // m
    const double alpha_5 = 0.094516665; // rad
    const double alpha_13 = 5*mahi::util::PI/180; // rad

    /// Motor continous and max torque limits [Nm]
    const double tau1_mot_cont = 0.187; 
    const double tau1_mot_max  = 2.560;
    const double tau2_mot_cont = 0.187; 
    const double tau2_mot_max  = 2.560;
    const double tau3_mot_cont = 0.0897; 
    const double tau3_mot_max  = 1.050;
    const double tau4_mot_cont = 0.187; 
    const double tau4_mot_max  = 2.560;
    const double tau5_mot_cont = 0.0897; 
    const double tau5_mot_max  = 1.050;

    // Transmission Rations [rad/m]
    const double eta1 = 0.23*0.0254;
    const double eta2 = 0.23*0.0254;
    const double eta3 = 0.23*0.0254;
    const double eta4 = 0.23*0.0254;
    const double eta5 = 0.23*0.0254;

    // Damping Coefficients [Nm*s/rad]
    const double b1 = 0.5 * 0.0252;  
    const double b2 = 0.5 * 0.0019;  
    const double b3 = 0.5 * 0.0029; 
    const double b4 = 0.5 * 0.0019;  
    const double b5 = 0.5 * 0.0029; 

    // Kinetic Friction [Nm]
    const double fk1 = 0.5 * 0.1891;
    const double fk2 = 0.5 * 0.0541;
    const double fk3 = 0.5 * 0.1339;
    const double fk4 = 0.5 * 0.1339;
    const double fk5 = 0.5 * 0.1339;

    // Gravity Constant [m/s^2]
    const double g = 9.80665;

    // ctpl::thread_pool p;

    bool logged = false;

private:
    // torque limiters
    mahi::robo::Limiter lim1, lim2, lim3, lim4, lim5;

    // Integrators
    mahi::util::Integrator q1dd_q1d, q2dd_q2d, q3dd_q3d, q4dd_q4d, q5dd_q5d, q1d_q1, q2d_q2, q3d_q3, q4d_q4, q5d_q5;

    Eigen::VectorXd Tau;
    Eigen::VectorXd B;
    Eigen::VectorXd Fk;

    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    Eigen::VectorXd x;

};