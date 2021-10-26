#pragma once
#include <Mahi/Util/Timing/Time.hpp>
#include <Mahi/Util/Math/Constants.hpp>
#include <Mahi/Util/Math/Integrator.hpp>
#include <Mahi/Robo/Control/Limiter.hpp>
#include <Eigen/Dense>
// #include <ctpl_stl.h>

/// Dynamic Model of the OpenWrist
class MoeModel {
public:

    MoeModel();

    void update(mahi::util::Time t);
    void set_torques(double tau0_, double tau1_, double tau2_, double tau3_);
    void set_positions(double q0_, double q1_, double q2_, double q3_);
    void set_velocities(double q0d_, double q1d_, double q2d_, double q3d_);
    void reset();

public:

    bool threadpool = true;

    // Joint torques [Nm]
    double tau0, tau1, tau2, tau3;
    // Joint Positions [m]
    double   q0,   q1,   q2,   q3;
    // Joint Velocities [m/s]
    double  q0d,  q1d,  q2d,  q3d;
    // Joint Accelerations [m/s^2]
    double q0dd, q1dd, q2dd, q3dd;

    // Hardstops
    const double q0min = -73.6285 * mahi::util::DEG2RAD;
    const double q0max = +28.6283 * mahi::util::DEG2RAD;
    const double q1min = -89.8249 * mahi::util::DEG2RAD;
    const double q1max = +89.8249 * mahi::util::DEG2RAD;
    const double q2min = -63.2490 * mahi::util::DEG2RAD;
    const double q2max = +68.2490 * mahi::util::DEG2RAD;
    const double q3min = -42.9322 * mahi::util::DEG2RAD;
    const double q3max = +35.0000 * mahi::util::DEG2RAD;

    double Khard0 = 100; // hardstop stiffness
    double Bhard0 = 1.0;  // hardstop damping
    double Khard1 = 100; // hardstop stiffness
    double Bhard1 = 1.0;  // hardstop damping
    double Khard2 = 100; // hardstop stiffness
    double Bhard2 = 1.0;  // hardstop damping
    double Khard3 = 100; // hardstop stiffness
    double Bhard3 = 1.0;  // hardstop damping

    double mat_calc_time = 0;
    double setup_time = 0;
    double comp_time = 0;

    /// Motor continous and max torque limits [Nm]
    const double tau0_mot_cont = 0.187; 
    const double tau0_mot_max  = 2.560;
    const double tau1_mot_cont = 0.187; 
    const double tau1_mot_max  = 2.560;
    const double tau2_mot_cont = 0.0897; 
    const double tau2_mot_max  = 1.050;
    const double tau3_mot_cont = 0.187; 
    const double tau3_mot_max  = 2.560;

    // Transmission Rations [rad/m]
    const double eta0 = 0.23*0.0254;
    const double eta1 = 0.23*0.0254;
    const double eta2 = 0.23*0.0254;
    const double eta3 = 0.23*0.0254;

    // Damping Coefficients [Nm*s/rad]
    // const double b0 = 0.5 * 0.0252;  
    // const double b1 = 0.5 * 0.0019;  
    // const double b2 = 0.5 * 0.0029; 
    // const double b3 = 0.5 * 0.0019;  

    // Kinetic Friction [Nm]
    // const double fk0 = 0.5 * 0.1891;
    // const double fk1 = 0.5 * 0.0541;
    // const double fk2 = 0.5 * 0.1339;
    // const double fk3 = 0.5 * 0.1339;

    // Gravity Constant [m/s^2]
    const double g = 9.80665;

    // ctpl::thread_pool p;

    bool logged = false;

private:
    // torque limiters
    mahi::robo::Limiter lim0, lim1, lim2, lim3;

    // Integrators
    mahi::util::Integrator q0dd_q0d, q1dd_q1d, q2dd_q2d, q3dd_q3d;
    mahi::util::Integrator q0d_q0, q1d_q1, q2d_q2, q3d_q3;

    Eigen::VectorXd Tau;
    Eigen::VectorXd B;
    Eigen::VectorXd Fk;

    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    Eigen::VectorXd x;

};