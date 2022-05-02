// #include "MoeModel.hpp"
#include <MOE/MOE.hpp>
#include <Mahi/Com.hpp>
#include <Mahi/Util.hpp>
#include <Mahi/Robo.hpp>
#include <thread>
#include <mutex>
#include <atomic>

// This is the dll that drives the model in the MOE visualization
// Unity intefaces via the the C methods below, mostly just to start/stop the simulation, and read joint positions

#define EXPORT extern "C" __declspec(dllexport)

// private functions that arent available from unity



using namespace mahi::com;
// using namespace mahi::util;
using namespace mahi::robo;

using Eigen::MatrixXd;
using Eigen::VectorXd;

moe::MoeDynamicModel model;
moe::MoeParameters moe_params;
std::thread thread;
std::mutex mtx;
std::atomic_bool sim_stop;

MatrixXd A, M;
VectorXd b, V, G, Tau, Friction;

VectorXd x, xd;

mahi::util::Integrator q0dd_q0d, q1dd_q1d, q2dd_q2d, q3dd_q3d;
mahi::util::Integrator q0d_q0, q1d_q1, q2d_q2, q3d_q3;

// compute hardstop torque if robot is in an unavailable range
inline double hardstop_torque(moe::MoeDynamicModel moe_model, int joint_id) {
    static const double hs_K = 100;
    static const double hs_B = 0.1;

    double qmin = moe_params.pos_limits_min_[joint_id];
    double qmax = moe_params.pos_limits_max_[joint_id];
    double q = moe_model.q[joint_id];
    double qd = moe_model.qd[joint_id];

    if      (q < qmin) return hs_K * (qmin - q) - hs_B * qd;
    else if (q > qmax) return hs_K * (qmax - q) - hs_B * qd;
    else               return 0;
}

void simulation()
{
    A        = MatrixXd::Zero(4, 4);
    M        = MatrixXd::Zero(4, 4);
    b        = VectorXd::Zero(4);
    V        = VectorXd::Zero(4);
    G        = VectorXd::Zero(4);
    Tau      = VectorXd::Zero(4);
    Friction = VectorXd::Zero(4);
    x        = VectorXd::Zero(4);
    xd       = VectorXd::Zero(4);

    bool started = false;
    MelShare ms_torque_0("ms_torque_0");
    MelShare ms_torque_1("ms_torque_1");
    MelShare ms_torque_2("ms_torque_2");
    MelShare ms_torque_3("ms_torque_3");
    MelShare ms_posvel_0("ms_posvel_0");
    MelShare ms_posvel_1("ms_posvel_1");
    MelShare ms_posvel_2("ms_posvel_2");
    MelShare ms_posvel_3("ms_posvel_3");
    double q0, q1, q2, q3;
    q0 = q1 = q2 = q3 = 0.0;
    double q0d, q1d, q2d, q3d;
    q0d = q1d = q2d = q3d = 0.0;
    double q0dd, q1dd, q2dd, q3dd;
    q0dd = q1dd = q2dd = q3dd = 0.0;
    double tau0, tau1, tau2, tau3;
    tau0 = tau1 = tau2 = tau3 = 0.0;
    std::vector<double> tau0_data, tau1_data, tau2_data, tau3_data;
    mahi::util::Timer timer(mahi::util::hertz(1000), mahi::util::Timer::Hybrid);
    mahi::util::Time t;
    mahi::util::Time sim_time = mahi::util::milliseconds(0);
    mahi::util::enable_realtime();
    while (!sim_stop)
    {
        if(!started){
            if(!ms_torque_0.read_data().empty()){
                started = true;
                std::lock_guard<std::mutex> lock(mtx);
                model.update(std::vector<double>(4, 0), std::vector<double>(4, 0));
                q0dd_q0d = q1dd_q1d = q2dd_q2d = q3dd_q3d = mahi::util::Integrator(0.0);
                q0d_q0 = q1d_q1 = q2d_q2 = q3d_q3 = mahi::util::Integrator(0.0);
                t = mahi::util::Time::Zero;
                timer.restart();
            }
        }

        tau0_data = ms_torque_0.read_data();
        tau1_data = ms_torque_1.read_data();
        tau2_data = ms_torque_2.read_data();
        tau3_data = ms_torque_3.read_data();

        tau0 = !tau0_data.empty() ? tau0_data[0] : 0.0;
        tau1 = !tau1_data.empty() ? tau1_data[0] : 0.0;
        tau2 = !tau2_data.empty() ? tau2_data[0] : 0.0;
        tau3 = !tau3_data.empty() ? tau3_data[0] : 0.0;

        {
            std::lock_guard<std::mutex> lock(mtx);
            // model.set_torques(tau0,tau1,tau2,tau3);
            if(started){
                Tau[0] = tau0 + hardstop_torque(model,0);
                Tau[1] = tau1 + hardstop_torque(model,1);
                Tau[2] = tau2 + hardstop_torque(model,2);
                Tau[3] = tau3 + hardstop_torque(model,3);

                constexpr double  B_coef[4] = {0.0393, 0.0691, 0.0068, 0.0025};
                constexpr double Fk_coef[4] = {0.1838, 0.1572, 0.0996, 0.1685};

                Friction[0] = B_coef[0]*q0d*1.0 + Fk_coef[0]*std::tanh(q0d*10);
                Friction[1] = B_coef[1]*q1d*1.0 + Fk_coef[1]*std::tanh(q1d*10);
                Friction[2] = B_coef[2]*q2d*1.0 + Fk_coef[2]*std::tanh(q2d*10);
                Friction[3] = B_coef[3]*q3d*1.0 + Fk_coef[3]*std::tanh(q3d*10);

                M = model.get_M();
                V = model.get_V();
                G = model.get_G();
                // Friction = model.get_Frisction();
                auto A = M;
                auto b = Tau - V - G - Friction;
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

                model.update({q0, q1, q2, q3}, {q0d, q1d, q2d, q3d});
            }

        }

        sim_time += mahi::util::milliseconds(1);
        ms_posvel_0.write_data({q0,q0d});
        ms_posvel_1.write_data({q1,q1d});
        ms_posvel_2.write_data({q2,q2d});
        ms_posvel_3.write_data({q3,q3d});
        t = timer.wait();
    }
    mahi::util::disable_realtime();
}

EXPORT void stop()
{
    sim_stop = true;
    if (thread.joinable())
        thread.join();
}

EXPORT void start()
{
    stop();
    model.update({0,0,0,0},{0,0,0,0});
    sim_stop = false;
    q0dd_q0d = q1dd_q1d = q2dd_q2d = q3dd_q3d = mahi::util::Integrator(0.0);
    q0d_q0 = q1d_q1 = q2d_q2 = q3d_q3 = mahi::util::Integrator(0.0);
    thread = std::thread(simulation);
}

EXPORT void get_positions(double *positions)
{
    std::lock_guard<std::mutex> lock(mtx);
    positions[0] = model.q[0];
    positions[1] = model.q[1];
    positions[2] = model.q[2];
    positions[3] = model.q[3];
}

