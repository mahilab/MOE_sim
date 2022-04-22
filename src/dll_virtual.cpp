#include "MoeModel.hpp"
#include <Mahi/MOE.hpp>
#include <Mahi/Com.hpp>
#include <Mahi/Util.hpp>
#include <Mahi/Robo.hpp>
#include <thread>
#include <mutex>
#include <atomic>

// This is the dll that drives the model in the MOE visualization
// Unity intefaces via the the C methods below, mostly just to start/stop the simulation, and read joint positions

#define EXPORT extern "C" __declspec(dllexport)

using namespace mahi::com;
using namespace mahi::util;
using namespace mahi::robo;

using Eigen::MatrixXd;
using Eigen::VectorXd;

moe::MoeDynamicModel model;
moe::MoeParams moe_params;
std::thread thread;
std::mutex mtx;
std::atomic_bool stop;

MatrixXd A, M;
VectorXd b, V, G, Tau, Friction;

VectorXd x, xd;

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
    double q0d, q1d, q2d, q3d;
    double tau0, tau1, tau2, tau3;
    std::vector<double> tau0_data, tau1_data, tau2_data, tau3_data;
    Timer timer(hertz(1000), Timer::Hybrid);
    Time t;
    Time sim_time = 0_ms;
    enable_realtime();
    while (!stop)
    {
        if(!started){
            if(!ms_torque_0.read_data().empty()){
                started = true;
                std::lock_guard<std::mutex> lock(mtx);
                model.reset();
                timer.stop();
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
            model.set_torques(tau0,tau1,tau2,tau3);
            if(started){
                Tau[0] = tau0 + hardstop_torque(moe_model,0);
                Tau[1] = tau1 + hardstop_torque(moe_model,1);
                Tau[2] = tau2 + hardstop_torque(moe_model,2);
                Tau[3] = tau3 + hardstop_torque(moe_model,3);

                M = model.get_M();
                V = model.get_V();
                G = model.get_G();
                Friction = model.get_Friction();
                auto A = M;
                auto B = Tau - V*qdot - G - B - Friction;
                x = A.inverse()*b;
            }
            q0 = model.q[0];
            q1 = model.q[1];
            q2 = model.q[2];
            q3 = model.q[3];
            q0d = model.qd[0];
            q1d = model.qd[1];
            q2d = model.qd[2];
            q3d = model.qd[3];
            
        }
        sim_time += 1_ms;
        ms_posvel_0.write_data({q0,q0d});
        ms_posvel_1.write_data({q1,q1d});
        ms_posvel_2.write_data({q2,q2d});
        ms_posvel_3.write_data({q3,q3d});
        t = timer.wait();
    }
    disable_realtime();
}

EXPORT void stop()
{
    stop = true;
    if (thread.joinable())
        thread.join();
}

EXPORT void start()
{
    stop();
    model.update({0,0,0,0},{0,0,0,0});
    stop = false;
    thread = std::thread(simulation);
}

EXPORT void get_positions(double *positions)
{
    std::lock_guard<std::mutex> lock(mtx);
    positions[0] = model.q0;
    positions[1] = model.q1;
    positions[2] = model.q2;
    positions[3] = model.q3;
}

// private functions that arent available from unity

// compute hardstop torque if robot is in an unavailable range
inline double hardstop_torque(MoeDynamicModel moe_model, int joint_id) {
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