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

constexpr int n_j = 4;

std::vector<double> q(n_j,0);
std::vector<double> qd(n_j,0);
std::vector<double> qdd(n_j,0);

// compute hardstop torque if robot is in an unavailable range
inline double hardstop_torque(moe::MoeDynamicModel moe_model, int joint_id) {
    static const double hs_K = 100;
    static const double hs_B = 0.1;

    double qmin = moe_params.pos_limits_min_[joint_id];
    double qmax = moe_params.pos_limits_max_[joint_id];

    if      (q[joint_id] < qmin) return hs_K * (qmin - q[joint_id]) - hs_B * qd[joint_id];
    else if (q[joint_id] > qmax) return hs_K * (qmax - q[joint_id]) - hs_B * qd[joint_id];
    else               return 0;
}

void simulation()
{
    std::vector<mahi::util::Integrator> qdd_qd, qd_q;
    for (int i = 0; i < n_j; i++) {
        qdd_qd.push_back(mahi::util::Integrator(0));
        qd_q.push_back(mahi::util::Integrator(0));
    }

    model.set_user_params({3,4,0});
    A        = MatrixXd::Zero(n_j, n_j);
    M        = MatrixXd::Zero(n_j, n_j);
    b        = VectorXd::Zero(n_j);
    V        = VectorXd::Zero(n_j);
    G        = VectorXd::Zero(n_j);
    Tau      = VectorXd::Zero(n_j);
    Friction = VectorXd::Zero(n_j);
    x        = VectorXd::Zero(n_j);
    xd       = VectorXd::Zero(n_j);

    bool started = false;
    MelShare ms_torque_0("ms_torque_0");
    MelShare ms_torque_1("ms_torque_1");
    MelShare ms_torque_2("ms_torque_2");
    MelShare ms_torque_3("ms_torque_3");
    MelShare ms_posvel_0("ms_posvel_0");
    MelShare ms_posvel_1("ms_posvel_1");
    MelShare ms_posvel_2("ms_posvel_2");
    MelShare ms_posvel_3("ms_posvel_3");
    std::vector<double> taus(n_j,0);
    std::vector<std::vector<double>> tau_datas;
    for (int i = 0; i < n_j; i++) tau_datas.push_back(std::vector<double>());
    mahi::util::Timer timer(mahi::util::hertz(1000), mahi::util::Timer::Hybrid);
    mahi::util::Time t;
    mahi::util::enable_realtime();
    while (!sim_stop)
    {
        if(!started){
            if(!ms_torque_0.read_data().empty()){
                started = true;
                std::lock_guard<std::mutex> lock(mtx);
                model.update(std::vector<double>(4, 0), std::vector<double>(4, 0));
                for (auto i = 0; i < n_j; i++) qdd_qd[i] = mahi::util::Integrator(0.0);
                for (auto i = 0; i < n_j; i++) qd_q[i] = mahi::util::Integrator(0.0);
                t = mahi::util::Time::Zero;
                timer.restart();
            }
        }

        tau_datas[0] = ms_torque_0.read_data();
        tau_datas[1] = ms_torque_1.read_data();
        tau_datas[2] = ms_torque_2.read_data();
        tau_datas[3] = ms_torque_3.read_data();

        for(auto i = 0; i < n_j; i++) {
            taus[i] = !tau_datas[i].empty() ? tau_datas[i][0] : 0.0;
        }

        {
            std::lock_guard<std::mutex> lock(mtx);
            // model.set_torques(tau0,tau1,tau2,tau3);
            if(started){
                Tau[0] = taus[0] + hardstop_torque(model,0);
                Tau[1] = taus[1] + hardstop_torque(model,1);
                Tau[2] = taus[2] + hardstop_torque(model,2);
                Tau[3] = taus[3] + hardstop_torque(model,3);

                // constexpr double  B_coef[4] = {0.0393, 0.0691, 0.0068, 0.0025};
                // constexpr double Fk_coef[4] = {0.1838, 0.1572, 0.0996, 0.1685};

                // Friction[0] = B_coef[0]*qd[0]*1.0 + Fk_coef[0]*std::tanh(qd[0]*10);
                // Friction[1] = B_coef[1]*qd[1]*1.0 + Fk_coef[1]*std::tanh(qd[1]*10);
                // Friction[2] = B_coef[2]*qd[2]*1.0 + Fk_coef[2]*std::tanh(qd[2]*10);
                // Friction[3] = B_coef[3]*qd[3]*1.0 + Fk_coef[3]*std::tanh(qd[3]*10);

                M = model.get_M();
                V = model.get_V();
                G = model.get_G();
                Friction = model.get_Friction();
                auto A = M;
                auto b = Tau - V - G - Friction;
                x = A.inverse()*b;

                for (auto i = 0; i < n_j; i++){
                    qdd[i] = x(i);
                    qd[i]  = qdd_qd[i].update(qdd[i], t);
                    q[i]   = qd_q[i].update(qd[i], t);
                }

                model.update(q, qd);
            }

        }

        ms_posvel_0.write_data({q[0],qd[0]});
        ms_posvel_1.write_data({q[1],qd[1]});
        ms_posvel_2.write_data({q[2],qd[2]});
        ms_posvel_3.write_data({q[3],qd[3]});
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
    q = std::vector<double>(n_j, 0.0);
    qd = std::vector<double>(n_j, 0.0);
    model.update(q,qd);
    sim_stop = false;
    thread = std::thread(simulation);
}

EXPORT void get_positions(double *positions)
{
    std::lock_guard<std::mutex> lock(mtx);
    positions[0] = q[0];
    positions[1] = q[1];
    positions[2] = q[2];
    positions[3] = q[3];
}

EXPORT void update_mass_props(int forearm_pos, int counterweight_pos, double shoulder_pos)
{
    std::lock_guard<std::mutex> lock(mtx);
    model.set_user_params({forearm_pos, counterweight_pos, shoulder_pos});
}