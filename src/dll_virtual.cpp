#include "MoeModel.hpp"
#include <Mahi/Com.hpp>
#include <Mahi/Util.hpp>
#include <Mahi/Robo.hpp>
#include <thread>
#include <mutex>
#include <atomic>

// This is the dll that drives the model in the Unity OpenWristSim visualization
// Unity intefaces via the the C methods below, mostly just to start/stop the simulation, and read joint positions
// End-users inteface through the MelShares or at a higher level, through OpenWristSim.hpp/cpp in their own C++ code
// This was all sort of hastily put together...it could be better!

#define EXPORT extern "C" __declspec(dllexport)

MoeModel g_model;
std::thread g_thread;
std::mutex g_mtx;
std::atomic_bool g_stop;

using namespace mahi::com;
using namespace mahi::util;
using namespace mahi::robo;

void simulation()
{
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
    while (!g_stop)
    {
        if(!started){
            if(!ms_torque_0.read_data().empty()){
                started = true;
                std::lock_guard<std::mutex> lock(g_mtx);
                g_model.reset();
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
            std::lock_guard<std::mutex> lock(g_mtx);
            g_model.set_torques(tau0,tau1,tau2,tau3);
            if(started) g_model.update(sim_time);
            q0 = g_model.q0;
            q1 = g_model.q1;
            q2 = g_model.q2;
            q3 = g_model.q3;
            q0d = g_model.q0d;
            q1d = g_model.q1d;
            q2d = g_model.q2d;
            q3d = g_model.q3d;
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
    g_stop = true;
    if (g_thread.joinable())
        g_thread.join();
}

EXPORT void start()
{
    stop();
    g_model.reset();
    g_stop = false;
    g_thread = std::thread(simulation);
}

EXPORT void set_torques(double tau0, double tau1, double tau2, double tau3)
{
    std::lock_guard<std::mutex> lock(g_mtx);
    g_model.set_torques(tau0, tau1, tau2, tau3);
}

EXPORT void set_positions(double q0, double q1, double q2, double q3)
{
    std::lock_guard<std::mutex> lock(g_mtx);
    g_model.set_positions(q0, q1, q2, q3);
}

EXPORT void set_velocities(double q0d, double q1d, double q2d, double q3d)
{
    std::lock_guard<std::mutex> lock(g_mtx);
    g_model.set_velocities(q0d, q1d, q2d, q3d);
}

EXPORT void get_positions(double *positions)
{
    std::lock_guard<std::mutex> lock(g_mtx);
    positions[0] = g_model.q0;
    positions[1] = g_model.q1;
    positions[2] = g_model.q2;
    positions[3] = g_model.q3;
}