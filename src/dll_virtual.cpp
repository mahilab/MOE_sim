#include "MeiiModel.hpp"
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

MeiiModel g_model;
std::thread g_thread;
std::mutex g_mtx;
std::atomic_bool g_stop;

using namespace mahi::com;
using namespace mahi::util;
using namespace mahi::robo;

void simulation()
{
    bool started = false;
    MelShare ms_torque_1("ms_torque_1");
    MelShare ms_torque_2("ms_torque_2");
    MelShare ms_torque_3("ms_torque_3");
    MelShare ms_torque_4("ms_torque_4");
    MelShare ms_torque_5("ms_torque_5");
    MelShare ms_posvel_1("ms_posvel_1");
    MelShare ms_posvel_2("ms_posvel_2");
    MelShare ms_posvel_3("ms_posvel_3");
    MelShare ms_posvel_4("ms_posvel_4");
    MelShare ms_posvel_5("ms_posvel_5");
    double q1, q2, q3, q4, q5;
    double q1d, q2d, q3d, q4d, q5d;
    double tau1, tau2, tau3, tau4, tau5;
    std::vector<double> tau1_data, tau2_data, tau3_data, tau4_data, tau5_data;
    Timer timer(hertz(1000), Timer::Hybrid);
    Time t;
    Time sim_time = 0_ms;
    enable_realtime();
    while (!g_stop)
    {
        if(!started){
            if(!ms_torque_1.read_data().empty()){
                started = true;
                std::lock_guard<std::mutex> lock(g_mtx);
                g_model.reset();
                timer.restart();
            }
        }

        tau1_data = ms_torque_1.read_data();
        tau2_data = ms_torque_2.read_data();
        tau3_data = ms_torque_3.read_data();
        tau4_data = ms_torque_4.read_data();
        tau5_data = ms_torque_5.read_data();

        tau1 = !tau1_data.empty() ? tau1_data[0] : 0.0;
        tau2 = !tau2_data.empty() ? tau2_data[0] : 0.0;
        tau3 = !tau3_data.empty() ? tau3_data[0] : 0.0;
        tau4 = !tau4_data.empty() ? tau4_data[0] : 0.0;
        tau5 = !tau5_data.empty() ? tau5_data[0] : 0.0;

        {
            std::lock_guard<std::mutex> lock(g_mtx);
            g_model.set_torques(tau1,tau2,tau3,tau4,tau5);
            if(started) g_model.update(sim_time);
            q1 = g_model.q1;
            q2 = g_model.q2;
            q3 = g_model.q3;
            q4 = g_model.q4;
            q5 = g_model.q5;
            q1d = g_model.q1d;
            q2d = g_model.q2d;
            q3d = g_model.q3d;
            q4d = g_model.q4d;
            q5d = g_model.q5d;
        }
        sim_time += 1_ms;
        ms_posvel_1.write_data({q1,q1d});
        ms_posvel_2.write_data({q2,q2d});
        ms_posvel_3.write_data({q3,q3d});
        ms_posvel_4.write_data({q4,q4d});
        ms_posvel_5.write_data({q5,q5d});
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

EXPORT void set_torques(double tau1, double tau2, double tau3, double tau4, double tau5)
{
    std::lock_guard<std::mutex> lock(g_mtx);
    g_model.set_torques(tau1, tau2, tau3, tau4, tau5);
}

EXPORT void set_positions(double q1, double q2, double q3, double q4, double q5, double q6, double q7, double q8, double q9, double q10, double q11, double q12, double q13, double q14)
{
    std::lock_guard<std::mutex> lock(g_mtx);
    g_model.set_positions(q1, q2, q3, q4, q5, q6, q7, q8, q9, q10, q11, q12, q13, q14);
}

EXPORT void set_velocities(double q1d, double q2d, double q3d, double q4d, double q5d, double q6d, double q7d, double q8d, double q9d, double q10d, double q11d, double q12d, double q13d, double q14d)
{
    std::lock_guard<std::mutex> lock(g_mtx);
    g_model.set_velocities(q1d, q2d, q3d, q4d, q5d, q6d, q7d, q8d, q9d, q10d, q11d, q12d, q13d, q14d);
}

EXPORT void get_positions(double *positions)
{
    std::lock_guard<std::mutex> lock(g_mtx);
    positions[0] = g_model.q1;
    positions[1] = g_model.q2;
    positions[2] = g_model.q3;
    positions[3] = g_model.q4;
    positions[4] = g_model.q5;
    positions[5] = g_model.q6;
    positions[6] = g_model.q7;
    positions[7] = g_model.q8;
    positions[8] = g_model.q9;
    positions[9] = g_model.q10;
    positions[10] = g_model.q11;
    positions[11] = g_model.q12;
    positions[12] = g_model.q13;
    positions[13] = g_model.q14;
}