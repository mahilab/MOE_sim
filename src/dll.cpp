#include <dll.hpp>

// This is the dll that drives the model in the Unity OpenWristSim visualization
// Unity intefaces via the the C methods below, mostly just to start/stop the simulation, and read joint positions
// End-users inteface through the MelShares or at a higher level, through OpenWristSim.hpp/cpp in their own C++ code
// This was all sort of hastily put together...it could be better!

using namespace mahi::com;
using namespace mahi::util;
using namespace mahi::robo;

void simulation()
{
    enable_realtime();
    MelShare ms_gains("gains");
    MelShare ms_refs("refs");
    MelShare ms_times_out("times");
    MelShare ms_qs_out("qs");
    double kp0 = 125.0;
    double kd0 = 1.75;
    double kp1 = 25;
    double kd1 = 1.15;
    double kp2 = 20.0;
    double kd2 = 1.0;
    double kp3 = 20.0;
    double kd3 = 0.25;
    double q_ref0 = 0.0;
    double q_ref1 = 0.0;
    double q_ref2 = 0.0;
    double q_ref3 = 0.0;
    double q0, q1, q2, q3;
    double tau0, tau1, tau2, tau3;
    double k_hard0 = 200;
    double b_hard0 = 10;
    double threadpooling = true;
    Timer timer(hertz(1000), Timer::Hybrid);
    Time t;
    Time t_last;
    Time sim_time = 0_ms;
    double comp_time = 0;
    while (!g_stop)
    {
        auto ms_gain_data = ms_gains.read_data();
        auto ms_ref_data = ms_refs.read_data();
        if (!ms_gain_data.empty()){
            kp0 = ms_gain_data[0];
            kd0 = ms_gain_data[1];
            kp1 = ms_gain_data[2];
            kd1 = ms_gain_data[3];
            kp2 = ms_gain_data[4];
            kd2 = ms_gain_data[5];
            kp3 = ms_gain_data[6];
            kd3 = ms_gain_data[7];
            k_hard0 = ms_gain_data[8];
            b_hard0 = ms_gain_data[9];

            q_ref0 = ms_ref_data[0];
            q_ref1 = ms_ref_data[1];
            q_ref2 = ms_ref_data[2]; 
            q_ref3 = ms_ref_data[3]; 
        }
        {
            std::lock_guard<std::mutex> lock(g_mtx);
            tau0 = kp0 * (q_ref0 - g_model.q0) - kd0 * g_model.q0d;
            tau1 = kp1 * (q_ref1 - g_model.q1) - kd1 * g_model.q1d;
            tau2 = kp2 * (q_ref2 - g_model.q2) - kd2 * g_model.q2d;
            tau3 = kp3 * (q_ref3 - g_model.q3) - kd3 * g_model.q3d;
            g_model.Khard0 = k_hard0;
            g_model.Bhard0 = b_hard0;
            g_model.set_torques(tau0,tau1,tau2,tau3);
            g_model.update(sim_time);
            q0 = g_model.q0;
            q1 = g_model.q1;
            q2 = g_model.q2;
            q3 = g_model.q3;
            comp_time = g_model.comp_time;
        }
        sim_time += 1_ms;
        ms_times_out.write_data({double((t-t_last).as_microseconds()),comp_time});
        ms_qs_out.write_data({tau0,tau1,tau2,tau3,q0,q1,q2,q3});
        t_last = t;
        t = timer.wait();
    }
    disable_realtime();
}

void stop()
{
    g_stop = true;
    if (g_thread.joinable())
        g_thread.join();
}

void start()
{
    // LOG(Info) << "Starting";
    stop();
    g_model.reset();
    g_stop = false;
    g_thread = std::thread(simulation);
}

void set_torques(double tau0, double tau1, double tau2, double tau3)
{
    std::lock_guard<std::mutex> lock(g_mtx);
    g_model.set_torques(tau0, tau1, tau2, tau3);
}

void set_positions(double q0, double q1, double q2, double q3)
{
    std::lock_guard<std::mutex> lock(g_mtx);
    g_model.set_positions(q0, q1, q2, q3);
}

void set_velocities(double q0d, double q1d, double q2d, double q3d)
{
    std::lock_guard<std::mutex> lock(g_mtx);
    g_model.set_velocities(q0d, q1d, q2d, q3d);
}

void get_positions(double *positions)
{
    std::lock_guard<std::mutex> lock(g_mtx);
    positions[0] = g_model.q0;
    positions[1] = g_model.q1;
    positions[2] = g_model.q2;
    positions[3] = g_model.q3;
}