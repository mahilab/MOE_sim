#pragma once

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

void simulation();

EXPORT void stop();

EXPORT void start();

EXPORT void set_torques(double tau1, double tau2, double tau3, double tau4, double tau5);

EXPORT void set_positions(double q1, double q2, double q3, double q4, double q5, double q6, double q7, double q8, double q9, double q10, double q11, double q12, double q13, double q14);

EXPORT void set_velocities(double q1d, double q2d, double q3d, double q4d, double q5d, double q6d, double q7d, double q8d, double q9d, double q10d, double q11d, double q12d, double q13d, double q14d);

EXPORT void get_positions(double *positions);