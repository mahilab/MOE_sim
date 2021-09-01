#pragma once

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

void simulation();

EXPORT void stop();

EXPORT void start();

EXPORT void set_torques(double tau0, double tau1, double tau2, double tau3);

EXPORT void set_positions(double q0, double q1, double q2, double q3);

EXPORT void set_velocities(double q0d, double q1d, double q2d, double q3d);

EXPORT void get_positions(double *positions);