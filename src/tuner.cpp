#define MAHI_GUI_NO_CONSOLE
#include <Mahi/Gui.hpp>
#include <Mahi/Util.hpp>
#include <Mahi/Com.hpp>
#include <thread>
#include <mutex>
#include <atomic>

using namespace mahi::gui;
using namespace mahi::util;
using namespace mahi::com;

struct ScrollingData {
    int MaxSize = 1000;
    int Offset  = 0;
    ImVector<ImVec2> Data;
    ScrollingData() { Data.reserve(MaxSize); }
    void AddPoint(float x, float y) {
        if (Data.size() < MaxSize)
            Data.push_back(ImVec2(x,y));
        else {
            Data[Offset] = ImVec2(x,y);
            Offset =  (Offset + 1) % MaxSize;
        }
    }
};

class SimTuner : public Application {
public:
    /// Constructor
    SimTuner() : Application(500,500,"Sim Tuner"),
    calc_times_1s(300)
    { 
        sim_rate.AddPoint(t,0);
        des_rate.AddPoint(t,0);
        tau0.AddPoint(t,0);
        tau1.AddPoint(t,0);
        tau2.AddPoint(t,0);
        tau3.AddPoint(t,0);
    }

    ~SimTuner() {
         stop = true;
     }

    /// Override update from Application, called once per frame
    void update() override
    {
        std::vector<double> ms_times_data = ms_times.read_data();
        std::vector<double> ms_qs_data = ms_qs.read_data();
        if(!ms_times_data.empty()) {
            sim_rate.AddPoint(t,ms_times_data[0]);
            des_rate.AddPoint(t,1000.0f);
            compTime.AddPoint(t,ms_times_data[1]);
            calc_times_1s.push_back(ms_times_data[1]);

            tau0.AddPoint(t,ms_qs_data[0]);
            tau1.AddPoint(t,ms_qs_data[1]);
            tau2.AddPoint(t,ms_qs_data[2]);
            tau3.AddPoint(t,ms_qs_data[3]);
    
            q0 = ms_qs_data[4];
            q1 = ms_qs_data[5];
            q2 = ms_qs_data[6];
            q3 = ms_qs_data[7];
        }
        ImGui::Begin("PD Tuner");
        ImGui::BeginGroup();
            ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.3f);
            ImGui::DragDouble("Kp Elbow",&kp0,1,0,250);
            ImGui::DragDouble("Kd Elbow",&kd0,1,0,10);
            ImGui::DragDouble("Kp Forearm",&kp1,1,0,50);
            ImGui::DragDouble("Kd Forearm",&kd1,1,0,10);
            ImGui::DragDouble("Kp Wrist FE",&kp2,1,0,50);
            ImGui::DragDouble("Kd Wrist FE",&kd2,1,0,10);
            ImGui::DragDouble("Kp Wrist RU",&kp3,1,0,50);
            ImGui::DragDouble("Kd Wrist RU",&kd3,1,0,10);
            ImGui::DragDouble("Khard",&k_hard,1,0,20000);
            ImGui::DragDouble("Bhard",&b_hard,1,0,2000);
        ImGui::EndGroup();
        ImGui::SameLine();
        ImGui::BeginGroup();
            ImGui::DragDouble("Elbow ref (rad)",&q_ref0,0.005f,-PI/2-0.5,PI/6+0.5,"%.4f");
            ImGui::DragDouble("Forearm ref (rad)",&q_ref1,0.005f,-PI-0.5,PI+0.5,"%.4f");
            ImGui::DragDouble("Wrist FE ref (rad)",&q_ref2,0.005f,-PI/2-0.5,PI/2+0.5,"%.4f");    
            ImGui::DragDouble("Wrist RU ref (rad)",&q_ref3,0.005f,-PI/2-0.5,PI/6+0.5,"%.4f");   
            ImGui::DragDouble("Elbow pos (rad)",&q0,0.0001f,0.03,0.15,"%.4f");
            ImGui::DragDouble("Forearm pos (rad)",&q1,0.0001f,0.03,0.15,"%.4f");    
            ImGui::DragDouble("Wrist FE pos (m)",&q2,0.0001f,0.03,0.15,"%.4f");
            ImGui::DragDouble("Wrist RU pos (m)",&q3,0.0001f,0.03,0.15,"%.4f");
            ImGui::PopItemWidth();
        ImGui::EndGroup();
        int mean_calc_time = int(mean(calc_times_1s.get_vector()));
        ImGui::InputInt("Mean Calc Time",&mean_calc_time);

        ImPlot::SetNextPlotLimits(t-10, t, 0, 1500, ImGuiCond_Always);
        if(ImPlot::BeginPlot("##Sim Time", "Time (s)", "Sim Rate (us)", {-1,300}, 0, rt_axis, rt_axis)){
            ImPlot::PlotLine("Sim Loop Time", &sim_rate.Data[0].x, &sim_rate.Data[0].y, sim_rate.Data.size(), sim_rate.Offset, 2 * sizeof(float));
            ImPlot::PlotLine("Desired Sim Loop Time", &des_rate.Data[0].x, &des_rate.Data[0].y, des_rate.Data.size(), des_rate.Offset, 2 * sizeof(float));
            ImPlot::PlotLine("Comp Time", &compTime.Data[0].x, &compTime.Data[0].y, compTime.Data.size(), compTime.Offset, 2 * sizeof(float));
            ImPlot::EndPlot();
        }

        ImPlot::SetNextPlotLimits(t-10, t, -10, 10, ImGuiCond_Always);
        if(ImPlot::BeginPlot("##Forces", "Time (s)", "Force (N)", {-1,300}, 0, rt_axis, rt_axis)){
            ImPlot::PlotLine("Force 0", &tau0.Data[0].x, &tau0.Data[0].y, tau0.Data.size(), tau1.Offset, 2 * sizeof(float));
            ImPlot::PlotLine("Force 1", &tau1.Data[0].x, &tau1.Data[0].y, tau1.Data.size(), tau1.Offset, 2 * sizeof(float));
            ImPlot::PlotLine("Force 2", &tau2.Data[0].x, &tau2.Data[0].y, tau2.Data.size(), tau2.Offset, 2 * sizeof(float));
            ImPlot::PlotLine("Force 3", &tau3.Data[0].x, &tau3.Data[0].y, tau3.Data.size(), tau3.Offset, 2 * sizeof(float));
            ImPlot::EndPlot();
        }

        t += ImGui::GetIO().DeltaTime;
        ImGui::End();
        ms_gains.write_data({kp0, kd0, kp1, kd1, kp2, kd2, kp3, kd3, k_hard, b_hard});
        ms_refs.write_data({q_ref0, q_ref1, q_ref2, q_ref3});
    }

    // Member Variables
    int rt_axis = 0;//ImPlotAxisFlags_Default & ~ImPlotAxisFlags_TickLabels;
    ScrollingData sim_rate;
    ScrollingData des_rate;
    ScrollingData tau0;
    ScrollingData tau1;
    ScrollingData tau2;
    ScrollingData tau3;
    ScrollingData compTime;
    float t = 0;
    bool tp = true;
    bool enabled = false;
    double k_hard = 100;
    double b_hard = 1.0;
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
    double q0 = 0.0;
    double q1 = 0.0;
    double q2 = 0.0;
    double q3 = 0.0;
    MelShare ms_gains = MelShare("gains");
    MelShare ms_refs = MelShare("refs");
    MelShare ms_times = MelShare("times");
    MelShare ms_qs = MelShare("qs");
    std::atomic_bool stop = false;
    std::mutex mtx;
    RingBuffer<double> calc_times_1s;
};

int main(int argc, char const *argv[])
{
    SimTuner tuner;
    tuner.run();
    return 0;
}