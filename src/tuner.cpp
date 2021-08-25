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
        tau1.AddPoint(t,0);
        tau2.AddPoint(t,0);
        tau3.AddPoint(t,0);
        tau4.AddPoint(t,0);
        tau5.AddPoint(t,0);
        compTime.AddPoint(t,0);
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

            tau1.AddPoint(t,ms_qs_data[0]);
            tau2.AddPoint(t,ms_qs_data[1]);
            tau3.AddPoint(t,ms_qs_data[2]);
            tau4.AddPoint(t,ms_qs_data[3]);
            tau5.AddPoint(t,ms_qs_data[4]);
            q1 = ms_qs_data[5];
            q2 = ms_qs_data[6];
            q3 = ms_qs_data[7];
            q4 = ms_qs_data[8];
            q5 = ms_qs_data[9];
        }
        ImGui::Begin("PD Tuner");
        ImGui::BeginGroup();
            ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.3f);
            ImGui::DragDouble("Kp Elbow",&kpe,0,0,50);
            ImGui::DragDouble("Kd Elbow",&kde,0,0,10);
            ImGui::DragDouble("Kp Forearm",&kpf,0,0,50);
            ImGui::DragDouble("Kd Forearm",&kdf,0,0,10);
            ImGui::DragDouble("Kp Parallel",&kp,0,0,1000);
            ImGui::DragDouble("Kd Parallel",&kd,0,0,100);
            ImGui::DragDouble("Khard",&k_hard,0,0,20000);
            ImGui::DragDouble("Bhard",&b_hard,0,0,2000);
        ImGui::EndGroup();
        ImGui::SameLine();
        ImGui::BeginGroup();
            ImGui::DragDouble("Elbow ref (rad)",&q_ref1,0.005f,-PI/2-0.5,0.5,"%.4f");
            ImGui::DragDouble("Forearm ref (rad)",&q_ref2,0.005f,-PI,PI,"%.4f");
            ImGui::DragDouble("Link 1 ref (m)",&q_ref3,0.0001f,0.03,0.15,"%.4f");    
            ImGui::DragDouble("Link 2 ref (m)",&q_ref4,0.0001f,0.03,0.15,"%.4f");   
            ImGui::DragDouble("Link 3 ref (m)",&q_ref5,0.0001f,0.03,0.15,"%.4f");    
            ImGui::DragDouble("Elbow pos (rad)",&q1,0.0001f,0.03,0.15,"%.4f");
            ImGui::DragDouble("Forearm pos (rad)",&q2,0.0001f,0.03,0.15,"%.4f");    
            ImGui::DragDouble("Link 1 pos (m)",&q3,0.0001f,0.03,0.15,"%.4f");
            ImGui::DragDouble("Link 2 pos (m)",&q4,0.0001f,0.03,0.15,"%.4f");
            ImGui::DragDouble("Link 3 pos (m)",&q5,0.0001f,0.03,0.15,"%.4f");
            ImGui::PopItemWidth();
        ImGui::EndGroup();
        int mean_calc_time = int(mean(calc_times_1s.get_vector()));
        ImGui::InputInt("Mean Calc Time",&mean_calc_time);

        ImPlot::SetNextPlotLimits(t-10, t, 0, 1500, ImGuiCond_Always);
        if(ImPlot::BeginPlot("##Sim Time", "Time (s)", "Sim Rate (us)", {-1,300}, ImPlotFlags_Default, rt_axis, rt_axis)){
            ImPlot::PlotLine("Sim Loop Time", &sim_rate.Data[0].x, &sim_rate.Data[0].y, sim_rate.Data.size(), sim_rate.Offset, 2 * sizeof(float));
            ImPlot::PlotLine("Desired Sim Loop Time", &des_rate.Data[0].x, &des_rate.Data[0].y, des_rate.Data.size(), des_rate.Offset, 2 * sizeof(float));
            ImPlot::PlotLine("Comp Time", &compTime.Data[0].x, &compTime.Data[0].y, compTime.Data.size(), compTime.Offset, 2 * sizeof(float));
            ImPlot::EndPlot();
        }

        ImPlot::SetNextPlotLimits(t-10, t, -10, 10, ImGuiCond_Always);
        if(ImPlot::BeginPlot("##Forces", "Time (s)", "Force (N)", {-1,300}, ImPlotFlags_Default, rt_axis, rt_axis)){
            ImPlot::PlotLine("Force 1", &tau1.Data[0].x, &tau1.Data[0].y, tau1.Data.size(), tau1.Offset, 2 * sizeof(float));
            ImPlot::PlotLine("Force 2", &tau2.Data[0].x, &tau2.Data[0].y, tau2.Data.size(), tau2.Offset, 2 * sizeof(float));
            ImPlot::PlotLine("Force 3", &tau3.Data[0].x, &tau3.Data[0].y, tau3.Data.size(), tau3.Offset, 2 * sizeof(float));
            ImPlot::PlotLine("Force 4", &tau4.Data[0].x, &tau4.Data[0].y, tau4.Data.size(), tau4.Offset, 2 * sizeof(float));
            ImPlot::PlotLine("Force 5", &tau5.Data[0].x, &tau5.Data[0].y, tau5.Data.size(), tau5.Offset, 2 * sizeof(float));
            ImPlot::EndPlot();
        }

        t += ImGui::GetIO().DeltaTime;
        ImGui::End();
        ms_gains.write_data({kp, kd, kpe, kde, kpf, kdf, k_hard, b_hard});
        ms_refs.write_data({q_ref1, q_ref2, q_ref3, q_ref4, q_ref5});
    }

    // Member Variables
    int rt_axis = ImPlotAxisFlags_Default & ~ImPlotAxisFlags_TickLabels;
    ScrollingData sim_rate;
    ScrollingData des_rate;
    ScrollingData tau1;
    ScrollingData tau2;
    ScrollingData tau3;
    ScrollingData tau4;
    ScrollingData tau5;
    ScrollingData compTime;
    float t = 0;
    bool tp = true;
    bool enabled = false;
    double k_hard = 200;
    double b_hard = 10;
    double kp = 2200;
    double kd = 30;
    double kpe = 100;
    double kde = 1.25;
    double kpf = 28;
    double kdf = 0.2;
    double q_ref1 = -mahi::util::PI/4;
    double q_ref2 = 0.0;
    double q_ref3 = 0.1;
    double q_ref4 = 0.1;
    double q_ref5 = 0.1;
    double q1 = 0.0;
    double q2 = 0.0;
    double q3 = 0.1;
    double q4 = 0.1;
    double q5 = 0.1;
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