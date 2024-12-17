/**
 * @file controller.h
 * @author Teruru-52
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "hardware/speaker.h"
#include "hardware/ir_sensor.h"
#include "controller/odometry.h"
#include "controller/pid_controller.h"
#include "controller/tracker.h"
#include "controller/identification.h"
#include "Agent.h"

#define ENABLE_LOG 1
#define ENABLE_SLALOM 1
#define ENABLE_SLALOM_CORRECTION 1
#define WALL_TIMING 0.8f
#define CNT_BACK 4
#define CNT_LOG 5

using SlalomType = trajectory::Slalom::SlalomType;
using AccType = trajectory::Acceleration::AccType;
using namespace hardware;

namespace undercarriage
{
    class Controller
    {
    public:
        Controller(undercarriage::Odometory *odom,
                   PID_Instances *pid,
                   undercarriage::TrackerBase *tracker,
                   trajectory::Slalom *slalom,
                   trajectory::Acceleration *acc,
                   hardware::IR_Param *ir_param,
                   trajectory::Velocity *velocity);

        typedef enum
        {
            forward,
            acc_curve,
            turn,
            pivot_turn_right_90,
            pivot_turn_left_90,
            pivot_turn_180,
            front_wall_correction,
            back,
            m_iden,
            step_iden,
            party_trick,
            stop,
            wait
        } CtrlMode;

        void InitializeOdometory() { odom->Initialize(); };
        void StartDMA() { motor.StartDMA(); };
        void InitializeMotor() { motor.CalcOffset(); };
        void BatteryCheck() { motor.BatteryCheck(); };
        int16_t GetPulseL() { return odom->GetPulseL(); };
        int16_t GetPulseR() { return odom->GetPulseR(); };

        void UpdateOdometory();
        // void UpdateIMU() { odom->UpdateIMU(); };
        void SetIRdata(const IR_Value &ir_value);
        void SetTrajectoryMode(int trj_mode = 1);
        // void SetBase() { theta_base = cur_pos.th; };
        bool ErrorFlag();

        void SetM_Iden();
        void SetStep_Iden();
        void SetPartyTrick();
        void PivotTurn(int angle);
        void Turn(const SlalomType &slalom_type);
        void Acceleration(const AccType &acc_type, uint8_t num_square = 1);
        void GoStraight();
        void FrontWallCorrection();
        void Back();
        void Wait_ms();

        void M_Iden();
        void Step_Iden();
        void PartyTrick();
        void SideWallCorrection();
        void PivotTurn();
        void CalcSlalomInput();
        float GetFrontWallPos(float ir_fmean);
        void Turn();
        void Acceleration();
        void GoStraight(float ref_l);
        void Back(int time);
        void Wait_ms(int time);
        void FrontWallCorrection(const IR_Value &ir_value);
        void BlindAlley();
        void StartMove();
        void Brake();
        void InputVelocity(float input_v, float input_w);

        void ResetCtrl();
        bool GetCtrlFlag() { return flag_ctrl; };
        bool GetMazeLoadFlag() { return flag_maze_load; };
        void ResetWallFlag() { flag_wall = false; };
        void ResetMazeLoadFlag() { flag_maze_load = false; };

        bool wallDataReady() { return flag_wall; };
        void updateWallData() { ir_wall_value = ir_value; };
        IR_Value GetIRWall() { return ir_wall_value; };
        Direction getWallData();
        void UpdatePos(const Direction &dir);
        void UpdateDir(const Direction &dir) { robot_dir = dir; };
        IndexVec getRobotPosition() { return robot_position; };

        void robotMove();
        void DirMove(const Direction &dir);
        void OpMove(const Operation &op);
        void CalcOpMovedState(const OperationList &runSequence);

        void Logger();
        void OutputLog();
        void OutputSlalomLog();
        void OutputPivotTurnLog();
        void OutputTranslationLog();
        void OutputMIdenLog() { iden_m.OutputLog(); };
        void OutputStepIdenLog() { iden_step.OutputLog(); };
        void MotorTest(float v_left, float v_right);

    private:
        undercarriage::Odometory *odom;
        hardware::Motor motor;
        PID_Instances *pid;
        undercarriage::TrackerBase *tracker;
        trajectory::Slalom *slalom;
        trajectory::Acceleration *acc;
        trajectory::PivotTurn180 pivot_turn180;
        trajectory::PivotTurn90 pivot_turn90;
        CtrlMode mode_ctrl;
        hardware::IR_Param *ir_param;
        trajectory::Velocity *velocity;
        undercarriage::M_Identification iden_m;
        undercarriage::Step_Identification iden_step;

        float v_left;
        float v_right;
        float u_w;
        float u_v;

        float ref_v;
        float ref_w;
        float ref_dw;
        int ref_size;

        float theta_base = 0.0; // theta_global_ref
        float theta_global = 0.0;
        float theta_error = 0.0;
        int angle_turn;
        float ref_theta = 0;
        float length;
        SlalomType slalom_type_;
        ctrl::Pose cur_pos{0, 0, 0};
        ctrl::Pose cur_vel{0, 0, 0};
        ctrl::Pose ref_pos{0, 0, 0}; // absolute coordinates
        ctrl::Pose ref_vel{0, 0, 0}; // robot coordinates
        ctrl::Pose ref_vel_ctrl{0, 0, 0};
        ctrl::Pose ref_acc{0, 0, 0}; // robot coordinates
        float acc_x;
        const float acc_x_err = 50.0 * 1e+3; // error threshold

        const float Tp1_w = 31.83;
        const float Kp_w = 144.2 * 1e+3;
        const float Tp1_v = 0.032;
        // const float Kp_v = 0.784493 * 1e+3;
        const float Kp_v = 0.65 * 1e+3;

        hardware::IR_Value ir_value;
        hardware::IR_Value ir_wall_value;
        bool flag_read_side_wall = false;
        typedef std::pair<uint32_t, float> xy_pair;
        // const std::vector<xy_pair> ir_table = {{2200, -48.9276}}; // dammy
        const std::vector<xy_pair> ir_table = {{2200, -33.0942}, {2201, -32.3786}, {2202, -31.6753}, {2203, -30.9838}, {2204, -30.3037}, {2205, -29.6348}, {2206, -28.9766}, {2207, -28.3288}, {2208, -27.6911}, {2209, -27.0631}, {2210, -26.4447}, {2211, -25.8354}, {2212, -25.2350}, {2213, -24.6433}, {2214, -24.0600}, {2215, -23.4849}, {2216, -22.9177}, {2217, -22.3583}, {2218, -21.8064}, {2219, -21.2619}, {2220, -20.7244}, {2221, -20.1940}, {2222, -19.6703}, {2223, -19.1532}, {2224, -18.6425}, {2225, -18.1382}, {2226, -17.6399}, {2227, -17.1477}, {2228, -16.6612}, {2229, -16.1805}, {2230, -15.7054}, {2231, -15.2357}, {2232, -14.7713}, {2233, -14.3121}, {2234, -13.8580}, {2235, -13.4089}, {2236, -12.9646}, {2237, -12.5251}, {2238, -12.0903}, {2239, -11.6600}, {2240, -11.2342}, {2241, -10.8127}, {2242, -10.3956}, {2243, -9.9827}, {2244, -9.5738}, {2245, -9.1690}, {2246, -8.7682}, {2247, -8.3713}, {2248, -7.9781}, {2249, -7.5887}, {2250, -7.2029}, {2251, -6.8208}, {2252, -6.4422}, {2253, -6.0670}, {2254, -5.6952}, {2255, -5.3268}, {2256, -4.9617}, {2257, -4.5998}, {2258, -4.2410}, {2259, -3.8854}, {2260, -3.5328}, {2261, -3.1832}, {2262, -2.8366}, {2263, -2.4929}, {2264, -2.1520}, {2265, -1.8140}, {2266, -1.4787}, {2267, -1.1461}, {2268, -0.8162}, {2269, -0.4890}, {2270, -0.1643}, {2271, 0.1578}, {2272, 0.4774}, {2273, 0.7946}, {2274, 1.1093}, {2275, 1.4216}, {2276, 1.7315}, {2277, 2.0391}, {2278, 2.3444}, {2279, 2.6475}, {2280, 2.9484}, {2281, 3.2470}, {2282, 3.5435}, {2283, 3.8379}, {2284, 4.1301}, {2285, 4.4203}, {2286, 4.7085}, {2287, 4.9947}, {2288, 5.2788}, {2289, 5.5610}, {2290, 5.8413}, {2291, 6.1197}, {2292, 6.3962}, {2293, 6.6708}, {2294, 6.9437}, {2295, 7.2147}, {2296, 7.4839}, {2297, 7.7514}, {2298, 8.0172}, {2299, 8.2812}, {2300, 8.5436}, {2301, 8.8042}, {2302, 9.0633}, {2303, 9.3207}, {2304, 9.5765}, {2305, 9.8307}, {2306, 10.0834}, {2307, 10.3345}, {2308, 10.5840}, {2309, 10.8321}, {2310, 11.0787}, {2311, 11.3238}, {2312, 11.5674}, {2313, 11.8096}, {2314, 12.0504}, {2315, 12.2898}, {2316, 12.5278}, {2317, 12.7644}, {2318, 12.9996}, {2319, 13.2336}, {2320, 13.4661}, {2321, 13.6974}, {2322, 13.9274}, {2323, 14.1561}, {2324, 14.3835}, {2325, 14.6097}, {2326, 14.8346}, {2327, 15.0583}, {2328, 15.2808}, {2329, 15.5021}, {2330, 15.7222}, {2331, 15.9412}, {2332, 16.1589}, {2333, 16.3755}, {2334, 16.5910}, {2335, 16.8054}, {2336, 17.0186}, {2337, 17.2307}, {2338, 17.4418}, {2339, 17.6517}, {2340, 17.8606}, {2341, 18.0685}, {2342, 18.2753}, {2343, 18.4810}, {2344, 18.6857}, {2345, 18.8894}, {2346, 19.0921}, {2347, 19.2938}, {2348, 19.4945}, {2349, 19.6943}, {2350, 19.8930}, {2351, 20.0908}, {2352, 20.2877}, {2353, 20.4836}, {2354, 20.6786}, {2355, 20.8727}, {2356, 21.0658}, {2357, 21.2580}, {2358, 21.4494}, {2359, 21.6398}, {2360, 21.8294}, {2361, 22.0181}, {2362, 22.2060}, {2363, 22.3929}, {2364, 22.5791}, {2365, 22.7644}, {2366, 22.9488}, {2367, 23.1325}, {2368, 23.3153}, {2369, 23.4973}, {2370, 23.6785}, {2371, 23.8589}, {2372, 24.0385}, {2373, 24.2173}, {2374, 24.3954}, {2375, 24.5727}, {2376, 24.7492}, {2377, 24.9249}, {2378, 25.1000}, {2379, 25.2742}, {2380, 25.4478}, {2381, 25.6206}, {2382, 25.7926}, {2383, 25.9640}, {2384, 26.1346}, {2385, 26.3046}, {2386, 26.4738}, {2387, 26.6423}, {2388, 26.8102}, {2389, 26.9773}, {2390, 27.1438}, {2391, 27.3096}, {2392, 27.4748}, {2393, 27.6393}, {2394, 27.8031}, {2395, 27.9663}, {2396, 28.1288}, {2397, 28.2907}, {2398, 28.4519}, {2399, 28.6125}, {2400, 28.7725}, {2401, 28.9319}, {2402, 29.0907}, {2403, 29.2488}, {2404, 29.4063}, {2405, 29.5633}, {2406, 29.7196}, {2407, 29.8753}, {2408, 30.0305}, {2409, 30.1850}, {2410, 30.3390}, {2411, 30.4924}, {2412, 30.6453}, {2413, 30.7975}, {2414, 30.9492}, {2415, 31.1004}, {2416, 31.2510}, {2417, 31.4010}, {2418, 31.5505}, {2419, 31.6994}, {2420, 31.8478}, {2421, 31.9957}, {2422, 32.1431}, {2423, 32.2899}, {2424, 32.4362}, {2425, 32.5819}, {2426, 32.7272}, {2427, 32.8719}, {2428, 33.0162}, {2429, 33.1599}, {2430, 33.3031}, {2431, 33.4458}, {2432, 33.5881}, {2433, 33.7298}, {2434, 33.8711}, {2435, 34.0118}, {2436, 34.1521}, {2437, 34.2919}, {2438, 34.4313}, {2439, 34.5701}, {2440, 34.7085}, {2441, 34.8464}, {2442, 34.9839}, {2443, 35.1209}, {2444, 35.2575}, {2445, 35.3935}, {2446, 35.5292}, {2447, 35.6644}, {2448, 35.7991}, {2449, 35.9335}, {2450, 36.0673}, {2451, 36.2008}, {2452, 36.3338}, {2453, 36.4663}, {2454, 36.5985}, {2455, 36.7302}, {2456, 36.8615}, {2457, 36.9924}, {2458, 37.1229}, {2459, 37.2529}, {2460, 37.3826}, {2461, 37.5118}, {2462, 37.6406}, {2463, 37.7690}, {2464, 37.8971}, {2465, 38.0247}, {2466, 38.1519}, {2467, 38.2787}, {2468, 38.4052}, {2469, 38.5312}, {2470, 38.6569}, {2471, 38.7822}, {2472, 38.9071}, {2473, 39.0316}, {2474, 39.1558}, {2475, 39.2795}, {2476, 39.4029}, {2477, 39.5260}, {2478, 39.6486}, {2479, 39.7709}, {2480, 39.8928}, {2481, 40.0144}, {2482, 40.1356}, {2483, 40.2565}, {2484, 40.3770}, {2485, 40.4971}, {2486, 40.6169}, {2487, 40.7364}, {2488, 40.8555}, {2489, 40.9742}, {2490, 41.0926}, {2491, 41.2107}, {2492, 41.3284}, {2493, 41.4458}, {2494, 41.5629}, {2495, 41.6796}, {2496, 41.7960}, {2497, 41.9121}, {2498, 42.0278}, {2499, 42.1432}, {2500, 42.2583}};
        float error_fl;
        float error_fr;

        int prev_wall_cnt = 0;
        int8_t dir_diff = 0;
        IndexVec robot_position = {0, 0};
        Direction robot_dir = NORTH;

        const int back_time = 700;       // ms
        const int correction_time = 700; // ms
        const int wait_time = 200;       // ms
        int cnt_blind_alley = 0;
        int cnt_can_back = 0;
        int cnt_time = 0;

        bool flag_ctrl = false;
        bool flag_slalom;
        bool flag_wall = false; // flag for sensors reading wall
        bool flag_wall_sl = true;
        bool flag_wall_sr = true;
        bool flag_wall_front = false;
        bool flag_safety = false;
        bool flag_maze_load = false;
        bool flag_side_correct = false;
        bool flag_straight_wall = false;
        bool flag_straight_time = false;

        int index_log = 0;
        float *log_x;
        float *log_y;
        // float *log_l;
        float *log_theta;
        float *log_omega;
        float *log_v;
        float *log_a;
        float *log_ref_x;
        float *log_ref_y;
        float *log_ref_theta;
        float *log_ref_omega;
        float *log_ref_v;
        float *log_ref_a;
        float *log_ctrl_v;
        float *log_ctrl_w;
        float *log_u_v;
        float *log_u_w;
    };
} // namespace undercarriage

#endif //  CONTROLLER_H_