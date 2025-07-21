/**
 * @file controller.h
 * @author Teruru-52
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "Agent.h"
#include "controller/identification.h"
#include "controller/odometry.h"
#include "controller/pid_controller.h"
#include "controller/tracker.h"
#include "hardware/ir_sensor.h"
#include "hardware/speaker.h"

#define ENABLE_SLALOM 1
#define ENABLE_SLALOM_CORRECTION 1
#define ENABLE_LOG 0
#define BUFFER_SIZE 10
#define CNT_LOG 2
#define CNT_BACK 4
#define CNT_BLIND_ALLAY 2

using SlalomType = trajectory::Slalom::SlalomType;
using AccType = trajectory::Acceleration::AccType;
using namespace hardware;

namespace undercarriage {
class Controller {
public:
  Controller(undercarriage::Odometory *odom, PID_Instances *pid,
             undercarriage::TrackerBase *tracker, trajectory::Slalom *slalom,
             trajectory::Acceleration *acc, hardware::IR_Param *ir_param,
             trajectory::Velocity *velocity);

  typedef enum {
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
  void FrontWallCorrection();
  void Back();
  void Wait_ms();

  void M_Iden();
  void Step_Iden();
  void PartyTrick();
  void SideWallCorrection();
  void PivotTurn();
  float GetFrontWallPos(float ir_fmean);
  void Turn();
  void Acceleration();
  void Back(int time);
  void Wait_ms(int time);
  void FrontWallCorrection(const IR_Value &ir_value);
  void BlindAlley();
  void StartMove();
  void Brake();
  void InputVelocity(float input_v, float input_w);

  void ResetCtrl();
  bool GetCtrlFlag() { return flag_ctrl; };
  bool GetMazeFlashFlag() { return flag_maze_flash; };
  void ResetWallFlag() { flag_wall = false; };
  void ResetMazeLoadFlag() { flag_maze_flash = false; };

  bool wallDataReady() { return flag_wall; };
  void updateWallData() { ir_wall_value = ir_value_; };
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
  void LoggerWall();
  void OutputLog();
  void OutputSlalomLog();
  void OutputPivotTurnLog();
  void OutputTranslationLog();
  void OutputMIdenLog() { iden_m.OutputLog(); };
  void OutputStepIdenLog() { iden_step.OutputLog(); };
  void MotorTest(float v_left, float v_right);

private:
  std::unique_ptr<undercarriage::Odometory> odom;
  hardware::Motor motor;
  std::unique_ptr<PID_Instances> pid;
  std::unique_ptr<undercarriage::TrackerBase> tracker;
  std::unique_ptr<trajectory::Slalom> slalom;
  std::unique_ptr<trajectory::Acceleration> acc;
  trajectory::PivotTurn180 pivot_turn180;
  trajectory::PivotTurn90 pivot_turn90;
  CtrlMode mode_ctrl;
  std::unique_ptr<hardware::IR_Param> ir_param;
  std::unique_ptr<trajectory::Velocity> velocity;
  undercarriage::M_Identification iden_m;
  undercarriage::Step_Identification iden_step;

  float v_left;
  float v_right;
  float u_w;
  float u_v;
  int ref_size;

  float theta_base = 0.0; // theta_global_ref
  float theta_global = 0.0;
  float theta_error = 0.0;
  int angle_turn;
  float ref_theta = 0;
  float x_diff = 0;
  float length;
  SlalomType slalom_type_;
  ctrl::Pose cur_pos{0, 0, 0};
  ctrl::Pose cur_vel{0, 0, 0};
  ctrl::Pose ref_pos{0, 0, 0}; // absolute coordinates
  ctrl::Pose ref_vel{0, 0, 0}; // robot coordinates
  ctrl::Pose ref_vel_ctrl{0, 0, 0};
  ctrl::Pose ref_acc{0, 0, 0}; // robot coordinates
  float acc_x;
  const float acc_x_err = 40.0 * 1e+3; // error threshold

  const float Tp1_w = 31.83;
  const float Kp_w = 144.2 * 1e+3;
  const float Tp1_v = 0.032;
  // const float Kp_v = 0.784493 * 1e+3;
  const float Kp_v = 0.65 * 1e+3;

  hardware::IR_Value ir_value_;
  hardware::IR_Value ir_wall_value;
  bool flag_read_side_wall = false;
  typedef std::pair<uint32_t, float> xy_pair;
  // const std::vector<xy_pair> ir_table = {{2200, -48.9276}}; // dammy
  // science tokyo
  const std::vector<xy_pair> ir_table = {
      {2400, -32.7445}, {2401, -32.2105}, {2402, -31.6816}, {2403, -31.1577},
      {2404, -30.6387}, {2405, -30.1246}, {2406, -29.6152}, {2407, -29.1104},
      {2408, -28.6103}, {2409, -28.1146}, {2410, -27.6233}, {2411, -27.1364},
      {2412, -26.6537}, {2413, -26.1752}, {2414, -25.7008}, {2415, -25.2304},
      {2416, -24.7640}, {2417, -24.3015}, {2418, -23.8429}, {2419, -23.3880},
      {2420, -22.9369}, {2421, -22.4894}, {2422, -22.0455}, {2423, -21.6051},
      {2424, -21.1682}, {2425, -20.7348}, {2426, -20.3047}, {2427, -19.8780},
      {2428, -19.4545}, {2429, -19.0342}, {2430, -18.6171}, {2431, -18.2032},
      {2432, -17.7923}, {2433, -17.3844}, {2434, -16.9796}, {2435, -16.5777},
      {2436, -16.1787}, {2437, -15.7825}, {2438, -15.3892}, {2439, -14.9986},
      {2440, -14.6108}, {2441, -14.2257}, {2442, -13.8433}, {2443, -13.4635},
      {2444, -13.0863}, {2445, -12.7116}, {2446, -12.3395}, {2447, -11.9699},
      {2448, -11.6027}, {2449, -11.2379}, {2450, -10.8756}, {2451, -10.5156},
      {2452, -10.1579}, {2453, -9.8025},  {2454, -9.4494},  {2455, -9.0986},
      {2456, -8.7499},  {2457, -8.4035},  {2458, -8.0592},  {2459, -7.7170},
      {2460, -7.3769},  {2461, -7.0390},  {2462, -6.7031},  {2463, -6.3692},
      {2464, -6.0373},  {2465, -5.7074},  {2466, -5.3794},  {2467, -5.0534},
      {2468, -4.7293},  {2469, -4.4071},  {2470, -4.0868},  {2471, -3.7683},
      {2472, -3.4516},  {2473, -3.1368},  {2474, -2.8237},  {2475, -2.5124},
      {2476, -2.2028},  {2477, -1.8950},  {2478, -1.5889},  {2479, -1.2844},
      {2480, -0.9816},  {2481, -0.6805},  {2482, -0.3810},  {2483, -0.0831},
      {2484, 0.2131},   {2485, 0.5078},   {2486, 0.8010},   {2487, 1.0926},
      {2488, 1.3826},   {2489, 1.6712},   {2490, 1.9582},   {2491, 2.2438},
      {2492, 2.5279},   {2493, 2.8105},   {2494, 3.0917},   {2495, 3.3715},
      {2496, 3.6498},   {2497, 3.9268},   {2498, 4.2024},   {2499, 4.4766},
      {2500, 4.7495},   {2501, 5.0210},   {2502, 5.2912},   {2503, 5.5601},
      {2504, 5.8277},   {2505, 6.0939},   {2506, 6.3590},   {2507, 6.6227},
      {2508, 6.8852},   {2509, 7.1464},   {2510, 7.4065},   {2511, 7.6653},
      {2512, 7.9229},   {2513, 8.1793},   {2514, 8.4345},   {2515, 8.6885},
      {2516, 8.9414},   {2517, 9.1931},   {2518, 9.4437},   {2519, 9.6932},
      {2520, 9.9415},   {2521, 10.1887},  {2522, 10.4348},  {2523, 10.6798},
      {2524, 10.9238},  {2525, 11.1667},  {2526, 11.4085},  {2527, 11.6492},
      {2528, 11.8889},  {2529, 12.1276},  {2530, 12.3653},  {2531, 12.6019},
      {2532, 12.8375},  {2533, 13.0721},  {2534, 13.3058},  {2535, 13.5384},
      {2536, 13.7701},  {2537, 14.0008},  {2538, 14.2305},  {2539, 14.4593},
      {2540, 14.6871},  {2541, 14.9140},  {2542, 15.1400},  {2543, 15.3651},
      {2544, 15.5892},  {2545, 15.8125},  {2546, 16.0348},  {2547, 16.2563},
      {2548, 16.4768},  {2549, 16.6965},  {2550, 16.9153},  {2551, 17.1333},
      {2552, 17.3504},  {2553, 17.5666},  {2554, 17.7820},  {2555, 17.9966},
      {2556, 18.2104},  {2557, 18.4233},  {2558, 18.6354},  {2559, 18.8466},
      {2560, 19.0571},  {2561, 19.2668},  {2562, 19.4757},  {2563, 19.6838},
      {2564, 19.8911},  {2565, 20.0977},  {2566, 20.3034},  {2567, 20.5085},
      {2568, 20.7127},  {2569, 20.9162},  {2570, 21.1190},  {2571, 21.3210},
      {2572, 21.5223},  {2573, 21.7228},  {2574, 21.9226},  {2575, 22.1217},
      {2576, 22.3201},  {2577, 22.5178},  {2578, 22.7148},  {2579, 22.9110},
      {2580, 23.1066},  {2581, 23.3015},  {2582, 23.4957},  {2583, 23.6892},
      {2584, 23.8821},  {2585, 24.0742},  {2586, 24.2657},  {2587, 24.4566},
      {2588, 24.6468},  {2589, 24.8363},  {2590, 25.0252},  {2591, 25.2135},
      {2592, 25.4011},  {2593, 25.5880},  {2594, 25.7744},  {2595, 25.9601},
      {2596, 26.1452},  {2597, 26.3296},  {2598, 26.5135},  {2599, 26.6968},
      {2600, 26.8794}};
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
  bool flag_maze_flash = false;
  bool flag_side_correct = false;
  bool flag_straight_time = false;

  int index_log = 0;
  float log_x[BUFFER_SIZE];
  float log_y[BUFFER_SIZE];
  float log_theta[BUFFER_SIZE];
  float log_omega[BUFFER_SIZE];
  float log_v[BUFFER_SIZE];
  float log_a[BUFFER_SIZE];
  float log_ref_x[BUFFER_SIZE];
  float log_ref_y[BUFFER_SIZE];
  float log_ref_theta[BUFFER_SIZE];
  float log_ref_omega[BUFFER_SIZE];
  float log_ref_v[BUFFER_SIZE];
  float log_ref_a[BUFFER_SIZE];
  float log_ctrl_v[BUFFER_SIZE];
  float log_ctrl_w[BUFFER_SIZE];
  float log_u_v[BUFFER_SIZE];
  float log_u_w[BUFFER_SIZE];
  float log_x_diff[10];
};
} // namespace undercarriage

#endif //  CONTROLLER_H_