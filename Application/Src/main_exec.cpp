/*
 * main_exec.cpp
 *
 *      Author: Teruru-52
 */

#include "main_exec.h"
#include "mouse_utils.h"
#include "instance.h"
#include "flash.h"
#include "hardware/led.h"
#include "hardware/speaker.h"

using SlalomType = trajectory::Slalom::SlalomType;
using AccType = trajectory::Acceleration::AccType;

Direction wallData = 0x0E;
Direction nextDir = NORTH;
IndexVec robotPos = IndexVec(0, 0);
Agent::State prevState = Agent::State::IDLE;
OperationList runSequence;

int cnt1Hz;
int16_t pulse_l;
int16_t pulse_r;

int trj_mode = 1;
int search_time = 0;
bool flag_start_cnt = false;
bool north_priority = false;

void StartupProcess()
{
    speaker.Beep(50);

    ir_sensors.StartDMA();
    ir_sensors.OnLed();
    controller.StartDMA();
    HAL_Delay(10);
    controller.BatteryCheck();
    HAL_Delay(500);

    Write_GPIO(FAN_PH, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 200);
    HAL_Delay(1000);
    // __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 400);
    // HAL_Delay(1000);
    // __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 600);
    // HAL_Delay(1000);
    // __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 1000);
    // HAL_Delay(1000);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);

    led.OffAll();
}

void UpdateUndercarriage()
{
    controller.UpdateOdometory();
}

void SetIRSensor()
{
    ir_sensors.SetIRSensorData();
    ir_value = ir_sensors.GetIRSensorData();
    controller.SetIRdata(ir_value);
}

void UpdateIRSensor()
{
    ir_sensors.UpdateLeftValue();
    ir_sensors.UpdateRightValue();
}

void Notification()
{
    // state.interruption_ = State::not_interrupt;
    speaker.Beep(20);
    led.Flash();
    speaker.Beep(20);
    // state.interruption_ = State::interrupt;
}

void Initialize()
{
    speaker.Beep(50);

    led.OffAll();
    switch (state.func_)
    {
    case State::func0: // start first searching (not load maze, SEARCHING_NOT_GOAL)
        trj_mode = 1;
        controller.SetTrajectoryMode(trj_mode);
        search_time = 120;
        state.mode_ = State::search;
        // printf("mode: search0\n");
        break;

    case State::func1: // resume searching (load maze, SEARCHING_NOT_GOAL)
        trj_mode = 2;
        controller.SetTrajectoryMode(trj_mode);
        search_time = 120;
        state.mode_ = State::search;
        // printf("mode: search1\n");
        break;

    case State::func2: // resume searching (load maze, SEARCHING_REACHED_GOAL or BACK_TO_START or FINISHED)
        trj_mode = 1;
        controller.SetTrajectoryMode(trj_mode);
        LoadMaze();
        agent.resumeAt(Agent::SEARCHING_NOT_GOAL, maze);
        search_time = 60;
        state.mode_ = State::search;
        // printf("mode: search2\n");
        break;

    case State::func3: // run sequence (load maze, SEARCHING_REACHED_GOAL or BACK_TO_START or FINISHED)
        trj_mode = 1;
        controller.SetTrajectoryMode(trj_mode);
        LoadMaze();
        agent.resumeAt(Agent::FINISHED, maze);
        // search_time = 30;
        agent.caclRunSequence(false);
        state.mode_ = State::run_sequence;
        // printf("mode: run_sequence1\n");
        break;

    case State::func4: // run sequence (load maze, SEARCHING_REACHED_GOAL or BACK_TO_START or FINISHED)
        trj_mode = 2;
        controller.SetTrajectoryMode(trj_mode);
        LoadMaze();
        agent.resumeAt(Agent::FINISHED, maze);
        // search_time = 20;
        agent.caclRunSequence(false);
        state.mode_ = State::run_sequence;
        // printf("mode: run_sequence2\n");
        break;

    case State::func5: // run sequence (load maze, SEARCHING_REACHED_GOAL or BACK_TO_START or FINISHED)
        // __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 2000);
        trj_mode = 3;
        controller.SetTrajectoryMode(trj_mode);
        LoadMaze();
        agent.resumeAt(Agent::FINISHED, maze);
        // search_time = 10;
        agent.caclRunSequence(false);
        state.mode_ = State::run_sequence;
        // printf("mode: run_sequence3\n");
        break;

    case State::func6:
        state.mode_ = State::m_identification;
        printf("mode: m_identification\n");
        break;

    case State::func7:
        state.mode_ = State::step_identification;
        printf("mode: step_identification\n");
        break;

    case State::func8:
        state.mode_ = State::party_trick;
        printf("mode: party_trick\n");
        break;

    case State::func9:
        trj_mode = 1;
        controller.SetTrajectoryMode(trj_mode);
        state.mode_ = State::test_slalom2;
        printf("mode: test_slalom2\n");
        break;

    case State::func10:
        trj_mode = 1;
        controller.SetTrajectoryMode(trj_mode);
        state.mode_ = State::test_slalom1;
        printf("mode: test_slalom1\n");
        break;

    case State::func11:
        trj_mode = 3;
        controller.SetTrajectoryMode(trj_mode);
        state.mode_ = State::test_translation2;
        printf("mode: test_translation2\n");
        break;

    case State::func12:
        trj_mode = 2;
        controller.SetTrajectoryMode(trj_mode);
        state.mode_ = State::test_translation1;
        printf("mode: test_translation1\n");
        break;

    case State::func13:
        state.mode_ = State::test_rotation;
        printf("mode: test_rotation\n");
        break;

    case State::func14:
        state.mode_ = State::test_odometory;
        printf("mode: test_odometory\n");
        break;

    case State::func15:
        state.mode_ = State::test_ir;
        // state.mode_ = State::output;
        printf("mode: test_ir\n");
        break;

    default:
        break;
    }

    // switch (state.mazeload)
    // {
    // case State::load:
    //     LoadMaze();
    //     break;

    // case State::not_load:
    //     break;
    // }

    speaker.Beep(20);
    speaker.Beep(20);
    controller.InitializeOdometory();
    speaker.Beep(20);
    controller.InitializeMotor();
    speaker.Beep(20);
    agent.setNorthPriority(north_priority);
    agent.update(robotPos, wallData);
    state.interruption_ = State::interrupt;
}

void StateProcess()
{
    if (state.mode_ == State::select_function)
    {
        ir_sensors.UpdateLeftValue();
        ir_sensors.UpdateRightValue();
        pulse_l = controller.GetPulseL();
        pulse_r = controller.GetPulseR();
        state.SelectLoadMaze(pulse_l);
        state.SelectFunction(pulse_r);

        static uint16_t cnt_sw = 0;
        if (Read_GPIO(USER_SW) == GPIO_PIN_RESET)
        {
            cnt_sw++;
            if (cnt_sw > 50000)
            {
                if (north_priority)
                {
                    north_priority = false;
                    Write_GPIO(LED_GREEN, GPIO_PIN_RESET);
                }
                else
                {
                    north_priority = true;
                    Write_GPIO(LED_GREEN, GPIO_PIN_SET);
                }
                cnt_sw = 0;
            }
        }

        if (ir_sensors.StartInitialize())
            Initialize();
    }

    else
    {
        switch (state.mode_)
        {
        case State::search: // func0, func1
            controller.StartMove();
            flag_start_cnt = true;
            MazeSearch();
            controller.PivotTurn(180);
            state.interruption_ = State::not_interrupt;
            FlashMaze();
            Notification();
            // agent.caclRunSequence(false);
            // state.interruption_ = State::interrupt;

            state.mode_ = State::select_function;
            // state.mode_ = State::run_sequence;
            break;

        case State::State::run_sequence: // func2, func3, func4, func5
            TimeAttack();
            // agent.resumeAt(Agent::SEARCHING_REACHED_GOAL, maze, robotPos);
            // trj_mode = 3;
            // controller.SetTrajectoryMode(trj_mode);
            // cnt1Hz = 0;
            // state.mode = State::search;
            state.interruption_ = State::not_interrupt;
            state.mode_ = State::select_function;
            break;

        case State::m_identification: // func6
            controller.SetM_Iden();
            state.log_ = State::m_iden;
            state.mode_ = State::output;
            break;

        case State::step_identification: // func7
            controller.SetStep_Iden();
            state.log_ = State::step_iden;
            state.mode_ = State::output;
            break;

        case State::party_trick: // func8
            controller.SetPartyTrick();
            // controller.FrontWallCorrection();
            break;

        case State::test_slalom2: // func9
            controller.StartMove();
            // controller.Acceleration(AccType::start);
            controller.Turn(SlalomType::right_90);
            // for (int i = 0; i < 5; i++)
            // {
            //     controller.Acceleration(AccType::forward, 1);
            //     wallData = controller.getWallData(); // front wall correction
            //     controller.Turn(SlalomType::right_90);
            //     wallData = controller.getWallData(); // front wall correction
            //     controller.Turn(SlalomType::right_90);
            // }
            // controller.Turn(SlalomType::right_45);
            // controller.Turn(SlalomType::left_45);
            controller.Acceleration(AccType::stop);
            controller.Brake();
            state.log_ = State::slalom;
            state.mode_ = State::output;
            break;

        case State::test_slalom1: // func10
            // controller.StartMove();
            controller.Acceleration(AccType::start);
            // HAL_Delay(1);
            wallData = controller.getWallData(); // front wall correction
            controller.Turn(SlalomType::right_90);
            // HAL_Delay(1);
            wallData = controller.getWallData(); // front wall correction
            controller.Turn(SlalomType::right_90);
            // HAL_Delay(1);
            wallData = controller.getWallData(); // front wall correction
            controller.Turn(SlalomType::left_90);
            // HAL_Delay(1);
            wallData = controller.getWallData(); // front wall correction
            controller.Turn(SlalomType::left_90);
            controller.Acceleration(AccType::stop);
            // controller.FrontWallCorrection();
            state.log_ = State::slalom;
            state.mode_ = State::output;
            break;

        case State::test_translation2: // func11
            controller.StartMove();
            for (int i = 0; i < 4; i++)
            {
                controller.Acceleration(AccType::forward, 1);
                controller.Acceleration(AccType::stop);
                controller.FrontWallCorrection();
                controller.PivotTurn(-90);
                controller.Acceleration(AccType::start_half);
                controller.Acceleration(AccType::stop);
                controller.FrontWallCorrection();
                controller.PivotTurn(-90);
                controller.Acceleration(AccType::start_half);
                // controller.Turn(SlalomType::right_90);
                // controller.Turn(SlalomType::right_90);
            }
            controller.Acceleration(AccType::stop);
            state.log_ = State::translation;
            state.mode_ = State::output;
            break;

        case State::test_translation1: // func12
            controller.StartMove();
            // controller.Acceleration(AccType::start_half);
            // controller.GoStraight();
            controller.Acceleration(AccType::forward, 1);
            controller.Acceleration(AccType::stop);
            controller.Brake();
            state.log_ = State::translation;
            state.mode_ = State::output;
            break;

        case State::test_rotation: // func13
            for (int i = 0; i < 40; i++)
                controller.PivotTurn(90);
            // for (int i = 0; i < 4; i++)
            //     controller.PivotTurn(180);
            state.log_ = State::pivot_turn;
            state.mode_ = State::output;
            break;

        case State::test_ir: // func15
            ir_sensors.UI_led_onoff(ir_value);
            // controller.MotorTest(0.0, 5.0);
            break;

        case State::output:
            controller.Brake();
            state.interruption_ = State::not_interrupt;

            if (Read_GPIO(USER_SW) == 0)
            {
                if (state.log_ == State::slalom)
                    controller.OutputSlalomLog();
                else if (state.log_ == State::m_iden)
                    controller.OutputMIdenLog();
                else if (state.log_ == State::step_iden)
                    controller.OutputStepIdenLog();
                else if (state.log_ == State::pivot_turn)
                    controller.OutputPivotTurnLog();
                else if (state.log_ == State::translation)
                    controller.OutputTranslationLog();
                else if (state.log_ == State::maze)
                {
                    for (int y = 0; y < MAZE_SIZE; y++)
                        for (int x = 0; x < MAZE_SIZE; x++)
                        {
                            printf("maze.wall[%d][%d].byte = %d\n", y, x, maze.wall[y][x].byte);
                            printf("maze_backup.wall[%d][%d].byte = %d\n", y, x, maze_backup.wall[y][x].byte);
                        }
                }
            }
            break;

        default:
            break;
        }
    }
}

void UpdateMovement()
{
    UpdateUndercarriage();
    SetIRSensor();
    controller.robotMove();

    if (controller.ErrorFlag())
    {
        controller.Brake();
        state.interruption_ = State::not_interrupt;
    }

    if (controller.ErrorFlag() && state.interruption_ == State::not_interrupt)
    {
        if (state.mode_ == State::search)
        {
            FlashMaze();
            Notification();
        }
    }

    if (controller.GetMazeFlashFlag())
    {
        maze_backup = maze;
        state.interruption_ = State::not_interrupt;
    }
}

bool GetInterruptionFlag()
{
    if (state.interruption_ == State::interrupt)
        return true;
    else
        return false;
}

void MazeSearch()
{
    while (1)
    {
        while (1)
        {
            if (controller.wallDataReady())
            {
                controller.ResetWallFlag();
                break;
            }
        }
        // controller.ResetWallFlag();
        ir_sensors.UI_led_onoff(controller.GetIRWall());
        // ir_sensors.PrintWalldata(controller.GetIRWall());
        controller.UpdateDir(nextDir);
        controller.UpdatePos(nextDir);
        wallData = controller.getWallData();
        robotPos = controller.getRobotPosition();
        agent.update(robotPos, wallData);
        if (agent.getState() == Agent::FINISHED)
        {
            controller.Acceleration(AccType::stop);
            controller.Brake();
            maze_backup = maze;
            break;
        }
        else if (prevState == Agent::SEARCHING_NOT_GOAL && agent.getState() == Agent::SEARCHING_REACHED_GOAL)
        {
            controller.Acceleration(AccType::stop);
            controller.Brake();
            maze_backup = maze;
            break;
            // maze_backup = maze;
            // state.interruption_ = State::not_interrupt;
            // FlashMaze();
            // state.interruption_ = State::interrupt;
        }
        else if (cnt1Hz > search_time && agent.getState() == Agent::SEARCHING_REACHED_GOAL)
        {
            agent.forceGotoStart();
        }
        nextDir = agent.getNextDirection();
        // printf("nextDir.byte = %d\n", nextDir.byte);
        controller.DirMove(nextDir);
        prevState = agent.getState();
    }
}

void TimeAttack()
{
    // get command list
    runSequence = agent.getRunSequence();
    runSequence.print();
    controller.CalcOpMovedState(runSequence);
    robotPos = controller.getRobotPosition();

    for (size_t i = 0; i < runSequence.size(); i++)
        controller.OpMove(runSequence[i]);
    controller.Acceleration(AccType::stop);
    controller.Brake();
}

bool GetStartCountFlag()
{
    return flag_start_cnt;
}

void CheckMazeFlashFlag()
{
    if (controller.GetMazeFlashFlag())
    {
        // speaker.Beep(20);
        FlashMaze();
        controller.ResetMazeLoadFlag();
        // speaker.Beep(20);
        state.interruption_ = State::interrupt;
    }
}

void FlashMaze()
{
    Flash_clear();
    // uint32_t *flash_data = (uint32_t *)GetWorkRamPointer();
    uint8_t *flash_data = GetWorkRamPointer();

    for (int y = 0; y < MAZE_SIZE; y++)
        for (int x = 0; x < MAZE_SIZE; x++)
            flash_data[MAZE_SIZE * y + x] = maze_backup.wall[y][x].byte;
    // flash_data[MAZE_SIZE * y + x] = maze.wall[y][x].byte;

    Flash_store();
    // if (!Flash_store())
    //     while (1)
    //         printf("Failed to write flash\n");
}

void LoadMaze()
{
    // uint32_t *flash_data = (uint32_t *)Flash_load();
    uint8_t *flash_data = Flash_load();

    for (int y = 0; y < MAZE_SIZE; y++)
        for (int x = 0; x < MAZE_SIZE; x++)
        {
            maze.wall[y][x].byte = flash_data[MAZE_SIZE * y + x];
            printf("maze.wall[%d][%d].byte = %d\n", y, x, maze.wall[y][x].byte);
        }
}

void PrintLog()
{
    if (state.mode_ == State::test_ir)
    {
        ir_sensors.PrintLog();
        // printf("cur_pos.x = %.3f\n", controller.GetFrontWallPos((ir_value.fl3 + ir_value.fr3) * 0.5));
    }
    else if (state.mode_ == State::test_odometory)
        controller.OutputLog();
}