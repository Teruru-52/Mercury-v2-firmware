/**
 * @file controller.cpp
 * @author Teruru-52
 */

#include "controller/controller.h"

namespace undercarriage {
Controller::Controller(undercarriage::Odometory *odom, PID_Instances *pid,
                       undercarriage::TrackerBase *tracker,
                       trajectory::Slalom *slalom,
                       trajectory::Acceleration *acc,
                       hardware::IR_Param *ir_param,
                       trajectory::Velocity *velocity)
    : odom(odom), pid(pid), tracker(tracker), slalom(slalom), acc(acc),
      mode_ctrl(stop), ir_param(ir_param), velocity(velocity) {
  if (ENABLE_LOG) {
    ref_size = slalom->GetRefSize();
    // ref_size = acc->GetRefSize();
    // ref_size = pivot_turn180.GetRefSize();
    // ref_size = pivot_turn90.GetRefSize();
    // ref_size = BUFFER_SIZE;
  }
}

void Controller::UpdateOdometory() {
  odom->Update();
  cur_pos = odom->GetPosition();
  theta_global = theta_base - theta_error + cur_pos.th;
  cur_vel = odom->GetVelocity();
  length = odom->GetLength();
  acc_x = odom->GetAccX();
}

void Controller::SetIRdata(const IR_Value &ir_value) {
  ir_value_ = ir_value;
  if (slalom->GetWallFlag() || acc->GetWallFlag()) {
    updateWallData();
    slalom->ResetWallFlag();
    acc->ResetWallFlag();
    flag_wall = true; // read wall
                      // speaker.ToggleSpeaker();
  }
}

void Controller::SetTrajectoryMode(int trj_mode) {
  slalom->SetMode(trj_mode);
  acc->SetMode(trj_mode);
}

bool Controller::ErrorFlag() {
  if (acc_x < -acc_x_err)
    flag_safety = true;
  return flag_safety;
}

void Controller::SetM_Iden() {
  mode_ctrl = m_iden;
  while (1) {
    if (flag_ctrl) {
      ResetCtrl();
      break;
    }
  }
}

void Controller::SetStep_Iden() {
  mode_ctrl = step_iden;
  while (1) {
    if (flag_ctrl) {
      ResetCtrl();
      break;
    }
  }
}

void Controller::SetPartyTrick() {
  mode_ctrl = party_trick;
  while (1) {
  }
}

void Controller::PivotTurn(int angle) {
  if (angle == 90) {
    mode_ctrl = pivot_turn_left_90;
    Write_GPIO(LED_TALE_LEFT, GPIO_PIN_SET);
  } else if (angle == -90) {
    mode_ctrl = pivot_turn_right_90;
    Write_GPIO(LED_TALE_RIGHT, GPIO_PIN_SET);
  } else if (angle == 180)
    mode_ctrl = pivot_turn_180;
  ref_theta = static_cast<float>(angle) * M_PI / 180.0;

  while (1) {
    if (flag_ctrl) {
      theta_base += ref_theta;
      theta_error += (ref_theta - cur_pos.th);
      // printf("theta_base: %.1f\n", theta_base);
      ResetCtrl();
      break;
    }
  }
  Write_GPIO(LED_TALE_LEFT, GPIO_PIN_RESET);
  Write_GPIO(LED_TALE_RIGHT, GPIO_PIN_RESET);
}

void Controller::Turn(const SlalomType &slalom_type) {
  slalom_type_ = slalom_type;
  switch (slalom_type_) {
  case SlalomType::left_90:
    ref_theta = M_PI * 0.5f;
    Write_GPIO(LED_TALE_LEFT, GPIO_PIN_SET);
    break;
  case SlalomType::right_90:
    ref_theta = -M_PI * 0.5f;
    Write_GPIO(LED_TALE_RIGHT, GPIO_PIN_SET);
    break;
  case SlalomType::left_45:
    ref_theta = M_PI * 0.25f;
    Write_GPIO(LED_TALE_LEFT, GPIO_PIN_SET);
    break;
  case SlalomType::right_45:
    ref_theta = -M_PI * 0.25f;
    Write_GPIO(LED_TALE_RIGHT, GPIO_PIN_SET);
    break;
  default:
    ref_theta = 0.0;
    break;
  }

  if (flag_wall_front && ENABLE_SLALOM_CORRECTION) // 前壁があれば前壁補正
  {
    if (!flag_slalom) // 前壁があり，slalomが始まっていなければ前壁補正
    {
      // Write_GPIO(LED_RED, G?PIO_PIN_SET);
      Toggle_GPIO(LED_RED);
      // float ir_fmean = (ir_value_.fl3 + ir_value_.fr3) * 0.5f;
      float ir_fmean = ir_value_.fr3;
      x_diff = GetFrontWallPos(ir_fmean);
      if (ENABLE_LOG)
        LoggerWall();
      flag_slalom = true;
      slalom->ResetTrajectory(slalom_type_, ref_theta, x_diff);
    }
  } else
    slalom->ResetTrajectory(slalom_type_, ref_theta);
  // slalom->ResetTrajectory(angle, ref_theta - theta_error);

  // tracker->SetXi(cur_vel.x);
  mode_ctrl = turn;

  while (1) {
    if (flag_ctrl) {
      theta_base += ref_theta;
      theta_error += (ref_theta - cur_pos.th);
      // printf("theta_base: %.1f\n", theta_base);
      ResetCtrl();
      break;
    }
  }
  Write_GPIO(LED_TALE_LEFT, GPIO_PIN_RESET);
  Write_GPIO(LED_TALE_RIGHT, GPIO_PIN_RESET);
}

void Controller::Acceleration(const AccType &acc_type, uint8_t num_square) {
  acc->ResetTrajectory(acc_type, cur_vel.x, num_square);
  mode_ctrl = acc_curve;
  if (acc_type == AccType::forward)
    flag_side_correct = true;
  while (1) {
    if (flag_ctrl) {
      theta_error += -cur_pos.th;
      ResetCtrl();
      break;
    }
  }
}

void Controller::FrontWallCorrection() {
  mode_ctrl = front_wall_correction;
  Write_GPIO(LED_RED, GPIO_PIN_SET);
  while (1) {
    if (flag_ctrl) {
      ResetCtrl();
      Write_GPIO(LED_RED, GPIO_PIN_RESET);
      break;
    }
  }
}

void Controller::Back() {
  mode_ctrl = back;
  while (1) {
    if (flag_ctrl) {
      odom->ResetTheta();
      theta_base = 0.0;
      // printf("theta_base: %.1f\n", theta_base);
      theta_error = 0.0;
      ResetCtrl();
      break;
    }
  }
}

void Controller::Wait_ms() {
  mode_ctrl = wait;
  while (1) {
    if (flag_ctrl) {
      ResetCtrl();
      break;
    }
  }
}

void Controller::M_Iden() {
  u_w = iden_m.GetInput(cur_vel.th);
  InputVelocity(0, u_w);

  // u_v = iden_m.GetInput(cur_vel.x);
  // InputVelocity(u_v, 0);

  if (iden_m.Finished()) {
    Brake();
    flag_ctrl = true;
  }
}

void Controller::Step_Iden() {
  // u_w = iden_step.GetInput(cur_vel.th);
  // InputVelocity(0, u_w);

  u_v = iden_step.GetInput(cur_vel.x);
  InputVelocity(u_v, 0);

  if (iden_step.Finished()) {
    Brake();
    flag_ctrl = true;
  }
}

void Controller::PartyTrick() {
  // u_v = pid->trans_vel->Update(-cur_vel[0]);
  u_v = 0.0;
  u_w = pid->angle->Update(-cur_pos.th) + pid->rot_vel->Update(-cur_vel.th);
  InputVelocity(u_v, u_w);
}

void Controller::SideWallCorrection() {
  if (flag_wall_sl) {
    if (ir_value_.fl2 > ir_param->is_wall.fl2)
      error_fl = ir_param->ctrl_base.fl2 - static_cast<float>(ir_value_.fl2);
    else
      error_fl = 0.0;
    if (ir_value_.fr2 < ir_param->is_wall.fr2)
      error_fl *= 1.8;
  } else
    error_fl = 0;

  if (flag_wall_sr) {
    if (ir_value_.fr2 > ir_param->is_wall.fr2)
      error_fr = ir_param->ctrl_base.fr2 - static_cast<float>(ir_value_.fr2);
    else
      error_fr = 0.0;
    if (ir_value_.fl2 < ir_param->is_wall.fl2)
      error_fr *= 1.8;
  } else
    error_fr = 0;
}

void Controller::PivotTurn() {
  if (mode_ctrl == pivot_turn_right_90) {
    pivot_turn90.UpdateRef();
    ref_vel.th = -pivot_turn90.GetRefVelocity();
    ref_acc.th = -pivot_turn90.GetRefAcceleration();
  } else if (mode_ctrl == pivot_turn_left_90) {
    pivot_turn90.UpdateRef();
    ref_vel.th = pivot_turn90.GetRefVelocity();
    ref_acc.th = pivot_turn90.GetRefAcceleration();
  } else if (mode_ctrl == pivot_turn_180) {
    pivot_turn180.UpdateRef();
    ref_vel.th = pivot_turn180.GetRefVelocity();
    ref_acc.th = pivot_turn180.GetRefAcceleration();
  }
  // u_v = pid->trans_vel->Update(-cur_vel[0]);
  u_v = 0.0;
  u_w = pid->rot_vel->Update(ref_vel.th - cur_vel.th) + ref_vel.th / Kp_w;
  // u_w = pid->rot_vel->Update(ref_vel.th - cur_vel.th) + (Tp1_w * ref_acc.th +
  // ref_vel.th) / Kp_w;
  InputVelocity(u_v, u_w);
  // if (ENABLE_LOG)
  //     Logger();
  if (pivot_turn90.Finished() || pivot_turn180.Finished()) {
    Brake();
    flag_ctrl = true;
  }
}

float Controller::GetFrontWallPos(float ir_fmean) {
  // logaritmic function
  // if (ir_param->log.b * ir_fmean + ir_param->log.c > 0)
  //     return ir_param->log.a * log(ir_param->log.b * ir_fmean +
  //     ir_param->log.c) + ir_param->log.d;
  // else
  //     return ir_param->log.a * log(1e-10) + ir_param->log.d;

  // table reference
  auto compareDistance = [ir_fmean](const xy_pair &a, const xy_pair &b) {
    return std::abs(a.first - ir_fmean) < std::abs(b.first - ir_fmean);
  };
  auto nearestIterator =
      std::min_element(ir_table.begin(), ir_table.end(), compareDistance);
  return nearestIterator->second;
};

void Controller::Turn() {
  slalom->UpdateRef();
  ref_pos = slalom->GetRefPosition();
  ref_vel = slalom->GetRefVelocity();
  ref_acc = slalom->GetRefAcceleration();

  tracker->UpdateRef(ref_pos, ref_vel, ref_acc);
  ref_vel_ctrl = tracker->CalcInput(cur_pos, cur_vel);
  // u_v = pid->trans_vel->Update(ref_vel_ctrl.x - cur_vel.x); // without
  // feedforward
  u_v = pid->trans_vel->Update(ref_vel_ctrl.x - cur_vel.x) +
        ref_vel_ctrl.x / Kp_v; // with feedforward
  u_w = pid->rot_vel->Update(ref_vel_ctrl.th - cur_vel.th) +
        ref_vel_ctrl.th / Kp_w;

  InputVelocity(u_v, u_w);
  // if (ENABLE_LOG)
  //     Logger();
  if (slalom->Finished())
    flag_ctrl = true;
}

void Controller::Acceleration() {
  acc->UpdateRef();
  ref_pos.x = acc->GetRefPosition();
  ref_pos.y = 0.0;
  ref_pos.th = theta_base;
  ref_vel.x = acc->GetRefVelocity();
  ref_vel.y = 0.0;
  ref_acc.x = acc->GetRefAcceleration();
  ref_acc.y = 0.0;
  // if (ENABLE_LOG)
  //     Logger();
  if (abs(ref_vel.x > 1000))
    ref_vel.x = 0;
  if (abs(ref_acc.x > 5000))
    ref_acc.x = 0;
  u_v = pid->trans_vel->Update(ref_vel.x - cur_vel.x) +
        (Tp1_v * ref_acc.x + ref_vel.x) / Kp_v;
  // u_v = pid->trans_vel->Update(ref_vel.x - cur_vel.x);

  if (flag_side_correct) {
    SideWallCorrection();
    u_w = pid->ir_side->Update(error_fl - error_fr) +
          pid->angle->Update(theta_base - theta_global);
  } else
    u_w = pid->angle->Update(theta_base - theta_global);

  InputVelocity(u_v, u_w);
  if (acc->Finished())
    flag_ctrl = true;
}

void Controller::Back(int time) {
  if (cnt_time < time) {
    InputVelocity(-0.5, 0.0);
    cnt_time++;
  } else {
    Brake();
    odom->ResetEncoder();
    flag_ctrl = true;
  }
}

void Controller::Wait_ms(int time) {
  if (cnt_time < time) {
    motor.Brake();
    cnt_time++;
  } else
    flag_ctrl = true;
}

void Controller::FrontWallCorrection(const IR_Value &ir_value) {
  if (cnt_time < correction_time) {
    float error_left =
        ir_param->ctrl_base.fl3 - static_cast<float>(ir_value_.fl3);
    float error_right =
        ir_param->ctrl_base.fr3 - static_cast<float>(ir_value_.fr3);
    v_left = pid->ir_front_l->Update(error_left);
    v_right = pid->ir_front_r->Update(error_right);
    motor.Drive(v_left, v_right);
    cnt_time++;
  } else
    flag_ctrl = true;
}

void Controller::BlindAlley() {
  Acceleration(AccType::stop);
  FrontWallCorrection();
  PivotTurn(-90);
  FrontWallCorrection();
  PivotTurn(-90);
  Back();
  Brake();
  if (cnt_blind_alley >= CNT_BLIND_ALLAY) {
    flag_maze_flash = true;
    while (1) {
      Write_GPIO(LED_TALE_LEFT, GPIO_PIN_SET);
      Write_GPIO(LED_TALE_RIGHT, GPIO_PIN_SET);
      if (!flag_maze_flash)
        break;
    }
    cnt_blind_alley = 0;
  }
  Acceleration(AccType::start);
  cnt_can_back = 0;
}

void Controller::StartMove() {
  PivotTurn(90);
  FrontWallCorrection();
  PivotTurn(-90);
  Back();
  Acceleration(AccType::start);
}

void Controller::Brake() {
  mode_ctrl = stop;
  motor.Brake();
  pid->Reset();
}

void Controller::InputVelocity(float input_v, float input_w) {
  v_left = input_v - input_w;
  v_right = input_v + input_w;
  motor.Drive(v_left, v_right);
}

void Controller::ResetCtrl() {
  tracker->Reset();
  pivot_turn180.Reset();
  pivot_turn90.Reset();
  slalom->Reset();
  acc->Reset();
  // pid->Reset();
  // pid->angle->Reset();
  // pid->ir_front_l->Reset();
  // pid->ir_front_r->Reset();
  // pid->ir_side->Reset();

  odom->Reset();
  odom->ResetTheta();
  // theta_base = 0.0;
  // mode_ctrl = forward;

  flag_slalom = false;
  flag_side_correct = false;
  index_log = 0;
  cnt_time = 0;
  flag_ctrl = false;
}

Direction Controller::getWallData() {
  Direction wall;

  // int8_t robot_dir_index = 0;
  // while (1)
  // {
  //     if (robot_dir.byte == NORTH << robot_dir_index)
  //         break;
  //     robot_dir_index++;
  // }
  int8_t robot_dir_index = __builtin_ctz(robot_dir.byte);

  if (ir_wall_value.fl3 > ir_param->is_wall.fl3 ||
      ir_wall_value.fr3 > ir_param->is_wall.fr3) {
    wall.byte |= NORTH << robot_dir_index;
    flag_wall_front = true;
  } else
    flag_wall_front = false;

  if (ir_wall_value.fl2 > ir_param->is_wall.fl2) {
    wall.byte |= NORTH << (robot_dir_index + 3) % 4;
    flag_wall_sl = true;
  } else
    flag_wall_sl = false;

  if (ir_wall_value.fr2 > ir_param->is_wall.fr2) {
    wall.byte |= NORTH << (robot_dir_index + 1) % 4;
    flag_wall_sr = true;
  } else
    flag_wall_sr = false;

  prev_wall_cnt = wall.nWall();

  return wall;
}

void Controller::UpdatePos(const Direction &dir) {
  if (NORTH == dir.byte)
    robot_position += IndexVec::vecNorth;
  else if (SOUTH == dir.byte)
    robot_position += IndexVec::vecSouth;
  else if (EAST == dir.byte)
    robot_position += IndexVec::vecEast;
  else if (WEST == dir.byte)
    robot_position += IndexVec::vecWest;
}

void Controller::robotMove() {
  switch (mode_ctrl) {
  case acc_curve:
    Acceleration();
    break;
  case turn:
    Turn();
    break;
  case pivot_turn_right_90:
    PivotTurn();
    break;
  case pivot_turn_left_90:
    PivotTurn();
    break;
  case pivot_turn_180:
    PivotTurn();
    break;
  case front_wall_correction:
    FrontWallCorrection(ir_value_);
    break;
  case back:
    Back(back_time);
    break;
  case m_iden:
    M_Iden();
    break;
  case step_iden:
    Step_Iden();
    break;
  case party_trick:
    PartyTrick();
    break;
  case stop:
    Brake();
    break;
  case wait:
    Wait_ms(wait_time);
    break;
  default:
    break;
  }
}

void Controller::DirMove(const Direction &dir) {
  int8_t robot_dir_index = 0;
  while (1) {
    if (robot_dir.byte == NORTH << robot_dir_index)
      break;
    robot_dir_index++;
  }

  int8_t next_dir_index = 0;
  while (1) {
    if (dir.byte == NORTH << next_dir_index)
      break;
    next_dir_index++;
  }

  dir_diff = next_dir_index - robot_dir_index;
  // printf("dir_diff = %d\n", dir_diff);
  // Straight Line
  if (dir_diff == 0)
    Acceleration(AccType::forward, 1);
  // Turn Right
  else if (dir_diff == 1 || dir_diff == -3) {
    if (ENABLE_SLALOM)
      Turn(SlalomType::right_90);
    else {
      Acceleration(AccType::stop);
      if (ir_value_.fl3 > ir_param->is_wall.fl3 &&
          ir_value_.fr3 > ir_param->is_wall.fr3)
        FrontWallCorrection();
      PivotTurn(-90);
      if (prev_wall_cnt == 2)
        cnt_can_back++;
      if (cnt_can_back >= CNT_BACK && flag_wall_front && flag_wall_sl) {
        Back();
        Acceleration(AccType::start);
        cnt_can_back = 0;
      } else
        Acceleration(AccType::start_half);
    }
  }
  // Turn Left
  else if (dir_diff == -1 || dir_diff == 3) {
    if (ENABLE_SLALOM)
      Turn(SlalomType::left_90);
    else {
      Acceleration(AccType::stop);
      if (ir_value_.fl3 > ir_param->is_wall.fl3 &&
          ir_value_.fr3 > ir_param->is_wall.fr3)
        FrontWallCorrection();
      PivotTurn(90);
      if (prev_wall_cnt == 2)
        cnt_can_back++;
      if (cnt_can_back >= CNT_BACK && flag_wall_front && flag_wall_sl) {
        Back();
        Acceleration(AccType::start);
        cnt_can_back = 0;
      } else
        Acceleration(AccType::start_half);
    }
  }
  // U-Turn
  else {
    if (prev_wall_cnt == 3) // Blind Alley
    {
      cnt_blind_alley++;
      BlindAlley();
    } else {
      Acceleration(AccType::stop);
      PivotTurn(180);
      Acceleration(AccType::start_half);
    }
  }
}

void Controller::OpMove(const Operation &op) {
  switch (op.op) {
  case Operation::START:
    StartMove();
    break;
  case Operation::FORWARD:
    flag_side_correct = true;
    Acceleration(AccType::forward, op.n);
    break;

  case Operation::TURN_LEFT90:
    if (ENABLE_SLALOM)
      Turn(SlalomType::left_90);
    else {
      Acceleration(AccType::stop);
      if (ir_value_.fl3 > ir_param->is_wall.fl3 &&
          ir_value_.fr3 > ir_param->is_wall.fr3)
        FrontWallCorrection();
      PivotTurn(90);
      Acceleration(AccType::start_half);
    }
    break;

  case Operation::TURN_RIGHT90:
    if (ENABLE_SLALOM)
      Turn(SlalomType::right_90);
    else {
      Acceleration(AccType::stop);
      if (ir_value_.fl3 > ir_param->is_wall.fl3 &&
          ir_value_.fr3 > ir_param->is_wall.fr3)
        FrontWallCorrection();
      PivotTurn(-90);
      Acceleration(AccType::start_half);
    }
    break;

  case Operation::TURN_LEFT45: // only for slalom
    Turn(SlalomType::left_45);
    break;

  case Operation::TURN_RIGHT45: // only for slalom
    Turn(SlalomType::right_45);

  case Operation::STOP:
    Acceleration(AccType::stop);
    Brake();
    break;
  default:
    break;
  }
}

void Controller::CalcOpMovedState(const OperationList &runSequence) {
  robot_position = IndexVec(0, 0);
  robot_dir = NORTH;
  for (size_t i = 0; i < runSequence.size(); i++) {
    switch (runSequence[i].op) {
    case Operation::START:
      robot_position = IndexVec::vecNorth;
      break;
    case Operation::FORWARD:
      if (robot_dir.byte == NORTH)
        robot_position += IndexVec::vecNorth;
      else if (robot_dir.byte == EAST)
        robot_position += IndexVec::vecEast;
      else if (robot_dir.byte == SOUTH)
        robot_position += IndexVec::vecSouth;
      else if (robot_dir.byte == WEST)
        robot_position += IndexVec::vecWest;
      break;
    case Operation::TURN_LEFT90:
      if (robot_dir.byte == NORTH) {
        robot_position += IndexVec::vecWest;
        robot_dir.byte = WEST;
      } else if (robot_dir.byte == EAST) {
        robot_position += IndexVec::vecNorth;
        robot_dir.byte = NORTH;
      } else if (robot_dir.byte == SOUTH) {
        robot_position += IndexVec::vecEast;
        robot_dir.byte = EAST;
      } else if (robot_dir.byte == WEST) {
        robot_position += IndexVec::vecSouth;
        robot_dir.byte = SOUTH;
      }
      break;
    case Operation::TURN_RIGHT90:
      if (robot_dir.byte == NORTH) {
        robot_position += IndexVec::vecEast;
        robot_dir.byte = EAST;
      } else if (robot_dir.byte == EAST) {
        robot_position += IndexVec::vecSouth;
        robot_dir.byte = SOUTH;
      } else if (robot_dir.byte == SOUTH) {
        robot_position += IndexVec::vecWest;
        robot_dir.byte = WEST;
      } else if (robot_dir.byte == WEST) {
        robot_position += IndexVec::vecNorth;
        robot_dir.byte = NORTH;
      }
      break;
    default:
      break;
    }
  }
  // printf("IndexVec(%d, %d)\n", robot_position.x, robot_position.y);
  // printf("robot_dir.byte = %d\n", robot_dir.byte);
}

void Controller::Logger() {
  static int cnt_log = 0;
  if (cnt_log % CNT_LOG == 0) {
    log_x[index_log] = cur_pos.x;
    log_y[index_log] = cur_pos.y;
    // log_l[index_log] = length;
    log_theta[index_log] = cur_pos.th;
    log_omega[index_log] = cur_vel.th;
    log_v[index_log] = cur_vel.x;
    log_a[index_log] = acc_x;
    log_ref_x[index_log] = ref_pos.x;
    log_ref_y[index_log] = ref_pos.y;
    log_ref_theta[index_log] = ref_pos.th;
    log_ref_omega[index_log] = ref_vel.th;
    log_ref_v[index_log] = ref_vel.x;

    if (mode_ctrl == turn) {
      log_ref_a[index_log] =
          ref_acc.x * cos(ref_pos.th) + ref_acc.y * sin(ref_pos.th);
      log_ctrl_v[index_log] = ref_vel_ctrl.x;
      log_ctrl_w[index_log] = ref_vel_ctrl.th;
    }
    if (mode_ctrl == acc_curve || mode_ctrl == forward)
      log_ref_a[index_log] = ref_acc.x;

    log_u_v[index_log] = u_v;
    log_u_w[index_log] = u_w;
    index_log = (index_log + 1) % BUFFER_SIZE;
  }
  cnt_log = (cnt_log + 1) % ref_size;
}

void Controller::LoggerWall() {
  static int cnt_wall = 0;
  log_x_diff[cnt_wall] = x_diff;
  cnt_wall++;
  if (cnt_wall >= 10)
    cnt_wall = 0;
}

void Controller::OutputLog() {
  // odom->OutputLog();
  motor.printLog();
}

void Controller::OutputSlalomLog() {
  ref_size = slalom->GetRefSize();
  for (int i = 0; i < ref_size / CNT_LOG; i++) {
    printf("%.2f, %.2f, %.3f, %.2f, %.3f, %.1f, ", log_x[i], log_y[i],
           log_theta[i], log_v[i], log_omega[i], log_a[i]);
    printf("%.2f, %.2f, %.3f, %.1f, %.3f, %.1f, ", log_ref_x[i], log_ref_y[i],
           log_ref_theta[i], log_ref_v[i], log_ref_omega[i], log_ref_a[i]);
    printf("%.2f, %.3f, %.3f, %.3f\n", log_ctrl_v[i], log_ctrl_w[i], log_u_v[i],
           log_u_w[i]);
  }
  // for (int i = 0; i < 10; i++)
  //     printf("%.2f, ", log_x_diff[i]);
}

void Controller::OutputPivotTurnLog() {
  if (mode_ctrl == pivot_turn_180)
    ref_size = pivot_turn180.GetRefSize();
  else
    ref_size = pivot_turn90.GetRefSize();
  // for (int i = 0; i < ref_size; i++)
  // printf("%.2f, %.2f\n", log_theta[i], log_omega[i]);
}

void Controller::OutputTranslationLog() {
  // ref_size = index_log;
  // ref_size = acc->GetRefSize();
  for (int i = 0; i < ref_size / CNT_LOG; i++) {
    printf("%.3f, %.3f, %.3f, %.3f, ", log_x[i], log_y[i], log_v[i], log_a[i]);
    printf("%.3f, %.3f, %.3f, ", log_ref_x[i], log_ref_v[i], log_ref_a[i]);
    printf("%.3f, %.3f\n", log_u_v[i], log_u_w[i]);
  }
}

void Controller::MotorTest(float v_left, float v_right) {
  motor.Drive(v_left, v_right); // voltage [V]
                                // motor.Free();
}
} // namespace undercarriage