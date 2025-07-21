/**
 * @file speaker.cpp
 * @author Teruru-52
 */

#include "hardware/speaker.h"

hardware::Speaker speaker(&htim8, TIM_CHANNEL_3);

namespace hardware {

Speaker::Speaker(TIM_HandleTypeDef* htim, uint32_t TIM_CHANNEL)
    : htim_(htim), TIM_CHANNEL_(TIM_CHANNEL) {
  __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_, 0);
}

void Speaker::Beep(uint32_t time_ms) {
  __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_, volume_);
  HAL_Delay(time_ms);
  // osDelay(time_ms);
  __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_, 0);
  HAL_Delay(time_ms);
}

void Speaker::On() {
  __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_, volume_);
  flag_ = true;
}

void Speaker::Off() {
  __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_, 0);
  flag_ = false;
}

void Speaker::Toggle() {
  if (flag_) {
    flag_ = false;
    Off();
  } else {
    flag_ = true;
    On();
  }
}

}  // namespace hardware