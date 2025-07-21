/**
 * @file speaker.h
 * @author Teruru-52
 */

#ifndef HARDWARE_SPEAKER_H_
#define HARDWARE_SPEAKER_H_

#include "main.h"

namespace hardware {
class Speaker {
 private:
  TIM_HandleTypeDef* htim_;
  uint32_t TIM_CHANNEL_;
  uint32_t volume_ = 50;
  bool flag_ = false;

 public:
  Speaker(TIM_HandleTypeDef* htim, uint32_t TIM_CHANNEL);
  void Beep(uint32_t time_ms);
  void On();
  void Off();
  void Toggle();
};
}  // namespace hardware

extern hardware::Speaker speaker;

#endif  //  HARDWARE_SPEAKER_H_