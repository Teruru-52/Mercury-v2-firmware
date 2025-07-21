/**
 * @file led.h
 * @author Teruru-52
 */

#ifndef HARDWARE_LED_H_
#define HARDWARE_LED_H_

#include "main.h"

namespace hardware {
class LED {
 public:
  void OnFrontLeft1();
  void OnFrontLeft2();
  void OnFrontLeft3();
  void OnFrontRight1();
  void OnFrontRight2();
  void OnFrontRight3();

  void OnBrake();
  void OnTaleLeft();
  void OnTaleRight();

  void OffFrontLeft1();
  void OffFrontLeft2();
  void OffFrontLeft3();
  void OffFrontRight1();
  void OffFrontRight2();
  void OffFrontRight3();

  void OffBrake();
  void OffTaleLeft();
  void OffTaleRight();

  void OnAll();
  void OffAll();

  void Func0();
  void Func1();
  void Func2();
  void Func3();
  void Func4();
  void Func5();
  void Func6();
  void Func7();
  void Func8();
  void Func9();
  void Func10();
  void Func11();
  void Func12();
  void Func13();
  void Func14();
  void Func15();

  void Flash();
};
}  // namespace hardware

extern hardware::LED led;

#endif  //  HARDWARE_LED_H_