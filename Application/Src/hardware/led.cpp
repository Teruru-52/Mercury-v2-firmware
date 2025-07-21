/**
 * @file led.cpp
 * @author Teruru-52
 */

#include "hardware/led.h"

hardware::LED led;

namespace hardware {
void LED::OnFrontLeft1() { Write_GPIO(LED_FRONT_LEFT1, GPIO_PIN_SET); }

void LED::OnFrontLeft2() { Write_GPIO(LED_FRONT_LEFT2, GPIO_PIN_SET); }

void LED::OnFrontLeft3() { Write_GPIO(LED_FRONT_LEFT3, GPIO_PIN_SET); }

void LED::OnFrontRight1() { Write_GPIO(LED_FRONT_RIGHT1, GPIO_PIN_SET); }

void LED::OnFrontRight2() { Write_GPIO(LED_FRONT_RIGHT2, GPIO_PIN_SET); }

void LED::OnFrontRight3() { Write_GPIO(LED_FRONT_RIGHT3, GPIO_PIN_SET); }

void LED::OnBrake() { Write_GPIO(LED_RED, GPIO_PIN_SET); }

void LED::OnTaleLeft() { Write_GPIO(LED_TALE_LEFT, GPIO_PIN_SET); }

void LED::OnTaleRight() { Write_GPIO(LED_TALE_RIGHT, GPIO_PIN_SET); }

void LED::OffFrontLeft1() { Write_GPIO(LED_FRONT_LEFT1, GPIO_PIN_RESET); }

void LED::OffFrontLeft2() { Write_GPIO(LED_FRONT_LEFT2, GPIO_PIN_RESET); }

void LED::OffFrontLeft3() { Write_GPIO(LED_FRONT_LEFT3, GPIO_PIN_RESET); }

void LED::OffFrontRight1() { Write_GPIO(LED_FRONT_RIGHT1, GPIO_PIN_RESET); }

void LED::OffFrontRight2() { Write_GPIO(LED_FRONT_RIGHT2, GPIO_PIN_RESET); }

void LED::OffFrontRight3() { Write_GPIO(LED_FRONT_RIGHT3, GPIO_PIN_RESET); }

void LED::OffBrake() { Write_GPIO(LED_RED, GPIO_PIN_RESET); }

void LED::OffTaleLeft() { Write_GPIO(LED_TALE_LEFT, GPIO_PIN_RESET); }

void LED::OffTaleRight() { Write_GPIO(LED_TALE_RIGHT, GPIO_PIN_RESET); }

void LED::OnAll() {
  OnFrontLeft1();
  // OnFrontLeft2();
  // OnFrontLeft3();
  OnFrontRight1();
  OnFrontRight2();
  OnFrontRight3();
}

void LED::OffAll() {
  OffFrontLeft1();
  OffFrontLeft2();
  OffFrontLeft3();
  OffFrontRight1();
  OffFrontRight2();
  OffFrontRight3();
}

void LED::Func0() { OffAll(); }

void LED::Func1() {
  OnFrontRight3();
  OffFrontRight2();
  OffFrontRight1();
  OffFrontLeft1();
}

void LED::Func2() {
  OffFrontRight3();
  OnFrontRight2();
  OffFrontRight1();
  OffFrontLeft1();
}

void LED::Func3() {
  OnFrontRight3();
  OnFrontRight2();
  OffFrontRight1();
  OffFrontLeft1();
}

void LED::Func4() {
  OffFrontRight3();
  OffFrontRight2();
  OnFrontRight1();
  OffFrontLeft1();
}

void LED::Func5() {
  OnFrontRight3();
  OffFrontRight2();
  OnFrontRight1();
  OffFrontLeft1();
}

void LED::Func6() {
  OffFrontRight3();
  OnFrontRight2();
  OnFrontRight1();
  OffFrontLeft1();
}

void LED::Func7() {
  OnFrontRight3();
  OnFrontRight2();
  OnFrontRight1();
  OffFrontLeft1();
}

void LED::Func8() {
  OffFrontRight3();
  OffFrontRight2();
  OffFrontRight1();
  OnFrontLeft1();
}

void LED::Func9() {
  OnFrontRight3();
  OffFrontRight2();
  OffFrontRight1();
  OnFrontLeft1();
}

void LED::Func10() {
  OffFrontRight3();
  OnFrontRight2();
  OffFrontRight1();
  OnFrontLeft1();
}

void LED::Func11() {
  OnFrontRight3();
  OnFrontRight2();
  OffFrontRight1();
  OnFrontLeft1();
}

void LED::Func12() {
  OffFrontRight3();
  OffFrontRight2();
  OnFrontRight1();
  OnFrontLeft1();
}

void LED::Func13() {
  OnFrontRight3();
  OffFrontRight2();
  OnFrontRight1();
  OnFrontLeft1();
}

void LED::Func14() {
  OffFrontRight3();
  OnFrontRight2();
  OnFrontRight1();
  OnFrontLeft1();
}

void LED::Func15() { OnAll(); }

void LED::Flash() {
  for (int i = 0; i < 2; i++) {
    OnAll();
    HAL_Delay(500);
    OffAll();
    HAL_Delay(500);
  }
}

}  // namespace hardware
