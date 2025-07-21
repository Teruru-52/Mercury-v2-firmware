/**
 * @file mouse_utils.cpp
 * @author Teruru-52
 * @date January 13th, 2024
 */

#include "mouse_utils.h"

State::State()
    : func_(not_selected),
      mazeload_(not_load),
      interruption_(not_interrupt),
      mode_(select_function),
      log_(slalom) {}

void State::SelectFunction(int16_t pulse) {
  if (pulse < PULSE_DIFF) {
    led.Func0();
    func_ = func0;
  } else if (pulse < PULSE_DIFF * 2) {
    led.Func1();
    func_ = func1;
  } else if (pulse < PULSE_DIFF * 3) {
    led.Func2();
    func_ = func2;
  } else if (pulse < PULSE_DIFF * 4) {
    led.Func3();
    func_ = func3;
  } else if (pulse < PULSE_DIFF * 5) {
    led.Func4();
    func_ = func4;
  } else if (pulse < PULSE_DIFF * 6) {
    led.Func5();
    func_ = func5;
  } else if (pulse < PULSE_DIFF * 7) {
    led.Func6();
    func_ = func6;
  } else if (pulse < PULSE_DIFF * 8) {
    led.Func7();
    func_ = func7;
  } else if (pulse < PULSE_DIFF * 9) {
    led.Func8();
    func_ = func8;
  } else if (pulse < PULSE_DIFF * 10) {
    led.Func9();
    func_ = func9;
  } else if (pulse < PULSE_DIFF * 11) {
    led.Func10();
    func_ = func10;
  } else if (pulse < PULSE_DIFF * 12) {
    led.Func11();
    func_ = func11;
  } else if (pulse < PULSE_DIFF * 13) {
    led.Func12();
    func_ = func12;
  } else if (pulse < PULSE_DIFF * 14) {
    led.Func13();
    func_ = func13;
  } else if (pulse < PULSE_DIFF * 13) {
    led.Func12();
    func_ = func12;
  } else if (pulse < PULSE_DIFF * 14) {
    led.Func13();
    func_ = func13;
  } else if (pulse < PULSE_DIFF * 15) {
    led.Func14();
    func_ = func14;
  } else {
    led.Func15();
    func_ = func15;
  }
  if (func_ != pre_func_) speaker.Beep(50);
  pre_func_ = func_;
}

void State::SelectLoadMaze(int16_t pulse) {
  if (pulse < PULSE_HALF) {
    led.OffTaleRight();
    mazeload_ = not_load;
  } else {
    led.OnTaleRight();
    mazeload_ = load;
  }
}