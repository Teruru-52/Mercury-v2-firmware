/**
 * @file filter_base.h
 * @author Teruru-52
 */

#ifndef FILTER_BASE_H_
#define FILTER_BASE_H_

#include "main.h"

namespace filter {
class FilterBase {
public:
  FilterBase() : pre_value_(0.0f), value_(0.0f) {}
  virtual ~FilterBase() = default;

  virtual void Update(float value) = 0;
  virtual void Reset() = 0;
  float GetValue() const { return value_; }

protected:
  float pre_value_ = 0.0f;
  float value_ = 0.0f;
};
} // namespace filter

#endif // FILTER_BASE_H_