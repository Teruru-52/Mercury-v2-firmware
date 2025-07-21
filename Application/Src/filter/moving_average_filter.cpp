/**
 * @file moving_average_filter.cpp
 * @author Teruru-52
 */

#include "filter/moving_average_filter.h"

namespace filter {

MovingAverageFilter::MovingAverageFilter(int filter_num)
    : filter_num_(filter_num) {}

void MovingAverageFilter::Update(float value) {
  if (buffer_.size() > filter_num_) buffer_.pop_front();
  buffer_.push_back(value);
  value_ = 0;
  for (auto& e : buffer_) value_ += e;
  value_ /= buffer_.size();
}

void MovingAverageFilter::Reset() {
  value_ = 0.0f;
  buffer_.clear();
}
}  // namespace filter
