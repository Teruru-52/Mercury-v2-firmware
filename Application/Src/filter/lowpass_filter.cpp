/**
 * @file lowpass_filter.cpp
 * @author Teruru-52
 */

#include "filter/lowpass_filter.h"
#include "filter/filter_base.h"

namespace filter {

LowpassFilter::LowpassFilter(float sampling_period, float cutoff_freq)
    : sampling_period_(sampling_period), cutoff_freq_(cutoff_freq) {
  coeff_ = 1.0f / (2.0f * M_PI * cutoff_freq_);
  alpha_ = coeff_ / (sampling_period_ + coeff_);
}

void LowpassFilter::Update(float value) {
  value_ = alpha_ * pre_value_ + (1.0f - alpha_) * value;
  pre_value_ = value_;
}

void LowpassFilter::Reset() {
  pre_value_ = 0.0f;
  value_ = 0.0f;
}
} // namespace filter