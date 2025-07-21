/**
 * @file lowpass_filter.h
 * @author Teruru-52
 */

#ifndef LOWPASS_FILTER_H_
#define LOWPASS_FILTER_H_

#include "filter/filter_base.h"
#include "main.h"

namespace filter {
class LowpassFilter : public FilterBase {
 public:
  LowpassFilter(float sampling_period, float cutoff_freq);
  ~LowpassFilter() override = default;

  void Update(float value) override;
  void Reset() override;

 private:
  float sampling_period_;  // [s]
  float coeff_;            // [1/s]
  float alpha_;            // [1]
  float cutoff_freq_;      // [Hz]
};
}  // namespace filter

#endif  // FILTER_BASE_H_