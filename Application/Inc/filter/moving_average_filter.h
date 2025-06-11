/**
 * @file moving_average_filter.h
 * @author Teruru-52
 */

#ifndef MOVING_AVERAGE_FILTER_H_
#define MOVING_AVERAGE_FILTER_H_

#include "filter/filter_base.h"
#include "main.h"
#include <deque>

namespace filter {
class MovingAverageFilter : public FilterBase {
public:
  MovingAverageFilter(int filter_num);
  ~MovingAverageFilter() override = default;

  void Update(float value) override;
  void Reset() override;

private:
  int filter_num_{10};
  std::deque<float> buffer_;
};
} // namespace filter

#endif // MOVING_AVERAGE_FILTER_H_