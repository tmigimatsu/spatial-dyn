/**
 * timer.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: July 2, 2018
 * Authors: Toki Migimatsu
 */

#include "utils/timer.h"

#include <thread>  // std::this_thread

namespace SpatialDyn {

void Timer::set_frequency(double frequency) {
  ns_interval_ = std::chrono::nanoseconds(static_cast<unsigned int>(1e9 / frequency));
}

double Timer::frequency() const {
  return 1e9 / ns_interval_.count();
}

void Timer::set_dt(double dt) {
  ns_interval_ = std::chrono::nanoseconds(static_cast<unsigned int>(dt / 1e9));
}

double Timer::dt() const {
  return ns_interval_.count() / 1e9;
}


double Timer::time() const {
  auto now = std::chrono::steady_clock::now();
  auto now_ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
  auto ns_since_epoch = now_ns.time_since_epoch();
  return ns_since_epoch.count() / 1e9;
}

double Timer::time_elapsed() const {
  auto now = std::chrono::steady_clock::now();
  auto elapsed = now - t_start_;
  auto ns_elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed);
  return ns_elapsed.count() / 1e9;
}

double Timer::time_sim() const {
  auto elapsed = t_next_ - t_start_;
  auto ns_elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed);
  return ns_elapsed.count() / 1e9;
}

unsigned long long Timer::num_iterations() const {
  return num_iters_;
}

void Timer::Reset() {
  t_start_ = std::chrono::steady_clock::now();
  t_next_  = t_start_ + ns_interval_;
  num_iters_ = 0;
}

void Timer::Sleep() {
  num_iters_++;
  if (!initialized_) {
    initialized_ = true;
    t_start_ = std::chrono::steady_clock::now();
    t_next_ = t_start_ + ns_interval_;
    return;
  }

  auto t_now = std::chrono::steady_clock::now();
  if (t_now < t_next_) {
    std::this_thread::sleep_for(t_next_ - t_now);
  }
  t_next_ += ns_interval_;
}

}  // namespace SpatialDyn
