/**
 * timer.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: July 2, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIALDYN_UTILS_TIMER_H_
#define SPATIALDYN_UTILS_TIMER_H_

#include <chrono>  // std::chrono

namespace SpatialDyn {

class Timer {

 public:

  Timer() {}

  Timer(double frequency)
      : ns_interval_(std::chrono::nanoseconds(static_cast<unsigned int>(1e9 / frequency))) {}

  /**
   * Timer loop frequency in Hz.
   */
  void set_frequency(double frequency);
  double frequency() const;

  /**
   * Timer loop time interval in seconds (1 / frequency).
   */
  void set_dt(double dt);
  double dt() const;

  /**
   * Current CPU time since epoch in seconds.
   */
  double time() const;

  /**
   * CPU time since last timer reset in seconds.
   */
  double time_elapsed() const;

  /**
   * Simulation time since last timer reset in seconds.
   */
  double time_sim() const;

  /**
   * Number of loop iterations since last timer reset.
   */
  unsigned long long num_iterations() const;

  /**
   * Reset timer.
   */
  void Reset();

  /**
   * Wait for next timer loop.
   */
  void Sleep();

 private:

  std::chrono::steady_clock::time_point t_start_;
  std::chrono::steady_clock::time_point t_next_;
  unsigned long long num_iters_ = 0;

  bool initialized_ = false;
  std::chrono::nanoseconds ns_interval_ = std::chrono::milliseconds(1);

};

}  // namespace SpatialDyn

#endif  // SPATIALDYN_UTILS_TIMER_H_
