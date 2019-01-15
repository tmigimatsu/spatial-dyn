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
#include <thread>  // std::this_thread

namespace SpatialDyn {

/**
 * @ingroup cpp_utils
 *
 * Timer utility class for use in control loops.
 *
 * @see Python: spatialdyn.Timer
 */
class Timer {

 public:

  /**
   * Default constructor that sets the loop frequency to 1000 Hz.
   */
  Timer() {}

  /**
   * Constructor that sets the loop frequency.
   *
   * @param frequency Frequency of the timer loop [Hz].
   * @see Python: spatialdyn.Timer.__init__()
   */
  Timer(double frequency) { set_freq(frequency); }

  /**
   * @return Timer loop frequency in Hz.
   * @see Python: spatialdyn.Timer.freq
   */
  double freq() const { return 1e9 / ns_interval_.count(); }

  /**
   * Set the timer loop frequency in Hz.
   */
  void set_freq(double frequency) {
    ns_interval_ = std::chrono::nanoseconds(static_cast<unsigned int>(1e9 / frequency));
  }

  /**
   * @return Timer loop time interval in seconds (1 / frequency).
   * @see Python: spatialdyn.Timer.dt
   */
  double dt() const { return ns_interval_.count() / 1e9; }

  /**
   * Set the timer loop time interval in seconds (1 / frequency).
   */
  void set_dt(double dt) {
    ns_interval_ = std::chrono::nanoseconds(static_cast<unsigned int>(dt / 1e9));
  }

  /**
   * @return Current CPU time since epoch in seconds.
   * @see Python: spatialdyn.Timer.time
   */
  double time() const {
    auto now = std::chrono::steady_clock::now();
    auto now_ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
    auto ns_since_epoch = now_ns.time_since_epoch();
    return ns_since_epoch.count() / 1e9;
  }

  /**
   * @return CPU time since last timer reset in seconds.
   * @see Python: spatialdyn.Timer.time_elapsed
   */
  double time_elapsed() const {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = now - t_start_;
    auto ns_elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed);
    return ns_elapsed.count() / 1e9;
  }

  /**
   * @return Simulation time since last timer reset in seconds.
   * @see Python: spatialdyn.Timer.time_sim
   */
  double time_sim() const {
    auto elapsed = t_next_ - t_start_;
    auto ns_elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed);
    return ns_elapsed.count() / 1e9;
  }

  /**
   * @return Number of loop iterations since last timer reset.
   * @see Python: spatialdyn.Timer.num_iters
   */
  unsigned long long num_iters() const { return num_iters_; }

  /**
   * Reset timer.
   *
   * @see Python: spatialdyn.Timer.reset()
   */
  void Reset() {
    num_iters_ = 0;
    t_start_ = std::chrono::steady_clock::now();
    t_next_  = t_start_ + ns_interval_;
  }

  /**
   * Wait for the next timer loop.
   *
   * @see Python: spatialdyn.Timer.sleep()
   */
  void Sleep() {
    ++num_iters_;
    if (!initialized_) {
      initialized_ = true;
      t_start_ = std::chrono::steady_clock::now();
      t_next_  = t_start_ + ns_interval_;
      return;
    }

    auto t_curr = std::chrono::steady_clock::now();
    if (t_curr < t_next_) {
      std::this_thread::sleep_for(t_next_ - t_curr);
    }
    t_next_ += ns_interval_;
  }

 private:

  /// @cond
  std::chrono::steady_clock::time_point t_start_;
  std::chrono::steady_clock::time_point t_next_;
  unsigned long long num_iters_ = 0;

  bool initialized_ = false;
  std::chrono::nanoseconds ns_interval_ = std::chrono::milliseconds(1);
  /// @endcond

};

}  // namespace SpatialDyn

#endif  // SPATIALDYN_UTILS_TIMER_H_
