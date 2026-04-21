#pragma once

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <stdexcept>

namespace ballistics {

class SolverInterrupted : public std::runtime_error {
 public:
  SolverInterrupted() : std::runtime_error("Solver terminated by user.") {}
};

class SolverControl {
 public:
  void requestPause() {
    paused_.store(true);
  }

  void resume() {
    paused_.store(false);
    cv_.notify_all();
  }

  void cancel() {
    cancelled_.store(true);
    paused_.store(false);
    cv_.notify_all();
  }

  bool isPaused() const { return paused_.load(); }
  bool isCancelled() const { return cancelled_.load(); }

  void checkpoint() const {
    if (cancelled_.load()) {
      throw SolverInterrupted();
    }
    if (!paused_.load()) {
      return;
    }

    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this]() { return !paused_.load() || cancelled_.load(); });
    if (cancelled_.load()) {
      throw SolverInterrupted();
    }
  }

 private:
  mutable std::mutex mutex_;
  mutable std::condition_variable cv_;
  std::atomic<bool> paused_{false};
  std::atomic<bool> cancelled_{false};
};

}  // namespace ballistics
