#ifndef HIGH_RESOLUTION_TIMER_H
#define HIGH_RESOLUTION_TIMER_H

#include <iostream>
#include <chrono>

namespace green {

class high_resolution_timer
{
public:
  void start() {
    tic_ = std::chrono::system_clock::now();
    is_running = true;
  }
  void stop() {
    toc_ = std::chrono::system_clock::now();
    is_running = false;
  }
  double duration(const std::string info="") const {
    std::chrono::time_point<std::chrono::system_clock> endtime;

    if ( is_running )
      endtime = std::chrono::system_clock::now();
    else
      endtime = toc_;

    std::chrono::duration<double, std::milli> fp_ms = endtime-tic_;
    const double elapsed_time = fp_ms.count();
    //    std::cout << "\t@Time for " << info << " " << elapsed_time << " ms" << std::endl;

    return elapsed_time;
  }
private:
  std::chrono::time_point<std::chrono::system_clock> tic_, toc_;
  bool is_running = false;
};

}
#endif
