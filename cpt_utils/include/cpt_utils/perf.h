#ifndef CPT_UTILS_PERF_H_
#define CPT_UTILS_PERF_H_
#include <chrono>
#include <iostream>
#include <map>
#include <stack>
#define PERF_MAX_STACK_SIZE 1024

namespace cad_percept {

/*
 * Very minimal performance counting library.
 * Supports two types of performance counters:
 *  - count - Just counts number of executions for this name
 *  - enter/exit: Measures time between enter and exit.
 */
class Perf {
 public:
  /*
   * Implementation of a singleton pattern.
   */
  static Perf* get() { return instance_; }

  /*
   * Enters a named region and stores time of entry.
   * Multiple regions can be open at the same time (but only if the second openend one is a subset
   * of the first.)
   */
  void enter(const std::string& name);

  /*
   * Exits the last openend named region and updates the statistic.
   */
  void exit();

  /*
   * Increases count for counter with given name
   */
  void count(const std::string& name);

  /*
   * Prints statistic to console.
   */
  void printStatistics();

 private:
  // Singleton instance
  static Perf* instance_;

  // Clock we are using
  using Clock = std::chrono::high_resolution_clock;

  // Typedefs for statistics.
  typedef struct {
    std::string name;
    Clock::time_point start_t;
  } StackRegion;

  typedef struct {
    Clock::duration total;
    Clock::duration min;
    Clock::duration max;
    uint count;
  } RegionStatistic;

  Perf() {}
  void printCountStatistics();
  void printRegionStatistics();

  std::stack<StackRegion> open_regions_;
  std::map<std::string, uint> counts_;
  std::map<std::string, RegionStatistic> statistics_;
};

}  // namespace cad_percept
#endif  // CPT_UTILS_PERF_H_
