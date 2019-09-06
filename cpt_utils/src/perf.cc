#include <cpt_utils/perf.h>
#include <iomanip>
namespace cad_percept {

Perf* Perf::instance_ = new Perf();

void Perf::count(const std::string& name) {
  if (counts_.find(name) == counts_.end()) {
    // does not exist in map.
    counts_[name] = 1;
  } else {
    counts_[name]++;
  }
}

void Perf::enter(const std::string& name) {
  Clock::time_point t_now = Clock::now();
  // have a max size, in case some bug never properly exits the regions.
  if (open_regions_.size() < PERF_MAX_STACK_SIZE) {
    // add to open regions stack
    open_regions_.push({name, t_now});
  }
}

/*
 * May take too long in current implementation.
 * Can be optimized later on.
 * Consult https://xkcd.com/1691/ if in doubt.
 */
void Perf::exit() {
  // Get clock first in any case.
  Clock::time_point t_now = Clock::now();

  // no stack frames, nothing to do.
  if (open_regions_.empty()) {
    return;
  }

  // Calc time elapsed and add to statistics.
  StackRegion& region = open_regions_.top();
  open_regions_.pop();

  Clock::duration t_elapsed = t_now - region.start_t;

  if (statistics_.find(region.name) == statistics_.end()) {
    // new region name
    RegionStatistic statistic;
    statistic.count = 1;
    statistic.total = t_elapsed;
    statistic.max = t_elapsed;
    statistic.min = t_elapsed;
    statistics_[region.name] = statistic;
  } else {
    // known region name
    statistics_[region.name].count++;
    statistics_[region.name].total += t_elapsed;
    statistics_[region.name].min = std::min(t_elapsed, statistics_[region.name].min);
    statistics_[region.name].max = std::max(t_elapsed, statistics_[region.name].max);
  }
}

void Perf::printStatistics() {
  std::cout << std::endl;
  if (!counts_.empty()) printCountStatistics();
  std::cout << std::endl;
  if (!statistics_.empty()) printRegionStatistics();
  std::cout << std::endl;
}

void Perf::printRegionStatistics() {
  std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~ Timings  ~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
  std::cout << std::left << std::setw(30) << "Name";
  std::cout << std::right << std::setw(8) << "Avg";
  std::cout << std::right << std::setw(8) << "Min";
  std::cout << std::right << std::setw(8) << "Max";
  std::cout << std::right << std::setw(8) << "Count";
  std::cout << std::endl;

  for (const auto& statistic : statistics_) {
    std::cout << std::left << std::setw(30) << statistic.first;
    std::cout << std::right << std::setw(8)
              << std::chrono::duration_cast<std::chrono::microseconds>(statistic.second.total /
                                                                       statistic.second.count)
                     .count();

    std::cout
        << std::right << std::setw(8)
        << std::chrono::duration_cast<std::chrono::microseconds>(statistic.second.min).count();

    std::cout
        << std::right << std::setw(8)
        << std::chrono::duration_cast<std::chrono::microseconds>(statistic.second.max).count();

    std::cout << std::right << std::setw(8) << statistic.second.count;
    std::cout << std::endl;
  }
};

void Perf::printCountStatistics() {
  if (!counts_.empty()) {
  }
  std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~ Counters ~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
  for (const auto& counts : counts_) {
    std::cout << std::left << std::setw(30) << counts.first;
    std::cout << std::right << std::setw(10) << counts.second << std::endl;
  }
}
}  // namespace cad_percept
