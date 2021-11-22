#include <cpt_utils/perf.h>
#include "gtest/gtest.h"

TEST(PerfOverheadTest, Count) {
  // go through 1'000'000 perf enter/exits and measure aveage overhead time.

  using Clock = std::chrono::high_resolution_clock;
  const uint num_tries = 1000000;

  Clock::time_point t_start = Clock::now();
  for (uint i = 0; i < num_tries; ++i) {
    cad_percept::Perf::get()->enter("test");
    cad_percept::Perf::get()->enter("test_inner");
    cad_percept::Perf::get()->exit();
    cad_percept::Perf::get()->exit();
  }
  Clock::time_point t_end = Clock::now();

  Clock::duration t_elapsed = t_end - t_start;
  Clock::duration t_per_call = t_elapsed / (2 * num_tries);

  std::cout << "Average duration per call: "
            << std::chrono::duration_cast<std::chrono::nanoseconds>(t_per_call).count() << " [ns]"
            << std::endl;

  EXPECT_TRUE(true);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}