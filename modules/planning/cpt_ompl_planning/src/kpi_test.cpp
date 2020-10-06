
#include <cpt_ompl_planning/path_kpi_calculator.h>

#include <chrono>
#include <iostream>
int main(int argc, char *argv[]) {
  PathKPICalcuator::PathArray path;

  // create spiral!
  std::string str_points = argv[1];
  int n_points = std::stoi(str_points);
  std::cout << "n_points = " << n_points << std::endl;
  for (int i = 0; i < n_points; i++) {
    double angle = (i / (double)n_points) * (2 * M_PI);
    path.push_back({cos(angle) * 0.5, sin(angle) * 0.5, 0});
  }
  path.push_back({cos(0.0) * 0.5, sin(0.0) * 0.5, 0});  // final point

  PathKPICalcuator calc;
  auto quality = calc.calculate(path);


  std::cout << quality << std::endl;
}