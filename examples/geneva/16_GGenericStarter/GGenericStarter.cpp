// Standard header files go here
#include <iostream>

// The new geneva optimizer interface #3
#include <geneva-interface/Go3.hpp>

// The lambda to be used
auto lambda = [](std::vector<double>& a) {
  return std::accumulate(a.begin(), a.end(), 0.0);
};

int main(int argc, char** argv)
{
  using namespace GO3;
  /*Set start value, left & right bounds*/
  std::vector<double> start{50, 50, 50}, left{0, 0, 0}, right{100, 100, 100};
  /*create optimizer,give CLI options and register algorithms in order*/
  Algorithm_EA ea{.Iterations = 30};
  ea[cfg::Iterations] = 40;
  GenevaOptimizer3 go3{argc, argv};
  // ea[cfg::Iterations] = 100;  // only accepts cfg's for EA
  auto pop = Population{start, left, right, .func = lambda,
			.Iterations = {10}};  // Now we only have to remove
  /* change population config,mutationprob,etc. */
  // pop[cfg::size] = 1000;
  pop[cfg::size] = 10000000;
  auto best_ptr = go3.optimize(pop, {ea, Algorithm_GD{.Iterations = 100}});
  // auto best_ptr = go->optimize(start, left, right, lambda);  // take Pop&
  std::cout << best_ptr << std::endl;
}
