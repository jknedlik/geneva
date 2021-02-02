/**
 * @file GFunctionMinimizer.cpp
 */

/********************************************************************************
 *
 * This file is part of the Geneva library collection. The following license
 * applies to this file:
 *
 * ------------------------------------------------------------------------------
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * ------------------------------------------------------------------------------
 *
 * Note that other files in the Geneva library collection may use a different
 * license. Please see the licensing information in each file.
 *
 ********************************************************************************
 *
 * Geneva was started by Dr. Rüdiger Berlich and was later maintained together
 * with Dr. Ariel Garcia under the auspices of Gemfony scientific. For further
 * information on Gemfony scientific, see http://www.gemfomy.eu .
 *
 * The majority of files in Geneva was released under the Apache license v2.0
 * in February 2020.
 *
 * See the NOTICE file in the top-level directory of the Geneva library
 * collection for a list of contributors and copyright information.
 *
 ********************************************************************************/

// Standard header files go here
#include <iostream>

// Geneva header files go here
#include <geneva/GPluggableOptimizationMonitors.hpp>

// The individual that should be optimized
#include <geneva-interface/Go3.hpp>

auto lambda = [](std::vector<double>& a) {
  return std::accumulate(a.begin(), a.end(), 0.0);
};

int main(int argc, char** argv)
{
  using namespace GO3;
  /*Set start value, left &right bounds*/
  std::vector<double> start{50, 50, 50}, left{0, 0, 0}, right{100, 100, 100};
  /*create optimizer and register Algorithms in order*/
  // Id be happy using nice designated initializers of c++20
  GenevaOptimizer3 go3{argc, argv, .algos = {Algorithm::EA, Algorithm::GD}};
  /*        v---------|tweak that algorithms config easily from here*/
  // go3.algos(0).config["xxx"] = 2;
  auto pop = Population(
      start, left, right,
      std::forward<decltype(lambda)>(lambda));	// Now we only have to remove
						// this forward<>()...
  /* change population config,mutationprob,etc. */
  /* optimize */
  auto best_ptr = go3.optimize(pop);
  /* alternatively like this*/
  // auto best_ptr = go->optimize(start, left, right, lambda);  // take Pop&
  std::cout << best_ptr << std::endl;
}
