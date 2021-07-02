#include <memory>
#include <stdexcept>
#include <variant>
#include <vector>

#include "geneva-interface/GenericIndividual.tcc"
#include "geneva/Go2.hpp"
namespace GO3 {

enum Algorithms { EA, GD };
namespace cfg {
template <size_t I>
using ic = std::integral_constant<std::size_t, I>;
static inline constexpr ic<0> Size{};
static inline constexpr ic<0> Iterations{};
static inline constexpr ic<1> test{};
static inline constexpr ic<2> x{};
}  // namespace cfg
using namespace Gem::Geneva;
class Population {
  public:
  std::vector<double> start, left, right;
  int Size{100};
  int numParents{5};
  std::function<double(std::vector<double>&)> func{[](std::vector<double>&) {
    throw std::runtime_error("no function defined");
    return 0.0;
  }};
  // std::tuple<int&, int&> cfg{std::forward_as_tuple(Size, numParents)};
  using Individual = GenericIndividual<decltype(func)>;
  using Factory = GenericIndividualFactory<decltype(func)>;
  std::shared_ptr<GenericIndividualFactory<decltype(func)>> factory_ptr;
  auto get()
  {
    if (!factory_ptr)  // late binding
      factory_ptr = std::make_shared<GenericIndividualFactory<decltype(func)>>(
	  "config/GenericIndividual.json", start, left, right, func);
    return factory_ptr;
  };
  template <typename Integral>
  constexpr auto& operator[](Integral)	// adress via 0,1 and integrals
  {
    return std::get<Integral::value>(std::forward_as_tuple(Size, numParents));
  }
};

template <typename FactoryT, typename AlgoT>
class Algorithm {
  public:
  int Iterations{5};
  int algonum{2};
  std::shared_ptr<AlgoT> get()
  {
    std::shared_ptr<AlgoT> algo = FactoryT().template get<AlgoT>();
    return algo;
  }
  template <typename Integral>
  constexpr auto& operator[](Integral)	// adress via 0,1 and integrals
  {
    return std::get<Integral::value>(
	std::forward_as_tuple(Iterations, algonum));
  }
};
using Algorithm_EA =
    Algorithm<GEvolutionaryAlgorithmFactory, GEvolutionaryAlgorithm>;
using Algorithm_GD = Algorithm<GGradientDescentFactory, GGradientDescent>;

using algorithmsT = std::vector<std::variant<Algorithm_EA, Algorithm_GD>>;
class GenevaOptimizer3 {
  public:
  Go2 go;
  GenevaOptimizer3(int argc, char** argv) : go(argc, argv, "config/Go2.json"){};

  template <typename Population>
  auto optimize(Population& pop, algorithmsT algos = {Algorithm_EA()})
  {
    if (go.clientMode()) {
      go.clientRun();
      exit(0);
    }

    int currMaxIteration = 0;
    for (auto algo : algos)
      std::visit(
	  [&](auto& al) {
	    using T = std::decay_t<decltype(al)>;
	    auto alg = al.get();
	    if constexpr (std::is_same_v<T, Algorithm_EA>)
	      alg->setPopulationSizes(pop.Size, 5);
	    /* User expects each algo to add its
	     * iterations to the process*/
	    std::cout << "iterations:" << al.Iterations << std::endl;
	    alg->setMaxIteration(currMaxIteration += al[cfg::Iterations]);
	    alg->setMaxStallIteration(currMaxIteration);
	    go& alg;
	  },
	  algo);

    go.registerContentCreator(pop.get());
    using Individual = typename Population::Individual;
    return go.optimize()->getBestGlobalIndividual<Individual>();
  }
};

}  // namespace GO3
