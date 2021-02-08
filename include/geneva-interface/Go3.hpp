#include <stdexcept>
#include <variant>

#include "geneva-interface/GenericOptimizer.hpp"
namespace GO3 {

enum Algorithm { EA, GD };
namespace cfg {
template <size_t I>
using ic = std::integral_constant<std::size_t, I>;
static inline constexpr ic<0> Iterations{};
static inline constexpr ic<1> test{};
static inline constexpr ic<2> x{};
}  // namespace cfg
template <typename Function>
class Population {
  std::tuple<int, std::string, double> cfg{42, "test", 0.1};

  public:
  std::shared_ptr<GenericIndividualFactory<Function>> factory_ptr;

  using Individual_t = GenericIndividual<Function>;
  Population(std::vector<double> start, std::vector<double> left,
	     std::vector<double> right, Function&& func)
      : factory_ptr(new GenericIndividualFactory<Function>(
	    "config/GenericIndividual.json", start, left, right,
	    std::forward<decltype(func)>(func)))
  {
    GenericIndividual<Function>::setFunc(std::forward<Function>(func));
  }

  template <typename Integral>
  constexpr auto& operator[](Integral)
  {
    return std::get<Integral::value>(cfg);
  }
};
class GenevaOptimizer3 {
  std::vector<Algorithm> algos;

  public:
  int argc;
  char** argv;
  Gem::Geneva::Go2 go;
  GenevaOptimizer3(int argc, char** argv,
		   std::vector<Algorithm> algos = {Algorithm::EA})
      : algos(algos),
	argc(argc),
	argv(argv),
	go(argc, argv, "config/Go2.json"){};

  template <typename Population>
  auto optimize(Population& pop)
  {
    if (go.clientMode()) {
      go.clientRun();
    }
    for (auto algo : algos) {
      switch (algo) {
	case Algorithm::EA:
	  go& Gem::Geneva::GEvolutionaryAlgorithmFactory()
	      .get<GEvolutionaryAlgorithm>();
	  break;
	case Algorithm::GD:
	  go& Gem::Geneva::GGradientDescentFactory().get<GGradientDescent>();
	  break;
      }
    }
    go.registerContentCreator(pop.factory_ptr);
    return go.optimize()
	->getBestGlobalIndividual<typename Population::Individual_t>();
  }
};
}  // namespace GO3
