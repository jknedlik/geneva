#include <variant>

#include "geneva-interface/GenericOptimizer.hpp"
namespace GO3 {
enum Algorithm { EA, GD };
template <typename Function>
class Population {
  public:
  std::shared_ptr<GenericIndividualFactory<Function>> factory_ptr;
  Population(std::vector<double> start, std::vector<double> left,
	     std::vector<double> right, Function&& func)
      : factory_ptr(new GenericIndividualFactory<Function>(
	    "config/GenericIndividual.json", start, left, right,
	    std::forward<decltype(func)>(func)))
  {
    GenericIndividual<Function>::setFunc(std::forward<Function>(func));
  }
  using Function_t = Function;
  using Individual_t = GenericIndividual<Function>;
};
class GenevaOptimizer3 {
  std::vector<Algorithm> algos;

  public:
  Gem::Geneva::Go2 go;
  GenevaOptimizer3(int argc, char** argv,
		   std::vector<Algorithm> algos = {Algorithm::EA})
      : algos(algos), go(argc, argv, "config/Go2.json"){};

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
