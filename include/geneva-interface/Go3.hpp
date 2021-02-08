#include <stdexcept>
#include <variant>

#include "geneva-interface/GenericOptimizer.hpp"
namespace GO3 {

enum Algorithms { EA, GD };
namespace cfg {
template <size_t I>
using ic = std::integral_constant<std::size_t, I>;
static inline constexpr ic<0> Iterations{};
static inline constexpr ic<1> test{};
static inline constexpr ic<2> x{};
}  // namespace cfg
template <typename... Ts>
class integralconfig {
  protected:
  public:
  std::tuple<Ts...> cfg;
  template <typename Integral>
  constexpr auto& operator[](Integral)
  {
    return std::get<Integral::value>(cfg);
  }
  integralconfig(std::tuple<Ts...> ts) : cfg(ts){};
};
using popconf = integralconfig<int, std::string, double>;
template <typename Function>
class Population : public popconf {
  public:
  std::shared_ptr<GenericIndividualFactory<Function>> factory_ptr;
  using Individual_t = GenericIndividual<Function>;
  Population(std::vector<double> start, std::vector<double> left,
	     std::vector<double> right, Function& func, int Iterations = 42)
      : Population(start, left, right, std::forward<Function>(func), Iterations)
  {
  }
  Population(std::vector<double> start, std::vector<double> left,
	     std::vector<double> right, Function&& func, int Iterations = 42)
      : factory_ptr(new GenericIndividualFactory<Function>(
	    "config/GenericIndividual.json", start, left, right,
	    std::forward<decltype(func)>(func))),
	popconf({Iterations, "testfile", 0.0})
  {
    GenericIndividual<Function>::setFunc(std::forward<Function>(func));
  }
};
using algoconf = integralconfig<int, int>;

template <typename FactoryT, typename AlgoT>
class Algorithm : public algoconf {
  public:
  Algorithm(int Iterations = 5) : algoconf({Iterations, 2}) {}
  std::shared_ptr<AlgoT> get()
  {
    std::shared_ptr<AlgoT> algo = FactoryT().template get<AlgoT>();
    algo->setMaxIteration((*this)[cfg::Iterations]);
    return algo;
  }
  void addToOptimization(Gem::Geneva::Go2& go)
  {
    std::shared_ptr<AlgoT> algo = FactoryT().template get<AlgoT>();
    algo->setMaxIteration((*this)[cfg::Iterations]);
    go& algo;
  };
};
using Algorithm_EA = Algorithm<Gem::Geneva::GEvolutionaryAlgorithmFactory,
			       Gem::Geneva::GEvolutionaryAlgorithm>;
using Algorithm_GD = Algorithm<Gem::Geneva::GGradientDescentFactory,
			       Gem::Geneva::GGradientDescent>;

class GenevaOptimizer3 {
  using algorithmsT = std::vector<std::variant<Algorithm_EA, Algorithm_GD>>;

  public:
  Gem::Geneva::Go2 go;
  GenevaOptimizer3(int argc, char** argv) : go(argc, argv, "config/Go2.json"){};

  template <typename Population>
  auto optimize(Population& pop, algorithmsT algos = {Algorithm_EA()})
  {
    if (go.clientMode()) go.clientRun();

    for (auto algo : algos) std::visit([&](auto& al) { go& al.get(); }, algo);
    // al.addToOptimization(go); }, algo);

    go.registerContentCreator(pop.factory_ptr);
    return go.optimize()
	->getBestGlobalIndividual<typename Population::Individual_t>();
  }
};
}  // namespace GO3
