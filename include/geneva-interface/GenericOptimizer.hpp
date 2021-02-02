#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include <vector>

#include "geneva/Go2.hpp"
// The individual that should be optimized
#include "boost/bind.hpp"
#include "boost/program_options/options_description.hpp"
#include "boost/system/error_code.hpp"
#include "geneva-interface/GOptions.hpp"
#include "geneva-interface/GenericIndividual.tcc"
using namespace Gem::Geneva;
using GPTL = GProcessingTimesLogger;

namespace GenevaOpt {
enum JSON { IndividualFactory, EAFactory, GDFactory, Go2, XPName };
enum THREADNUM { Producer };
std::map<JSON, std::string> conf{
    {JSON::IndividualFactory, "config/GGenericIndividual.json"},
    {JSON::EAFactory, "config/GEvolutionaryAlgorithm.json"},
    {JSON::GDFactory, "config/GGradientDescentAlgorithm.json"},
    {JSON::Go2, "config/Go2.json"},
    {JSON::XPName, ""}};
std::map<THREADNUM, int> threads{{THREADNUM::Producer, 4}};
template <typename Function>
struct Types {
  using Function_t = Function;
  using Individual_t = GenericIndividual<Function>;
  using Factory_t = GenericIndividualFactory<Function>;
};

template <typename Function>
auto createFunctionFactory(std::string fn, std::vector<double> &start,
			   std::vector<double> &left,
			   std::vector<double> &right, Function &&func)
{
  std::shared_ptr<GenericIndividualFactory<Function>> gsif_ptr(
      new GenericIndividualFactory<Function>(
	  fn, start, left, right, std::forward<decltype(func)>(func)));
  return gsif_ptr;
}

class ClientTerminationException : public std::exception {
  public:
  virtual const char *what() const noexcept
  {
    const char *emsg =
	"The Geneva run has ended and this client will terminate now, which is "
	"the expected behavior. If you "
	"need this client to continue, catch this exception and reset the "
	"termination handler with std::set_terminate(NULL)";
    return emsg;
  }
};
template <typename Factory>
class GenevaOptimizer {
  public:
  using Function_t = typename Factory::Function_t;
  using Individual_t = GenericIndividual<Function_t>;

  private:
  std::shared_ptr<Gem::Geneva::Go2> go;

  public:
  struct Factories {
    std::shared_ptr<Gem::Geneva::GEvolutionaryAlgorithmFactory> ea;
    std::shared_ptr<Gem::Geneva::GGradientDescentFactory> gd;
    std::shared_ptr<Factory> fd;
  };

  Factories facts;

  GenevaOptimizer(std::shared_ptr<Factory> factory_ptr)
  {
    std::cout << "GenevaOptimizer:Building Factories and Go2:" << std::endl;
    facts.fd = factory_ptr;
    facts.ea = decltype(facts.ea)(
	new Gem::Geneva::GEvolutionaryAlgorithmFactory(conf[JSON::EAFactory]));
    facts.gd = decltype(facts.gd)(
	new Gem::Geneva::GGradientDescentFactory(conf[JSON::GDFactory]));
    GRANDOMFACTORY->setNProducerThreads(threads[THREADNUM::Producer]);
    std::cout << "Using Command Line Options:";
    std::vector<std::string> args =
	GenevaOpt::OptionRegistry::registry->flatMap();
    args.insert(args.begin(), "Run.exe");
    std::vector<char *> cstrings{};
    for (auto &string : args) cstrings.push_back(&string.front());
    go = std::shared_ptr<Gem::Geneva::Go2>(
	new Gem::Geneva::Go2(cstrings.size(), &cstrings[0], conf[JSON::Go2]));

    std::cout << "Number of Random Number Producers Threads:"
	      << go->getNProducerThreads() << std::endl;
    int resultint;
    go->registerContentCreator(factory_ptr);
  }
  std::shared_ptr<G_OptimizationAlgorithm_Base> EAOptimizerFromCp(
      std::string filename, std::shared_ptr<G_OptimizationAlgorithm_Base> gopt)
  {
    return gopt;
  }

  void copyIndividualsFromCP(
      std::string filename,
      std::shared_ptr<Gem::Geneva::G_OptimizationAlgorithm_Base> sourcePop)
  {
    std::cout << "Checkpoint file:" << filename
	      << std::string(sourcePop->cp_personality_fits(
				 std::filesystem::path(filename))
				 ? "\ndoes fit the algorithm:\n"
				 : "\ndoes not fit the algorithm:\n")
	      << std::endl;
    // Take the format used in the algorithm-configuration file
    sourcePop->fromFile(filename, sourcePop->getCheckpointSerializationMode());
    std::cout << "Best individual from checkpoint:"
	      << std::get<0>(sourcePop->getBestKnownPrimaryFitness()) << "//"
	      << std::get<1>(sourcePop->getBestKnownPrimaryFitness())
	      << "\nCopying " << sourcePop->size()
	      << "individuals from the checkpoint into the new population"
	      << std::endl;

    // also sorts them and keeps the best individual in the population
    std::vector<double> resultvector;
    for (auto a : *sourcePop) {
      go->push_back(a);
    }

    std::sort(go->begin(), go->end(),
	      [](std::shared_ptr<GParameterSet> x,
		 std::shared_ptr<GParameterSet> y) -> bool {
		return x->raw_fitness() < y->raw_fitness();
	      });
    std::cout << "Best fitness in population: " << std::setprecision(20)
	      << (*go->begin())->raw_fitness() << "\nwith parameters:\n";

    auto r = go->begin();
    (*(go->begin()))->streamline(resultvector);
    for (auto a : resultvector) {
      std::cout << "" << a;
    }
    resultvector.clear();
    std::cout << "\n  Best global fitness: " << std::setprecision(20)
	      << sourcePop
		     ->getBestGlobalIndividual<GenericIndividual<Function_t>>()
		     ->raw_fitness()
	      << "\nwith parameters:\n";
    sourcePop->getBestGlobalIndividual<GenericIndividual<Function_t>>()
	->streamline(resultvector);
    for (auto a : resultvector) {
      std::cout << "" << a;
    }
    resultvector.clear();
    if ((*go->begin())->raw_fitness() >
	sourcePop->getBestGlobalIndividual<GenericIndividual<Function_t>>()
	    ->raw_fitness()) {
      std::cout
	  << "\n  Replacing the last individual with the best global individual"
	  << std::endl;
      go->pop_back();
      go->insert(
	  go->begin(),
	  sourcePop->getBestGlobalIndividual<GenericIndividual<Function_t>>());
    }
    std::cout << "\n  marking all Individuals for reprocessing/recalculation"
	      << std::endl;
    for (auto a : *go) a->mark_as_due_for_processing();
  }

  std::vector<std::shared_ptr<G_OptimizationAlgorithm_Base>> algos;
  void addAlgorithm(std::shared_ptr<G_OptimizationAlgorithm_Base> algo)
  {
    (*go) & algo;
    algos.push_back(algo);
  }

  auto optimize()
  {
    //  Add a content creator so Go2 can generate its own individuals, if
    //  necessary
    std::shared_ptr<GenericIndividual<Function_t>> bestIndividual_ptr;
    if (go->clientMode()) {
      std::cout << "Running in client Mode" << std::endl;
      go->clientRun();
      // set the termination handler to print exception
      std::set_terminate([]() {
	if (auto exc = std::current_exception()) {
	  // we have an exception
	  try {
	    rethrow_exception(exc);  // throw to recognize the type
	  }
	  catch (ClientTerminationException const
		     &exc) {  // simply exit in this case
	    std::cerr << "Caught ClientTerminationException: " << exc.what()
		      << std::endl;
	    exit(0);
	  }
	  catch (...) {
	    std::set_terminate(NULL);  // reset termination handler and rethrow;
	    rethrow_exception(exc);
	  }
	}
      });
      throw ClientTerminationException();
    }
    else {
      // Add Pluggable Monitor for creating histo of computing time
      std::string monitorTimings = "notempty";
      if (monitorTimings != "empty") {
	std::shared_ptr<GPTL> processingTimesLogger_ptr(new GPTL(
	    "hist_" + monitorTimings + ".C", "hist2D_" + monitorTimings + ".C",
	    monitorTimings + ".txt",
	    100	 // nBins in x-directionS
	    ,
	    100	 // nBins in y-direction
	    ));
      }

      bestIndividual_ptr =
	  go->optimize()
	      ->getBestGlobalIndividual<GenericIndividual<Function_t>>();
    }

    return bestIndividual_ptr;
  }

  auto reevaluateBest()
  {
    std::cout << "Reevaluating best parameter fitness" << std::endl;
    assert(go->begin() != go->end());

    std::shared_ptr<GenericIndividual<Function_t>> bestIndividual_ptr =
	std::dynamic_pointer_cast<GenericIndividual<Function_t>>(
	    *(go->begin()));
    bestIndividual_ptr->mark_as_due_for_processing();
    bestIndividual_ptr->process();
    return bestIndividual_ptr;
  }
};  // namespace GenevaOpt

template <typename lambda_t>
std::shared_ptr<GenericIndividual<lambda_t>> FindMinimum(
    std::array<std::vector<double>, 3> vals, bool eaOrGd, lambda_t &&lambda)
{
  GenericIndividual<lambda_t>::setFunc(std::forward<lambda_t>(lambda));
  // GenevaOptimizer<GenericIndividualFactory<lambda_t>> goz(
  GenevaOptimizer<GenericIndividualFactory<lambda_t>> goz(
      createFunctionFactory(conf[JSON::IndividualFactory], vals[0], vals[1],
			    vals[2], std::forward<lambda_t>(lambda)));

  Gem::Geneva::GEvolutionaryAlgorithmFactory ea_f = *goz.facts.ea;
  Gem::Geneva::GGradientDescentFactory gd_f = *goz.facts.gd;

  if (conf[JSON::XPName].compare("") != 0) {
    goz.copyIndividualsFromCP(conf[JSON::XPName],
			      ea_f.get<GEvolutionaryAlgorithm>());
  }

  if (eaOrGd)
    goz.addAlgorithm(ea_f.get<GEvolutionaryAlgorithm>());

  else
    goz.addAlgorithm(gd_f.get<GGradientDescent>());

  return revalBest ? goz.reevaluateBest() : goz.optimize();
}
}  // namespace GenevaOpt
