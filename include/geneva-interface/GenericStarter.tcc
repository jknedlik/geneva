// Geneva header files go here
#include <geneva/GPluggableOptimizationMonitors.hpp>
#include <geneva/Go2.hpp>

// Template GStarter header and implementation
#include <geneva-add/GenericIndividual.hpp>
#include <geneva-add/GenericIndividual.tcc>

using namespace Gem::Geneva;

template <typename Function>
auto GOptimize(int argc, char **argv, Function &&func)
{
  std::vector<double> params;
  GenericIndividual<Function>::setFunc(std::forward<Function>(func));
  Go2 go(argc, argv, "./config/Go2.json");

  // Client mode
  if (go.clientMode()) {
    go.clientRun();
    exit(0);
  }
  // Create a factory for GStarterIndividual objects and perform
  // any necessary initial work.
  std::shared_ptr<GenericIndividualFactory<Function>> gsif_ptr(
      new GenericIndividualFactory<Function>(
          "./config/GStarterIndividual.json",
          std::forward<decltype(func)>(func)));

  // Add a content creator so Go2 can generate its own individuals, if necessary
  go.registerContentCreator(gsif_ptr);

  // Perform the actual optmization
  auto bestIndividual_ptr =
      go.optimize()->getBestGlobalIndividual<GenericIndividual<Function>>();
  return bestIndividual_ptr;
}

template <typename Function, typename Factory_t>
auto GOptimize(int argc, char **argv, Function &&func,
               std::shared_ptr<Factory_t> &Factory)
{
  std::vector<double> params;
  Go2 go(argc, argv, "./config/Go2.json");

  // Client mode
  GenericIndividual<Function>::setFunc(std::forward<Function>(func));
  if (go.clientMode()) {
    go.clientRun();
    exit(0);
  }
  // Create a factory for GStarterIndividual objects and perform
  // any necessary initial work.
  std::shared_ptr<Factory_t> gsif_ptr(new GenericIndividualFactory<Function>(
      "./config/GStarterIndividual.json", std::forward<decltype(func)>(func)));

  // Add a content creator so Go2 can generate its own individuals, if necessary
  go.registerContentCreator(gsif_ptr);

  // Perform the actual optmization
  auto bestIndividual_ptr =
      go.optimize()->getBestGlobalIndividual<GenericIndividual<Function>>();
  return bestIndividual_ptr;
}
