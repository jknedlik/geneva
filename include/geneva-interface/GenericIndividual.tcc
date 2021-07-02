#include "boost/serialization/level.hpp"
#include "geneva-interface/GenericIndividual.hpp"

namespace Gem {
namespace Geneva {

/******************************************************************************/
/**
 * The default constructor -- intentionally private. Note that some data members
 * may be initialized in the class body.
 */
template <typename F>
GenericIndividual<F>::GenericIndividual()
    : GParameterSet()  //, func(std::forward<decltype(f)>(f))
{		       /* nothing */
}

/******************************************************************************/
/**
 * The standard constructor. The number of parameters is determined using the
 * number of entries in the startValues vector. Note that all vector arguments
 * need to have the same dimension.
 */

template <typename F>
GenericIndividual<F>::GenericIndividual(
    const std::size_t &prod_id, const std::vector<double> startValues,
    const std::vector<double> lowerBoundaries,
    const std::vector<double> upperBoundaries, const double &sigma,
    const double &sigmaSigma, const double &minSigma, const double &maxSigma,
    const double &adProb)
    : GParameterSet()
{
  try {
    // The following is a static function used both here
    // and in the factory, so setup code cannot diverge
    GenericIndividual<F>::addContent(*this, prod_id, startValues,
				     lowerBoundaries, upperBoundaries, sigma,
				     sigmaSigma, minSigma, maxSigma, adProb);
  }
  catch (const gemfony_exception &e) {
    glogger << e.what() << GTERMINATION;
  }
  catch (...) {
    glogger << "Unknown exception caught" << std::endl << GTERMINATION;
  }
}

/******************************************************************************/
/**
 * A standard copy constructor
 *
 * @param cp A copy of another GFunctionIndidivual
 */
template <typename Function>
GenericIndividual<Function>::GenericIndividual(
    const GenericIndividual<Function> &cp)
    : GParameterSet(cp)	 //, func(cp.func)
{			 /* nothing */
}

/******************************************************************************/
/**
 * The standard destructor
 */
template <typename F>
GenericIndividual<F>::~GenericIndividual()
{ /* nothing */
}

/******************************************************************************/
/**
 * Searches for compliance with expectations with respect to another object
 * of the same type
 *
 * @param cp A constant reference to another GObject object
 * @param e The expected outcome of the comparison
 * @param limit The maximum deviation for floating point values (important for
 * similarity checks)
 */
template <typename F>
void GenericIndividual<F>::compare(const GObject &cp,
				   const Gem::Common::expectation &e,
				   const double &limit) const
{
  using namespace Gem::Common;

  // Check that we are dealing with a GenericIndividual reference independent
  // of this object and convert the pointer
  const GenericIndividual *p_load =
      Gem::Common::g_convert_and_compare<GObject, GenericIndividual>(&cp, this);

  Gem::Common::GToken token("GenericIndividual", e);

  // Compare our parent data ...
  //  Gem::Common::compare_t<Gem::Geneva::GParameterSet>(
  //    IDENTITY(*this, *p_load), token);

  Gem::Common::compare_base_t<Gem::Geneva::GParameterSet>(*this, *p_load,
							  token);
}

/******************************************************************************/
/**
 * Adds local configuration options to a GParserBuilder object
 *
 * @param gpb The GParserBuilder object to which configuration options should be
 * added
 */
template <typename F>
void GenericIndividual<F>::addConfigurationOptions(
    Gem::Common::GParserBuilder &gpb)
{
  // Call our parent class'es function
  GParameterSet::addConfigurationOptions(gpb);
  // Add local data. We use C++11 lambda expressions to
  // specify the function to be called for setting the
  // target functions. An alternative would be bind expressions.
}

/*******************************************************************************************/
/**
 * Retrieves the average value of all sigmas used in Gauss adaptors.
 *
 * @return The average value of sigma used in Gauss adaptors
 */
template <typename F>
double GenericIndividual<F>::getAverageSigma() const
{
  std::vector<double> sigmas;

  // Loop over all parameter objects
  for (std::size_t i = 0; i < this->size(); i++) sigmas.push_back(getSigma(i));
  // Return the average
  return Gem::Common::GMean(sigmas);
}

template <typename F>
double GenericIndividual<F>::getSigma(size_t indx) const
{
  // Extract the parameter object
  auto gcdo_ptr = this->at<GConstrainedDoubleObject>(indx);

  // Extract the adaptor
  auto adaptor_ptr = gcdo_ptr->getAdaptor<GDoubleGaussAdaptor>();

  // Extract the sigma value
  return adaptor_ptr->getSigma();
}
/******************************************************************************/
/**
 * Emit information about this individual
 */
template <typename F>
std::string GenericIndividual<F>::print()
{
  std::ostringstream result;

  // Retrieve the parameters
  std::vector<double> parVec;
  this->streamline(parVec);

  result << "GenericIndividual: raw fitness " << this->raw_fitness(0)
	 << " has the following parameter values:" << std::endl;

  for (std::size_t i = 0; i < parVec.size(); i++) {
    result << i << ": " << parVec.at(i) << std::endl;
  }
  result << "The average sigma of this individual is "
	 << this->getAverageSigma() << std::endl;

  return result.str();
}

/******************************************************************************/
/**
 * Loads the data of another GenericIndividual, camouflaged as a GObject
 *
 * @param cp A copy of another GenericIndividual, camouflaged as a GObject
 */

template <typename F>
void GenericIndividual<F>::load_(const GObject *cp)
{
  // Check that we are dealing with a GenericIndividual reference independent
  // of this object and convert the pointer
  const GenericIndividual *p_load =
      Gem::Common::g_convert_and_compare<GObject, GenericIndividual>(cp, this);

  // Load our parent class'es data ...
  GParameterSet::load_(cp);
}

/******************************************************************************/
/**
 * Creates a deep clone of this object
 *
 * @return A deep clone of this object, camouflaged as a GObject
 */
template <typename F>
GObject *GenericIndividual<F>::clone_() const
{
  return new GenericIndividual(*this);
}

/******************************************************************************/
/**
 * The actual value calculation takes place here
 *
 * @param The id of the target function (ignored here)
 * @return The value of this object, as calculated with the evaluation function
 */
template <typename F>
double GenericIndividual<F>::fitnessCalculation()
{
  // Retrieve the parameters
  std::vector<double> parVec;
  this->streamline(parVec);

  return Fitness(parVec);
}

/******************************************************************************/
/**
 * Allows to output a GenericIndividual or convert it to a string using
 * boost::lexical_cast
 */
template <typename F>
std::ostream &operator<<(std::ostream &stream,
			 std::shared_ptr<GenericIndividual<F>> gsi_ptr)
{
  stream << gsi_ptr->print();
  return stream;
}

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/**
 * A constructor with the ability to switch the parallelization mode. It
 * initializes a target item as needed.
 *
 * @param configFile The name of the configuration file
 */
template <typename F>
GenericIndividualFactory<F>::GenericIndividualFactory(
    const std::string &configFile, F &f)
    : Gem::Common::GFactoryT<GParameterSet>(configFile),
      adProb_(GSI_DEF_ADPROB),
      sigma_(GSI_DEF_SIGMA),
      sigmaSigma_(GSI_DEF_SIGMASIGMA),
      minSigma_(GSI_DEF_MINSIGMA),
      maxSigma_(GSI_DEF_MAXSIGMA),
      func(f)
{ /* nothing */
}
template <typename F>
GenericIndividualFactory<F>::GenericIndividualFactory(
    const std::string &configFile, const std::vector<double> &start,
    const std::vector<double> &left, const std::vector<double> &right, F &f)
    : Gem::Common::GFactoryT<GParameterSet>(configFile),
      startValues_(start),
      lowerBoundaries_(left),
      upperBoundaries_(right),
      adProb_(GSI_DEF_ADPROB),
      sigma_(GSI_DEF_SIGMA),
      sigmaSigma_(GSI_DEF_SIGMASIGMA),
      minSigma_(GSI_DEF_MINSIGMA),
      maxSigma_(GSI_DEF_MAXSIGMA),
      func(f){/* nothing */};
/**
 * The destructor
 */

template <typename F>
GenericIndividualFactory<F>::~GenericIndividualFactory()
{ /* nothing */
}

/******************************************************************************/
/**
 * Creates items of this type
 *
 * @return Items of the desired type
 */
template <typename F>
std::shared_ptr<GParameterSet> GenericIndividualFactory<F>::getObject_(
    Gem::Common::GParserBuilder &gpb, const std::size_t &id)
{
  // Will hold the result
  std::shared_ptr<GenericIndividual<F>> target(new GenericIndividual<F>());

  // Make the object's local configuration options known
  target->addConfigurationOptions(gpb);
  // target->print();

  return target;
}

/******************************************************************************/
/**
 * Allows to describe local configuration options for gradient descents
 */
template <typename F>
void GenericIndividualFactory<F>::describeLocalOptions_(
    Gem::Common::GParserBuilder &gpb)
{
  // Describe our own options
  using namespace Gem::Courtier;

  // Allow our parent class to describe its options
  Gem::Common::GFactoryT<GParameterSet>::describeLocalOptions_(gpb);

  // Local data
  gpb.registerFileParameter<double>("adProb", adProb_, GSI_DEF_ADPROB)
      << "The probability for random adaptions of values in evolutionary "
	 "algorithms";

  gpb.registerFileParameter<double>("sigma", sigma_, GSI_DEF_SIGMA)
      << "The sigma for gauss-adaption in ES";

  gpb.registerFileParameter<double>("sigmaSigma", sigmaSigma_,
				    GSI_DEF_SIGMASIGMA)
      << "Influences the self-adaption of gauss-mutation in ES";

  gpb.registerFileParameter<double>("minSigma", minSigma_, GSI_DEF_MINSIGMA)
      << "The minimum amount value of sigma";

  gpb.registerFileParameter<double>("maxSigma", maxSigma_, GSI_DEF_MAXSIGMA)
      << "The maximum amount value of sigma";
}

/******************************************************************************/
/**
 * Allows to act on the configuration options received from the configuration
 * file. Here we can add the options described in describeLocalOptions to the
 * object. In practice, we will usually add the parameter objects here. Note
 * that a very similar constructor exists for GenericIndividual, so it may be
 * used independently of the factory.
 *
 * @param p A smart-pointer to be acted on during post-processing
 */
template <typename F>
void GenericIndividualFactory<F>::postProcess_(
    std::shared_ptr<GParameterSet> &p_base)
{
  // Convert the base pointer to our local type
  std::shared_ptr<GenericIndividual<F>> p =
      Gem::Common::convertSmartPointer<GParameterSet, GenericIndividual<F>>(
	  p_base);

  // We simply use a static function defined in the GStartIndividual header
  // to set up all parameter objects. It is used both here in the factory and
  // in one of the constructors.
  //
  GenericIndividual<F>::addContent(*p, this->getId(), startValues_,
				   lowerBoundaries_, upperBoundaries_, sigma_,
				   sigmaSigma_, minSigma_, maxSigma_, adProb_);
}

template <typename F>
std::function<double(std::vector<double> &)> GenericIndividual<F>::func;

} /* namespace Geneva */
} /* namespace Gem */

