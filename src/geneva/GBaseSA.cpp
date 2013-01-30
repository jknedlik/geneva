/**
 * @file GBaseSA.cpp
 */

/*
 * Copyright (C) Gemfony scientific UG (haftungsbeschraenkt)
 *
 * See the AUTHORS file in the top-level directory for a list of authors.
 *
 * Contact: contact [at] gemfony (dot) com
 *
 * This file is part of the Geneva library collection.
 *
 * Geneva was developed with kind support from Karlsruhe Institute of
 * Technology (KIT) and Steinbuch Centre for Computing (SCC). Further
 * information about KIT and SCC can be found at http://www.kit.edu/english
 * and http://scc.kit.edu .
 *
 * Geneva is free software: you can redistribute and/or modify it under
 * the terms of version 3 of the GNU Affero General Public License
 * as published by the Free Software Foundation.
 *
 * Geneva is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with the Geneva library. If not, see <http://www.gnu.org/licenses/>.
 *
 * For further information on Gemfony scientific and Geneva, visit
 * http://www.gemfony.com .
 */
#include "geneva/GBaseSA.hpp"

BOOST_CLASS_EXPORT_IMPLEMENT(Gem::Geneva::GBaseSA::GSAOptimizationMonitor)

namespace Gem {
namespace Geneva {

/******************************************************************************/
/**
 * The default constructor, As we do not have any individuals yet, we set the population
 * size, and number of parents to 0. It is the philosophy of this class not
 * to provide constructors for each and every use case. Instead, you should set
 * vital parameters, such as the population size or the parent individuals by hand.
 */
GBaseSA::GBaseSA()
   : GBaseParChildT<GParameterSet>()
   , t0_(SA_T0)
   , t_(t0_)
   , alpha_(SA_ALPHA)
{
   // Register the default optimization monitor
   this->registerOptimizationMonitor(
         boost::shared_ptr<GOptimizationAlgorithmT<GParameterSet>::GOptimizationMonitorT>(
               new GSAOptimizationMonitor()
         )
   );

   // Make sure we start with a valid population size if the user does not supply these values
   this->setDefaultPopulationSize(100,1);
}

/******************************************************************************/
/**
 * A standard copy constructor. Note that the generation number is reset to 0 and
 * is not copied from the other object. We assume that a new optimization run will
 * be started.
 *
 * @param cp Another GBaseSA object
 */
GBaseSA::GBaseSA(const GBaseSA& cp)
   : GBaseParChildT<GParameterSet>(cp)
   , t0_(cp.t0_)
   , t_(cp.t_)
   , alpha_(cp.alpha_)
{
   // Copying / setting of the optimization algorithm id is done by the parent class. The same
   // applies to the copying of the optimization monitor.
}

/******************************************************************************/
/**
 * The standard destructor. All work is done in the parent class.
 */
GBaseSA::~GBaseSA()
{ /* nothing */ }

/******************************************************************************/
/**
 * The standard assignment operator.
 *
 * @param cp Another GBaseSA object
 * @return A constant reference to this object
 */
const GBaseSA& GBaseSA::operator=(const GBaseSA& cp) {
   GBaseSA::load_(&cp);
   return *this;
}

/******************************************************************************/
/**
 * Loads the data of another GBaseSA object, camouflaged as a GObject.
 *
 * @param cp A pointer to another GBaseSA object, camouflaged as a GObject
 */
void GBaseSA::load_(const GObject * cp)
{
   const GBaseSA *p_load = gobject_conversion<GBaseSA>(cp);

   // First load the parent class'es data ...
   GBaseParChildT<GParameterSet>::load_(cp);

   // ... and then our own data
   t0_ = p_load->t0_;
   t_ = p_load->t_;
   alpha_ = p_load->alpha_;
}

/******************************************************************************/
/**
 * Checks for equality with another GBaseSA object
 *
 * @param  cp A constant reference to another GBaseSA object
 * @return A boolean indicating whether both objects are equal
 */
bool GBaseSA::operator==(const GBaseSA& cp) const {
   using namespace Gem::Common;
   // Means: The expectation of equality was fulfilled, if no error text was emitted (which converts to "true")
   return !checkRelationshipWith(cp, CE_EQUALITY, 0.,"GBaseSA::operator==","cp", CE_SILENT);
}

/******************************************************************************/
/**
 * Checks for inequality with another GBaseSA object
 *
 * @param  cp A constant reference to another GBaseSA object
 * @return A boolean indicating whether both objects are inequal
 */
bool GBaseSA::operator!=(const GBaseSA& cp) const {
   using namespace Gem::Common;
   // Means: The expectation of inequality was fulfilled, if no error text was emitted (which converts to "true")
   return !checkRelationshipWith(cp, CE_INEQUALITY, 0.,"GBaseSA::operator!=","cp", CE_SILENT);
}

/******************************************************************************/
/**
 * Checks whether a given expectation for the relationship between this object and another object
 * is fulfilled.
 *
 * @param cp A constant reference to another object, camouflaged as a GObject
 * @param e The expected outcome of the comparison
 * @param limit The maximum deviation for floating point values (important for similarity checks)
 * @param caller An identifier for the calling entity
 * @param y_name An identifier for the object that should be compared to this one
 * @param withMessages Whether or not information should be emitted in case of deviations from the expected outcome
 * @return A boost::optional<std::string> object that holds a descriptive string if expectations were not met
 */
boost::optional<std::string> GBaseSA::checkRelationshipWith(const GObject& cp,
      const Gem::Common::expectation& e,
      const double& limit,
      const std::string& caller,
      const std::string& y_name,
      const bool& withMessages) const
{
    using namespace Gem::Common;

   // Check that we are indeed dealing with a GParamterBase reference
   const GBaseSA *p_load = GObject::gobject_conversion<GBaseSA>(&cp);

   // Will hold possible deviations from the expectation, including explanations
   std::vector<boost::optional<std::string> > deviations;

   // Check our parent class'es data ...
   deviations.push_back(GBaseParChildT<GParameterSet>::checkRelationshipWith(cp, e, limit, "GBaseSA", y_name, withMessages));

   // ... and then our local data
   deviations.push_back(checkExpectation(withMessages, "GBaseSA", t0_, p_load->t0_, "t0_", "p_load->t0_", e , limit));
   deviations.push_back(checkExpectation(withMessages, "GBaseSA", t_, p_load->t_, "t_", "p_load->t_", e , limit));
   deviations.push_back(checkExpectation(withMessages, "GBaseSA", alpha_, p_load->alpha_, "alpha_", "p_load->alpha_", e , limit));

   return evaluateDiscrepancies("GBaseSA", caller, deviations, e);
}

/******************************************************************************/
/**
 * Returns information about the type of optimization algorithm. This function needs
 * to be overloaded by the actual algorithms to return the correct type.
 *
 * @return The type of optimization algorithm
 */
personality_oa GBaseSA::getOptimizationAlgorithm() const {
   return PERSONALITY_SA;
}

/******************************************************************************/
/**
 * Returns the name of this optimization algorithm
 *
 * @return The name assigned to this optimization algorithm
 */
std::string GBaseSA::getAlgorithmName() const {
   return std::string("Simulated Annealing");
}

/******************************************************************************/
/**
 * The function checks that the population size meets the requirements and does some
 * tagging. It is called from within GOptimizationAlgorithmT<Gem::Geneva::GParameterSet>::optimize(), before the
 * actual optimization cycle starts.
 */
void GBaseSA::init() {
   // To be performed before any other action
   GBaseParChildT<GParameterSet>::init();
}

/******************************************************************************/
/**
 * Does any necessary finalization work
 */
void GBaseSA::finalize() {
   // Last action
   GBaseParChildT<GParameterSet>::finalize();
}

/******************************************************************************/
/**
 * Performs a simulated annealing style sorting and selection
 */
void GBaseSA::sortSAMode() {
   // Position the nParents best children of the population right behind the parents
   if(this->getMaxMode()){
      std::partial_sort(data.begin() + nParents_, data.begin() + 2*nParents_, data.end(),
           boost::bind(&GParameterSet::fitness, _1, 0) > boost::bind(&GParameterSet::fitness, _2, 0));
   }
   else{
      std::partial_sort(data.begin() + nParents_, data.begin() + 2*nParents_, data.end(),
           boost::bind(&GParameterSet::fitness, _1, 0) < boost::bind(&GParameterSet::fitness, _2, 0));
   }

   // Check for each parent whether it should be replaced by the corresponding child
   for(std::size_t np=0; np<nParents_; np++) {
      double pPass = saProb(this->at(np)->fitness(0), this->at(nParents_+np)->fitness(0));
      if(pPass >= 1.) {
         this->at(np)->load(this->at(nParents_+np));
      } else {
         double challenge = gr.uniform_01<double>();
         if(challenge < pPass) {
            this->at(np)->load(this->at(nParents_+np));
         }
      }
   }

   // Sort the parents -- it is possible that a child with a worse fitness has replaced a parent
   if(this->getMaxMode()){
      std::sort(data.begin(), data.begin() + nParents_,
           boost::bind(&GParameterSet::fitness, _1, 0) > boost::bind(&GParameterSet::fitness, _2, 0));
   }
   else{
      std::sort(data.begin(), data.begin() + nParents_,
           boost::bind(&GParameterSet::fitness, _1, 0) < boost::bind(&GParameterSet::fitness, _2, 0));
   }

   // Make sure the temperature gets updated
   updateTemperature();
}

/******************************************************************************/
/**
 * Calculates the simulated annealing probability for a child to replace a parent
 *
 * @param qParent The fitness of the parent
 * @param qChild The fitness of the child
 * @return A double value in the range [0,1[, representing the likelihood for the child to replace the parent
 */
double GBaseSA::saProb(const double& qParent, const double& qChild) {
   // We do not have to do anything if the child is better than the parent
   if(isBetter(qChild, qParent)) {
      return 2.;
   }

   double result = 0.;
   if(this->getMaxMode()){
      result = exp(-(qParent-qChild)/t_);
   } else {
      result = exp(-(qChild-qParent)/t_);
   }

   return result;
}

/******************************************************************************/
/**
 * Updates the temperature. This function is used for simulated annealing.
 */
void GBaseSA::updateTemperature() {
   t_ *= alpha_;
}

/******************************************************************************/
/**
 * Determines the strength of the temperature degradation. This function is used for simulated annealing.
 *
 * @param alpha The degradation speed of the temperature
 */
void GBaseSA::setTDegradationStrength(double alpha) {
   if(alpha <=0.) {
      glogger
      << "In GBaseSA::setTDegradationStrength(const double&):" << std::endl
      << "Got negative alpha: " << alpha << std::endl
      << GEXCEPTION;
   }

   alpha_ = alpha;
}

/******************************************************************************/
/**
 * Retrieves the temperature degradation strength. This function is used for simulated annealing.
 *
 * @return The temperature degradation strength
 */
double GBaseSA::getTDegradationStrength() const {
   return alpha_;
}

/******************************************************************************/
/**
 * Sets the start temperature. This function is used for simulated annealing.
 *
 * @param t0 The start temperature
 */
void GBaseSA::setT0(double t0) {
   if(t0 <=0.) {
      glogger
      << "In GBaseSA::setT0(const double&):" << std::endl
      << "Got negative start temperature: " << t0 << std::endl
      << GEXCEPTION;
   }

   t0_ = t0;
}

/******************************************************************************/
/**
 * Retrieves the start temperature. This function is used for simulated annealing.
 *
 * @return The start temperature
 */
double GBaseSA::getT0() const {
   return t0_;
}

/******************************************************************************/
/**
 * Retrieves the current temperature. This function is used for simulated annealing.
 *
 * @return The current temperature
 */
double GBaseSA::getT() const {
   return t_;
}

/******************************************************************************/
/**
 * Adds local configuration options to a GParserBuilder object
 *
 * @param gpb The GParserBuilder object to which configuration options should be added
 * @param showOrigin Makes the function indicate the origin of parameters in comments
 */
void GBaseSA::addConfigurationOptions (
   Gem::Common::GParserBuilder& gpb
   , const bool& showOrigin
) {
   std::string comment;

   // Call our parent class'es function
   GBaseParChildT<GParameterSet>::addConfigurationOptions(gpb, showOrigin);

   comment = ""; // Reset the comment string
   comment += "The start temperature used in simulated annealing;";
   if(showOrigin) comment += "[GBaseEA]";
   gpb.registerFileParameter<double>(
      "t0" // The name of the variable
      , SA_T0 // The default value
      , boost::bind(
         &GBaseSA::setT0
         , this
         , _1
        )
      , Gem::Common::VAR_IS_ESSENTIAL // Alternative: VAR_IS_SECONDARY
      , comment
   );

   comment = ""; // Reset the comment string
   comment += "The degradation strength used in the cooling;";
   comment += "schedule in simulated annealing;";
   if(showOrigin) comment += "[GBaseEA]";
   gpb.registerFileParameter<double>(
      "alpha" // The name of the variable
      , SA_ALPHA // The default value
      , boost::bind(
         &GBaseSA::setTDegradationStrength
         , this
         , _1
      )
      , Gem::Common::VAR_IS_ESSENTIAL // Alternative: VAR_IS_SECONDARY
      , comment
   );
}

/******************************************************************************/
/**
 * Some error checks related to population sizes
 */
void GBaseSA::populationSanityChecks() const {
   // First check that we have been given a suitable value for the number of parents.
   // Note that a number of checks (e.g. population size != 0) has already been done
   // in the parent class.
   if(nParents_ == 0) {
      glogger
      << "In GBaseSA::populationSanityChecks(): Error!" << std::endl
      << "Number of parents is set to 0"
      << GEXCEPTION;
   }

   // We need at least as many children as parents
   std::size_t popSize = getPopulationSize();
   if(popSize<=nParents_) {
      glogger
      << "In GBaseSA::populationSanityChecks() :" << std::endl
      << "Requested size of population is too small :" << popSize << " " << nParents_ << std::endl
      << GEXCEPTION;
   }
}

/******************************************************************************/
/**
 * Choose new parents, based on the SA selection scheme.
 */
void GBaseSA::selectBest()
{
   // Sort according to the "Simulated Annealing" scheme
   sortSAMode();

   // Let parents know they are parents
   markParents();
}

/******************************************************************************/
/**
 * Retrieves the evaluation range in a given iteration. The start point will be
 * different, depending on the iteration. The end-point is not meant
 * to be inclusive.
 *
 * @return The range inside which evaluation should take place
 */
boost::tuple<std::size_t,std::size_t> GBaseSA::getEvaluationRange() const {
   return boost::tuple<std::size_t, std::size_t>(
         inFirstIteration()?0:getNParents()
         ,  data.size()
   );
}

/******************************************************************************/
/**
 * Applies modifications to this object. This is needed for testing purposes
 *
 * @return A boolean which indicates whether modifications were made
 */
bool GBaseSA::modify_GUnitTests() {
#ifdef GEM_TESTING

   bool result = false;

   // Call the parent class'es function
   if(GBaseParChildT<GParameterSet>::modify_GUnitTests()) result = true;

   return result;

#else /* GEM_TESTING */  // If this function is called when GEM_TESTING isn't set, throw
   condnotset("GBaseSA::modify_GUnitTests", "GEM_TESTING");
   return false;
#endif /* GEM_TESTING */
}

/******************************************************************************/
/**
 * Performs self tests that are expected to succeed. This is needed for testing purposes
 */
void GBaseSA::specificTestsNoFailureExpected_GUnitTests() {
#ifdef GEM_TESTING
   // Call the parent class'es function
   GBaseParChildT<GParameterSet>::specificTestsNoFailureExpected_GUnitTests();

   //------------------------------------------------------------------------------
   //------------------------------------------------------------------------------

#else /* GEM_TESTING */ // If this function is called when GEM_TESTING isn't set, throw
   condnotset("GBaseSA::specificTestsNoFailureExpected_GUnitTests", "GEM_TESTING");
#endif /* GEM_TESTING */
}

/******************************************************************************/
/**
 * Performs self tests that are expected to fail. This is needed for testing purposes
 */
void GBaseSA::specificTestsFailuresExpected_GUnitTests() {
#ifdef GEM_TESTING
   // Call the parent class'es function
   GBaseParChildT<GParameterSet>::specificTestsFailuresExpected_GUnitTests();

   //------------------------------------------------------------------------------
   //------------------------------------------------------------------------------

#else /* GEM_TESTING */
   condnotset("GBaseSA::specificTestsFailuresExpected_GUnitTests", "GEM_TESTING");
#endif /* GEM_TESTING */
}

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/**
 * The default constructor
 */
GBaseSA::GSAOptimizationMonitor::GSAOptimizationMonitor()
   : xDim_(DEFAULTXDIMOM)
   , yDim_(DEFAULTYDIMOM)
   , nMonitorInds_(0)
{ /* nothing */ }

/******************************************************************************/
/**
 * The copy constructor
 *
 * @param cp A copy of another GSAOptimizationMonitor object
 */
GBaseSA::GSAOptimizationMonitor::GSAOptimizationMonitor(const GBaseSA::GSAOptimizationMonitor& cp)
   : GOptimizationAlgorithmT<GParameterSet>::GOptimizationMonitorT(cp)
   , xDim_(cp.xDim_)
   , yDim_(cp.yDim_)
   , nMonitorInds_(cp.nMonitorInds_)
{ /* nothing */ }

/******************************************************************************/
/**
 * The destructor
 */
GBaseSA::GSAOptimizationMonitor::~GSAOptimizationMonitor()
{ /* nothing */ }

/******************************************************************************/
/**
 * A standard assignment operator.
 *
 * @param cp A copy of another GSAOptimizationMonitor object
 * @return A constant reference to this object
 */
const GBaseSA::GSAOptimizationMonitor& GBaseSA::GSAOptimizationMonitor::operator=(const GBaseSA::GSAOptimizationMonitor& cp){
   GBaseSA::GSAOptimizationMonitor::load_(&cp);
   return *this;
}

/******************************************************************************/
/**
 * Checks for equality with another GParameter Base object
 *
 * @param  cp A constant reference to another GSAOptimizationMonitor object
 * @return A boolean indicating whether both objects are equal
 */
bool GBaseSA::GSAOptimizationMonitor::operator==(const GBaseSA::GSAOptimizationMonitor& cp) const {
   using namespace Gem::Common;
   // Means: The expectation of equality was fulfilled, if no error text was emitted (which converts to "true")
   return !checkRelationshipWith(cp, CE_EQUALITY, 0.,"GBaseSA::GSAOptimizationMonitor::operator==","cp", CE_SILENT);
}

/******************************************************************************/
/**
 * Checks for inequality with another GSAOptimizationMonitor object
 *
 * @param  cp A constant reference to another GSAOptimizationMonitor object
 * @return A boolean indicating whether both objects are inequal
 */
bool GBaseSA::GSAOptimizationMonitor::operator!=(const GBaseSA::GSAOptimizationMonitor& cp) const {
   using namespace Gem::Common;
   // Means: The expectation of inequality was fulfilled, if no error text was emitted (which converts to "true")
   return !checkRelationshipWith(cp, CE_INEQUALITY, 0.,"GBaseSA::GSAOptimizationMonitor::operator!=","cp", CE_SILENT);
}

/******************************************************************************/
/**
 * Checks whether a given expectation for the relationship between this object and another object
 * is fulfilled.
 *
 * @param cp A constant reference to another object, camouflaged as a GObject
 * @param e The expected outcome of the comparison
 * @param limit The maximum deviation for floating point values (important for similarity checks)
 * @param caller An identifier for the calling entity
 * @param y_name An identifier for the object that should be compared to this one
 * @param withMessages Whether or not information should be emitted in case of deviations from the expected outcome
 * @return A boost::optional<std::string> object that holds a descriptive string if expectations were not met
 */
boost::optional<std::string> GBaseSA::GSAOptimizationMonitor::checkRelationshipWith(
      const GObject& cp
      , const Gem::Common::expectation& e
      , const double& limit
      , const std::string& caller
      , const std::string& y_name
      , const bool& withMessages
) const {
   using namespace Gem::Common;

   // Check that we are indeed dealing with a GParamterBase reference
   const GBaseSA::GSAOptimizationMonitor *p_load = GObject::gobject_conversion<GBaseSA::GSAOptimizationMonitor >(&cp);

   // Will hold possible deviations from the expectation, including explanations
   std::vector<boost::optional<std::string> > deviations;

   // Check our parent class'es data ...
   deviations.push_back(GOptimizationAlgorithmT<GParameterSet>::GOptimizationMonitorT::checkRelationshipWith(cp, e, limit, "GBaseSA::GSAOptimizationMonitor", y_name, withMessages));

   // ... and then our local data.
   deviations.push_back(checkExpectation(withMessages, "GBaseSA::GSAOptimizationMonitor", xDim_, p_load->xDim_, "xDim_", "p_load->xDim_", e , limit));
   deviations.push_back(checkExpectation(withMessages, "GBaseSA::GSAOptimizationMonitor", yDim_, p_load->yDim_, "yDim_", "p_load->yDim_", e , limit));
   deviations.push_back(checkExpectation(withMessages, "GBaseSA::GSAOptimizationMonitor", nMonitorInds_, p_load->nMonitorInds_, "nMonitorInds_", "p_load->nMonitorInds_", e , limit));

   return evaluateDiscrepancies("GBaseSA::GSAOptimizationMonitor", caller, deviations, e);
}

/******************************************************************************/
/**
 * A function that is called once before the optimization starts
 *
 * @param goa A pointer to the current optimization algorithm for which information should be emitted
 * @return A string containing information to written to the output file (if any)
 */
std::string GBaseSA::GSAOptimizationMonitor::firstInformation(GOptimizationAlgorithmT<GParameterSet> * const goa) {
   // This should always be the first statement in a custom optimization monitor
   std::cout << GOptimizationAlgorithmT<GParameterSet>::GOptimizationMonitorT::firstInformation(goa);

   // Perform the conversion to the target algorithm
#ifdef DEBUG
   if(goa->getOptimizationAlgorithm() != PERSONALITY_SA) {
      glogger
      << "In GBaseSA::GSAOptimizationMonitor::firstInformation():" << std::endl
      << "Provided optimization algorithm has wrong type: " << goa->getOptimizationAlgorithm() << std::endl
      << GEXCEPTION;
   }
#endif /* DEBUG */
   GBaseSA * const sa = static_cast<GBaseSA * const>(goa);

   // Determine a suitable number of monitored individuals, if it hasn't already
   // been set externally. We allow a maximum of 3 monitored individuals by default
   // (or the number of parents, if <= 3).
   if(nMonitorInds_ == 0) {
      nMonitorInds_ = std::min(sa->getNParents(), std::size_t(3));
   }

   // Output the header to the summary stream
   return saFirstInformation(sa);
}

/******************************************************************************/
/**
 * A function that is called during each optimization cycle. It is possible to
 * extract quite comprehensive information in each iteration. For examples, see
 * the standard overloads provided for the various optimization algorithms.
 *
 * @param goa A pointer to the current optimization algorithm for which information should be emitted
 * @return A string containing information to written to the output file (if any)
 */
std::string GBaseSA::GSAOptimizationMonitor::cycleInformation(GOptimizationAlgorithmT<GParameterSet> * const goa) {
   // Let the audience know what the parent has to say
   std::cout << GOptimizationAlgorithmT<GParameterSet>::GOptimizationMonitorT::cycleInformation(goa);

   // Perform the conversion to the target algorithm
#ifdef DEBUG
   if(goa->getOptimizationAlgorithm() != PERSONALITY_SA) {
      glogger
      <<  "In GBaseSA::GSAOptimizationMonitor::cycleInformation():" << std::endl
      << "Provided optimization algorithm has wrong type: " << goa->getOptimizationAlgorithm() << std::endl
      << GEXCEPTION;
   }
#endif /* DEBUG */
   GBaseSA * const sa = static_cast<GBaseSA * const>(goa);
   return saCycleInformation(sa);
}

/******************************************************************************/
/**
 * A function that is called once at the end of the optimization cycle
 *
 * @param goa A pointer to the current optimization algorithm for which information should be emitted
 * @return A string containing information to written to the output file (if any)
 */
std::string GBaseSA::GSAOptimizationMonitor::lastInformation(GOptimizationAlgorithmT<GParameterSet> * const goa) {
   // Perform the conversion to the target algorithm
#ifdef DEBUG
   if(goa->getOptimizationAlgorithm() != PERSONALITY_SA) {
      glogger
      << "In GBaseSA::GSAOptimizationMonitor::lastInformation():" << std::endl
      << "Provided optimization algorithm has wrong type: " << goa->getOptimizationAlgorithm() << std::endl
      << GEXCEPTION;
   }
#endif /* DEBUG */
   GBaseSA * const sa = static_cast<GBaseSA * const>(goa);

   // Do the actual information gathering
   std::ostringstream result;
   result << saLastInformation(sa);

   // This should always be the last statement in a custom optimization monitor
   std::cout << GOptimizationAlgorithmT<GParameterSet>::GOptimizationMonitorT::lastInformation(goa);

   return result.str();
}

/******************************************************************************/
/**
 * A function that is called once before the optimization starts, acting on
 * Simulated Annealing algorithms
 */
std::string GBaseSA::GSAOptimizationMonitor::saFirstInformation(GBaseSA * const sa) {
   std::ostringstream result;

   // Output the header to the summary stream
   result << "{" << std::endl
         << "  gROOT->Reset();" << std::endl
         << "  gStyle->SetOptTitle(0);" << std::endl
         << "  TCanvas *cc = new TCanvas(\"cc\",\"cc\",0,0," << xDim_ << "," << yDim_ << ");" << std::endl
         << "  cc->Divide(1," << nMonitorInds_ << ");" << std::endl
         << std::endl;

   result << "  std::vector<long> iteration;" << std::endl
         << std::endl;
   for(std::size_t i=0; i<nMonitorInds_; i++) {
      result << "  std::vector<double> evaluation" << i << ";" << std::endl
            << std::endl;
   }

   return result.str();
}

/******************************************************************************/
/**
 * A function that is called during each optimization cycle, acting on evolutionary
 * algorithms
 *
 */
std::string GBaseSA::GSAOptimizationMonitor::saCycleInformation(GBaseSA * const sa) {
   std::ostringstream result;

   bool isDirty = false;
   double currentEvaluation = 0.;

   // Retrieve the current iteration
   boost::uint32_t iteration = sa->getIteration();

   result << "  iteration.push_back(" << iteration << ");" << std::endl;

   for(std::size_t i=0; i<nMonitorInds_; i++) {
      // Get access to the individual
      boost::shared_ptr<GParameterSet> gi_ptr = sa->individual_cast<GParameterSet>(i);

      // Retrieve the fitness of this individual
      isDirty = false;
      currentEvaluation = gi_ptr->getCachedFitness(isDirty);

      // Write information to the output stream
      result << "  evaluation" << i << ".push_back(" <<  currentEvaluation << ");" << (isDirty?" // dirty flag is set":"") << std::endl;
   }

   result << std::endl; // Improves readability of the output data

   return result.str();
}

/******************************************************************************/
/**
 * A function that is called once at the end of the optimization cycle acting
 * on evolutionary algorithms
 *
 */
std::string GBaseSA::GSAOptimizationMonitor::saLastInformation(GBaseSA * const sa) {
   std::ostringstream result;

   // Output final print logic to the stream
   result << "  // Transfer the vectors into arrays" << std::endl
         << "  double iteration_arr[iteration.size()];" << std::endl;

   for(std::size_t i=0; i<nMonitorInds_; i++) {
      result << "  double evaluation" << i << "_arr[evaluation" << i << ".size()];" << std::endl
            << std::endl
            << "  for(std::size_t i=0; i<iteration.size(); i++) {" << std::endl
            << "     iteration_arr[i] = (double)iteration[i];"
            << "     evaluation" << i << "_arr[i] = evaluation" << i << "[i];" << std::endl
            << "  }" << std::endl
            << std::endl
            << "  // Create a TGraph object" << std::endl
            << "  TGraph *evGraph" << i << " = new TGraph(evaluation" << i << ".size(), iteration_arr, evaluation" << i << "_arr);" << std::endl
            << "  // Set the axis titles" << std::endl
            << "  evGraph" << i << "->GetXaxis()->SetTitle(\"Iteration\");" << std::endl
            << "  evGraph" << i << "->GetYaxis()->SetTitleOffset(1.1);" << std::endl
            << "  evGraph" << i << "->GetYaxis()->SetTitle(\"Fitness\");" << std::endl
            << std::endl;
   }

   result << "  // Do the actual drawing" << std::endl;

   for(std::size_t i=0; i<nMonitorInds_; i++) {
      result << "  cc->cd(" << i+1 << ");" << std::endl
            << "  evGraph" << i << "->Draw(\"APL\");" << std::endl;
   }

   result << "  cc->cd();" << std::endl
         << "}" << std::endl;

   return result.str();
}

/******************************************************************************/
/**
 * Allows to set the dimensions of the canvas
 *
 * @param xDim The desired dimension of the canvas in x-direction
 * @param yDim The desired dimension of the canvas in y-direction
 */
void GBaseSA::GSAOptimizationMonitor::setDims(const boost::uint16_t& xDim, const boost::uint16_t& yDim) {
   xDim_ = xDim;
   yDim_ = yDim;
}

/******************************************************************************/
/**
 * Retrieves the dimension of the canvas in x-direction
 *
 * @return The dimension of the canvas in x-direction
 */
boost::uint16_t GBaseSA::GSAOptimizationMonitor::getXDim() const {
   return xDim_;
}

/******************************************************************************/
/**
 * Retrieves the dimension of the canvas in y-direction
 *
 * @return The dimension of the canvas in y-direction
 */
boost::uint16_t GBaseSA::GSAOptimizationMonitor::getYDim() const {
   return yDim_;
}

/******************************************************************************/
/**
 * Sets the number of individuals in the population that should be monitored
 *
 * @oaram nMonitorInds The number of individuals in the population that should be monitored
 */
void GBaseSA::GSAOptimizationMonitor::setNMonitorIndividuals(const std::size_t& nMonitorInds) {
   if(nMonitorInds == 0) {
      glogger
      << "In GBaseSA::GSAOptimizationMonitor::setNMonitorIndividuals():" << std::endl
      << "Number of monitored individuals is set to 0." << std::endl
      << GEXCEPTION;
   }

   nMonitorInds_ = nMonitorInds;
}

/******************************************************************************/
/**
 * Retrieves the number of individuals that are being monitored
 *
 * @return The number of individuals in the population being monitored
 */
std::size_t GBaseSA::GSAOptimizationMonitor::getNMonitorIndividuals() const {
   return nMonitorInds_;
}

/******************************************************************************/
/**
 * Loads the data of another object
 *
 * cp A pointer to another GBaseSA::GSAOptimizationMonitor object, camouflaged as a GObject
 */
void GBaseSA::GSAOptimizationMonitor::load_(const GObject* cp) {
   const GBaseSA::GSAOptimizationMonitor *p_load = gobject_conversion<GBaseSA::GSAOptimizationMonitor>(cp);

   // Load the parent classes' data ...
   GOptimizationAlgorithmT<GParameterSet>::GOptimizationMonitorT::load_(cp);

   // ... and then our local data
   xDim_ = p_load->xDim_;
   yDim_ = p_load->yDim_;
   nMonitorInds_ = p_load->nMonitorInds_;
}

/******************************************************************************/
/**
 * Creates a deep clone of this object
 *
 * @return A deep clone of this object
 */
GObject* GBaseSA::GSAOptimizationMonitor::clone_() const {
   return new GBaseSA::GSAOptimizationMonitor(*this);
}

/******************************************************************************/
/**
 * Applies modifications to this object. This is needed for testing purposes
 */
bool GBaseSA::GSAOptimizationMonitor::modify_GUnitTests() {
#ifdef GEM_TESTING
   bool result = false;

   // Call the parent class'es function
   if(GOptimizationAlgorithmT<GParameterSet>::GOptimizationMonitorT::modify_GUnitTests()) result = true;

   return result;

#else /* GEM_TESTING */  // If this function is called when GEM_TESTING isn't set, throw
   condnotset("GBaseSA::GSAOptimizationMonitor::modify_GUnitTests", "GEM_TESTING");
   return false;
#endif /* GEM_TESTING */
}

/******************************************************************************/
/**
 * Performs self tests that are expected to succeed. This is needed for testing purposes
 */
void GBaseSA::GSAOptimizationMonitor::specificTestsNoFailureExpected_GUnitTests() {
#ifdef GEM_TESTING
   // Call the parent class'es function
   GOptimizationAlgorithmT<GParameterSet>::GOptimizationMonitorT::specificTestsNoFailureExpected_GUnitTests();

#else /* GEM_TESTING */ // If this function is called when GEM_TESTING isn't set, throw
   condnotset("GBaseSA::GSAOptimizationMonitor::specificTestsNoFailureExpected_GUnitTests", "GEM_TESTING");
#endif /* GEM_TESTING */
}

/******************************************************************************/
/**
 * Performs self tests that are expected to fail. This is needed for testing purposes
 */
void GBaseSA::GSAOptimizationMonitor::specificTestsFailuresExpected_GUnitTests() {
#ifdef GEM_TESTING
   // Call the parent class'es function
   GOptimizationAlgorithmT<GParameterSet>::GOptimizationMonitorT::specificTestsFailuresExpected_GUnitTests();

#else /* GEM_TESTING */
   condnotset("GBaseSA::GSAOptimizationMonitor::specificTestsFailuresExpected_GUnitTests", "GEM_TESTING");
#endif /* GEM_TESTING */
}

/******************************************************************************/

} /* namespace Geneva */
} /* namespace Gem */