/**
 * @file GMultiThreadedSA.cpp
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

#include "geneva/GMultiThreadedSA.hpp"

BOOST_CLASS_EXPORT_IMPLEMENT(Gem::Geneva::GMultiThreadedSA)

namespace Gem {
namespace Geneva {

/******************************************************************************/
/**
 * A standard constructor. No local, dynamically allocated data,
 * hence this function is empty.
 */
GMultiThreadedSA::GMultiThreadedSA()
   : GBaseSA()
   , nThreads_(boost::numeric_cast<boost::uint16_t>(Gem::Common::getNHardwareThreads(DEFAULTNBOOSTTHREADS)))
   , storedServerMode_(true)
{ /* nothing */ }

/******************************************************************************/
/**
 * A standard copy constructor. Note that we do not copy storedServerMode_ as
 * it is used for internal caching only.
 *
 * @param cp Reference to another GMultiThreadedSA object
 */
GMultiThreadedSA::GMultiThreadedSA(const GMultiThreadedSA& cp)
   : GBaseSA(cp)
   , nThreads_(cp.nThreads_)
   , storedServerMode_(true)
{ /* nothing */ }

/******************************************************************************/
/**
 * The standard destructor. We clear remaining work items in the
 * thread pool and wait for active tasks to finish.
 */
GMultiThreadedSA::~GMultiThreadedSA()
{ /* nothing */ }

/******************************************************************************/
/**
 * A standard assignment operator for GMultiThreadedSA objects.
 *
 * @param cp Reference to another GMultiThreadedSA object
 * @return A constant reference to this object
 */
const GMultiThreadedSA& GMultiThreadedSA::operator=(const GMultiThreadedSA& cp) {
   GMultiThreadedSA::load_(&cp);
   return *this;
}

/******************************************************************************/
/**
 * Loads the data from another GMultiThreadedSA object.
 *
 * @param vp Pointer to another GMultiThreadedSA object, camouflaged as a GObject
 */
void GMultiThreadedSA::load_(const GObject *cp) {
   // Convert GObject pointer to local format
   const GMultiThreadedSA *p_load = this->gobject_conversion<GMultiThreadedSA>(cp);

   // First load our parent class'es data ...
   GBaseSA::load_(cp);

   // ... and then our own
   nThreads_ = p_load->nThreads_;

   // Note that we do not copy storedServerMode_ as it is used for internal caching only
}

/******************************************************************************/
/**
 * Creates a deep clone of this object
 *
 * @return A deep copy of this object, camouflaged as a GObject
 */
GObject *GMultiThreadedSA::clone_() const  {
   return new GMultiThreadedSA(*this);
}

/******************************************************************************/
/**
 * Checks for equality with another GMultiThreadedSA object
 *
 * @param  cp A constant reference to another GMultiThreadedSA object
 * @return A boolean indicating whether both objects are equal
 */
bool GMultiThreadedSA::operator==(const GMultiThreadedSA& cp) const {
   using namespace Gem::Common;
   // Means: The expectation of equality was fulfilled, if no error text was emitted (which converts to "true")
   return !checkRelationshipWith(cp, CE_EQUALITY, 0.,"GMultiThreadedSA::operator==","cp", CE_SILENT);
}

/******************************************************************************/
/**
 * Checks for inequality with another GMultiThreadedSA object
 *
 * @param  cp A constant reference to another GMultiThreadedSA object
 * @return A boolean indicating whether both objects are inequal
 */
bool GMultiThreadedSA::operator!=(const GMultiThreadedSA& cp) const {
   using namespace Gem::Common;
   // Means: The expectation of inequality was fulfilled, if no error text was emitted (which converts to "true")
   return !checkRelationshipWith(cp, CE_INEQUALITY, 0.,"GMultiThreadedSA::operator!=","cp", CE_SILENT);
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
boost::optional<std::string> GMultiThreadedSA::checkRelationshipWith(
      const GObject& cp
      , const Gem::Common::expectation& e
      , const double& limit
      , const std::string& caller
      , const std::string& y_name
      , const bool& withMessages
) const {
    using namespace Gem::Common;

   // Check that we are indeed dealing with a GParamterBase reference
   const GMultiThreadedSA *p_load = GObject::gobject_conversion<GMultiThreadedSA>(&cp);

   // Will hold possible deviations from the expectation, including explanations
   std::vector<boost::optional<std::string> > deviations;

   // Check our parent class'es data ...
   deviations.push_back(GBaseSA::checkRelationshipWith(cp, e, limit, "GMultiThreadedSA", y_name, withMessages));

   // ... and then our local data
   deviations.push_back(checkExpectation(withMessages, "GMultiThreadedSA", nThreads_, p_load->nThreads_, "nThreads_", "p_load->nThreads_", e , limit));

   return evaluateDiscrepancies("GMultiThreadedSA", caller, deviations, e);
}

/******************************************************************************/
/**
 * Necessary initialization work before the start of the optimization
 */
void GMultiThreadedSA::init() {
   // GBaseSA sees exactly the environment it would when called from its own class
   GBaseSA::init();

   // Initialize our thread pool
   tp_.reset(new Gem::Common::GThreadPool(nThreads_));

   // We want to confine re-evaluation to defined places. However, we also want to restore
   // the original flags. We thus record the previous setting when setting the flag to true.
   // The function will throw if not all individuals have the same server mode flag.

   // Set the server mode and store the original flag
   bool first = true;
   std::vector<boost::shared_ptr<GParameterSet> >::iterator it;
   for(it=data.begin(); it!=data.end(); ++it){
      if(first){
         storedServerMode_ = (*it)->getServerMode();
         first = false;
      }

      if(storedServerMode_ != (*it)->setServerMode(true)) {
         glogger
         << "In GMultiThreadedSA::init():" << std::endl
         << "Not all server mode flags have the same value!" << std::endl
         << GEXCEPTION;
      }
   }
}

/******************************************************************************/
/**
 * Necessary clean-up work after the optimization has finished
 */
void GMultiThreadedSA::finalize() {
   // Restore the original values
   std::vector<boost::shared_ptr<GParameterSet> >::iterator it;
   for(it=data.begin(); it!=data.end(); ++it) {
      (*it)->setServerMode(storedServerMode_);
   }

   // Terminate our thread pool
   tp_.reset();

   // GBaseSA sees exactly the environment it would when called from its own class
   GBaseSA::finalize();
}

/******************************************************************************/
/**
 * Adapt all children in parallel. Evaluation is done in a seperate function (evaluateChildren).
 */
void GMultiThreadedSA::adaptChildren()
{
   boost::tuple<std::size_t,std::size_t> range = getAdaptionRange();
   std::vector<boost::shared_ptr<GParameterSet> >::iterator it;

   for(it=data.begin()+boost::get<0>(range); it!=data.begin()+boost::get<1>(range); ++it) {
      tp_->schedule(boost::function<void()>(boost::bind(&GParameterSet::adapt, *it)));
   }

   // Wait for all threads in the pool to complete their work
   tp_->wait();
}

/******************************************************************************/
/**
 * Evaluate all children (and possibly parents, depending on the iteration and sorting mode) in parallel
 */
void GMultiThreadedSA::evaluateChildren()
{
   boost::tuple<std::size_t,std::size_t> range = getEvaluationRange();
   std::vector<boost::shared_ptr<GParameterSet> >::iterator it;

#ifdef DEBUG
   // There should be no situation in which a "clean" individual is submitted
   // through this function
   for(std::size_t i=boost::get<0>(range); i<boost::get<1>(range); i++) {
      if(!this->at(i)->isDirty()) {
         glogger
         << "In GMultiThreadedSA::evaluateChildren(): Error!" << std::endl
         << "Tried to evaluate \"clean\" children." << std::endl
         << GEXCEPTION;
      }
   }
#endif

   // Make evaluation possible and initiate the worker threads
   for(it=data.begin() + boost::get<0>(range); it!=data.begin() + boost::get<1>(range); ++it) {
      (*it)->setServerMode(false);
      tp_->schedule(boost::function<double()>(boost::bind(&GParameterSet::doFitnessCalculation, *it)));
   }

   // Wait for all threads in the pool to complete their work
   tp_->wait();

   // Make re-evaluation impossible
   for(it=data.begin() + boost::get<0>(range); it!=data.begin() + boost::get<1>(range); ++it) {
      (*it)->setServerMode(true);
   }
}

/******************************************************************************/
/**
 * Adds local configuration options to a GParserBuilder object
 *
 * @param gpb The GParserBuilder object to which configuration options should be added
 * @param showOrigin Makes the function indicate the origin of parameters in comments
 */
void GMultiThreadedSA::addConfigurationOptions (
   Gem::Common::GParserBuilder& gpb
   , const bool& showOrigin
) {
   std::string comment;

   // Call our parent class'es function
   GBaseSA::addConfigurationOptions(gpb, showOrigin);

   // Add local data
   comment = ""; // Reset the comment string
   comment += "The number of threads used to simultaneously process individuals;";
   if(showOrigin) comment += "[GMultiThreadedSA]";
   gpb.registerFileParameter<boost::uint16_t>(
      "nEvaluationThreads" // The name of the variable
      , 0 // The default value
      , boost::bind(
         &GMultiThreadedSA::setNThreads
         , this
         , _1
        )
      , Gem::Common::VAR_IS_ESSENTIAL // Alternative: VAR_IS_SECONDARY
      , comment
   );
}

/******************************************************************************/
/**
 * Allows to assign a name to the role of this individual(-derivative). This is mostly important for the
 * GBrokerEA class which should prevent objects of its type from being stored as an individual in its population.
 * All other objects do not need to re-implement this function (unless they rely on the name for some reason).
 */
std::string GMultiThreadedSA::getIndividualCharacteristic() const {
   return std::string("GENEVA_MTOPTALG");
}

/******************************************************************************/
/**
 * Sets the number of threads for this population. If nThreads is set
 * to 0, an attempt will be made to set the number of threads to the
 * number of hardware threading units (e.g. number of cores or hyperthreading
 * units).
 *
 * @param nThreads The number of threads this class uses
 */
void GMultiThreadedSA::setNThreads(boost::uint16_t nThreads) {
   if(nThreads == 0) {
      nThreads_ = boost::numeric_cast<boost::uint16_t>(Gem::Common::getNHardwareThreads(DEFAULTNBOOSTTHREADS));
   }
   else {
      nThreads_ = nThreads;
   }
}

/******************************************************************************/
/**
 * Retrieves the number of threads this population uses.
 *
 * @return The maximum number of allowed threads
 */
boost::uint16_t GMultiThreadedSA::getNThreads() const  {
   return nThreads_;
}

/******************************************************************************/
/**
 * Applies modifications to this object. This is needed for testing purposes
 *
 * @return A boolean which indicates whether modifications were made
 */
bool GMultiThreadedSA::modify_GUnitTests() {
#ifdef GEM_TESTING
   bool result = false;

   // Call the parent class'es function
   if(GBaseSA::modify_GUnitTests()) result = true;

   return result;

#else /* GEM_TESTING */  // If this function is called when GEM_TESTING isn't set, throw
   condnotset("GMultiThreadedSA::modify_GUnitTests", "GEM_TESTING");
   return false;
#endif /* GEM_TESTING */
}

/******************************************************************************/
/**
 * Performs self tests that are expected to succeed. This is needed for testing purposes
 */
void GMultiThreadedSA::specificTestsNoFailureExpected_GUnitTests() {
#ifdef GEM_TESTING
   // Call the parent class'es function
   GBaseSA::specificTestsNoFailureExpected_GUnitTests();

#else /* GEM_TESTING */ // If this function is called when GEM_TESTING isn't set, throw
   condnotset("GMultiThreadedSA::specificTestsNoFailureExpected_GUnitTests", "GEM_TESTING");
#endif /* GEM_TESTING */
}

/******************************************************************************/
/**
 * Performs self tests that are expected to fail. This is needed for testing purposes
 */
void GMultiThreadedSA::specificTestsFailuresExpected_GUnitTests() {
#ifdef GEM_TESTING
   // Call the parent class'es function
   GBaseSA::specificTestsFailuresExpected_GUnitTests();

#else /* GEM_TESTING */ // If this function is called when GEM_TESTING isn't set, throw
   condnotset("GMultiThreadedSA::specificTestsFailuresExpected_GUnitTests", "GEM_TESTING");
#endif /* GEM_TESTING */
}

/******************************************************************************/

} /* namespace Geneva */
} /* namespace Gem */