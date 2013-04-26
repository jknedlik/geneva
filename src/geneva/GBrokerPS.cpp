/**
 * @file GBrokerPS.cpp
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

#include "geneva/GBrokerPS.hpp"

BOOST_CLASS_EXPORT_IMPLEMENT(Gem::Geneva::GBrokerPS)

namespace Gem {
namespace Geneva {

/******************************************************************************/
/**
 * The default constructor
 */
GBrokerPS::GBrokerPS() :
   GBasePS()
   , Gem::Courtier::GBrokerConnectorT<GParameterSet>()
   , storedServerMode_(true)
{ /* nothing */ }

/******************************************************************************/
/**
 * A standard copy constructor
 */
GBrokerPS::GBrokerPS(const GBrokerPS& cp)
   : GBasePS(cp)
   , Gem::Courtier::GBrokerConnectorT<GParameterSet>(cp)
   , storedServerMode_(true)
{ /* nothing */ }

/******************************************************************************/
/**
 * The destructor.
 */
GBrokerPS::~GBrokerPS()
{ /* nothing */ }

/******************************************************************************/
/**
 * A standard assignment operator for GBrokerPS objects.
 *
 * @param cp Reference to another GBrokerPS object
 * @return A constant reference to this object
 */
const GBrokerPS& GBrokerPS::operator=(const GBrokerPS& cp) {
   GBrokerPS::load_(&cp);
   return *this;
}

/******************************************************************************/
/**
 * Checks for equality with another GBrokerPS object
 *
 * @param  cp A constant reference to another GBrokerPS object
 * @return A boolean indicating whether both objects are equal
 */
bool GBrokerPS::operator==(const GBrokerPS& cp) const
{
   using namespace Gem::Common;
   // Means: The expectation of equality was fulfilled, if no error text was emitted (which converts to "true")
   return !checkRelationshipWith(cp, CE_EQUALITY, 0., "GBrokerPS::operator==",
         "cp", CE_SILENT);
}

/******************************************************************************/
/**
 * Checks for inequality with another GBrokerPS object
 *
 * @param  cp A constant reference to another GBrokerPS object
 * @return A boolean indicating whether both objects are inequal
 */
bool GBrokerPS::operator!=(const GBrokerPS& cp) const
{
   using namespace Gem::Common;
   // Means: The expectation of inequality was fulfilled, if no error text was emitted (which converts to "true")
   return !checkRelationshipWith(cp, CE_INEQUALITY, 0.,
         "GBrokerPS::operator!=", "cp", CE_SILENT);
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
boost::optional<std::string> GBrokerPS::checkRelationshipWith(
      const GObject& cp
      , const Gem::Common::expectation& e
      , const double& limit
      , const std::string& caller
      , const std::string& y_name
      , const bool& withMessages
) const {
   using namespace Gem::Common;
   using namespace Gem::Courtier;

   // Check that we are indeed dealing with a GParamterBase reference
   const GBrokerPS *p_load = GObject::gobject_conversion<GBrokerPS>(&cp);

   // Will hold possible deviations from the expectation, including explanations
   std::vector < boost::optional<std::string> > deviations;

   // Check our parent classes' data ...
   deviations.push_back(GBasePS::checkRelationshipWith(cp, e, limit, "GBrokerPS",   y_name, withMessages));
   deviations.push_back(GBrokerConnectorT<GParameterSet>::checkRelationshipWith(*p_load, e, limit, "GBrokerPS", y_name, withMessages));

   // no local data

   return evaluateDiscrepancies("GBrokerPS", caller, deviations, e);
}

/***********************************************************************************/
/**
 * Emits a name for this class / object
 */
std::string GBrokerPS::name() const {
   return std::string("GBrokerPS");
}

/******************************************************************************/
/**
 * Checks whether this algorithm communicates via the broker. This is an overload from the corresponding
 * GOptimizableI function
 *
 * @return A boolean indicating whether this algorithm communicates via the broker
 */
bool GBrokerPS::usesBroker() const {
   return true;
}

/******************************************************************************/
/**
 * Loads the data from another GBrokerPS object.
 *
 * @param vp Pointer to another GBrokerPS object, camouflaged as a GObject
 */
void GBrokerPS::load_(const GObject *cp) {
   const GBrokerPS *p_load = gobject_conversion<GBrokerPS> (cp);

   // Load the parent classes' data ...
   GBasePS::load_(cp);
   Gem::Courtier::GBrokerConnectorT<GParameterSet>::load(p_load);

   // ... no local data. We do not load storedServerMode_, which is a temporary
}

/******************************************************************************/
/**
 * Creates a deep clone of this object
 *
 * @return A deep copy of this object, camouflaged as a GObject
 */
GObject *GBrokerPS::clone_() const {
   return new GBrokerPS(*this);
}

/******************************************************************************/
/**
 * Necessary initialization work before the start of the optimization
 */
void GBrokerPS::init() {
   // GGradientDesccent sees exactly the environment it would when called from its own class
   GBasePS::init();

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
         << "In GBrokerPS::init():" << std::endl
         << "Not all server mode flags have the same value!" << std::endl
         << GEXCEPTION;
      }
   }
}

/******************************************************************************/
/**
 * Necessary clean-up work after the optimization has finished
 */
void GBrokerPS::finalize() {
   // Restore the original values
   std::vector<boost::shared_ptr<GParameterSet> >::iterator it;
   for(it=data.begin(); it!=data.end(); ++it) {
      (*it)->setServerMode(storedServerMode_);
   }

   // GBasePS sees exactly the environment it would when called from its own class
   GBasePS::finalize();
}

/******************************************************************************/
/**
 * Adds local configuration options to a GParserBuilder object
 *
 * @param gpb The GParserBuilder object to which configuration options should be added
 * @param showOrigin Makes the function indicate the origin of parameters in comments
 */
void GBrokerPS::addConfigurationOptions (
   Gem::Common::GParserBuilder& gpb
   , const bool& showOrigin
) {
   std::string comment;

   // Call our parent class'es function
   GBasePS::addConfigurationOptions(gpb, showOrigin);
   Gem::Courtier::GBrokerConnectorT<GParameterSet>::addConfigurationOptions(gpb, showOrigin);

   // no local data
}

/******************************************************************************/
/**
 * Allows to assign a name to the role of this individual(-derivative). This is mostly important for the
 * GBrokerEA class which should prevent objects of its type from being stored as an individual in its population.
 * All other objects do not need to re-implement this function (unless they rely on the name for some reason).
 */
std::string GBrokerPS::getIndividualCharacteristic() const {
   return std::string("GENEVA_BROKEROPTALG");
}

/******************************************************************************/
/**
 * Triggers fitness calculation of a number of individuals. This function performs the same task as done
 * in GBasePS, albeit by delegating work to the broker. Items are evaluated up to a maximum position
 * in the vector. Note that we always start the evaluation with the first item in the vector.
 *
 * @param finalPos The position in the vector up to which the fitness calculation should be performed
 * @return The best fitness found amongst all parents
 */
double GBrokerPS::doFitnessCalculation(const std::size_t& finalPos) {
   using namespace Gem::Courtier;
   bool complete = false;

#ifdef DEBUG
   for (std::size_t i = 0; i < finalPos; i++) {
      // Make sure the evaluated individuals have the dirty flag set
      if(!this->at(i)->isDirty()) {
         glogger
         << "In GBrokerPS::doFitnessCalculation(const std::size_t&):" << std::endl
         << "Found individual in position " << i << " whose dirty flag isn't set" << std::endl
         << GEXCEPTION;
      }
   }
#endif /* DEBUG */

   //--------------------------------------------------------------------------------
   // Submit all work items and wait for their return
   boost::tuple<std::size_t,std::size_t> range(0, this->size());
   complete = GBrokerConnectorT<GParameterSet>::workOn(
         data
         , range
         , EXPECTFULLRETURN
   );

   if(!complete) {
      glogger
      << "In GBrokerPS::doFitnessCalculation(): Error!" << std::endl
      << "No complete set of items received" << std::endl
      << GEXCEPTION;
   }

   //--------------------------------------------------------------------------------
   // Retrieve information about the best fitness found
   double bestFitness = getWorstCase(); // Holds the best fitness found so far
   double fitnessFound = 0.;
   for (std::size_t i = 0; i < this->size(); i++) {
      fitnessFound = this->at(i)->fitness();

      if (isBetter(fitnessFound, bestFitness)) {
         bestFitness = fitnessFound;
      }
   }

   return bestFitness;
}

/******************************************************************************/
/**
 * Applies modifications to this object. This is needed for testing purposes
 *
 * @return A boolean which indicates whether modifications were made
 */
bool GBrokerPS::modify_GUnitTests() {
#ifdef GEM_TESTING

   bool result = false;

   // Call the parent class'es function
   if(GBasePS::modify_GUnitTests()) result = true;

   return result;

#else /* GEM_TESTING */  // If this function is called when GEM_TESTING isn't set, throw
   condnotset("GBrokerPS::modify_GUnitTests", "GEM_TESTING");
   return false;
#endif /* GEM_TESTING */
}

/******************************************************************************/
/**
 * Performs self tests that are expected to succeed. This is needed for testing purposes
 */
void GBrokerPS::specificTestsNoFailureExpected_GUnitTests() {
#ifdef GEM_TESTING
   // Call the parent class'es function
   GBasePS::specificTestsNoFailureExpected_GUnitTests();

#else /* GEM_TESTING */ // If this function is called when GEM_TESTING isn't set, throw
   condnotset("GBrokerPS::specificTestsNoFailureExpected_GUnitTests", "GEM_TESTING");
#endif /* GEM_TESTING */
}

/******************************************************************************/
/**
 * Performs self tests that are expected to fail. This is needed for testing purposes
 */
void GBrokerPS::specificTestsFailuresExpected_GUnitTests() {
#ifdef GEM_TESTING
   // Call the parent class'es function
   GBasePS::specificTestsFailuresExpected_GUnitTests();

#else /* GEM_TESTING */ // If this function is called when GEM_TESTING isn't set, throw
   condnotset("GBrokerPS::specificTestsFailuresExpected_GUnitTests", "GEM_TESTING");
#endif /* GEM_TESTING */
}

/******************************************************************************/

} /* namespace Geneva */
} /* namespace Gem */
