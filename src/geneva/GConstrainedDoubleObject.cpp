/**
 * @file GConstrainedDoubleObject.cpp
 */

/*
 * Copyright (C) Gemfony scientific UG (haftungsbeschraenkt)
 *
 * See the AUTHORS file in the top-level directory for a list of authors.
 *
 * Contact: contact [at] gemfony (dot) eu
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
 * http://www.gemfony.eu .
 */

#include "geneva/GConstrainedDoubleObject.hpp"

BOOST_CLASS_EXPORT_IMPLEMENT(Gem::Geneva::GConstrainedDoubleObject)

namespace Gem {
namespace Geneva {

/******************************************************************************/
/**
 * The default constructor
 */
GConstrainedDoubleObject::GConstrainedDoubleObject()
	: GConstrainedFPT<double>()
{ /* nothing */ }

/******************************************************************************/
/**
 * Initialization with boundaries only. The value is set randomly.
 *
 * @param lowerBoundary The lower boundary of the value range
 * @param upperBoundary The upper boundary of the value range
 */
GConstrainedDoubleObject::GConstrainedDoubleObject(
   const double& lowerBoundary
   , const double& upperBoundary
)
	: GConstrainedFPT<double>(lowerBoundary, upperBoundary)
{ /* nothing */ }

/******************************************************************************/
/**
 * Initialization with value and boundaries
 *
 * @param val Initialization value
 * @param lowerBoundary The lower boundary of the value range
 * @param upperBoundary The upper boundary of the value range
 */
GConstrainedDoubleObject::GConstrainedDoubleObject (
     const double& val
   , const double& lowerBoundary
   , const double& upperBoundary
)
	: GConstrainedFPT<double>(val, lowerBoundary, upperBoundary)
{ /* nothing */ }

/******************************************************************************/
/**
 * The copy constructor
 *
 * @param cp A copy of another GConstrainedDoubleObject object
 */
GConstrainedDoubleObject::GConstrainedDoubleObject(const GConstrainedDoubleObject& cp)
	: GConstrainedFPT<double>(cp)
{ /* nothing */ }

/******************************************************************************/
/**
 * Initialization by contained value
 *
 * @param val A value used for the initialization
 */
GConstrainedDoubleObject::GConstrainedDoubleObject(const double& val)
	: GConstrainedFPT<double>(val)
{ /* nothing */ }

/******************************************************************************/
/**
 * The destructor
 */
GConstrainedDoubleObject::~GConstrainedDoubleObject()
{ /* nothing */ }

/***************************************************************************/
/**
 * The standard assignment operator
 */
const GConstrainedDoubleObject& GConstrainedDoubleObject::operator=(
   const GConstrainedDoubleObject& cp
) {
   this->load_(&cp);
   return *this;
}

/******************************************************************************/
/**
 * An assignment operator for the contained value type
 *
 * @param val The value to be assigned to this object
 * @return The value that was just assigned to this object
 */
double GConstrainedDoubleObject::operator=(const double& val) {
	return GConstrainedFPT<double>::operator=(val);
}

/******************************************************************************/
/**
 * Creates a deep clone of this object.
 *
 * @return A copy of this object, camouflaged as a GObject
 */
GObject* GConstrainedDoubleObject::clone_() const {
	return new GConstrainedDoubleObject(*this);
}

/******************************************************************************/
/**
 * Checks for equality with another GConstrainedDoubleObject object
 *
 * @param  cp A constant reference to another GConstrainedDoubleObject object
 * @return A boolean indicating whether both objects are equal
 */
bool GConstrainedDoubleObject::operator==(const GConstrainedDoubleObject& cp) const {
   using namespace Gem::Common;
   try {
      this->compare(cp, CE_EQUALITY, CE_DEF_SIMILARITY_DIFFERENCE);
      return true;
   } catch(g_expectation_violation&) {
      return false;
   }
}

/******************************************************************************/
/**
 * Checks for inequality with another GConstrainedDoubleObject object
 *
 * @param  cp A constant reference to another GConstrainedDoubleObject object
 * @return A boolean indicating whether both objects are inequal
 */
bool GConstrainedDoubleObject::operator!=(const GConstrainedDoubleObject& cp) const {
   using namespace Gem::Common;
   try {
      this->compare(cp, CE_INEQUALITY, CE_DEF_SIMILARITY_DIFFERENCE);
      return true;
   } catch(g_expectation_violation&) {
      return false;
   }
}

/******************************************************************************/
/**
 * Searches for compliance with expectations with respect to another object
 * of the same type
 *
 * @param cp A constant reference to another GObject object
 * @param e The expected outcome of the comparison
 * @param limit The maximum deviation for floating point values (important for similarity checks)
 */
void GConstrainedDoubleObject::compare(
   const GObject& cp
   , const Gem::Common::expectation& e
   , const double& limit
) const {
   using namespace Gem::Common;

   // Check that we are indeed dealing with a GBaseEA reference
   const GConstrainedDoubleObject *p_load = GObject::gobject_conversion<GConstrainedDoubleObject>(&cp);

   try {
      BEGIN_COMPARE;

      // Check our parent class'es data ...
      COMPARE_PARENT(GConstrainedFPT<double>, cp, e, limit);

      // ... no local data

      END_COMPARE;

   } catch(g_expectation_violation& g) { // Create a suitable stack-trace
      throw g("g_expectation_violation caught by GConstrainedDoubleObject");
   }
}

/***********************************************************************************/
/**
 * Emits a name for this class / object
 */
std::string GConstrainedDoubleObject::name() const {
   return std::string("GConstrainedDoubleObject");
}

/******************************************************************************/
/**
 * Attach our local value to the vector. This is used to collect all parameters of this type
 * in the sequence in which they were registered.
 *
 * @param parVec The vector to which the local value should be attached
 */
void GConstrainedDoubleObject::doubleStreamline(
   std::vector<double>& parVec
   , const activityMode& am
) const {
   // Note: application of the transfer function happens in GConstrainedNumT inside value()
	parVec.push_back(this->value());
}

/******************************************************************************/
/**
 * Attach our local value to the map.
 *
 * @param parVec The map to which the local value should be attached
 */
void GConstrainedDoubleObject::doubleStreamline(
   std::map<std::string
   , std::vector<double> >& parVec
   , const activityMode& am
) const {
   std::vector<double> parameters;
   // Note: application of the transfer function happens in GConstrainedNumT inside value()
   parameters.push_back(this->value());
   parVec[this->getParameterName()]=parameters;
}

/******************************************************************************/
/**
 * Attach boundaries of type double to the vectors.
 *
 * @param lBndVec A vector of lower double parameter boundaries
 * @param uBndVec A vector of upper double parameter boundaries
 */
void GConstrainedDoubleObject::doubleBoundaries(
   std::vector<double>& lBndVec
   , std::vector<double>& uBndVec
   , const activityMode& am
) const {
	lBndVec.push_back(this->getLowerBoundary());
	uBndVec.push_back(this->getUpperBoundary());
}

/******************************************************************************/
/**
 * Tell the audience that we own a double value
 *
 * @param am An enum indicating whether only information about active, inactive or all parameters of this type should be extracted
 * @return The number 1, as we own a single double parameter
 */
std::size_t GConstrainedDoubleObject::countDoubleParameters(
   const activityMode& am
) const {
   return 1;
}

/******************************************************************************/
/**
 * Assigns part of a value vector to the parameter. Note that we apply a transformation to
 * the parameter value, so that it lies inside of the allowed value range.
 */
void GConstrainedDoubleObject::assignDoubleValueVector(
   const std::vector<double>& parVec
   , std::size_t& pos
   , const activityMode& am
) {
#ifdef DEBUG
	// Do we have a valid position ?
	if(pos >= parVec.size()) {
	   glogger
	   << "In GConstrainedDoubleObject::assignDoubleValueVector(const std::vector<double>&, std::size_t&):" << std::endl
      << "Tried to access position beyond end of vector: " << parVec.size() << "/" << pos << std::endl
      << GEXCEPTION;
	}
#endif

	this->setValue(this->transfer(parVec[pos]));
	pos++;
}

/******************************************************************************/
/**
 * Assigns part of a value map to the parameter
 */
void GConstrainedDoubleObject::assignDoubleValueVectors(
   const std::map<std::string, std::vector<double> >& parMap
   , const activityMode& am
) {
   this->setValue(
      this->transfer(
         Gem::Common::getMapItem(
            parMap
            ,this->getParameterName()
         ).at(0)
      )
   );
}

/******************************************************************************/
/**
 * Multiplication with a random value in a given range
 */
void GConstrainedDoubleObject::doubleMultiplyByRandom(
   const double& min
   , const double& max
   , const activityMode& am
) {
   GParameterT<double>::setValue(
      transfer(
         GParameterT<double>::value() *
         this->GObject::gr_ptr()->uniform_real<double>(min, max)
      )
   );
}

/******************************************************************************/
/**
 * Multiplication with a random value in the range [0,1[
 */
void GConstrainedDoubleObject::doubleMultiplyByRandom(
   const activityMode& am
) {
   GParameterT<double>::setValue(
      transfer(
         GParameterT<double>::value() *
         this->GObject::gr_ptr()->uniform_01<double>()
      )
   );
}

/******************************************************************************/
/**
 * Multiplication with a constant value
 */
void GConstrainedDoubleObject::doubleMultiplyBy(
   const double& val
   , const activityMode& am
) {
   GParameterT<double>::setValue(transfer(val * GParameterT<double>::value()));
}

/******************************************************************************/
/**
 * Initialization with a constant value
 */
void GConstrainedDoubleObject::doubleFixedValueInit(
   const double& val
   , const activityMode& am
) {
   GParameterT<double>::setValue(transfer(val));
}

/******************************************************************************/
/**
 * Adds the "same-type" parameters of another GParameterBase object to this one
 */
void GConstrainedDoubleObject::doubleAdd(
   boost::shared_ptr<GParameterBase> p_base
   , const activityMode& am
) {
   // We first need to convert p_base into the local type
   boost::shared_ptr<GConstrainedDoubleObject> p
      = GParameterBase::parameterbase_cast<GConstrainedDoubleObject>(p_base);
   GParameterT<double>::setValue(transfer(this->value() + p->value()));
}

/******************************************************************************/
/**
 * Adds the "same-type" parameters of another GParameterBase object to this one
 */
void GConstrainedDoubleObject::doubleSubtract(
   boost::shared_ptr<GParameterBase> p_base
   , const activityMode& am
) {
   // We first need to convert p_base into the local type
   boost::shared_ptr<GConstrainedDoubleObject> p
      = GParameterBase::parameterbase_cast<GConstrainedDoubleObject>(p_base);
   GParameterT<double>::setValue(transfer(this->value() - p->value()));
}


/******************************************************************************/
/**
 * Loads the data of another GObject
 *
 * @param cp A copy of another GConstrainedDoubleObject object, camouflaged as a GObject
 */
void GConstrainedDoubleObject::load_(const GObject* cp){
	// Check for a possible self-assignment
    GObject::selfAssignmentCheck<GConstrainedDoubleObject>(cp);

	// Load our parent class'es data ...
	GConstrainedFPT<double>::load_(cp);

	// ... no local data
}

/******************************************************************************/
/**
 * Applies modifications to this object. This is needed for testing purposes
 *
 * @return A boolean which indicates whether modifications were made
 */
bool GConstrainedDoubleObject::modify_GUnitTests() {
#ifdef GEM_TESTING
	bool result = false;

	// Call the parent class'es function
	if(GConstrainedFPT<double>::modify_GUnitTests()) result = true;

	return result;

#else /* GEM_TESTING */  // If this function is called when GEM_TESTING isn't set, throw
   condnotset("GConstrainedDoubleObject::modify_GUnitTests", "GEM_TESTING");
   return false;
#endif /* GEM_TESTING */
}

/******************************************************************************/
/**
 * Performs self tests that are expected to succeed. This is needed for testing purposes
 */
void GConstrainedDoubleObject::specificTestsNoFailureExpected_GUnitTests() {
#ifdef GEM_TESTING
	// Some general settings
	const double testVal = 42.;
	const double testVal2 = 17.;
	double testVal3 = 0.;
	const double lowerBoundary = 0.;
	const double upperBoundary = 100.;
	const std::size_t NTESTS = 100;

	// Make sure we have an appropriate adaptor loaded when performing these tests
	bool adaptorStored = false;
	boost::shared_ptr<GAdaptorT<double> > storedAdaptor;

	if(this->hasAdaptor()) {
		storedAdaptor = this->getAdaptor();
		adaptorStored = true;
	}

	boost::shared_ptr<GDoubleGaussAdaptor> gdga_ptr(new GDoubleGaussAdaptor(0.025, 0.1, 0., 1., 1.0));
	gdga_ptr->setAdaptionThreshold(0); // Make sure the adaptor's internal parameters don't change through the adaption
	gdga_ptr->setAdaptionMode(true); // Always adapt
	this->addAdaptor(gdga_ptr);

	// Call the parent class'es function
	GConstrainedFPT<double>::specificTestsNoFailureExpected_GUnitTests();

	// --------------------------------------------------------------------------

	{ // Check that assignment of a value with operator= works both for set and unset boundaries
		boost::shared_ptr<GConstrainedDoubleObject> p_test = this->GObject::clone<GConstrainedDoubleObject>();

		// Reset the boundaries so we are free to do what we want
		BOOST_CHECK_NO_THROW(p_test->resetBoundaries());

		// Assign a value with operator=
		BOOST_CHECK_NO_THROW(*p_test = testVal2);

		// Check the value
		BOOST_CHECK(p_test->value() == testVal2);

		// Assign boundaries and values
		BOOST_CHECK_NO_THROW(p_test->setValue(testVal2, lowerBoundary, upperBoundary));

		// Check the value again
		BOOST_CHECK(p_test->value() == testVal2);

		// Assign a value with operator=
		BOOST_CHECK_NO_THROW(*p_test = testVal);

		// Check the value again, should have changed
		BOOST_CHECK(p_test->value() == testVal);
	}

	// --------------------------------------------------------------------------

	{ // Check construction with two boundaries plus initialization with a random value and extraction of that value
		boost::shared_ptr<GConstrainedDoubleObject> p_test(new GConstrainedDoubleObject(0.3, 0.6));
		BOOST_CHECK_NO_THROW(testVal3 = p_test->value());
	}

   // --------------------------------------------------------------------------

   { // Check construction with two boundaries and a value and extraction of that value
      const double TESTVAL = 0.4;
      boost::shared_ptr<GConstrainedDoubleObject> p_test(new GConstrainedDoubleObject(0.4, 0.3, 0.6));
      BOOST_CHECK_NO_THROW(testVal3 = p_test->value());
      BOOST_CHECK(testVal3 == TESTVAL);
   }

   // --------------------------------------------------------------------------

   { // Check that repeated retrieval of the value always yields the same value
      const double TESTVAL = 0.4;
      boost::shared_ptr<GConstrainedDoubleObject> p_test(new GConstrainedDoubleObject(0.4, 0.3, 0.6));
      for(std::size_t i=0; i<NTESTS; i++) {
         BOOST_CHECK_NO_THROW(testVal3 = p_test->value());
         BOOST_CHECK_MESSAGE(
               testVal3 == TESTVAL
               , "The value has changed: " << testVal3 << " / " << TESTVAL
         );
      }
   }

	// --------------------------------------------------------------------------

	// Remove the test adaptor
	this->resetAdaptor();

	// Load the old adaptor, if needed
	if(adaptorStored) {
		this->addAdaptor(storedAdaptor);
	}

   // --------------------------------------------------------------------------

#else /* GEM_TESTING */ // If this function is called when GEM_TESTING isn't set, throw
   condnotset("GConstrainedDoubleObject::specificTestsNoFailureExpected_GUnitTests", "GEM_TESTING");
#endif /* GEM_TESTING */
}

/******************************************************************************/
/**
 * Performs self tests that are expected to fail. This is needed for testing purposes
 */
void GConstrainedDoubleObject::specificTestsFailuresExpected_GUnitTests() {
#ifdef GEM_TESTING
	// Make sure we have an appropriate adaptor loaded when performing these tests
	bool adaptorStored = false;
	boost::shared_ptr<GAdaptorT<double> > storedAdaptor;

	if(this->hasAdaptor()) {
		storedAdaptor = this->getAdaptor();
		adaptorStored = true;
	}

	boost::shared_ptr<GDoubleGaussAdaptor> gdga_ptr(new GDoubleGaussAdaptor(0.025, 0.1, 0., 1., 1.0));
	gdga_ptr->setAdaptionThreshold(0); // Make sure the adaptor's internal parameters don't change through the adaption
	gdga_ptr->setAdaptionMode(true); // Always adapt
	this->addAdaptor(gdga_ptr);

	// Call the parent class'es function
	GConstrainedFPT<double>::specificTestsFailuresExpected_GUnitTests();

	// Remove the test adaptor
	this->resetAdaptor();

	// Load the old adaptor, if needed
	if(adaptorStored) {
		this->addAdaptor(storedAdaptor);
	}

#else /* GEM_TESTING */ // If this function is called when GEM_TESTING isn't set, throw
   condnotset("GConstrainedDoubleObject::specificTestsFailuresExpected_GUnitTests", "GEM_TESTING");
#endif /* GEM_TESTING */
}

/******************************************************************************/

} /* namespace Geneva */
} /* namespace Gem */
