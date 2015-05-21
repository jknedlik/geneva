/**
 * @file GInt32FlipAdaptor.cpp
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

#include "geneva/GInt32FlipAdaptor.hpp"

// Make sure the class gets instantiated at least once
BOOST_CLASS_EXPORT_IMPLEMENT(Gem::Geneva::GInt32FlipAdaptor)

namespace Gem {
namespace Geneva {

/******************************************************************************/
/**
 * The default constructor
 */
GInt32FlipAdaptor::GInt32FlipAdaptor()
	: GIntFlipAdaptorT<boost::int32_t>() { /* nothing */ }

/******************************************************************************/
/**
 * The copy constructor
 *
 * @param cp A copy of another GInt32FlipAdaptor object
 */
GInt32FlipAdaptor::GInt32FlipAdaptor(const GInt32FlipAdaptor &cp)
	: GIntFlipAdaptorT<boost::int32_t>(cp) { /* nothing */ }

/******************************************************************************/
/**
 * Initialization with a adaption probability
 *
 * @param adProb The adaption probability
 */
GInt32FlipAdaptor::GInt32FlipAdaptor(const double &adProb)
	: GIntFlipAdaptorT<boost::int32_t>(adProb) { /* nothing */ }

/******************************************************************************/
/**
 * The destructor
 */
GInt32FlipAdaptor::~GInt32FlipAdaptor() { /* nothing */ }

/******************************************************************************/
/**
 * Creates a deep clone of this object.
 *
 * @return A copy of this object, camouflaged as a GObject
 */
GObject *GInt32FlipAdaptor::clone_() const {
	return new GInt32FlipAdaptor(*this);
}

/***************************************************************************/
/**
 * The standard assignment operator
 */
const GInt32FlipAdaptor &GInt32FlipAdaptor::operator=(const GInt32FlipAdaptor &cp) {
	this->load_(&cp);
	return *this;
}

/******************************************************************************/
/**
 * Checks for equality with another GInt32FlipAdaptor object
 *
 * @param  cp A constant reference to another GInt32FlipAdaptor object
 * @return A boolean indicating whether both objects are equal
 */
bool GInt32FlipAdaptor::operator==(const GInt32FlipAdaptor &cp) const {
	using namespace Gem::Common;
	try {
		this->compare(cp, CE_EQUALITY, CE_DEF_SIMILARITY_DIFFERENCE);
		return true;
	} catch (g_expectation_violation &) {
		return false;
	}
}

/******************************************************************************/
/**
 * Checks for inequality with another GInt32FlipAdaptor object
 *
 * @param  cp A constant reference to another GInt32FlipAdaptor object
 * @return A boolean indicating whether both objects are inequal
 */
bool GInt32FlipAdaptor::operator!=(const GInt32FlipAdaptor &cp) const {
	using namespace Gem::Common;
	try {
		this->compare(cp, CE_INEQUALITY, CE_DEF_SIMILARITY_DIFFERENCE);
		return true;
	} catch (g_expectation_violation &) {
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
void GInt32FlipAdaptor::compare(
	const GObject &cp, const Gem::Common::expectation &e, const double &limit
) const {
	using namespace Gem::Common;

	// Check that we are indeed dealing with a GBaseEA reference
	const GInt32FlipAdaptor *p_load = GObject::gobject_conversion<GInt32FlipAdaptor>(&cp);

	GToken token("GInt32FlipAdaptor", e);

	// Compare our parent data ...
	Gem::Common::compare_base<GIntFlipAdaptorT<boost::int32_t> >(IDENTITY(*this, *p_load), token);

	// ... no local data

	// React on deviations from the expectation
	token.evaluate();
}

/***********************************************************************************/
/**
 * Emits a name for this class / object
 */
std::string GInt32FlipAdaptor::name() const {
	return std::string("GInt32FlipAdaptor");
}

/******************************************************************************/
/**
 * Loads the data of another GObject
 *
 * @param cp A copy of another GInt32FlipAdaptor object, camouflaged as a GObject
 */
void GInt32FlipAdaptor::load_(const GObject *cp) {
	// Check that we are not accidently assigning this object to itself
	GObject::selfAssignmentCheck<GInt32FlipAdaptor>(cp);

	// Load our parent class'es data ...
	GIntFlipAdaptorT<boost::int32_t>::load_(cp);

	// ... no local data
}

/******************************************************************************/
/**
 * Retrieves the id of this adaptor
 *
 * @return The id of this adaptor
 */
Gem::Geneva::adaptorId GInt32FlipAdaptor::getAdaptorId() const {
	return Gem::Geneva::GINT32FLIPADAPTOR;
}

/* ----------------------------------------------------------------------------------
 * Tested in GInt32FlipAdaptor::specificTestsNoFailuresExpected_GUnitTests()
 * ----------------------------------------------------------------------------------
 */

/******************************************************************************/
/**
 * Applies modifications to this object. This is needed for testing purposes
 *
 * @return A boolean which indicates whether modifications were made
 */
bool GInt32FlipAdaptor::modify_GUnitTests() {
#ifdef GEM_TESTING

	using boost::unit_test_framework::test_suite;
	using boost::unit_test_framework::test_case;

	bool result = false;

	// Call the parent class'es function
	if (GIntFlipAdaptorT<boost::int32_t>::modify_GUnitTests()) result = true;

	return result;

#else /* GEM_TESTING */  // If this function is called when GEM_TESTING isn't set, throw
   condnotset("GInt32FlipAdaptor::modify_GUnitTests", "GEM_TESTING");
   return false;
#endif /* GEM_TESTING */
}

/******************************************************************************/
/**
 * Performs self tests that are expected to succeed. This is needed for testing purposes
 */
void GInt32FlipAdaptor::specificTestsNoFailureExpected_GUnitTests() {
#ifdef GEM_TESTING

	using boost::unit_test_framework::test_suite;
	using boost::unit_test_framework::test_case;

	// Call the parent class'es function
	GIntFlipAdaptorT<boost::int32_t>::specificTestsNoFailureExpected_GUnitTests();

	// --------------------------------------------------------------------------

	{ // Check that the adaptor returns the correct adaptor id
		std::shared_ptr <GInt32FlipAdaptor> p_test = this->clone<GInt32FlipAdaptor>();

		BOOST_CHECK_MESSAGE(
			p_test->getAdaptorId() == GINT32FLIPADAPTOR, "\n"
																		<< "p_test->getAdaptorId() = " << p_test->getAdaptorId()
																		<< "GINT32FLIPADAPTOR      = " << GINT32FLIPADAPTOR << "\n"
		);
	}

	// --------------------------------------------------------------------------

#else /* GEM_TESTING */ // If this function is called when GEM_TESTING isn't set, throw
   condnotset("GInt32FlipAdaptor::specificTestsNoFailureExpected_GUnitTests", "GEM_TESTING");
#endif /* GEM_TESTING */
}

/******************************************************************************/
/**
 * Performs self tests that are expected to fail. This is needed for testing purposes
 */
void GInt32FlipAdaptor::specificTestsFailuresExpected_GUnitTests() {
#ifdef GEM_TESTING

	using boost::unit_test_framework::test_suite;
	using boost::unit_test_framework::test_case;

	// Call the parent class'es function
	GIntFlipAdaptorT<boost::int32_t>::specificTestsFailuresExpected_GUnitTests();

	// no local data - nothing to test

#else /* GEM_TESTING */ // If this function is called when GEM_TESTING isn't set, throw
   condnotset("GInt32FlipAdaptor::specificTestsFailuresExpected_GUnitTests", "GEM_TESTING");
#endif /* GEM_TESTING */
}

/******************************************************************************/

} /* namespace Geneva */
} /* namespace Gem */
