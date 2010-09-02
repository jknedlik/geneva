/**
 * @file GSwarmPersonalityTraits.cpp
 */

/*
 * Copyright (C) Authors of the Geneva library collection and Karlsruhe
 * Institute of Technology (University of the State of Baden-Wuerttemberg
 * and National Laboratory of the Helmholtz Association).
 *
 * See the AUTHORS file in the top-level directory for a list of authors.
 *
 * Contact: info [at] gemfony (dot) com
 *
 * This file is part of the Geneva library collection
 *
 * Geneva is free software: you can redistribute it and/or modify
 * it under the terms of version 3 of the GNU Affero General Public License
 * as published by the Free Software Foundation.
 *
 * Geneva is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with the Geneva library.  If not, see <http://www.gnu.org/licenses/>.
 *
 * For further information on Gemfony scientific and Geneva, visit
 * http://www.gemfony.com .
 */

#include "geneva/GSwarmPersonalityTraits.hpp"

// Included here so no conflicts occur. See explanation at
// http://www.boost.org/libs/serialization/doc/special.html#derivedpointers
#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT(Gem::Geneva::GSwarmPersonalityTraits)

namespace Gem {
namespace Geneva {

/*****************************************************************************/
/**
 * The default constructor
 */
GSwarmPersonalityTraits::GSwarmPersonalityTraits()
	: GPersonalityTraits()
	, neighborhood_(0)
	, command_("")
	, noPositionUpdate_(false)
{ /* nothing */ }

/*****************************************************************************/
/**
 * The copy contructor
 *
 * @param cp A copy of another GSwarmPersonalityTraits object
 */
GSwarmPersonalityTraits::GSwarmPersonalityTraits(const GSwarmPersonalityTraits& cp)
	: GPersonalityTraits(cp)
	, neighborhood_(cp.neighborhood_)
	, command_(cp.command_)
	, noPositionUpdate_(cp.noPositionUpdate_)
{ /* nothing */ }

/*****************************************************************************/
/**
 * The standard destructor
 */
GSwarmPersonalityTraits::~GSwarmPersonalityTraits()
{ /* nothing */ }

/*****************************************************************************/
/**
 * Checks for equality with another GSwarmPersonalityTraits object
 *
 * @param  cp A constant reference to another GSwarmPersonalityTraits object
 * @return A boolean indicating whether both objects are equal
 */
bool GSwarmPersonalityTraits::operator==(const GSwarmPersonalityTraits& cp) const {
	using namespace Gem::Common;
	// Means: The expectation of equality was fulfilled, if no error text was emitted (which converts to "true")
	return !checkRelationshipWith(cp, CE_EQUALITY, 0.,"GSwarmPersonalityTraits::operator==","cp", CE_SILENT);
}

/*****************************************************************************/
/**
 * Checks for inequality with another GSwarmPersonalityTraits object
 *
 * @param  cp A constant reference to another GSwarmPersonalityTraits object
 * @return A boolean indicating whether both objects are inequal
 */
bool GSwarmPersonalityTraits::operator!=(const GSwarmPersonalityTraits& cp) const {
	using namespace Gem::Common;
	// Means: The expectation of inequality was fulfilled, if no error text was emitted (which converts to "true")
	return !checkRelationshipWith(cp, CE_INEQUALITY, 0.,"GSwarmPersonalityTraits::operator!=","cp", CE_SILENT);
}

/*****************************************************************************/
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
boost::optional<std::string> GSwarmPersonalityTraits::checkRelationshipWith(const GObject& cp,
		const Gem::Common::expectation& e,
		const double& limit,
		const std::string& caller,
		const std::string& y_name,
		const bool& withMessages) const
{
    using namespace Gem::Common;

	// Check that we are indeed dealing with a GParamterBase reference
	const GSwarmPersonalityTraits *p_load = GObject::conversion_cast<GSwarmPersonalityTraits>(&cp);

	// Will hold possible deviations from the expectation, including explanations
    std::vector<boost::optional<std::string> > deviations;

	// Check our parent class'es data ...
	deviations.push_back(GPersonalityTraits::checkRelationshipWith(cp, e, limit, "GSwarmPersonalityTraits", y_name, withMessages));

	// ... and then our local data
	deviations.push_back(checkExpectation(withMessages, "GSwarmPersonalityTraits", neighborhood_, p_load->neighborhood_, "neighborhood_", "p_load->neighborhood_", e , limit));
	deviations.push_back(checkExpectation(withMessages, "GSwarmPersonalityTraits", command_, p_load->command_, "command_", "p_load->command_", e , limit));
	deviations.push_back(checkExpectation(withMessages, "GSwarmPersonalityTraits", noPositionUpdate_, p_load->noPositionUpdate_, "noPositionUpdate_", "p_load->noPositionUpdate_", e , limit));

	return evaluateDiscrepancies("GEAPersonalityTraits", caller, deviations, e);
}

/*****************************************************************************/
/**
 * Sets the noPositionUpdate_ flag
 */
void GSwarmPersonalityTraits::setNoPositionUpdate() {
	noPositionUpdate_ = true;
}

/* ----------------------------------------------------------------------------------
 * Tested in GSwarmPersonalityTraits::specificTestsNoFailuresExpected_GUnitTests()
 * ----------------------------------------------------------------------------------
 */

/*****************************************************************************/
/**
 * Retrieves the current value of the noPositionUpdate_ flag
 *
 * @return The current value of the noPositionUpdate_ flag
 */
bool GSwarmPersonalityTraits::noPositionUpdate() const {
	return noPositionUpdate_;
}

/* ----------------------------------------------------------------------------------
 * Tested in GSwarmPersonalityTraits::specificTestsNoFailuresExpected_GUnitTests()
 * ----------------------------------------------------------------------------------
 */

/*****************************************************************************/
/**
 * Retrieves and resets the current value of the noPositionUpdate_ flag
 *
 * @return The value of the noPositionUpdate_ flag when the function was called
 */
bool GSwarmPersonalityTraits::checkNoPositionUpdateAndReset() {
	bool current = noPositionUpdate_;
	if(noPositionUpdate_) noPositionUpdate_ = false;
	return current;
}

/* ----------------------------------------------------------------------------------
 * Tested in GSwarmPersonalityTraits::specificTestsNoFailuresExpected_GUnitTests()
 * ----------------------------------------------------------------------------------
 */

/*****************************************************************************/
/**
 * Creates a deep clone of this object
 *
 * @return A clone of this object, camouflaged as a GObject
 */
GObject* GSwarmPersonalityTraits::clone_() const {
	return new GSwarmPersonalityTraits(*this);
}

/*****************************************************************************/
/**
 * Loads the data of another GSwarmPersonalityTraits object
 *
 * @param cp A copy of another GSwarmPersonalityTraits object, camouflaged as a GObject
 */
void GSwarmPersonalityTraits::load_(const GObject* cp) {
	const GSwarmPersonalityTraits *p_load = this->conversion_cast<GSwarmPersonalityTraits>(cp);

	// Load the parent class'es data
	GObject::load_(cp);

	// and then the local data
	neighborhood_ = p_load->neighborhood_;
	command_ = p_load->command_;
	noPositionUpdate_ = p_load->noPositionUpdate_;
}

/*****************************************************************************/
/**
 * Specifies in which of the populations neighborhood the individual lives
 *
 * @param neighborhood The current neighborhood of this individual
 */
void GSwarmPersonalityTraits::setNeighborhood(const std::size_t& neighborhood) {
	neighborhood_ = neighborhood;
}

/*****************************************************************************/
/**
 * Retrieves the position of the individual in the population
 *
 * @return The current position of this individual in the population
 */
std::size_t GSwarmPersonalityTraits::getNeighborhood(void) const {
	return neighborhood_;
}

/*****************************************************************************/
/**
 * Sets a command to be performed by a remote client.
 *
 * @param command The command to be performed by a remote client
 */
void GSwarmPersonalityTraits::setCommand(const std::string& command) {
	if(command != "evaluate") { // The allowed "grammar"
		std::ostringstream error;
		error << "In GSwarmPersonalityTraits::setCommand(): Got invalid command " << command << std::endl;
		throw(Gem::Common::gemfony_error_condition(error.str()));
	}

	command_ = command;
}

/* ----------------------------------------------------------------------------------
 * Tested in GSwarmPersonalityTraits::specificTestsNoFailuresExpected_GUnitTests()
 * Tested in GSwarmPersonalityTraits::specificTestsFailuresExpected_GUnitTests()
 * ----------------------------------------------------------------------------------
 */

/*****************************************************************************/
/**
 * Retrieves the command to be performed by a remote client.
 *
 * @return The command to be performed by a remote client.
 */
std::string GSwarmPersonalityTraits::getCommand() const {
#ifdef DEBUG
	// Some error checking
	if(command_.empty() || command_=="" || command_=="empty") {
		std::ostringstream error;
		error << "In GSwarmPersonalityTraits::getCommand(): Error " << std::endl
			  << "Tried to retrieve a command while a command hasn't been set" << std::endl;
		throw(Gem::Common::gemfony_error_condition(error.str()));
	}
#endif /* DEBUG */

	return command_;
}

/* ----------------------------------------------------------------------------------
 * Tested in GSwarmPersonalityTraits::specificTestsNoFailuresExpected_GUnitTests()
 * Tested in GSwarmPersonalityTraits::specificTestsFailuresExpected_GUnitTests()
 * ----------------------------------------------------------------------------------
 */

/*****************************************************************************/
/**
 * Resets the command string
 */
void GSwarmPersonalityTraits::resetCommand() {
	command_ = "";
}

/* ----------------------------------------------------------------------------------
 * Used in GSwarmPersonalityTraits::specificTestsFailuresExpected_GUnitTests()
 * ----------------------------------------------------------------------------------
 */

#ifdef GENEVATESTING
/*****************************************************************************/
/**
 * Applies modifications to this object. This is needed for testing purposes
 *
 * @return A boolean which indicates whether modifications were made
 */
bool GSwarmPersonalityTraits::modify_GUnitTests() {
	bool result = false;

	// Call the parent class'es function
	if(GPersonalityTraits::modify_GUnitTests()) result = true;

	return result;
}

/*****************************************************************************/
/**
 * Performs self tests that are expected to succeed. This is needed for testing purposes
 */
void GSwarmPersonalityTraits::specificTestsNoFailureExpected_GUnitTests() {
	using boost::unit_test_framework::test_suite;
	using boost::unit_test_framework::test_case;

	// Call the parent class'es function
	GPersonalityTraits::specificTestsNoFailureExpected_GUnitTests();

	//------------------------------------------------------------------------------

	{ // Test setting and retrieval of the allowed commands
		boost::shared_ptr<GSwarmPersonalityTraits> p_test = this->clone<GSwarmPersonalityTraits>();

		BOOST_CHECK_NO_THROW(p_test->setCommand("evaluate"));
		BOOST_CHECK(p_test->getCommand() == "evaluate");
	}

	//------------------------------------------------------------------------------

	{ // Test setting and retrieval of the noPositionUpdate_ flag
		boost::shared_ptr<GSwarmPersonalityTraits> p_test = this->clone<GSwarmPersonalityTraits>();

		// Check setting and retrieval
		BOOST_CHECK_NO_THROW(p_test->setNoPositionUpdate());
		BOOST_CHECK(p_test->noPositionUpdate() == true);

		// Check retrieval and reset
		bool noPositionUpdate = false; // This value should be changed by the following call
		BOOST_CHECK_NO_THROW(noPositionUpdate = p_test->checkNoPositionUpdateAndReset());
		BOOST_CHECK(noPositionUpdate == true);
		BOOST_CHECK(p_test->noPositionUpdate() == false);

		// Try again -- the value "false" should not change
		noPositionUpdate = true; // This value should be changed by the following call
		BOOST_CHECK_NO_THROW(noPositionUpdate = p_test->checkNoPositionUpdateAndReset());
		BOOST_CHECK(noPositionUpdate == false);
		BOOST_CHECK(p_test->noPositionUpdate() == false);
	}

	//------------------------------------------------------------------------------

	{ // Test setting and retrieval of the neighborhood
		boost::shared_ptr<GSwarmPersonalityTraits> p_test = this->clone<GSwarmPersonalityTraits>();

		// Setting and retrieval of the neighborhood
		for(std::size_t i=0; i<10; i++) {
			BOOST_CHECK_NO_THROW(p_test->setNeighborhood(i));
			BOOST_CHECK(p_test->getNeighborhood() == i);
		}
	}

	//------------------------------------------------------------------------------
}

/*****************************************************************************/
/**
 * Performs self tests that are expected to fail. This is needed for testing purposes
 */
void GSwarmPersonalityTraits::specificTestsFailuresExpected_GUnitTests() {
	using boost::unit_test_framework::test_suite;
	using boost::unit_test_framework::test_case;

	// Call the parent class'es function
	GPersonalityTraits::specificTestsFailuresExpected_GUnitTests();

	//------------------------------------------------------------------------------

#ifdef DEBUG
	{ // Test that retrieval of an unset command throws in DEBUG mode
		boost::shared_ptr<GSwarmPersonalityTraits> p_test = this->clone<GSwarmPersonalityTraits>();

		// Reset the command string
		BOOST_CHECK_NO_THROW(p_test->resetCommand());

		// Try to retrieve the command string
		BOOST_CHECK_THROW(p_test->getCommand(), Gem::Common::gemfony_error_condition);
	}
#endif /* DEBUG */

	//------------------------------------------------------------------------------

	{ // Check that setting any other command than "evaluate" throws. In particular, "adapt" should throw
		boost::shared_ptr<GSwarmPersonalityTraits> p_test = this->clone<GSwarmPersonalityTraits>();

		// Try to set an unknown command
		BOOST_CHECK_THROW(p_test->setCommand("adapt"), Gem::Common::gemfony_error_condition);
	}

	//------------------------------------------------------------------------------
}

/*****************************************************************************/
#endif /* GENEVATESTING */

} /* namespace Geneva */
} /* namespace Gem */
