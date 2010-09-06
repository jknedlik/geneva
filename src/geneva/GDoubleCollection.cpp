/**
 * @file GDoubleCollection.cpp
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

#include "geneva/GDoubleCollection.hpp"

/**
 * Included here so no conflicts occur. See explanation at
 * http://www.boost.org/libs/serialization/doc/special.html#derivedpointers
 */
#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT(Gem::Geneva::GDoubleCollection)


namespace Gem {
namespace Geneva {

/*******************************************************************************************/
/**
 * The default constructor
 */
GDoubleCollection::GDoubleCollection()
{ /* nothing */ }

/*******************************************************************************************/
/**
 * Initialization with a number of random values in a given range
 *
 * @param nval The amount of random values
 * @param min The minimum random value
 * @param max The maximum random value
 */
GDoubleCollection::GDoubleCollection(const std::size_t& nval, const double& min, const double& max)
	: GFPNumCollectionT<double>(nval, min, max)
{ /* nothing */ }

/*******************************************************************************************/
/**
 * The copy constructor
 *
 * @param cp A copy of another GDoubleCollection object
 */
GDoubleCollection::GDoubleCollection(const GDoubleCollection& cp)
	: GFPNumCollectionT<double>(cp)
{ /* nothing */ }

/*******************************************************************************************/
/**
 * The destructor
 */
GDoubleCollection::~GDoubleCollection()
{ /* nothing */ }

/*******************************************************************************************/
/**
 * A standard assignment operator.
 *
 * @param cp A copy of another GDoubleCollection object
 * @return A constant reference to this object
 */
const GDoubleCollection& GDoubleCollection::operator=(const GDoubleCollection& cp){
	GDoubleCollection::load_(&cp);
	return *this;
}

/*******************************************************************************************/
/**
 * Creates a deep clone of this object.
 *
 * @return A copy of this object, camouflaged as a GObject
 */
GObject* GDoubleCollection::clone_() const {
	return new GDoubleCollection(*this);
}

/*******************************************************************************************/
/**
 * Checks for equality with another GDoubleCollection object
 *
 * @param  cp A constant reference to another GDoubleCollection object
 * @return A boolean indicating whether both objects are equal
 */
bool GDoubleCollection::operator==(const GDoubleCollection& cp) const {
	using namespace Gem::Common;
	// Means: The expectation of equality was fulfilled, if no error text was emitted (which converts to "true")
	return !checkRelationshipWith(cp, CE_EQUALITY, 0.,"GDoubleCollection::operator==","cp", CE_SILENT);
}

/*******************************************************************************************/
/**
 * Checks for inequality with another GDoubleCollection object
 *
 * @param  cp A constant reference to another GDoubleCollection object
 * @return A boolean indicating whether both objects are inequal
 */
bool GDoubleCollection::operator!=(const GDoubleCollection& cp) const {
	using namespace Gem::Common;
	// Means: The expectation of inequality was fulfilled, if no error text was emitted (which converts to "true")
	return !checkRelationshipWith(cp, CE_INEQUALITY, 0.,"GDoubleCollection::operator!=","cp", CE_SILENT);
}

/*******************************************************************************************/
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
boost::optional<std::string> GDoubleCollection::checkRelationshipWith(const GObject& cp,
		const Gem::Common::expectation& e,
		const double& limit,
		const std::string& caller,
		const std::string& y_name,
		const bool& withMessages) const
{
    using namespace Gem::Common;

    // Check that we are not accidently assigning this object to itself
    GObject::selfAssignmentCheck<GDoubleCollection>(&cp);

	// Will hold possible deviations from the expectation, including explanations
    std::vector<boost::optional<std::string> > deviations;

	// Check our parent class'es data ...
	deviations.push_back(GFPNumCollectionT<double>::checkRelationshipWith(cp, e, limit, "GDoubleCollection", y_name, withMessages));

	// no local data ...

	return evaluateDiscrepancies("GDoubleCollection", caller, deviations, e);
}

/*******************************************************************************************/
/**
 * Loads the data of another GObject
 *
 * @param cp A copy of another GDoubleCollection object, camouflaged as a GObject
 */
void GDoubleCollection::load_(const GObject* cp){
    // Check that we are not accidently assigning this object to itself
    GObject::selfAssignmentCheck<GDoubleCollection>(cp);

	// Load our parent class'es data ...
	GFPNumCollectionT<double>::load_(cp);

	// ... no local data
}

#ifdef GENEVATESTING
/*******************************************************************************************/
/**
 * Applies modifications to this object. This is needed for testing purposes
 *
 * @return A boolean which indicates whether modifications were made
 */
bool GDoubleCollection::modify_GUnitTests() {
	bool result = false;

	// Call the parent class'es function
	if(GFPNumCollectionT<double>::modify_GUnitTests()) result = true;

	return result;
}

/*******************************************************************************************/
/**
 * Performs self tests that are expected to succeed. This is needed for testing purposes
 */
void GDoubleCollection::specificTestsNoFailureExpected_GUnitTests() {
	// A few settings
	const std::size_t nItems = 10000;
	const std::size_t nTests = 10;
	const double FIXEDVALUEINIT = 1.;

	// Make sure we have an appropriate adaptor loaded when performing these tests
	bool adaptorStored = false;
	boost::shared_ptr<GAdaptorT<double> > storedAdaptor;

	if(this->hasAdaptor()) {
		storedAdaptor = this->getAdaptor();
		adaptorStored = true;
	}

	boost::shared_ptr<GDoubleGaussAdaptor> gdga_ptr(new GDoubleGaussAdaptor(0.5, 0.8, 0., 2., 1.0));
	gdga_ptr->setAdaptionThreshold(0); // Make sure the adaptor's internal parameters don't change through the adaption
	gdga_ptr->setAdaptionMode(true); // Always adapt
	this->addAdaptor(gdga_ptr);

	// Call the parent class'es function
	GFPNumCollectionT<double>::specificTestsNoFailureExpected_GUnitTests();

	//------------------------------------------------------------------------------

	{ // Test the GParameterT<T>::adaptImpl() implementation
		boost::shared_ptr<GDoubleCollection> p_test1 = this->clone<GDoubleCollection>();
		boost::shared_ptr<GDoubleCollection> p_test2 = this->clone<GDoubleCollection>();

		if(p_test1->hasAdaptor()) {
			// Make sure the collection is clean
			p_test1->clear();

			// Add a few items
			for(std::size_t i=0; i<nItems; i++) {
				p_test1->push_back(FIXEDVALUEINIT);
			}

			for(std::size_t t=0; t<nTests; t++) {
				// Load p_test1 into p_test2
				BOOST_CHECK_NO_THROW(p_test2->load(p_test1));

				// Make sure the objects match
				BOOST_CHECK(*p_test1 == *p_test2);

				// Adapt p_test1, using the internal function
				BOOST_CHECK_NO_THROW(p_test1->adaptImpl());

				// Test whether the two objects differ now
				BOOST_CHECK(*p_test1 != *p_test2);

				// Check that each element differs
				for(std::size_t i=0; i<nItems; i++) {
					BOOST_CHECK(p_test1->at(i) != p_test2->at(i));
				}
			}
		}
	}

	//------------------------------------------------------------------------------

	{ // Test of GParameterT<T>::swap(const GParameterT<T>&)
		boost::shared_ptr<GDoubleCollection> p_test1 = this->clone<GDoubleCollection>();
		boost::shared_ptr<GDoubleCollection> p_test2 = this->clone<GDoubleCollection>();
		boost::shared_ptr<GDoubleCollection> p_test3 = this->clone<GDoubleCollection>();

		if(p_test1->hasAdaptor()) {
			// Make sure the collection is clean
			p_test1->clear();

			// Add a few items
			for(std::size_t i=0; i<nItems; i++) {
				p_test1->push_back(FIXEDVALUEINIT);
			}

			// Load p_test1 into p_test2 and p_test3
			BOOST_CHECK_NO_THROW(p_test2->load(p_test1));
			BOOST_CHECK_NO_THROW(p_test3->load(p_test1));

			// Make sure the objects match
			BOOST_CHECK(*p_test1 == *p_test2);
			BOOST_CHECK(*p_test1 == *p_test3);
			BOOST_CHECK(*p_test3 == *p_test2);

			// Adapt p_test1, using the internal function
			BOOST_CHECK_NO_THROW(p_test1->adaptImpl());

			// Test whether p_test1 and p_test2/3 differ now
			BOOST_CHECK(*p_test1 != *p_test2);
			BOOST_CHECK(*p_test1 != *p_test3);
			// Test whether p_test2 is still the same as p_test3
			BOOST_CHECK(*p_test3 == *p_test2);

			// Swap the data of p_test2 and p_test1
			BOOST_CHECK_NO_THROW(p_test2->swap(*p_test1));

			// Now p_test1 and p_test3 should be the same, while p_test2 differs from both
			BOOST_CHECK(*p_test1 == *p_test3);
			BOOST_CHECK(*p_test2 != *p_test1);
			BOOST_CHECK(*p_test2 != *p_test3);
		}
	}

	//------------------------------------------------------------------------------

	// Remove the test adaptor
	this->resetAdaptor();

	// Load the old adaptor, if needed
	if(adaptorStored) {
		this->addAdaptor(storedAdaptor);
	}
}

/*******************************************************************************************/
/**
 * Performs self tests that are expected to fail. This is needed for testing purposes
 */
void GDoubleCollection::specificTestsFailuresExpected_GUnitTests() {
	// Make sure we have an appropriate adaptor loaded when performing these tests
	bool adaptorStored = false;
	boost::shared_ptr<GAdaptorT<double> > storedAdaptor;

	if(this->hasAdaptor()) {
		storedAdaptor = this->getAdaptor();
		adaptorStored = true;
	}

	boost::shared_ptr<GDoubleGaussAdaptor> gdga_ptr(new GDoubleGaussAdaptor(0.5, 0.8, 0., 2., 1.0));
	gdga_ptr->setAdaptionThreshold(0); // Make sure the adaptor's internal parameters don't change through the adaption
	gdga_ptr->setAdaptionMode(true); // Always adapt
	this->addAdaptor(gdga_ptr);

	// Call the parent class'es function
	GFPNumCollectionT<double>::specificTestsFailuresExpected_GUnitTests();

	// Nothing to check -- no local data

	// Remove the test adaptor
	this->resetAdaptor();

	// Load the old adaptor, if needed
	if(adaptorStored) {
		this->addAdaptor(storedAdaptor);
	}
}

/*******************************************************************************************/

#endif /* GENEVATESTING */

} /* namespace Geneva */
} /* namespace Gem */
