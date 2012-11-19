/**
 * @file GNumT.hpp
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


// Standard headers go here

// Boost headers go here

#ifndef GNUMT_HPP_
#define GNUMT_HPP_

// For Microsoft-compatible compilers
#if defined(_MSC_VER)  &&  (_MSC_VER >= 1020)
#pragma once
#endif


// Geneva headers go here
#include "geneva/GParameterT.hpp"

namespace Gem {
namespace Geneva {

const double DEFAULTLOWERINITBOUNDARYSINGLE=0.;
const double DEFAULTUPPERINITBOUNDARYSINGLE=1.;

/******************************************************************************/
/**
 * This class represents numeric values. The most likely types to be stored
 * in this class are double and boost::int32_t . By using the framework provided
 * by GParameterT, this class becomes rather simple.
 */
template <typename T>
class GNumT
	: public GParameterT<T>
{
	///////////////////////////////////////////////////////////////////////
	friend class boost::serialization::access;

	template<typename Archive>
	void serialize(Archive & ar, const unsigned int) {
		using boost::serialization::make_nvp;
		ar & make_nvp("GParameterT",	boost::serialization::base_object<GParameterT<T> >(*this))
		   & BOOST_SERIALIZATION_NVP(lowerInitBoundary_)
		   & BOOST_SERIALIZATION_NVP(upperInitBoundary_);
	}
	///////////////////////////////////////////////////////////////////////

public:
	/** @brief Specifies the type of parameters stored in this collection */
	typedef T collection_type;

	/***************************************************************************/
	/**
	 * The default constructor.
	 */
	GNumT()
		: GParameterT<T> ()
		, lowerInitBoundary_(T(DEFAULTLOWERINITBOUNDARYSINGLE))
		, upperInitBoundary_(T(DEFAULTUPPERINITBOUNDARYSINGLE))
	{ /* nothing */ }

	/*****************************************************************/
	/*
	 * Initialize with a single value
	 *
	 * @param val The value used for the initialization
	 */
	explicit GNumT(const T& val)
		: GParameterT<T>(val)
		, lowerInitBoundary_(T(DEFAULTLOWERINITBOUNDARYSINGLE))
		, upperInitBoundary_(T(DEFAULTUPPERINITBOUNDARYSINGLE))
	{ /* nothing */ }

	/***************************************************************************/
	/**
	 * Initialize the boundaries. The internal value will be
	 * initialized with the lower boundary.
	 *
	 * @param min The lower boundary for random entries
	 * @param max The upper boundary for random entries
	 */
	GNumT(const T& min, const T& max)
		: GParameterT<T> (min)
		, lowerInitBoundary_(min)
		, upperInitBoundary_(max)
	{ /* nothing */ }

	/***************************************************************************/
	/**
	 * The standard copy constructor
	 */
	GNumT(const GNumT<T>& cp)
		: GParameterT<T> (cp)
		, lowerInitBoundary_(cp.lowerInitBoundary_)
		, upperInitBoundary_(cp.upperInitBoundary_)
	{ /* nothing */ }

	/***************************************************************************/
	/**
	 * The standard destructor
	 */
	virtual ~GNumT()
	{ /* nothing */ }

	/***************************************************************************/
	/**
	 * The standard assignment operator.
	 *
	 * @param cp A copy of another GNumT<T> object
	 * @return A constant reference to this object
	 */
	const GNumT<T>& operator=(const GNumT<T>& cp){
		GNumT<T>::load_(&cp);
		return *this;
	}

	/***************************************************************************/
	/**
	 * An assignment operator for the contained value type
	 */
	virtual T operator=(const T& val) {
		return GParameterT<T>::operator=(val);
	}

	/***************************************************************************/
	/**
	 * Checks for equality with another GNumT<T> object
	 *
	 * @param  cp A constant reference to another GNumT<T> object
	 * @return A boolean indicating whether both objects are equal
	 */
	bool operator==(const GNumT<T>& cp) const {
		using namespace Gem::Common;
		// Means: The expectation of equality was fulfilled, if no error text was emitted (which converts to "true")
		return !checkRelationshipWith(cp, CE_EQUALITY, 0.,"GNumT<T>::operator==","cp", CE_SILENT);
	}

	/***************************************************************************/
	/**
	 * Checks for inequality with another GNumT<T> object
	 *
	 * @param  cp A constant reference to another GNumT<T> object
	 * @return A boolean indicating whether both objects are inequal
	 */
	bool operator!=(const GNumT<T>& cp) const {
		using namespace Gem::Common;
		// Means: The expectation of inequality was fulfilled, if no error text was emitted (which converts to "true")
		return !checkRelationshipWith(cp, CE_INEQUALITY, 0.,"GNumT<T>::operator!=","cp", CE_SILENT);
	}

	/***************************************************************************/
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
	boost::optional<std::string> checkRelationshipWith(const GObject& cp,
			const Gem::Common::expectation& e,
			const double& limit,
			const std::string& caller,
			const std::string& y_name,
			const bool& withMessages) const
	{
	    using namespace Gem::Common;

		// Check that we are indeed dealing with a GParamterBase reference
		const GNumT<T>  *p_load = GObject::gobject_conversion<GNumT<T> >(&cp);

		// Will hold possible deviations from the expectation, including explanations
	    std::vector<boost::optional<std::string> > deviations;

		// Check our parent class'es data ...
		deviations.push_back(GParameterT<T>::checkRelationshipWith(cp, e, limit, "GNumT<T>", y_name, withMessages));

		// ... and then our local data
		deviations.push_back(checkExpectation(withMessages, "GNumT<T>", lowerInitBoundary_, p_load->lowerInitBoundary_, "lowerInitBoundary_", "p_load->lowerInitBoundary_", e , limit));
		deviations.push_back(checkExpectation(withMessages, "GNumT<T>", upperInitBoundary_, p_load->upperInitBoundary_, "upperInitBoundary_", "p_load->upperInitBoundary_", e , limit));

		return evaluateDiscrepancies("GNumT<T>", caller, deviations, e);
	}

	/***************************************************************************/
	/**
	 * Sets the initialization boundaries
	 *
	 * @param lowerInitBoundary The lower boundary for random initialization
	 * @param upperInitBoundary The upper boundary for random initialization
	 */
	void setInitBoundaries(const T& lowerInitBoundary, const T& upperInitBoundary) {
		// Do some error checking
		if(lowerInitBoundary >= upperInitBoundary) {
			raiseException(
					"In GNumT<T>::setInitBoundaries():" << std::endl
					<< "Invalid boundaries provided: " << std::endl
					<< "lowerInitBoundary = " << lowerInitBoundary << std::endl
					<< "upperInitBoundary = " << upperInitBoundary
			);
		}

		lowerInitBoundary_ = lowerInitBoundary;
		upperInitBoundary_ = upperInitBoundary;
	}

	/* ----------------------------------------------------------------------------------
	 * Tested GNumT<T>::specificTestsNoFailureExpected_GUnitTests()
	 * Setting of invalid boundaries is tested in GNumT<T>::specificTestsFailuresExpected_GUnitTests()
	 * ----------------------------------------------------------------------------------
	 */

	/***************************************************************************/
	/**
	 * Retrieves the value of the lower initialization boundary
	 *
	 * @return The value of the lower initialization boundary
	 */
	T getLowerInitBoundary() const {
		return lowerInitBoundary_;
	}

	/* ----------------------------------------------------------------------------------
	 * Tested GNumT<T>::specificTestsNoFailureExpected_GUnitTests()
	 * ----------------------------------------------------------------------------------
	 */

	/***************************************************************************/
	/**
	 * Retrieves the value of the upper initialization boundary
	 *
	 * @return The value of the upper initialization boundary
	 */
	T getUpperInitBoundary() const {
		return upperInitBoundary_;
	}

	/* ----------------------------------------------------------------------------------
	 * Tested GNumT<T>::specificTestsNoFailureExpected_GUnitTests()
	 * ----------------------------------------------------------------------------------
	 */

protected:
	/***************************************************************************/
	/**
	 * Loads the data of another GNumT<T> object,
	 * camouflaged as a GObject. We have no local data, so
	 * all we need to do is to the standard identity check,
	 * preventing that an object is assigned to itself.
	 *
	 * @param cp A copy of another GNumT<T> object, camouflaged as a GObject
	 */
	virtual void load_(const GObject *cp){
		// Convert cp into local format
		const GNumT<T> *p_load = GObject::gobject_conversion<GNumT<T> >(cp);

		// Load our parent class'es data ...
		GParameterT<T>::load_(cp);

		// ... and then our local data
		lowerInitBoundary_ = p_load->lowerInitBoundary_;
		upperInitBoundary_ = p_load->upperInitBoundary_;
	}

	/***************************************************************************/
	/**
	 * Creates a deep copy of this object. Purely virtual as this class
	 * should not be instantiable.
	 *
	 * @return A pointer to a deep clone of this object
	 */
	virtual GObject *clone_() const = 0;

	/***************************************************************************/
	/** @brief Triggers random initialization of the parameter */
	virtual void randomInit_() = 0;

private:
	/***************************************************************************/
	T lowerInitBoundary_; ///< The lower boundary for random initialization
	T upperInitBoundary_; ///< The upper boundary for random initialization

public:

	/***************************************************************************/
	/**
	 * Applies modifications to this object. This is needed for testing purposes
	 *
	 * @return A boolean which indicates whether modifications were made
	 */
	virtual bool modify_GUnitTests() {
#ifdef GEM_TESTING
		bool result = false;

		// Call the parent classes' functions
		if(GParameterT<T>::modify_GUnitTests()) result = true;

		return result;

#else /* GEM_TESTING */  // If this function is called when GEM_TESTING isn't set, throw
		condnotset("GNumT<>::modify_GUnitTests", "GEM_TESTING");
		return false;
#endif /* GEM_TESTING */
	}

	/***************************************************************************/
	/**
	 * Performs self tests that are expected to succeed. This is needed for testing purposes
	 */
	virtual void specificTestsNoFailureExpected_GUnitTests() {
#ifdef GEM_TESTING
		// Call the parent classes' functions
		GParameterT<T>::specificTestsNoFailureExpected_GUnitTests();

		// A few settings
		const T LOWERTESTINITVAL = T(1); // Do not choose a negative value as T might be an unsigned type
		const T UPPERTESTINITVAL = T(3);

		//------------------------------------------------------------------------------

		{ // Test setting and retrieval of initialization boundaries
			boost::shared_ptr<GNumT<T> > p_test = this->GObject::template clone<GNumT<T> >();

			// Set the boundaries
			BOOST_CHECK_NO_THROW(p_test->setInitBoundaries(LOWERTESTINITVAL, UPPERTESTINITVAL));

			// Check that these values have indeed been assigned
			BOOST_CHECK(p_test->getLowerInitBoundary() == LOWERTESTINITVAL);
			BOOST_CHECK(p_test->getUpperInitBoundary() == UPPERTESTINITVAL);
		}

		//------------------------------------------------------------------------------

#else /* GEM_TESTING */  // If this function is called when GEM_TESTING isn't set, throw
      condnotset("GNumT<>::specificTestsNoFailureExpected_GUnitTests", "GEM_TESTING");
#endif /* GEM_TESTING */
	}

	/***************************************************************************/
	/**
	 * Performs self tests that are expected to fail. This is needed for testing purposes
	 */
	virtual void specificTestsFailuresExpected_GUnitTests() {
#ifdef GEM_TESTING
		// Call the parent classes' functions
		GParameterT<T>::specificTestsFailuresExpected_GUnitTests();

		// A few settings
		const T LOWERTESTINITVAL = T(1); // Do not choose a negative value as T might be an unsigned type
		const T UPPERTESTINITVAL = T(3);

		//------------------------------------------------------------------------------

		{ // Check that assignement of initialization boundaries throws for invalid boundaries
			boost::shared_ptr<GNumT<T> > p_test = this->GObject::template clone<GNumT<T> >();

			BOOST_CHECK_THROW(p_test->setInitBoundaries(UPPERTESTINITVAL, LOWERTESTINITVAL), Gem::Common::gemfony_error_condition);
		}

		//------------------------------------------------------------------------------


#else /* GEM_TESTING */  // If this function is called when GEM_TESTING isn't set, throw
      condnotset("GNumT<>::specificTestsFailuresExpected_GUnitTests", "GEM_TESTING");
#endif /* GEM_TESTING */
	}

};

} /* namespace Geneva */
} /* namespace Gem */

/******************************************************************************/
// The content of BOOST_SERIALIZATION_ASSUME_ABSTRACT(T)

namespace boost {
	namespace serialization {
		template<typename T>
		struct is_abstract<Gem::Geneva::GNumT<T> > : public boost::true_type {};
		template<typename T>
		struct is_abstract< const Gem::Geneva::GNumT<T> > : public boost::true_type {};
	}
}

/******************************************************************************/

#endif /* GNUMT_HPP_ */
