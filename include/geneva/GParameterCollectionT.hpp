/**
 * @file GParameterCollectionT.hpp
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


// Standard header files go here

// Boost header files go here

#ifndef GPARAMETERCOLLECTIONT_HPP_
#define GPARAMETERCOLLECTIONT_HPP_

// For Microsoft-compatible compilers
#if defined(_MSC_VER)  &&  (_MSC_VER >= 1020)
#pragma once
#endif

// Geneva header files go here
#include "geneva/GObject.hpp"
#include "geneva/GParameterBaseWithAdaptorsT.hpp"
#include "geneva/GStdSimpleVectorInterfaceT.hpp"

namespace Gem {
namespace Geneva {

/***********************************************************************************************/
/**
 * A class holding a collection of mutable parameters - usually just an atomic value (double,
 * long, bool, ...).
 */
template<typename T>
class GParameterCollectionT
	:public GParameterBaseWithAdaptorsT<T>,
	 public GStdSimpleVectorInterfaceT<T>
{
	///////////////////////////////////////////////////////////////////////
	friend class boost::serialization::access;

	template<typename Archive>
	void serialize(Archive & ar, const unsigned int) {
		using boost::serialization::make_nvp;
		ar & make_nvp("GParameterBaseWithAdaptorsT_T", boost::serialization::base_object<GParameterBaseWithAdaptorsT<T> >(*this))
		   & make_nvp("GStdSimpleVectorInterfaceT_T", boost::serialization::base_object<GStdSimpleVectorInterfaceT<T> >(*this));
	}
	///////////////////////////////////////////////////////////////////////

public:
	/*******************************************************************************************/
	/**
	 * The default constructor
	 */
	GParameterCollectionT()
		: GParameterBaseWithAdaptorsT<T> ()
		, GStdSimpleVectorInterfaceT<T>()
	{ /* nothing */ }

	/*******************************************************************************************/
	/**
	 * The copy constructor
	 *
	 * @param cp A copy of another GParameterCollectionT<T> object
	 */
	GParameterCollectionT(const GParameterCollectionT<T>& cp)
		: GParameterBaseWithAdaptorsT<T> (cp)
		, GStdSimpleVectorInterfaceT<T>(cp)
	{  /* nothing */ }

	/*******************************************************************************************/
	/**
	 * The standard destructor
	 */
	virtual ~GParameterCollectionT()
	{ /* nothing */ }

	/*******************************************************************************************/
	/**
	 * A standard assignment operator.
	 *
	 * @param cp A copy of another GParameterCollectionT object
	 * @return A constant reference to this object
	 */
	const GParameterCollectionT<T>& operator=(const GParameterCollectionT<T>& cp)
	{
		GParameterCollectionT<T>::load_(cp);
		return *this;
	}

	/*******************************************************************************************/
	/**
	 * Checks for equality with another GParameterCollectionT<T> object
	 *
	 * @param  cp A constant reference to another GParameterCollectionT<T> object
	 * @return A boolean indicating whether both objects are equal
	 */
	bool operator==(const GParameterCollectionT<T>& cp) const {
		using namespace Gem::Common;
		// Means: The expectation of equality was fulfilled, if no error text was emitted (which converts to "true")
		return !checkRelationshipWith(cp, CE_EQUALITY, 0.,"GParameterCollectionT<T>::operator==","cp", CE_SILENT);
	}

	/*******************************************************************************************/
	/**
	 * Checks for inequality with another GParameterCollectionT<T> object
	 *
	 * @param  cp A constant reference to another GParameterCollectionT<T> object
	 * @return A boolean indicating whether both objects are inequal
	 */
	bool operator!=(const GParameterCollectionT<T>& cp) const {
		using namespace Gem::Common;
		// Means: The expectation of inequality was fulfilled, if no error text was emitted (which converts to "true")
		return !checkRelationshipWith(cp, CE_INEQUALITY, 0.,"GParameterCollectionT<T>::operator==","cp", CE_SILENT);
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
	boost::optional<std::string> checkRelationshipWith(const GObject& cp,
			const Gem::Common::expectation& e,
			const double& limit,
			const std::string& caller,
			const std::string& y_name,
			const bool& withMessages) const
	{
	    using namespace Gem::Common;

		// Check that we are indeed dealing with a GParamterBase reference
		const GParameterCollectionT<T>  *p_load = GObject::conversion_cast<GParameterCollectionT<T> >(&cp);

		// Will hold possible deviations from the expectation, including explanations
	    std::vector<boost::optional<std::string> > deviations;

		// Check our parent classes' data ...
		deviations.push_back(GParameterBaseWithAdaptorsT<T>::checkRelationshipWith(cp, e, limit, "GParameterCollectionT<T>", y_name, withMessages));
		deviations.push_back(GStdSimpleVectorInterfaceT<T>::checkRelationshipWith(*p_load, e, limit, "GParameterCollectionT<T>", y_name, withMessages));

		// no local data ...

		return evaluateDiscrepancies("GParameterCollectionT<T>", caller, deviations, e);
	}

	/*******************************************************************************************/
	/**
	 * Checks whether a given expectation for the relationship between this object and a vector
	 * of base-type items is fulfilled. Only the content of the vector underlying this class
	 * and cp is checked.
	 *
	 * @param cp A constant reference to a std::vector<value_type>
	 * @param e The expected outcome of the comparison
	 * @param limit The maximum deviation for floating point values (important for similarity checks)
	 * @param caller An identifier for the calling entity
	 * @param y_name An identifier for the object that should be compared to this one
	 * @param withMessages Whether or not information should be emitted in case of deviations from the expected outcome
	 * @return A boost::optional<std::string> object that holds a descriptive string if expectations were not met
	 */
	boost::optional<std::string> checkRelationshipWith(const std::vector<T>& cp,
			const Gem::Common::expectation& e,
			const double& limit,
			const std::string& caller,
			const std::string& y_name,
			const bool& withMessages) const
	{
	    using namespace Gem::Common;

		// Will hold possible deviations from the expectation, including explanations
	    std::vector<boost::optional<std::string> > deviations;

	    deviations.push_back(GStdSimpleVectorInterfaceT<T>::checkRelationshipWith(cp, e, limit, caller, y_name, withMessages));

		return evaluateDiscrepancies("GParameterCollectionT<T>", caller, deviations, e);
	}

	/*******************************************************************************************/
	/**
	 * Allows to adapt the values stored in this class. applyAdaptor expects a reference
	 * to a std::vector<T>. As we are derived from a wrapper of this class, we can just pass
	 * a reference to its data vector to the function.
	 */
	virtual void adaptImpl() {
		GParameterBaseWithAdaptorsT<T>::applyAdaptor(GStdSimpleVectorInterfaceT<T>::data);
	}

	/* ----------------------------------------------------------------------------------
	 * Tested in GDoubleObject::specificTestsNoFailureExpected_GUnitTests()
	 * ----------------------------------------------------------------------------------
	 */

	/*******************************************************************************************/
	/**
	 * Swap another object's vector with ours
	 */
	inline void swap(GParameterCollectionT<T>& cp) {
		GStdSimpleVectorInterfaceT<T>::swap(cp.data);
	}

	/* ----------------------------------------------------------------------------------
	 * Tested in GDoubleObject::specificTestsNoFailureExpected_GUnitTests()
	 * ----------------------------------------------------------------------------------
	 */

	/*******************************************************************************************/
	/**
	 * Retrieval of the value at a given position
	 *
	 * @param pos The position for which the value needs to be returned
	 * @return The value of val_
	 */
	virtual T value(const std::size_t& pos) {
		return this->at(pos);
	}

	/*******************************************************************************************/
	/**
	 * Allows to set the internal (and usually externally visible) value at a given position. Note
	 * that we assume here that T has an operator=() or is a basic value type, such as double
	 * or int.
	 *
	 * @param pos The position at which the value shout be stored
	 * @param val The new T value stored in this class
	 */
	virtual void setValue(const std::size_t& pos, const T& val)  {
		this->at(pos) = val;
	}

protected:
	/*******************************************************************************************/
	/**
	 * Loads the data of another GParameterCollectionT<T> object, camouflaged as a GObject
	 *
	 * @param cp A copy of another GParameterCollectionT<T> object, camouflaged as a GObject
	 */
	virtual void load_(const GObject* cp) {
		// Convert cp into local format and check for self-assignment
		const GParameterCollectionT<T> *p_load = GObject::conversion_cast<GParameterCollectionT<T> >(cp);

		// Load our parent class'es data ...
		GParameterBaseWithAdaptorsT<T>::load_(cp);
		GStdSimpleVectorInterfaceT<T>::operator=(*p_load);
	}

	/*******************************************************************************************/
	/**
	 * Creates a deep clone of this object. Purely virtual, so this class cannot be instantiated.
	 */
	virtual GObject* clone_() const = 0;

	/*******************************************************************************************/
	/**
	 * Re-implementation of a corresponding function in GStdSimpleVectorInterface.
	 * Making the vector wrapper purely virtual allows the compiler to perform
	 * further optimizations.
	 */
	virtual void dummyFunction() { /* nothing */ }

#ifdef GENEVATESTING
public:
	/*******************************************************************************************/
	/**
	 * Applies modifications to this object. This is needed for testing purposes
	 *
	 * @return A boolean which indicates whether modifications were made
	 */
	virtual bool modify_GUnitTests() {
		bool result = false;

		// Call the parent classes' functions
		if(GParameterBaseWithAdaptorsT<T>::modify_GUnitTests()) result = true;
		if(GStdSimpleVectorInterfaceT<T>::modify_GUnitTests()) result = true;

		return result;
	}

	/*******************************************************************************************/
	/**
	 * Performs self tests that are expected to succeed. This is needed for testing purposes
	 */
	virtual void specificTestsNoFailureExpected_GUnitTests() {
		// Call the parent classes' functions
		GParameterBaseWithAdaptorsT<T>::specificTestsNoFailureExpected_GUnitTests();
		GStdSimpleVectorInterfaceT<T>::specificTestsNoFailureExpected_GUnitTests();
	}

	/*******************************************************************************************/
	/**
	 * Performs self tests that are expected to fail. This is needed for testing purposes
	 */
	virtual void specificTestsFailuresExpected_GUnitTests() {
		// Call the parent classes' functions
		GParameterBaseWithAdaptorsT<T>::specificTestsFailuresExpected_GUnitTests();
		GStdSimpleVectorInterfaceT<T>::specificTestsFailuresExpected_GUnitTests();
	}
#endif /* GENEVATESTING */
};

/*********************************************************************************************/

} /* namespace Geneva */
} /* namespace Gem */

/**************************************************************************************************/
/**
 * @brief The content of the BOOST_SERIALIZATION_ASSUME_ABSTRACT(T) macro. Needed for Boost.Serialization
 */
namespace boost {
  namespace serialization {
    template<typename T>
    struct is_abstract<Gem::Geneva::GParameterCollectionT<T> > : public boost::true_type {};
    template<typename T>
    struct is_abstract< const Gem::Geneva::GParameterCollectionT<T> > : public boost::true_type {};
  }
}

/**************************************************************************************************/

#endif /* GPARAMETERCOLLECTIONT_HPP_ */
