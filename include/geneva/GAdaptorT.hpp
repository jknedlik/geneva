/**
 * @file GAdaptorT.hpp
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


// Standard headers go here
#include <iostream>
#include <string>
#include <sstream>
#include <vector>

// Includes check for correct Boost version(s)
#include "common/GGlobalDefines.hpp"

// Boost headers go here

#include <boost/cstdint.hpp>
#include <boost/logic/tribool.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/tracking.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/export.hpp>

#ifndef GADAPTORT_HPP_
#define GADAPTORT_HPP_

// For Microsoft-compatible compilers
#if defined(_MSC_VER)  &&  (_MSC_VER >= 1020)
#pragma once
#endif

// Geneva headers go here

#include "common/GTriboolSerialization.hpp"
#include "hap/GRandomT.hpp"
#include "hap/GRandomBaseT.hpp"
#include "GObject.hpp"
#include "GObjectExpectationChecksT.hpp"
#include "GOptimizationEnums.hpp"

namespace Gem {
namespace Geneva {

/******************************************************************************************/
/**
 * In Geneva, two mechanisms exist that let the user specify the
 * type of adaption he wants to have executed on collections of
 * items (basic types or any other types).  The most basic
 * possibility is for the user to overload the GIndividual::customAdaptions()
 * function and manually specify the types of adaptions (s)he
 * wants. This allows great flexibility, but is not very practicable
 * for standard adaptions.
 *
 * Classes derived from GParameterBaseWithAdaptorsT<T> can additionally store
 * "adaptors". These are templatized function objects that can act
 * on the items of a collection of user-defined types. Predefined
 * adaptors exist for standard types (with the most prominent
 * examples being bits and double values).
 *
 * The GAdaptorT class mostly acts as an interface for these
 * adaptors, but also implements some functionality of its own. E.g., it is possible
 * to specify a function that shall be called every adaptionThreshold_ calls of the
 * adapt() function. It is also possible to set a adaption probability, only a certain
 * percentage of adaptions is actually performed at run-time.
 *
 * In order to use this class, the user must derive a class from
 * GAdaptorT<T> and specify the type of adaption he wishes to
 * have applied to items, by overloading of
 * GAdaptorT<T>::customAdaptions(T&) .  T will often be
 * represented by a basic value (double, long, bool, ...). Where
 * this is not the case, the adaptor will only be able to access
 * public functions of T, unless T declares the adaptor as a friend.
 *
 * As a derivative of GObject, this class follows similar rules as
 * the other Geneva classes.
 */
template <typename T>
class GAdaptorT:
	public GObject
{
	///////////////////////////////////////////////////////////////////////
	friend class boost::serialization::access;

	template<typename Archive>
	void serialize(Archive & ar, const unsigned int) {
		using boost::serialization::make_nvp;
		ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(GObject)
		   & BOOST_SERIALIZATION_NVP(adaptionCounter_)
		   & BOOST_SERIALIZATION_NVP(adaptionThreshold_)
		   & BOOST_SERIALIZATION_NVP(adProb_)
		   & BOOST_SERIALIZATION_NVP(adaptionMode_);
	}
	///////////////////////////////////////////////////////////////////////

public:
	/***********************************************************************************/
	/**
	 * Allows external callers to find out about the type stored in this object
	 */
	typedef T adaption_type;

	/***********************************************************************************/
	/**
	 * The default constructor.
	 */
	GAdaptorT()
		: GObject()
		, gr_local(new Gem::Hap::GRandomT<Gem::Hap::RANDOMLOCAL, double, boost::int32_t>())
		, gr(gr_local)
		, adaptionCounter_(0)
		, adaptionThreshold_(0)
		, adProb_(DEFAULTADPROB)
		, adaptionMode_(boost::logic::indeterminate)
	{ /* nothing */ }

	/***********************************************************************************/
	/**
	 * This constructor allows to set the probability with which a adaption is indeed
	 * performed.
	 */
	GAdaptorT(const double& prob)
		: GObject()
		, gr_local(new Gem::Hap::GRandomT<Gem::Hap::RANDOMLOCAL, double, boost::int32_t>())
		, gr(gr_local)
		, adaptionCounter_(0)
		, adaptionThreshold_(0)
		, adProb_(prob)
		, adaptionMode_(boost::logic::indeterminate)
	{
		// Do some error checking
		if(prob < 0. || prob > 1.) {
			std::cerr << "In GAdaptorT<T>::GadaptorT(const double& prob): Error!" << std::endl
					  << "Provided daption probability is invalid: " << prob << std::endl;
			std::terminate();
		}
	}

	/***********************************************************************************/
	/**
	 * A standard copy constructor.
	 *
	 * @param cp A copy of another GAdaptorT<T>
	 */
	GAdaptorT(const GAdaptorT<T>& cp)
		: GObject(cp)
		, gr_local(new Gem::Hap::GRandomT<Gem::Hap::RANDOMLOCAL, double, boost::int32_t>()) // We do *not* copy the other object's generator
		, gr(gr_local)
		, adaptionCounter_(cp.adaptionCounter_)
		, adaptionThreshold_(cp.adaptionThreshold_)
		, adProb_(cp.adProb_)
		, adaptionMode_(cp.adaptionMode_)
	{ /* nothing */ }

	/***********************************************************************************/
	/**
	 * The standard destructor. Gets rid of the local random number generator, unless
	 * an external generator has been assigned.
	 */
	virtual ~GAdaptorT() {
		if(gr_local) delete gr_local;
	}

	/***********************************************************************************/
	/**
	 * A standard assignment operator for GAdaptorT<T> objects,
	 *
	 * @param cp A copy of another GAdaptorT<T> object
	 */
	const GAdaptorT<T>& operator=(const GAdaptorT<T>& cp) {
		GAdaptorT<T>::load_(&cp);
		return *this;
	}

	/***********************************************************************************/
	/**
	 * Checks for equality with another GAdaptorT<T> object
	 *
	 * @param  cp A constant reference to another GAdaptorT<T> object
	 * @return A boolean indicating whether both objects are equal
	 */
	bool operator==(const GAdaptorT<T>& cp) const {
		using namespace Gem::Common;
		// Means: The expectation of equality was fulfilled, if no error text was emitted (which converts to "true")
		return !checkRelationshipWith(cp, CE_EQUALITY, 0.,"GAdaptorT<T>::operator==","cp", CE_SILENT);
	}

	/***********************************************************************************/
	/**
	 * Checks for inequality with another GAdaptorT<T> object
	 *
	 * @param  cp A constant reference to another GAdaptorT<T> object
	 * @return A boolean indicating whether both objects are inequal
	 */
	bool operator!=(const GAdaptorT<T>& cp) const {
		using namespace Gem::Common;
		// Means: The expectation of inequality was fulfilled, if no error text was emitted (which converts to "true")
		return !checkRelationshipWith(cp, CE_INEQUALITY, 0.,"GAdaptorT<T>::operator!=","cp", CE_SILENT);
	}

	/***********************************************************************************/
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
		const GAdaptorT<T>  *p_load = conversion_cast<GAdaptorT<T> >(&cp);

		// Will hold possible deviations from the expectation, including explanations
	    std::vector<boost::optional<std::string> > deviations;

		// Check our parent class'es data ...
		deviations.push_back(GObject::checkRelationshipWith(cp, e, limit, "GAdaptorT<T>", y_name, withMessages));

		// ... and then our local data
		deviations.push_back(checkExpectation(withMessages, "GAdaptorT<T>", adaptionCounter_, p_load->adaptionCounter_, "adaptionCounter_", "p_load->adaptionCounter_", e , limit));
		deviations.push_back(checkExpectation(withMessages, "GAdaptorT<T>", adaptionThreshold_, p_load->adaptionThreshold_, "adaptionThreshold_", "p_load->adaptionThreshold_", e , limit));
		deviations.push_back(checkExpectation(withMessages, "GAdaptorT<T>", adProb_, p_load->adProb_, "adProb_", "p_load->adProb_", e , limit));
		deviations.push_back(checkExpectation(withMessages, "GAdaptorT<T>", adaptionMode_, p_load->adaptionMode_, "adaptionMode_", "p_load->adaptionMode_", e , limit));

		return evaluateDiscrepancies("GAdaptorT<T>", caller, deviations, e);
	}

	/***********************************************************************************/
	/**
	 * Retrieves the id of the adaptor. Purely virtual, must be implemented by the
	 * actual adaptors.
	 *
	 * @return The id of the adaptor
	 */
	virtual Gem::Geneva::adaptorId getAdaptorId() const = 0;

	/***********************************************************************************/
	/**
	 * Sets the adaption probability to a given value. This function will throw
	 * if the probability is not in the allowed range.
	 *
	 * @test {
	 * 	 <ul>
	 *     <li>Setting of valid probabilities is tested in GAdaptorT<T>::specificTestsNoFailuresExpected_GUnitTests()</li>
	 *     <li>Checks for setting of invalid probabilities is tested in GAdaptorT<T>::specificTestsFailuresExpected_GUnitTests()</li>
	 *     <li>The effects on the probability of adaptions actually taking place are tested in GBooleanAdaptor</li>
	 *   </ul>
	 * }
	 *
	 * @param val The new value of the probability for integer flips
	 */
	void setAdaptionProbability(const double& probability) {
		// Check the supplied probability value
		if(probability < 0. || probability > 1.) {
			std::ostringstream error;
			error << "In GAdaptorT<T>::setAdaptionProbability(const double&) : Error!" << std::endl
				  << "Bad probability value given: " << probability << std::endl;

			// throw an exception. Add some information so that if the exception
			// is caught through a base object, no information is lost.
			throw Gem::Common::gemfony_error_condition(error.str());
		}

		adProb_ = probability;
	}

	/***********************************************************************************/
	/**
	 * Retrieves the current value of the adaption probability
	 *
	 * @test {
	 * 	 <ul>
	 *     <li>retrieval of probabilities is tested in GAdaptorT<T>::specificTestsNoFailuresExpected_GUnitTests()</li>
	 *   </ul>
	 * }
	 *
	 * @return The current value of the adaption probability
	 */
	double getAdaptionProbability() const {
		return adProb_;
	}

	/***********************************************************************************/
	/**
	 * Retrieves the current value of the adaptionCounter_ variable.
	 *
	 * @return The value of the adaptionCounter_ variable
	 */
	virtual boost::uint32_t getAdaptionCounter() const  {
		return adaptionCounter_;
	}

	/***********************************************************************************/
	/**
	 * Sets the value of adaptionThreshold_. If set to 0, no adaption of the optimization
	 * parameters will take place
	 *
	 * @param adaptionCounter The value that should be assigned to the adaptionCounter_ variable
	 */
	void setAdaptionThreshold(const boost::uint32_t& adaptionThreshold)  {
		adaptionThreshold_ = adaptionThreshold;
	}

	/***********************************************************************************/
	/**
	 * Retrieves the value of the adaptionThreshold_ variable.
	 *
	 * @return The value of the adaptionThreshold_ variable
	 */
	boost::uint32_t getAdaptionThreshold() const  {
		return adaptionThreshold_;
	}

	/***********************************************************************************/
	/**
	 * Allows to specify whether adaptions should happen always, never, or with a given
	 * probability. This uses the boost::logic::tribool class. The function is declared
	 * virtual so adaptors requiring adaptions to happen always or never can prevent
	 * resetting of the adaptionMode_ variable.
	 *
	 * @param adaptionMode The desired mode (always/never/with a given probability)
	 */
	virtual void setAdaptionMode(boost::logic::tribool adaptionMode) {
		adaptionMode_ = adaptionMode;
	}

	/***********************************************************************************/
	/**
	 * Returns the current value of the adaptionMode_ variable
	 *
	 * @return The current value of the adaptionMode_ variable
	 */
	boost::logic::tribool getAdaptionMode() const {
		return adaptionMode_;
	}

	/***********************************************************************************/
	/**
	 * Common interface for all adaptors to the adaption
	 * functionality. The user specifies this functionality in the
	 * customAdaptions() function.
	 *
	 * @param val The value that needs to be adapted
	 */
	void adapt(T& val)  {
		if(boost::logic::indeterminate(adaptionMode_)) { // The most likely case
			// The adaption parameters are modified every adaptionThreshold_ number
			// of calls to this function.
			if(gr->uniform_01() <= adProb_) {
				if(adaptionThreshold_ && adaptionCounter_++ >= adaptionThreshold_){
					adaptionCounter_ = 0;
					adaptAdaption();
				}

				customAdaptions(val);
			}
		}
		else if(adaptionMode_ == true) { // always adapt
			customAdaptions(val);
		}
		// No need to test for adaptionMode_ == false as no action is needed in this case
	}

	/***********************************************************************************/
	/**
	 * Assign a random number generator from another object.
	 *
	 * @param gr_cp A reference to another object's GRandomBaseT object derivative
	 */
	void assignGRandomPointer(Gem::Hap::GRandomBaseT<double, boost::int32_t> *gr_cp) {
		gr = gr_cp;
		if(gr_local) delete gr_local;
		gr_local = NULL;
	}

protected:
	/***********************************************************************************/
    /**
     * A random number generator. This reference and the associated pointer is either
     * connected to a local random number generator assigned in the constructor, or
     * to a "factory" generator located in the surrounding GParameterSet object.
     */
	Gem::Hap::GRandomBaseT<double, boost::int32_t> *gr_local;
	Gem::Hap::GRandomBaseT<double, boost::int32_t> *gr;

	/***********************************************************************************/
	/**
	 * Loads the contents of another GAdaptorT<T>. The function
	 * is similar to a copy constructor (but with a pointer as
	 * argument). As this function might be called in an environment
	 * where we do not know the exact type of the class, the
	 * GAdaptorT<T> is camouflaged as a GObject . This implies the
	 * need for dynamic conversion.
	 *
	 * @param gb A pointer to another GAdaptorT<T>, camouflaged as a GObject
	 */
	virtual void load_(const GObject *cp) {
		// Convert cp into local format
		const GAdaptorT<T> *p_load = conversion_cast<GAdaptorT<T> >(cp);

		// Load the parent class'es data
		GObject::load_(cp);

		// Then our own data
		adaptionCounter_ = p_load->adaptionCounter_;
		adaptionThreshold_ = p_load->adaptionThreshold_;
		adProb_ = p_load->adProb_;
		adaptionMode_ = p_load->adaptionMode_;
	}

	/***********************************************************************************/
	/** @brief Creates a deep copy of this object */
	virtual GObject *clone_(void) const =0;

	/***********************************************************************************/
	/**
	 *  This function is re-implemented by derived classes, if they wish to
	 *  implement special behavior upon a new adaption run. E.g., an internal
	 *  variable could be set to a new value. The function will be called every
	 *  adaptionThreshold_ calls of the adapt() function, unless the threshold is
	 *  set to 0 . It is not purely virtual, as we do not force derived classes
	 *  to re-implement this function. Note though that, if the function is
	 *  re-implemented, this class'es function should be called as the last action,
	 *  as later versions of this function might implement local logic. Adaption
	 *  of adaption parameters can be switched off by setting the adaptionThreshold_
	 *  variable to 0.
	 */
	virtual void adaptAdaption() { /* nothing */ }

	/***********************************************************************************/
	/** @brief Adaption of values as specified by the user */
	virtual void customAdaptions(T& val)=0;

private:
	/***********************************************************************************/
	boost::uint32_t adaptionCounter_; ///< A local counter
	boost::uint32_t adaptionThreshold_; ///< Specifies after how many adaptions the adaption itself should be adapted
	double adProb_; ///< internal representation of the adaption probability
	boost::logic::tribool adaptionMode_; ///< false == never adapt; indeterminate == adapt with adProb_ probability; true == always adapt

#ifdef GENEVATESTING
public:
	/***********************************************************************************/
	/**
	 * Applies modifications to this object. This is needed for testing purposes
	 *
	 * @return A boolean which indicates whether modifications were made
	 */
	virtual bool modify_GUnitTests() {
		bool result = false;

		// Call the parent classes' functions
		if(GObject::modify_GUnitTests()) result = true;

		return result;
	}

	/***********************************************************************************/
	/**
	 * Performs self tests that are expected to succeed. This is needed for testing purposes
	 */
	virtual void specificTestsNoFailureExpected_GUnitTests() {
		using boost::unit_test_framework::test_suite;
		using boost::unit_test_framework::test_case;

		// Call the parent classes' functions
		GObject::specificTestsNoFailureExpected_GUnitTests();

		//------------------------------------------------------------------------------

		{ // Test of GAdaptorT<T>::set/getAdaptionProbability()
			boost::shared_ptr<GAdaptorT<T> > p_test = this->clone<GAdaptorT<T> >();

			// The adaption probability should have been cloned
			BOOST_CHECK_MESSAGE(
					p_test->getAdaptionProbability() == this->getAdaptionProbability()
					,  "\n"
					<< "p_test->getAdaptionProbability() = " << p_test->getAdaptionProbability() << "\n"
					<< "this->getAdaptionProbability() = " << this->getAdaptionProbability() << "\n"
			);

			// Set the adaption probability to a sensible value and check the new setting
			double testAdProb = 0.5;
			BOOST_CHECK_NO_THROW(
					p_test->setAdaptionProbability(testAdProb);
			);
			BOOST_CHECK_MESSAGE(
					p_test->getAdaptionProbability() == testAdProb
					,  "\n"
					<< "p_test->getAdaptionProbability() = " << p_test->getAdaptionProbability() << "\n"
					<< "testAdProb = " << testAdProb << "\n"
			);
		}

		//------------------------------------------------------------------------------

		{ // Check that mutating a value with this class actually work with different likelihoods
			boost::shared_ptr<GAdaptorT<T> > p_test = this->clone<GAdaptorT<T> >();

			T testVal = T(0);
			for(double prob=0.; prob<1.; prob+=0.01) {
				// Account for rounding problems
				if(prob > 1.) prob = 1.;

				BOOST_CHECK_NO_THROW(p_test->setAdaptionProbability(prob));
				BOOST_CHECK_NO_THROW(p_test->adapt(testVal));
			}
		}

		//------------------------------------------------------------------------------

		{ // Test of GAdaptorT<T>::setAdaptionProbability() regarding the effects on the likelihood for adaption of the variable
			boost::shared_ptr<GAdaptorT<T> > p_test = this->clone<GAdaptorT<T> >();

			const std::size_t nTests=100000;

			for(double prob=0.; prob<1.; prob+=0.1) {
				// Account for rounding problems
				if(prob > 1.) prob = 1.;

				std::size_t nChanged=0;

				T testVal = T(0);
				T prevTestVal = testVal;

				// Set the likelihood for adaption to "prob"
				p_test->setAdaptionProbability(prob);

				// Mutating a boolean value a number of times should now result in a certain number of changed values
				for(std::size_t i=0; i<nTests; i++) {
					p_test->adapt(testVal);
					if(testVal != prevTestVal) {
						nChanged++;
						prevTestVal=testVal;
					}
				}

				double changeProb = double(nChanged)/double(nTests);

				BOOST_CHECK_MESSAGE(
						changeProb>0.95*prob
						,  "\n"
						<< "changeProb = " << changeProb << "\n"
						<< "prob = " << prob << "\n"
				);
			}
		}

		//------------------------------------------------------------------------------
	}

	/***********************************************************************************/
	/**
	 * Performs self tests that are expected to fail. This is needed for testing purposes
	 */
	virtual void specificTestsFailuresExpected_GUnitTests() {
		using boost::unit_test_framework::test_suite;
		using boost::unit_test_framework::test_case;

		// Call the parent classes' functions
		GObject::specificTestsFailuresExpected_GUnitTests();

		//------------------------------------------------------------------------------

		{ // Test of GAdaptorT<T>::setAdaptionProbability(): Setting a value < 0. should throw
			boost::shared_ptr<GAdaptorT<T> > p_test = this->clone<GAdaptorT<T> >();

			// Setting a probability < 0 should throw
			BOOST_CHECK_THROW(
					p_test->setAdaptionProbability(-1.);
					, Gem::Common::gemfony_error_condition
			);
		}

		//------------------------------------------------------------------------------

		{ // Test of GAdaptorT<T>::setAdaptionProbability(): Setting a value > 1. should throw
			boost::shared_ptr<GAdaptorT<T> > p_test = this->clone<GAdaptorT<T> >();

			// Setting a probability < 0 should throw
			BOOST_CHECK_THROW(
					p_test->setAdaptionProbability(2.);
					, Gem::Common::gemfony_error_condition
			);
		}

		//------------------------------------------------------------------------------
	}

#endif /* GENEVATESTING */
};

/******************************************************************************************/

} /* namespace Geneva */
} /* namespace Gem */

/********************************************************************************************/
/** @brief Mark this class as abstract. This is the content of
 * BOOST_SERIALIZATION_ASSUME_ABSTRACT(T) */

namespace boost {
	namespace serialization {
		template<typename T>
		struct is_abstract<Gem::Geneva::GAdaptorT<T> > : public boost::true_type {};
		template<typename T>
		struct is_abstract< const Gem::Geneva::GAdaptorT<T> > : public boost::true_type {};
	}
}

/********************************************************************************************/

#endif /* GADAPTORT_HPP_ */
