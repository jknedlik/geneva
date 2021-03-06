/********************************************************************************
 *
 * This file is part of the Geneva library collection. The following license
 * applies to this file:
 *
 * ------------------------------------------------------------------------------
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * ------------------------------------------------------------------------------
 *
 * Note that other files in the Geneva library collection may use a different
 * license. Please see the licensing information in each file.
 *
 ********************************************************************************
 *
 * Geneva was started by Dr. Rüdiger Berlich and was later maintained together
 * with Dr. Ariel Garcia under the auspices of Gemfony scientific. For further
 * information on Gemfony scientific, see http://www.gemfomy.eu .
 *
 * The majority of files in Geneva was released under the Apache license v2.0
 * in February 2020.
 *
 * See the NOTICE file in the top-level directory of the Geneva library
 * collection for a list of contributors and copyright information.
 *
 ********************************************************************************/

#pragma once

// Global checks, defines and includes needed for all of Geneva
#include "common/GGlobalDefines.hpp"

// Standard headers go here
#include <type_traits>

// Boost headers go here

// Geneva headers go here

#include "common/GSerializationHelperFunctionsT.hpp"
#include "hap/GRandomT.hpp"
#include "geneva/GObject.hpp"
#include "geneva/GOptimizationEnums.hpp"

namespace Gem {
namespace Geneva {

/******************************************************************************/
/**
 * The following applies mostly to evolutionary algorithms.
 *
 * In Geneva, two mechanisms exist that let the user specify the
 * type of adaption he wants to have executed on collections of
 * items (basic types or any other types).  The most basic
 * possibility is for the user to overload the GParameterSet::customAdaptions()
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
 * adapt() function. It is also possible to set an adaption probability, so only a certain
 * percentage of adaptions is actually performed at run-time.
 *
 * In order to use this class, the user must derive a class from
 * GAdaptorT<T, fp_type> and specify the type of adaption he wishes to
 * have applied to items, by overloading of
 * GAdaptorT<T, fp_type>::customAdaptions(T&) .  T will often be
 * represented by a basic value (double, long, bool, ...). Where
 * this is not the case, the adaptor will only be able to access
 * public functions of T, unless T declares the adaptor as a friend.
 *
 * As a derivative of GObject, this class follows similar rules as
 * the other Geneva classes.
 */
template<typename T, typename fp_type = double>
class GAdaptorT
	: public GObject
{
	 ///////////////////////////////////////////////////////////////////////
	 friend class boost::serialization::access;

	 template<typename Archive>
	 void serialize(Archive &ar, const unsigned int)
	 {
		 using boost::serialization::make_nvp;
		 ar
		 & BOOST_SERIALIZATION_BASE_OBJECT_NVP(GObject)
		 & BOOST_SERIALIZATION_NVP(m_adaptionCounter)
		 & BOOST_SERIALIZATION_NVP(m_adaptionThreshold)
		 & BOOST_SERIALIZATION_NVP(m_adProb)
		 & BOOST_SERIALIZATION_NVP(m_adaptAdProb)
		 & BOOST_SERIALIZATION_NVP(m_minAdProb)
		 & BOOST_SERIALIZATION_NVP(m_maxAdProb)
		 & BOOST_SERIALIZATION_NVP(m_adaptionMode)
		 & BOOST_SERIALIZATION_NVP(m_adaptAdaptionProbability)
		 & BOOST_SERIALIZATION_NVP(m_adProb_reset);
	 }
	 ///////////////////////////////////////////////////////////////////////

public:
	 /***************************************************************************/
	 /**
	  * Allows external callers to find out about the type stored in this object
	  */
	 using adaption_type = T;

	/***************************************************************************/
	/**
     * This constructor allows to set the probability with which an adaption is indeed
     * performed.
     *
     * @param adProb The likelihood for a an adaption to be actually carried out
     */
	explicit GAdaptorT(const fp_type &adProb)
			: GObject()
			, m_adProb(adProb)
	{
		// Do some error checking
		// Check that m_adProb is in the allowed range. Adapt, if necessary
		if (not Gem::Common::checkRangeCompliance<fp_type>(
				m_adProb
				, m_minAdProb
				, m_maxAdProb
				, "GAdaptorT<>::GAdaptorT(" + Gem::Common::to_string(adProb) + ")"
		)) {
			glogger
					<< "In GAdaptorT<T, fp_type>::GadaptorT(const fp_type& adProb):" << std::endl << "adProb value " << m_adProb
					<< " is outside of allowed value range [" << m_minAdProb << ", " << m_maxAdProb << "]" << std::endl
					<< "The value will be adapted to fit this range." << std::endl << GWARNING;

			Gem::Common::enforceRangeConstraint<fp_type>(
					m_adProb
					, m_minAdProb
					, m_maxAdProb
					, "GAdaptorT<>::GAdaptorT(" + Gem::Common::to_string(adProb) + " / 1)"
			);
			Gem::Common::enforceRangeConstraint<fp_type>(
					m_adProb_reset
					, m_minAdProb
					, m_maxAdProb
					, "GAdaptorT<>::GAdaptorT(" + Gem::Common::to_string(adProb) + " / 2)"
			);
		}
	}

	/***************************************************************************/
	// Defaulted constructors, destructor and assignment operators -- rule of five

	 GAdaptorT() = default;
	 GAdaptorT(GAdaptorT<T, fp_type> const & cp) = default;
	 GAdaptorT(GAdaptorT<T, fp_type> && cp) = default;

     ~GAdaptorT() override = default;

	 GAdaptorT<T, fp_type>& operator=(GAdaptorT<T, fp_type> const&) = default;
	 GAdaptorT<T, fp_type>& operator=(GAdaptorT<T, fp_type> &&) = default;

	 /***************************************************************************/
	 /**
	  * Retrieves the id of the adaptor.
	  *
	  * @return The id of the adaptor
	  */
	 Gem::Geneva::adaptorId getAdaptorId() const {
	 	return getAdaptorId_();
	 }

	 /* ----------------------------------------------------------------------------------
	  * Tested in GBooleanAdaptor
	  * Tested in GInt32FlipAdaptor
	  * Tested in GInt32GaussAdaptor
	  * Tested in GDoubleGaussAdaptor
	  * ----------------------------------------------------------------------------------
	  */

	 /***************************************************************************/
	 /**
	  * Sets the adaption probability to a given value. This function will throw
	  * if the probability is not in the allowed range.
	  *
	  * @param adProb The new value of the probability of adaptions taking place
	  */
	 void setAdaptionProbability(const fp_type &adProb)
	 {
		 // Check the supplied probability value
		 if (adProb < fp_type(0.) || adProb > fp_type(1.)) {
			 throw gemfony_exception(
				 g_error_streamer(DO_LOG, time_and_place)
					 << "In GAdaptorT<T, fp_type>::setAdaptionProbability(const fp_type&):" << std::endl
					 << "Bad probability value given: " << adProb << std::endl
			 );
		 }

		 // Check that the new value fits in the allowed value range
		 if (not Gem::Common::checkRangeCompliance<fp_type>(
			 adProb
			 , m_minAdProb
			 , m_maxAdProb
			 , "GAdaptorT<>::setAdaptionProbability(" + Gem::Common::to_string(adProb) + ")"
		 )) {
			 throw gemfony_exception(
				 g_error_streamer(DO_LOG, time_and_place)
					 << "In GAdaptorT<T, fp_type>::setAdaptionProbability(const fp_type& adProb):" << std::endl
					 << "adProb value " << adProb << " is outside of allowed value range [" << m_minAdProb << ", " << m_maxAdProb
					 << "]" << std::endl
					 << "Set new boundaries first before setting a new \"adProb\" value" << std::endl
			 );
		 }

		 m_adProb = adProb;
	 }

	 /* ----------------------------------------------------------------------------------
	  * Setting of valid probabilities is tested in GAdaptorT<T, fp_type>::specificTestsNoFailuresExpected_GUnitTests()
	  * Checks for setting of invalid probabilities is tested in GAdaptorT<T, fp_type>::specificTestsFailuresExpected_GUnitTests()
	  * The effects on the probability of adaptions actually taking place are tested in GAdaptorT<T, fp_type>::specificTestsNoFailuresExpected_GUnitTests()
	  * ----------------------------------------------------------------------------------
	  */

	 /***************************************************************************/
	 /**
	  * Retrieves the current value of the adaption probability
	  *
	  * @return The current value of the adaption probability
	  */
	 fp_type getAdaptionProbability() const
	 {
		 return m_adProb;
	 }

	 /* ----------------------------------------------------------------------------------
	  * Retrieval of probabilities is tested in GAdaptorT<T, fp_type>::specificTestsNoFailuresExpected_GUnitTests()
	  * ----------------------------------------------------------------------------------
	  */

	 /***************************************************************************/
	 /**
	  * Sets the "reset" adaption probability to a given value. This is the probability
	  * to which adProb_ will be reset if updateOnStall() is called. This function will
	  * throw if the probability is not in the allowed range.
	  *
	  * @param adProb_reset The new value of the "reset" probability
	  */
	 void setResetAdaptionProbability(const fp_type &adProb_reset)
	 {
		 // Check the supplied probability value
		 if (not Gem::Common::checkRangeCompliance<fp_type>(
			 adProb_reset
			 , m_minAdProb
			 , m_maxAdProb
			 , "GAdaptorT<>::setResetAdaptionProbability(" + Gem::Common::to_string(adProb_reset) + ")"
		 )) {
			 throw gemfony_exception(
				 g_error_streamer(DO_LOG, time_and_place)
					 << "In GAdaptorT<T, fp_type>::setResetAdaptionProbability(const fp_type&):" << std::endl
					 << "adProb_reset value " << adProb_reset << " is outside of allowed value range [" << m_minAdProb << ", "
					 << m_maxAdProb << "]" << std::endl
					 << "Set new boundaries first before setting a new \"adProb_reset\" value" << std::endl
			 );
		 }

		 m_adProb_reset = adProb_reset;
	 }

	 /***************************************************************************/
	 /**
	  * Retrieves the current value of the "reset" adaption probability
	  *
	  * @return The current value of the "reset" adaption probability
	  */
	 fp_type getResetAdaptionProbability() const
	 {
		 return m_adProb_reset;
	 }

	 /***************************************************************************/
	 /**
	  * Sets the probability for the adaption of adaption parameters
	  *
	  * @param probability The new value of the probability of adaptions of adaption parameters
	  */
	 void setAdaptAdaptionProbability(const fp_type &probability)
	 {
		 // Check the supplied probability value
		 if (not Gem::Common::checkRangeCompliance<fp_type>(
			 probability
			 , 0.
			 , 1.
			 , "GAdaptorT<>::setAdaptAdaptionProbability(" + Gem::Common::to_string(probability) + ")"
		 )) {
			 throw gemfony_exception(
				 g_error_streamer(DO_LOG, time_and_place)
					 << "In GAdaptorT<T, fp_type>::setAdaptAdaptionProbability(const fp_type&) :" << std::endl
					 << "Probability " << probability << " not in allowed range [0.,1.]" << std::endl
			 );
		 }

		 m_adaptAdaptionProbability = probability;
	 }

	 /* ----------------------------------------------------------------------------------
	  * Setting of valid probabilities is tested in GAdaptorT<T, fp_type>::specificTestsNoFailuresExpected_GUnitTests()
	  * Checks for setting of invalid probabilities is tested in GAdaptorT<T, fp_type>::specificTestsFailuresExpected_GUnitTests()
	  * ----------------------------------------------------------------------------------
	  */

	 /***************************************************************************/
	 /**
	  * Retrieves the current value of the adaptAdaptionProbability_ variable
	  *
	  * @return The current value of the adaptAdaptionProbability_ variable
	  */
	 fp_type getAdaptAdaptionProbability() const
	 {
		 return m_adaptAdaptionProbability;
	 }

	 /* ----------------------------------------------------------------------------------
	  * Retrieval of probabilities is tested in GAdaptorT<T, fp_type>::specificTestsNoFailuresExpected_GUnitTests()
	  * ----------------------------------------------------------------------------------
	  */

	 /***************************************************************************/
	 /**
	  * Allows to specify an adaption factor for adProb_ (or 0, if you do not
	  * want this feature)
	  */
	 void setAdaptAdProb(fp_type adaptAdProb)
	 {
#ifdef DEBUG
		 if (adaptAdProb < fp_type(0.)) {
			 throw gemfony_exception(
				 g_error_streamer(DO_LOG, time_and_place)
					 << "In GAdaptorT<>::setAdaptAdProb(): Error!" << std::endl
					 << "adaptAdProb < 0: " << adaptAdProb << std::endl
			 );
		 }
#endif /* DEBUG */

		 m_adaptAdProb = adaptAdProb;
	 }

	 /***************************************************************************/
	 /**
	  * Allows to retrieve the rate of evolutionary adaption of adProb_
	  */
	 fp_type getAdaptAdProb() const
	 {
		 return m_adaptAdProb;
	 }

	 /***************************************************************************/
	 /**
	  * Retrieves the current value of the m_adaptionCounter variable.
	  *
	  * @return The value of the m_adaptionCounter variable
	  */
	 std::uint32_t getAdaptionCounter() const
	 {
		 return m_adaptionCounter;
	 }

	 /* ----------------------------------------------------------------------------------
	  * It is tested in GAdaptorT<T, fp_type>::specificTestsNoFailuresExpected_GUnitTests() that the
	  * adaption counter does not exceed the set adaption threshold
	  * ----------------------------------------------------------------------------------
	  */

	 /***************************************************************************/
	 /**
	  * Sets the value of adaptionThreshold_. If set to 0, no adaption of the optimization
	  * parameters will take place
	  *
	  * @param adaptionCounter The value that should be assigned to the adaptionCounter_ variable
	  */
	 void setAdaptionThreshold(const std::uint32_t &adaptionThreshold)
	 {
		 m_adaptionThreshold = adaptionThreshold;
	 }

	 /* ----------------------------------------------------------------------------------
	  * Setting of adaption thresholds is tested in GAdaptorT<T, fp_type>::specificTestsNoFailuresExpected_GUnitTests()
	  * ----------------------------------------------------------------------------------
	  */

	 /***************************************************************************/
	 /**
	  * Retrieves the value of the adaptionThreshold_ variable.
	  *
	  * @return The value of the adaptionThreshold_ variable
	  */
	 std::uint32_t getAdaptionThreshold() const
	 {
		 return m_adaptionThreshold;
	 }

	 /* ----------------------------------------------------------------------------------
	  * Retrieval of adaption threshold is tested in GAdaptorT<T, fp_type>::specificTestsNoFailuresExpected_GUnitTests()
	  * ----------------------------------------------------------------------------------
	  */

	 /***************************************************************************/
	 /**
	  * Allows to specify whether adaptions should happen always, never, or with a given
	  * probability. The function is declared virtual so adaptors requiring adaptions to
	  * happen always or never can prevent resetting of the m_adaptionMode variable.
	  *
	  * @param adaptionMode The desired mode (always/never/with a given probability)
	  */
	 virtual void setAdaptionMode(adaptionMode am) BASE {
		 m_adaptionMode = am;
	 }

	 /* ----------------------------------------------------------------------------------
	  * Setting of the adaption mode is tested in GAdaptorT<T, fp_type>::specificTestsNoFailuresExpected_GUnitTests()
	  * The effect of setting the adaption mode is tested in GAdaptorT<T, fp_type>::specificTestsNoFailuresExpected_GUnitTests()
	  * ----------------------------------------------------------------------------------
	  */

	 /***************************************************************************/
	 /**
	  * Returns the current value of the m_adaptionMode variable
	  *
	  * @return The current value of the m_adaptionMode variable
	  */
	 adaptionMode getAdaptionMode() const {
		 return m_adaptionMode;
	 }

	 /* ----------------------------------------------------------------------------------
	  * Retrieval of the adaption mode is tested in GAdaptorT<T, fp_type>::specificTestsNoFailuresExpected_GUnitTests()
	  * ----------------------------------------------------------------------------------
	  */

	 /***************************************************************************/
	 /**
	  * Allows to set the allowed range for adaption probability variation.
	  * NOTE that this function will silently adapt the values of adProb_ and
	  * adProb_reset_, if they fall outside of the new range.
	  */
	 void setAdProbRange(fp_type minAdProb, fp_type maxAdProb)
	 {
#ifdef DEBUG
		 if (minAdProb < 0.) {
			 throw gemfony_exception(
				 g_error_streamer(DO_LOG, time_and_place)
					 << "In GAdaptorT<T, fp_type>::setAdProbRange(): Error!" << std::endl
					 << "minAdProb < 0: " << minAdProb << std::endl
			 );
		 }

		 if (maxAdProb > 1.) {
			 throw gemfony_exception(
				 g_error_streamer(DO_LOG, time_and_place)
					 << "In GAdaptorT<T, fp_type>::setAdProbRange(): Error!" << std::endl
					 << "maxAdProb > 1: " << maxAdProb << std::endl
			 );
		 }

		 if (minAdProb > maxAdProb) {
			 throw gemfony_exception(
				 g_error_streamer(DO_LOG, time_and_place)
					 << "In GAdaptorT<T, fp_type>::setAdProbRange(): Error!" << std::endl
					 << "Invalid minAdProb and/or maxAdProb: " << minAdProb << " / " << maxAdProb << std::endl
			 );
		 }
#endif /* DEBUG */

		 // Store the new values
		 m_minAdProb = minAdProb;
		 if (m_minAdProb < DEFMINADPROB) {
			 m_minAdProb = DEFMINADPROB;
		 }
		 m_maxAdProb = maxAdProb;

		 // Make sure m_adProb and m_adProb_reset fit the new allowed range
		 Gem::Common::enforceRangeConstraint<fp_type>(
			 m_adProb
			 , m_minAdProb
			 , m_maxAdProb
			 , "GAdaptorT<>::setAdProbRange() / 1"
		 );
		 Gem::Common::enforceRangeConstraint<fp_type>(
			 m_adProb_reset
			 , m_minAdProb
			 , m_maxAdProb
			 , "GAdaptorT<>::setAdProbRange() / 2"
		 );
	 }

	 /***************************************************************************/
	 /**
	  * Allows to retrieve the allowed range for adProb_ variation
	  */
	 auto getAdProbRange() const
	 {
		 return std::tuple<fp_type, fp_type>{m_minAdProb, m_maxAdProb};
	 }

	 /***************************************************************************/
	 /**
	  * Common interface for all adaptors to the adaption functionality. The user
	  * specifies the actual actions in the customAdaptions() function.
	  *
	  * @param val The value that needs to be adapted
	  * @param range A typical value range for type T
	  * @param gr A reference to a random number generator
	  * @return The number of adaptions that were carried out
	  */
	 std::size_t adapt(
		 T &val
		 , const T &range
		 , Gem::Hap::GRandomBase& gr
	 ) {
		 using namespace Gem::Common;
		 using namespace Gem::Hap;

		 bool adapted = false;

		 // Update the adaption probability, if requested by the user
		 if (m_adaptAdProb > fp_type(0.)) {
			 m_adProb *= gexp(
				 m_normal_distribution(
					 gr
					 , typename std::normal_distribution<fp_type>::param_type(
						 0.
						 , m_adaptAdProb
					 ))
			 );
			 Gem::Common::enforceRangeConstraint<fp_type>(
				 m_adProb
				 , m_minAdProb
				 , m_maxAdProb
				 , "GAdaptorT<>::adapt() / 1"
			 );
		 }

		 if (adaptionMode::WITHPROBABILITY == m_adaptionMode) { // The most likely case is indeterminate (means: "sometimes" here)
			 if (m_weighted_bool(gr, std::bernoulli_distribution::param_type(gfabs(m_adProb)))) { // Likelihood of m_adProb for the adaption
				 adaptAdaption(range, gr);
				 customAdaptions(
					 val
					 , range
					 , gr
				 );
				 adapted = true;
			 }
		 } else if (adaptionMode::ALWAYS == m_adaptionMode) { // always adapt
			 adaptAdaption(range, gr);
			 customAdaptions(
				 val
				 , range
				 , gr
			 );
			 adapted = true;
		 }

		 // No need to test for "m_adaptionMode == adaptionMode::NEVER" as no action is needed in this case

		 if (adapted) {
			 return std::size_t(1);
		 } else {
			 return std::size_t(0);
		 }
	 }

	 /* ----------------------------------------------------------------------------------
	  * Adaption is tested in GAdaptorT<T, fp_type>::specificTestsNoFailuresExpected_GUnitTests()
	  * ----------------------------------------------------------------------------------
	  */

	 /***************************************************************************/
	 /**
	  * Common interface for all adaptors to the adaption functionality. The user
	  * specifies the actual actions in the customAdaptions() function. This function
	  * deals with entire parameter vectors. The philosophy behind these vectors is
	  * that they represent a common logical entity and should thus be mutated together,
	  * using a single adaptor. However, it is not clear whether adaptions of mutation
	  * parameters (such as adaption of the sigma value) should happen whenever
	  * customAdaptions() is called (which would be equivalent to individual parameter
	  * objects) or only once, before customAdaptions is applied to each position in
	  * turn. As adaption e.g. of the sigma value slightly favors changes towards smaller
	  * values, we incur a small bias in the first case, where mutations of parameters
	  * at the end of the array might be smaller than at the beginning. In the second case,
	  * metaAdaption might not be called often enough to adapt the mutation process
	  * to different geometries of the quality surface. Our tests show that the latter
	  * might be more severe, so we have implemented repeated adaption of mutation parameters
	  * in this function.
	  *
	  * @param valVec A vector of values that need to be adapted
	  * @param range A typical value range for type T
	  * @return The number of adaptions that were carried out
	  */
	 std::size_t adapt(
		 std::vector<T> &valVec
		 , const T &range
		 , Gem::Hap::GRandomBase& gr
	 ) {
		 using namespace Gem::Common;
		 using namespace Gem::Hap;

		 std::size_t nAdapted = 0;

		 // Update the adaption probability, if requested by the user
		 if (m_adaptAdProb > fp_type(0.)) {
			 m_adProb *= gexp(
				 m_normal_distribution(
					 gr
					 , typename std::normal_distribution<fp_type>::param_type(
						 0.
						 , m_adaptAdProb
					 ))
			 );
			 Gem::Common::enforceRangeConstraint<fp_type>(
				 m_adProb
				 , m_minAdProb
				 , m_maxAdProb
				 , "GAdaptorT<>::adapt() / 2"
			 );
		 }

		 if (adaptionMode::WITHPROBABILITY == m_adaptionMode) { // The most likely case
			 for (auto &val: valVec) {
				 // A likelihood of m_adProb for adaption
				 if (m_weighted_bool(gr, std::bernoulli_distribution::param_type(gfabs(m_adProb)))) {
					 adaptAdaption(range, gr);
					 customAdaptions(
						 val
						 , range
						 , gr
					 );

					 nAdapted += 1;
				 }
			 }
		 } else if (adaptionMode::ALWAYS == m_adaptionMode) { // always adapt
			 for (auto &val: valVec) {
				 adaptAdaption(range, gr);
				 customAdaptions(
					 val
					 , range
					 , gr
				 );

				 nAdapted += 1;
			 }
		 }

		 // No need to test for "m_adaptionMode == adaptionMode::NEVER" as no action is needed in this case

		 return nAdapted;
	 }

	 /* ----------------------------------------------------------------------------------
	  * Adaption is tested in GAdaptorT<T, fp_type>::specificTestsNoFailuresExpected_GUnitTests()
	  * ----------------------------------------------------------------------------------
	  */

	 /***************************************************************************/
	 /**
	  * Triggers updates when the optimization process has stalled. This function
	  * resets the adaption probability to its original value
	  *
	  * @param nStalls The number of consecutive stalls up to this point
	  * @param range A typical value range for type T
	  * @return A boolean indicating whether updates were performed
	  */
	 virtual bool updateOnStall(
		 const std::size_t &nStalls
		 , const T &range
	 ) BASE {
#ifdef DEBUG
		 if (0 == nStalls) {
			 throw gemfony_exception(
				 g_error_streamer(DO_LOG, time_and_place)
					 << "In GAdaptorT<>::updateOnStall(" << nStalls << "): Error!" << std::endl
					 << "Function called for zero nStalls" << std::endl
			 );
		 }
#endif

		 // Reset the adaption probability
		 if (m_adProb == m_adProb_reset) {
			 return false;
		 } else {
			 m_adProb = m_adProb_reset;
			 return true;
		 }
	 }

	 /***************************************************************************/
	 /**
	  * Allows derived classes to print diagnostic messages
	  *
	  * @return A diagnostic message
	  */
	 virtual std::string printDiagnostics() const BASE
	 {
		 return std::string();
	 }

	 /***************************************************************************/
	 /**
	  * Allows to query specific properties of a given adaptor. Note that the
	  * adaptor must have implemented a "response" for the query, as the function
	  * will otherwise throw. This function is meant for debugging and profiling.
	  * It might e.g. be useful if you want to know why an EA-based optimization has
	  * stalled. Note that the permanent use of this function, e.g. from a permanently
	  * enabled "pluggable optimization monitor, will be inefficient due to the
	  * constant need to compare strings.
	  *
	  * @param adaoptorName The name of the adaptor to be queried
	  * @param property The property for which information is sought
	  * @param data A vector, to which the properties should be added
	  */
	 void queryPropertyFrom(
		 const std::string &adaptorName
		 , const std::string &property
		 , std::vector<boost::any> &data
	 ) const BASE {
		 // Do nothing, if this query is not for us
		 if (adaptorName != this->name()) {
			 return;
		 } else { // O.k., this query is for us!
			 if (property == "adProb") { // The only property that can be queried for this class
				 data.push_back(boost::any(m_adProb));
			 } else { // Ask derived classes
				 if (not this->customQueryProperty(
					 property
					 , data
				 )) {
					 throw gemfony_exception(
						 g_error_streamer(DO_LOG, time_and_place)
							 << "In GAdaptorT<T, fp_type>::queryPropertyFrom(): Error!" << std::endl
							 << "Function was called for unimplemented property " << property << std::endl
							 << "on adaptor " << adaptorName << std::endl
					 );
				 }
			 }
		 }
	 }

	 /***************************************************************************/
	 /** @brief Allows derived classes to randomly initialize parameter members */
	 virtual bool randomInit(Gem::Hap::GRandomBase&) BASE = 0;

protected:
	/***************************************************************************/
	 /**
	  * Loads the contents of another GAdaptorT<T, fp_type>. The function
	  * is similar to a copy constructor (but with a pointer as
	  * argument). As this function might be called in an environment
	  * where we do not know the exact type of the class, the
	  * GAdaptorT<T, fp_type> is camouflaged as a GObject . This implies the
	  * need for dynamic conversion.
	  *
	  * @param gb A pointer to another GAdaptorT<T, fp_type>, camouflaged as a GObject
	  */
	 void load_(const GObject *cp) override
	 {
		 // Check that we are dealing with a GAdaptorT<T, fp_type> reference independent of this object and convert the pointer
		 const GAdaptorT<T, fp_type> *p_load = Gem::Common::g_convert_and_compare<GObject, GAdaptorT<T, fp_type>>(cp, this);

		 // Load the parent class'es data
		 GObject::load_(cp);

		 // Then our own data
		 m_adaptionCounter = p_load->m_adaptionCounter;
		 m_adaptionThreshold = p_load->m_adaptionThreshold;
		 m_adProb = p_load->m_adProb;
		 m_adaptAdProb = p_load->m_adaptAdProb;
		 m_minAdProb = p_load->m_minAdProb;
		 m_maxAdProb = p_load->m_maxAdProb;
		 m_adaptionMode = p_load->m_adaptionMode;
		 m_adaptAdaptionProbability = p_load->m_adaptAdaptionProbability;
		 m_adProb_reset = p_load->m_adProb_reset;
	 }

	/***************************************************************************/
	/** @brief Allow access to this classes compare_ function */
	friend void Gem::Common::compare_base_t<GAdaptorT<T, fp_type>>(
		GAdaptorT<T, fp_type> const &
		, GAdaptorT<T, fp_type> const &
		, Gem::Common::GToken &
	);

	/***************************************************************************/
	/**
     * Searches for compliance with expectations with respect to another object
     * of the same type
     *
     * @param cp A constant reference to another GObject object
     * @param e The expected outcome of the comparison
     * @param limit The maximum deviation for floating point values (important for similarity checks)
     */
	void compare_(
		const GObject &cp
		, const Gem::Common::expectation &e
		, const fp_type &limit
	) const override {
		using namespace Gem::Common;

		// Check that we are dealing with a GAdaptorT<T, fp_type> reference independent of this object and convert the pointer
		const GAdaptorT<T, fp_type> *p_load = Gem::Common::g_convert_and_compare<GObject, GAdaptorT<T, fp_type>>(cp, this);

		GToken token("GAdaptorT<T, fp_type>", e);

		// Compare our parent data ...
		Gem::Common::compare_base_t<GObject>(*this, *p_load, token);

		// ... and then the local data
		compare_t(IDENTITY(m_adaptionCounter, p_load->m_adaptionCounter), token);
		compare_t(IDENTITY(m_adaptionThreshold, p_load->m_adaptionThreshold), token);
		compare_t(IDENTITY(m_adProb, p_load->m_adProb), token);
		compare_t(IDENTITY(m_adaptAdProb, p_load->m_adaptAdProb), token);
		compare_t(IDENTITY(m_minAdProb, p_load->m_minAdProb), token);
		compare_t(IDENTITY(m_maxAdProb, p_load->m_maxAdProb), token);
		compare_t(IDENTITY(m_adaptionMode, p_load->m_adaptionMode), token);
		compare_t(IDENTITY(m_adaptAdaptionProbability, p_load->m_adaptAdaptionProbability), token);
		compare_t(IDENTITY(m_adProb_reset, p_load->m_adProb_reset), token);

		// React on deviations from the expectation
		token.evaluate();
	}

	/***************************************************************************/
	 /**
	  * This function helps to adapt the adaption parameters, if certain conditions are met.
	  * Adaption is triggered by the parameter object.
	  *
	  *  @param range A typical range for the parameter with type T
	  */
	 void adaptAdaption(
		 const T &range
		 , Gem::Hap::GRandomBase& gr
	 ) {
		 using namespace Gem::Common;
		 using namespace Gem::Hap;

		 // The adaption parameters are modified every m_adaptionThreshold number of adaptions.
		 if (m_adaptionThreshold > 0) {
			 if (++m_adaptionCounter >= m_adaptionThreshold) {
				 m_adaptionCounter = 0;
				 customAdaptAdaption(range, gr);
			 }
		 } else if (m_adaptAdaptionProbability) { // Do the same with probability settings
			 // Likelihood of m_adaptAdaptionProbability for the adaption
			 if (m_weighted_bool(gr, std::bernoulli_distribution::param_type(gfabs(m_adaptAdaptionProbability)))) {
				 customAdaptAdaption(range, gr);
			 }
		 }
	 }

	 /***************************************************************************/
	 /**
	  * Adds a given property value to the vector or returns false, if the property
	  * was not found. We do not check anymore if this query was for as, as this was
	  * already done by  queryPropertyFrom(). Thus function needs to be re-implemented
	  * by derived classes wishing to emit information. If there is no re-implementation,
	  * this function will simply return false.
	  */
	 virtual bool customQueryProperty(
		 const std::string &property
		 , std::vector<boost::any> &data
	 ) const BASE {
		 return false;
	 }

	 /***************************************************************************/
	 /**
	  *  This function is re-implemented by derived classes, if they wish to
	  *  implement special behavior for a new adaption run. E.g., an internal
	  *  variable could be set to a new value.
	  *
	  *  @param range A typical range for the parameter with type T
	  */
	 virtual void customAdaptAdaption(
		 const T &
		 , Gem::Hap::GRandomBase& gr
	 ) BASE { /* nothing */}

	 /***************************************************************************/

	 /** @brief Adaption of values as specified by the user */
	 virtual void customAdaptions(T &, const T &, Gem::Hap::GRandomBase&) BASE = 0;

	 /** @brief Creates a deep copy of this object */
	 GObject *clone_() const override = 0;


    /***************************************************************************/
    /**
     * Applies modifications to this object. This is needed for testing purposes
     *
     * @return A boolean which indicates whether modifications were made
     */
    bool modify_GUnitTests_() override
    {
#ifdef GEM_TESTING
        using boost::unit_test_framework::test_suite;
        using boost::unit_test_framework::test_case;

        bool result = false;

        // Call the parent classes' functions
        if (GObject::modify_GUnitTests_()) {
            result = true;
        }

        // Modify some local parameters
        if (this->getAdaptionProbability() <= 0.5) {
            this->setAdaptionProbability(0.75);
        } else {
            this->setAdaptionProbability(0.25);
        }

        result = true;

        return result;

#else /* GEM_TESTING */  // If this function is called when GEM_TESTING isn't set, throw
        Gem::Common::condnotset("GAdaptorT<>::modify_GUnitTests", "GEM_TESTING");
		 return false;
#endif /* GEM_TESTING */
    }

    /***************************************************************************/
    /**
     * Performs self tests that are expected to succeed. This is needed for testing purposes
     */
    void specificTestsNoFailureExpected_GUnitTests_() override
    {
#ifdef GEM_TESTING
        using boost::unit_test_framework::test_suite;
        using boost::unit_test_framework::test_case;

        // Call the parent classes' functions
        GObject::specificTestsNoFailureExpected_GUnitTests_();

        // Retrieve a random number generator
        Gem::Hap::GRandomT<Gem::Hap::RANDFLAVOURS::RANDOMPROXY> gr;

        //------------------------------------------------------------------------------

        { // Test of GAdaptorT<T, fp_type>::set/getAdaptionProbability()
            std::shared_ptr<GAdaptorT<T, fp_type>> p_test = this->clone<GAdaptorT<T, fp_type>>();

            // The adaption probability should have been cloned
            BOOST_CHECK_MESSAGE(
                    p_test->getAdaptionProbability() == this->getAdaptionProbability()
            , "\n"
                            << "p_test->getAdaptionProbability() = " << p_test->getAdaptionProbability() << "\n"
                            << "this->getAdaptionProbability() = " << this->getAdaptionProbability() << "\n"
            );

            // Set an appropriate range for the adaption
            p_test->setAdProbRange(
                    0.001
                    , 1.
            );

            // Set the adaption probability to a sensible value and check the new setting
            fp_type testAdProb = fp_type(0.5);
            BOOST_CHECK_NO_THROW(
                    p_test->setAdaptionProbability(testAdProb);
            );
            BOOST_CHECK_MESSAGE(
                    p_test->getAdaptionProbability() == testAdProb
            , "\n"
                            << "p_test->getAdaptionProbability() = " << p_test->getAdaptionProbability() << "\n"
                            << "testAdProb = " << testAdProb << "\n"
            );
        }

        //------------------------------------------------------------------------------

        { // Check that mutating a value with this class actually work with different likelihoods
            std::shared_ptr<GAdaptorT<T, fp_type>> p_test = this->clone<GAdaptorT<T, fp_type>>();

            // Make sure the adaption probability is taken into account
            p_test->setAdaptionMode(adaptionMode::WITHPROBABILITY);
            // Set an appropriate range for the adaption
            p_test->setAdProbRange(
                    0.001
                    , 1.
            );

            T testVal = T(0);
            for (fp_type prob = 0.001; prob < 1.; prob += 0.01) {
                // Account for rounding problems
                if (prob > 1.) {
                    prob = 1.;
                }

                p_test->setAdaptionProbability(prob);
                BOOST_CHECK_NO_THROW(p_test->setAdaptionProbability(prob));
                BOOST_CHECK_NO_THROW(p_test->adapt(
                        testVal
                        , T(1)
                        , gr
                ));
            }
        }

        //------------------------------------------------------------------------------

        { // Test of GAdaptorT<T, fp_type>::setAdaptionProbability() regarding the effects on the likelihood for adaption of the variable
            std::shared_ptr<GAdaptorT<T, fp_type>> p_test = this->clone<GAdaptorT<T, fp_type>>();

            // Make sure the adaption probability is taken into account
            p_test->setAdaptionMode(adaptionMode::WITHPROBABILITY);
            // Prevent changes to m_adProb
            p_test->setAdaptAdProb(0.);

            p_test->setAdProbRange(
                    0.
                    , 1.
            );

            const std::size_t nTests = 100000;

            for (fp_type prob = 0.1; prob < 1.; prob += 0.1) {
                // Account for rounding problems
                if (prob > 1.) {
                    prob = 1.;
                }

                std::size_t nChanged = 0;

                T testVal = T(0);
                T prevTestVal = testVal;

                // Set the likelihood for adaption to "prob"
                p_test->setAdaptionProbability(prob);

                // Mutating a boolean value a number of times should now result in a certain number of changed values
                for (std::size_t i = 0; i < nTests; i++) {
                    p_test->adapt(
                            testVal
                            , T(1)
                            , gr
                    );
                    if (testVal != prevTestVal) {
                        nChanged++;
                        prevTestVal = testVal;
                    }
                }

                fp_type changeProb = fp_type(nChanged) / fp_type(nTests);

                BOOST_CHECK_MESSAGE(
                        changeProb > 0.8 * prob && changeProb < 1.2 * prob
                , "\n"
                                << "changeProb = " << changeProb << "\n"
                                << "prob = " << prob << "\n"
                                << "with allowed window = [" << 0.8 * prob << " : " << 1.2 * prob << "]" << "\n"
                );
            }
        }

        //------------------------------------------------------------------------------

        { // Check setting and retrieval of the adaption mode
            std::shared_ptr<GAdaptorT<T, fp_type>> p_test = this->clone<GAdaptorT<T, fp_type>>();

            // Check setting of the different allowed values
            // false
            BOOST_CHECK_NO_THROW (p_test->setAdaptionMode(adaptionMode::NEVER));
            BOOST_CHECK_MESSAGE (
                    p_test->getAdaptionMode() == adaptionMode::NEVER
            , "\n"
                            << "p_test->getAdaptionMode() = " << p_test->getAdaptionMode() << "\n"
                            << "required value            = adaptionMode::NEVER\n"
            );

            // true
            BOOST_CHECK_NO_THROW (p_test->setAdaptionMode(adaptionMode::ALWAYS));
            BOOST_CHECK_MESSAGE (
                    adaptionMode::ALWAYS == p_test->getAdaptionMode()
            , "\n"
                            << "p_test->getAdaptionMode() = " << p_test->getAdaptionMode() << "\n"
                            << "required value            = adaptionMode::ALWAYS\n"
            );

            // boost::logic::indeterminate
            BOOST_CHECK_NO_THROW (p_test->setAdaptionMode(adaptionMode::WITHPROBABILITY));
            BOOST_CHECK_MESSAGE (
                    adaptionMode::WITHPROBABILITY == p_test->getAdaptionMode()
            , "\n"
                            << "p_test->getAdaptionMode() = " << p_test->getAdaptionMode() << "\n"
                            << "required value            = boost::logic::indeterminate\n"
            );
        }

        //------------------------------------------------------------------------------

        { // Check the effect of the adaption mode settings
            std::shared_ptr<GAdaptorT<T, fp_type>> p_test = this->clone<GAdaptorT<T, fp_type>>();
            p_test->setAdaptionProbability(0.5);

            const std::size_t nTests = 10000;

            // false: There should never be adaptions, independent of the adaption probability
            BOOST_CHECK_NO_THROW (p_test->setAdaptionMode(adaptionMode::NEVER));
            T currentValue = T(0);
            T oldValue = currentValue;
            for (std::size_t i = 0; i < nTests; i++) {
                p_test->adapt(
                        currentValue
                        , T(1)
                        , gr
                );
                BOOST_CHECK_MESSAGE (
                        currentValue == oldValue
                , "\n"
                                << "Values differ, when they shouldn't:"
                                << "currentValue = " << currentValue << "\n"
                                << "oldValue     = " << oldValue << "\n"
                                << "iteration    = " << i << "\n"
                );
            }

            // true: Adaptions should happen always, independent of the adaption probability
            BOOST_CHECK_NO_THROW (p_test->setAdaptionMode(adaptionMode::ALWAYS));
            currentValue = T(0);
            oldValue = currentValue;
            for (std::size_t i = 0; i < nTests; i++) {
                p_test->adapt(
                        currentValue
                        , T(1)
                        , gr
                );
                BOOST_CHECK_MESSAGE (
                        currentValue != oldValue
                , "\n"
                                << "Values are identical when they shouldn't be:" << "\n"
                                << "currentValue = " << currentValue << "\n"
                                << "oldValue     = " << oldValue << "\n"
                                << "iteration    = " << i << "\n"
                                << (this->printDiagnostics()).c_str()
                );
                oldValue = currentValue;
            }

            // boost::logic::indeterminate: Adaptions should happen with a certain adaption probability
            // No tests -- we already know that this works
        }

        //------------------------------------------------------------------------------

        { // Test of GAdaptorT<T, fp_type>::set/getAdaptAdaptionProbability()
            std::shared_ptr<GAdaptorT<T, fp_type>> p_test = this->clone<GAdaptorT<T, fp_type>>();

            // The adaption probability should have been cloned
            BOOST_CHECK_MESSAGE(
                    p_test->getAdaptAdaptionProbability() == this->getAdaptAdaptionProbability()
            , "\n"
                            << "p_test->getAdaptAdaptionProbability() = " << p_test->getAdaptAdaptionProbability() << "\n"
                            << "this->getAdaptAdaptionProbability() = " << this->getAdaptAdaptionProbability() << "\n"
            );

            // Set the adaption probability to a sensible value and check the new setting
            fp_type testAdProb = 0.5;
            BOOST_CHECK_NO_THROW(
                    p_test->setAdaptAdaptionProbability(testAdProb);
            );
            BOOST_CHECK_MESSAGE(
                    p_test->getAdaptAdaptionProbability() == testAdProb
            , "\n"
                            << "p_test->getAdaptAdaptionProbability() = " << p_test->getAdaptAdaptionProbability() << "\n"
                            << "testAdProb = " << testAdProb << "\n"
            );
        }

        //------------------------------------------------------------------------------

        { // Test retrieval and setting of the adaption threshold and whether the adaptionCounter behaves nicely
            std::shared_ptr<GAdaptorT<T, fp_type>> p_test = this->clone<GAdaptorT<T, fp_type>>();

            // Make sure we have the right adaption mode
            p_test->setAdaptionMode(adaptionMode::WITHPROBABILITY);
            // Make sure we always adapt
            p_test->setAdaptionProbability(1.0);

            // The value that will be adapted
            T testVal = T(0);
            T oldTestVal = T(0);

            // The old adaption counter
            std::uint32_t oldAdaptionCounter = p_test->getAdaptionCounter();

            // Set the adaption threshold to a specific value
            for (std::uint32_t adThr = 10; adThr > 0; adThr--) {
                // Just make sure our logic is right and we stay in the right window
                BOOST_CHECK(adThr <= 10);

                BOOST_CHECK_NO_THROW(p_test->setAdaptionThreshold(adThr));
                BOOST_CHECK_MESSAGE(
                        p_test->getAdaptionThreshold() == adThr
                , "\n"
                                << "p_test->getAdaptionThreshold() = " << p_test->getAdaptionThreshold() << "\n"
                                << "adThr = " << adThr << "\n"
                );

                // Check that the adaption counter does not exceed the threshold by
                // adapting a value a number of times > adThr
                for (std::uint32_t adCnt = 0; adCnt < 3 * adThr; adCnt++) {
                    // Do the actual adaption
                    if (p_test->adapt(
                            testVal
                            , T(1)
                            , gr
                    )) {
                        // Check that testVal has indeed been adapted
                        BOOST_CHECK_MESSAGE(
                                testVal != oldTestVal
                        , "\n"
                                        << "testVal = " << testVal << "\n"
                                        << "oldTestVal = " << oldTestVal << "\n"
                                        << "adThr = " << adThr << "\n"
                                        << "adCnt = " << adCnt << "\n"
                        );
                        oldTestVal = testVal;

                        // Check that the adaption counter has changed at all, as it should
                        // for adaption thresholds > 1
                        if (adThr > 1) {
                            BOOST_CHECK_MESSAGE(
                                    p_test->getAdaptionCounter() != oldAdaptionCounter
                            , "\n"
                                            << "p_test->getAdaptionCounter() = " << p_test->getAdaptionCounter() << "\n"
                                            << "oldAdaptionCounter = " << oldAdaptionCounter << "\n"
                                            << "adThr = " << adThr << "\n"
                                            << "adCnt = " << adCnt << "\n"
                            );
                            oldAdaptionCounter = p_test->getAdaptionCounter();
                        }

                        // Check that the adaption counter is behaving nicely
                        BOOST_CHECK_MESSAGE(
                                p_test->getAdaptionCounter() < adThr
                        , "\n"
                                        << "p_test->getAdaptionCounter() = " << p_test->getAdaptionCounter() << "\n"
                                        << "adThr = " << adThr << "\n"
                                        << "adCnt = " << adCnt << "\n"
                        );
                    }
                }
            }
        }

        //------------------------------------------------------------------------------

        { // Test that customAdaptions() in derived classes changes a test value on every call
            std::shared_ptr<GAdaptorT<T, fp_type>> p_test = this->clone<GAdaptorT<T, fp_type>>();

            std::size_t nTests = 10000;

            T testVal = T(0);
            T oldTestVal = T(0);
            for (std::size_t i = 0; i < nTests; i++) {
                BOOST_CHECK_NO_THROW(p_test->customAdaptions(
                        testVal
                        , T(1)
                        , gr
                ));
                BOOST_CHECK_MESSAGE(
                        testVal != oldTestVal
                , "\n"
                                << "Found identical values after adaption took place" << "\n"
                                << "testVal = " << testVal << "\n"
                                << "oldTestVal = " << oldTestVal << "\n"
                                << "iteration = " << i << "\n"
                );
                oldTestVal = testVal;
            }
        }

        //------------------------------------------------------------------------------

#else /* GEM_TESTING */ // If this function is called when GEM_TESTING isn't set, throw
        Gem::Common::condnotset("GAdaptorT<>::specificTestsNoFailureExpected_GUnitTests", "GEM_TESTING");
#endif /* GEM_TESTING */
    }

    /***************************************************************************/
    /**
     * Performs self tests that are expected to fail. This is needed for testing purposes.
     */
    void specificTestsFailuresExpected_GUnitTests_() override
    {
#ifdef GEM_TESTING
        using boost::unit_test_framework::test_suite;
        using boost::unit_test_framework::test_case;

        // Call the parent classes' functions
        GObject::specificTestsFailuresExpected_GUnitTests_();

        // Retrieve a random number generator
        Gem::Hap::GRandomT<Gem::Hap::RANDFLAVOURS::RANDOMPROXY> gr;

        //------------------------------------------------------------------------------

        { // Test of GAdaptorT<T, fp_type>::setAdaptionProbability(): Setting a value < 0. should throw
            std::shared_ptr<GAdaptorT<T, fp_type>> p_test = this->clone<GAdaptorT<T, fp_type>>();

            // Setting a probability < 0 should throw
            BOOST_CHECK_THROW(
                    p_test->setAdaptionProbability(-1.);
            , gemfony_exception
            );
        }

        //------------------------------------------------------------------------------

        { // Test of GAdaptorT<T, fp_type>::setAdaptionProbability(): Setting a value > 1. should throw
            std::shared_ptr<GAdaptorT<T, fp_type>> p_test = this->clone<GAdaptorT<T, fp_type>>();

            // Setting a probability > 1 should throw
            BOOST_CHECK_THROW(
                    p_test->setAdaptionProbability(2.);
            , gemfony_exception
            );
        }

        //------------------------------------------------------------------------------

        { // Test of GAdaptorT<T, fp_type>::setAdaptAdaptionProbability(): Setting a value < 0. should throw
            std::shared_ptr<GAdaptorT<T, fp_type>> p_test = this->clone<GAdaptorT<T, fp_type>>();

            // Setting a probability < 0 should throw
            BOOST_CHECK_THROW(
                    p_test->setAdaptAdaptionProbability(-1.);
            , gemfony_exception
            );
        }

        //------------------------------------------------------------------------------

        { // Test of GAdaptorT<T, fp_type>::setAdaptAdaptionProbability(): Setting a value > 1. should throw
            std::shared_ptr<GAdaptorT<T, fp_type>> p_test = this->clone<GAdaptorT<T, fp_type>>();

            // Setting a probability > 1 should throw
            BOOST_CHECK_THROW(
                    p_test->setAdaptAdaptionProbability(2.);
            , gemfony_exception
            );
        }

        //------------------------------------------------------------------------------

#else /* GEM_TESTING */ // If this function is called when GEM_TESTING isn't set, throw
        Gem::Common::condnotset("GAdaptorT<>::specificTestsFailuresExpected_GUnitTests", "GEM_TESTING");
#endif /* GEM_TESTING */
    }

	 /***************************************************************************/
	 // Protected data

	 std::normal_distribution<fp_type> m_normal_distribution; ///< Helps with gauss-type mutation
	 std::uniform_real_distribution<fp_type> m_uniform_real_distribution; ///< Access to uniformly distributed floating point random numbers
	 std::bernoulli_distribution m_weighted_bool; ///< Access to boolean random numbers with a given probability structure

private:
	/***************************************************************************/
	/**
     * Retrieves the id of the adaptor. Purely virtual, must be implemented by the
     * actual adaptors.
     *
     * @return The id of the adaptor
     */
	virtual Gem::Geneva::adaptorId getAdaptorId_() const BASE = 0;

	/* ----------------------------------------------------------------------------------
     * Tested in GBooleanAdaptor
     * Tested in GInt32FlipAdaptor
     * Tested in GInt32GaussAdaptor
     * Tested in GDoubleGaussAdaptor
     * ----------------------------------------------------------------------------------
     */

	/***************************************************************************/
	 /**
	  * Emits a name for this class / object
	  */
	 std::string name_() const override {
		 return std::string("GAdaptorT");
	 }

	 /***************************************************************************/

	 std::uint32_t m_adaptionCounter = 0;///< A local counter
	 std::uint32_t m_adaptionThreshold = DEFAULTADAPTIONTHRESHOLD;///< Specifies after how many adaptions the adaption itself should be adapted
	 fp_type m_adProb = DEFAULTADPROB;///< internal representation of the adaption probability
	 fp_type m_adaptAdProb = DEFAUPTADAPTADPROB;///< The rate, at which adProb_ should be adapted
	 fp_type m_minAdProb = DEFMINADPROB;///< The lower allowed value for adProb_ during variation
	 fp_type m_maxAdProb = DEFMAXADPROB;///< The upper allowed value for adProb_ during variation
	 adaptionMode m_adaptionMode = adaptionMode::WITHPROBABILITY; ///< Whether to adapt always, never, or with a given probability
	 fp_type m_adaptAdaptionProbability = DEFAULTADAPTADAPTIONPROB;///< Influences the likelihood for the adaption of the adaption parameters
	 fp_type m_adProb_reset = m_adProb;///< The value to which adProb_ will be reset if "updateOnStall()" is called
};

/******************************************************************************/
/** @brief Specialization of GAdaptorT<T,fp_type>::adapt(vec) for the T==bool */
template<>
std::size_t GAdaptorT<bool, double>::adapt(std::vector<bool> &, const bool &, Gem::Hap::GRandomBase&);

/******************************************************************************/

} /* namespace Geneva */
} /* namespace Gem */

/******************************************************************************/
/** @brief Mark this class as abstract. This is the content of
 * BOOST_SERIALIZATION_ASSUME_ABSTRACT(T) */

namespace boost {
namespace serialization {
template<typename T, typename fp_type>
struct is_abstract<Gem::Geneva::GAdaptorT<T, fp_type>> : public boost::true_type
{};
template<typename T, typename fp_type>
struct is_abstract<const Gem::Geneva::GAdaptorT<T, fp_type>> : public boost::true_type
{};
}
}

/******************************************************************************/

