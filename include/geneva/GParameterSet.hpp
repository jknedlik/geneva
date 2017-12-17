/**
 * @file GParameterSet.hpp
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

#ifndef GPARAMETERSET_HPP_
#define GPARAMETERSET_HPP_

// Global checks, defines and includes needed for all of Geneva
#include "common/GGlobalDefines.hpp"

// Standard header files go here
#include <map>
#include <typeinfo>
#include <limits>

// Boost header files go here
#include <boost/numeric/conversion/bounds.hpp>
#include <boost/serialization/split_member.hpp>

// Geneva headers go here
#include "common/GExceptions.hpp"
#include "common/GCommonHelperFunctionsT.hpp"
#include "common/GCommonMathHelperFunctions.hpp"
#include "common/GLockVarT.hpp"
#include "common/GLogger.hpp"
#include "common/GCommonHelperFunctionsT.hpp"
#include "common/GStdPtrVectorInterfaceT.hpp"
#include "hap/GRandomT.hpp"
#include "courtier/GProcessingContainerT.hpp"
#include "geneva/GObject.hpp"
#include "geneva/GParameterBase.hpp"
#include "geneva/GenevaHelperFunctionsT.hpp"
#include "geneva/GPersonalityTraits.hpp"
#include "geneva/G_Interface_Mutable.hpp"
#include "geneva/G_Interface_Rateable.hpp"
#include "geneva/GPersonalityTraits.hpp"
#include "geneva/GMultiConstraintT.hpp"

#ifdef GEM_TESTING
#include "geneva/GBooleanObject.hpp"
#include "geneva/GConstrainedInt32Object.hpp"
#include "geneva/GConstrainedDoubleObject.hpp"
#include "geneva/GConstrainedDoubleObjectCollection.hpp"
#include "geneva/GConstrainedInt32ObjectCollection.hpp"
#include "geneva/GDoubleCollection.hpp"
#include "geneva/GDoubleObject.hpp"
#include "geneva/GParameterObjectCollection.hpp"
#include "geneva/GInt32Collection.hpp"
#include "common/GUnitTestFrameworkT.hpp"
#include "hap/GRandomT.hpp"
#endif /* GEM_TESTING */

namespace Gem {

namespace Tests {
class GTestIndividual1; // forward declaration, needed for testing purposes
} /* namespace Tests */

namespace Geneva {

/******************************************************************************/
/**
 * This class implements a collection of GParameterBase objects. It
 * will form the basis of many user-defined individuals.
 */
class GParameterSet
	: public GObject
   , public G_Interface_Mutable
   , public G_Interface_Rateable
   , public Gem::Common::GStdPtrVectorInterfaceT<GParameterBase, GObject>
   , public Gem::Courtier::GProcessingContainerT<GParameterSet, double>
{
	 friend class Gem::Tests::GTestIndividual1; ///< Needed for testing purposes

	 ///////////////////////////////////////////////////////////////////////
	 friend class boost::serialization::access;

	 template<typename Archive>
	 void serialize(Archive & ar, const unsigned int){
		 using boost::serialization::make_nvp;
		 ar
		 & BOOST_SERIALIZATION_BASE_OBJECT_NVP(GObject)
		 & make_nvp("GStdPtrVectorInterfaceT_GParameterBase", boost::serialization::base_object<Gem::Common::GStdPtrVectorInterfaceT<GParameterBase, GObject>>(*this))
		 & make_nvp("GProcessingContainerT_ParameterSet_double", boost::serialization::base_object<Gem::Courtier::GProcessingContainerT<GParameterSet, double>>(*this))
		 & BOOST_SERIALIZATION_NVP(m_perItemCrossOverProbability)
		 & BOOST_SERIALIZATION_NVP(m_n_fitness_criteria)
		 & BOOST_SERIALIZATION_NVP(m_best_past_primary_fitness)
		 & BOOST_SERIALIZATION_NVP(m_n_stalls)
		 & BOOST_SERIALIZATION_NVP(m_dirty_flag)
		 & BOOST_SERIALIZATION_NVP(m_maxmode)
		 & BOOST_SERIALIZATION_NVP(m_assigned_iteration)
		 & BOOST_SERIALIZATION_NVP(m_validity_level)
		 & BOOST_SERIALIZATION_NVP(m_pt_ptr)
		 & BOOST_SERIALIZATION_NVP(m_eval_policy)
		 & BOOST_SERIALIZATION_NVP(m_individual_constraint_ptr)
		 & BOOST_SERIALIZATION_NVP(m_sigmoid_steepness)
		 & BOOST_SERIALIZATION_NVP(m_sigmoid_extremes)
		 & BOOST_SERIALIZATION_NVP(m_marked_as_invalid_by_user)
		 & BOOST_SERIALIZATION_NVP(m_max_unsuccessful_adaptions)
		 & BOOST_SERIALIZATION_NVP(m_max_retries_until_valid)
		 & BOOST_SERIALIZATION_NVP(m_n_adaptions)
		 & BOOST_SERIALIZATION_NVP(m_evaluation_id)
		 & BOOST_SERIALIZATION_NVP(m_current_fitness_vec)
		 & BOOST_SERIALIZATION_NVP(m_worst_known_valids_vec);
	 }
	 ///////////////////////////////////////////////////////////////////////

public:
	 /** @brief The default constructor */
	 G_API_GENEVA GParameterSet() = default;
	 /** @brief Initialization with the number of fitness criteria */
	 explicit G_API_GENEVA GParameterSet(const std::size_t&);
	 /** @brief The copy constructor */
	 G_API_GENEVA GParameterSet(const GParameterSet&);
	 /** @brief The destructor */
	 G_API_GENEVA ~GParameterSet() override = default;

	 /** @brief Searches for compliance with expectations with respect to another object of the same type */
	 G_API_GENEVA void compare(
		 const GObject& // the other object
		 , const Gem::Common::expectation& // the expectation for this object, e.g. equality
		 , const double& // the limit for allowed deviations of floating point types
	 ) const override;

	 /** Swap another object's vector with ours. */
	 void swap(GParameterSet& cp);

	 /** @brief Allows to randomly initialize parameter members */
	 G_API_GENEVA bool randomInit(const activityMode&);

	 /** @brief Specify whether we want to work in maximization (maxMode::MAXIMIZE) or minimization (maxMode::MINIMIZE) mode */
	 G_API_GENEVA void setMaxMode(const maxMode&);

	 /** @brief Transformation of the individual's parameter objects into a boost::property_tree object */
	 G_API_GENEVA void toPropertyTree(pt::ptree&, const std::string& = "parameterset") const BASE;
	 /** @brief Transformation of the individual's parameter objects into a list of comma-separated values */
	 G_API_GENEVA std::string toCSV(bool=false, bool=true, bool=true, bool=true) const;

	 /** @brief Prevent shadowing of std::vector<GParameterBase>::at() */
	 G_API_GENEVA Gem::Common::GStdPtrVectorInterfaceT<GParameterBase, GObject>::reference at(const std::size_t& pos);

	 /** @brief Checks whether this object is better than a given set of evaluations */
	 G_API_GENEVA bool isGoodEnough(const std::vector<double>&);

	 /** @brief Perform a fusion operation between this object and another */
	 virtual G_API_GENEVA std::shared_ptr<GParameterSet> amalgamate(const std::shared_ptr<GParameterSet>&) const BASE;

	 /** @brief Performs a cross-over with another GParameterSet object on a "per item" basis */
	 G_API_GENEVA void perItemCrossOver(const GParameterSet&, const double&);

	 /** @brief Allows to set the "per item" cross-over probability */
	 G_API_GENEVA void setPerItemCrossOverProbability(double);
	 /** @brief Allows to retrieve the "per item" cross-over probability */
	 G_API_GENEVA double getPerItemCrossOverProbability() const;

	 /** @brief Triggers updates of adaptors contained in this object */
	 virtual G_API_GENEVA void updateAdaptorsOnStall(const std::uint32_t&);
	 /** @brief Retrieves information from adaptors with a given property */
	 virtual G_API_GENEVA void queryAdaptor(
		 const std::string& adaptorName
		 , const std::string& property
		 , std::vector<boost::any>& data
	 ) const BASE;

	 /** @brief Retrieves parameters relevant for the evaluation from another GParameterSet */
	 virtual G_API_GENEVA void cannibalize(GParameterSet&);

	 /** @brief The adaption interface */
	 G_API_GENEVA std::size_t adapt() override;

	 /** @brief Returns the raw result of a fitness function with a given id */
	 G_API_GENEVA double fitness(std::size_t = 0) const override;

	 /** @brief Calculate or returns the result of a fitness function with a given id */
	 G_API_GENEVA double fitness(std::size_t, bool, bool) override;
	 /** @brief Calculate or returns the result of a fitness function with a given id */
	 G_API_GENEVA double fitness(std::size_t, bool, bool) const override;

	 /** @brief Returns the transformed result of a fitness function with a given id */
	 G_API_GENEVA double transformedFitness(std::size_t = 0) const override;

	 /** @brief Returns a fitness targetted at optimization algorithms, taking into account maximization and minimization */
	 G_API_GENEVA double minOnly_fitness(std::size_t = 0) const override;

	 /** @brief Returns all raw fitness results in a std::vector */
	 G_API_GENEVA std::vector<double> fitnessVec() const override;
	 /** @brief Returns all raw or transformed results in a std::vector */
	 G_API_GENEVA std::vector<double> fitnessVec(bool) const override;
	 /** @brief Returns all transformed fitness results in a std::vector */
	 G_API_GENEVA std::vector<double> transformedFitnessVec() const override;

	 /** @brief A wrapper for the non-const fitness function, so we can bind to it */
	 G_API_GENEVA double nonConstFitness(std::size_t, bool, bool);
	 /** @brief A wrapper for the const fitness function, so we can bind to it */
	 G_API_GENEVA double constFitness(std::size_t, bool, bool) const;

	 /** @brief Retrieve the current (not necessarily up-to-date) fitness */
	 G_API_GENEVA double getCachedFitness(std::size_t = 0, bool = USETRANSFORMEDFITNESS) const;

	 /** @brief Enforce fitness (re-)calculation */
	 G_API_GENEVA void enforceFitnessUpdate(std::function<std::vector<double>()> =  std::function<std::vector<double>()>());

	 /** @brief Registers a new, secondary result value of the custom fitness calculation */
	 G_API_GENEVA void registerSecondaryResult(const std::size_t&, const double&);
	 /** @brief Determines the overall number of fitness criteria present for this individual */
	 G_API_GENEVA std::size_t getNumberOfFitnessCriteria() const;
	 /** @brief Allows to reset the number of fitness criteria */
	 G_API_GENEVA void setNumberOfFitnessCriteria(std::size_t);
	 /** @brief Determines whether more than one fitness criterion is present for this individual */
	 G_API_GENEVA bool hasMultipleFitnessCriteria() const;

	 /** @brief Checks the worst fitness and updates it when needed */
	 G_API_GENEVA void challengeWorstValidFitness(std::tuple<double, double>&, const std::size_t&);
	 /** @brief Retrieve the fitness tuple at a given evaluation position */
	 G_API_GENEVA std::tuple<double,double> getFitnessTuple(const std::uint32_t& = 0) const;

	 /** @brief Check whether this individual is "clean", i.e neither "dirty" nor has a delayed evaluation */
	 G_API_GENEVA bool isClean() const;
	 /** @brief Check whether the dirty flag is set */
	 G_API_GENEVA bool isDirty() const ;
	 /** @brief Sets the dirtyFlag_ */
	 G_API_GENEVA void setDirtyFlag();
	 /** @brief Checks whether evaluation was delayed */
	 G_API_GENEVA bool evaluationDelayed() const;

	 /** @brief Allows to retrieve the m_maxmode parameter */
	 G_API_GENEVA maxMode getMaxMode() const;

	 /** @brief Retrieves the worst possible evaluation result, depending on whether we are in maximization or minimization mode */
	 virtual G_API_GENEVA double getWorstCase() const BASE;
	 /** @brief Retrieves the best possible evaluation result, depending on whether we are in maximization or minimization mode */
	 virtual G_API_GENEVA double getBestCase() const BASE;

	 /** @brief Retrieves the steepness_ variable (used for the sigmoid transformation) */
	 G_API_GENEVA double getSteepness() const;
	 /** @brief Sets the steepness variable (used for the sigmoid transformation) */
	 G_API_GENEVA void setSteepness(double);

	 /** @brief Retrieves the barrier_ variable (used for the sigmoid transformation) */
	 G_API_GENEVA double getBarrier() const;
	 /** @brief Sets the barrier variable (used for the sigmoid transformation) */
	 G_API_GENEVA void setBarrier(double);

	 /** @brief Sets the maximum number of adaption attempts that may pass without actual modifications */
	 G_API_GENEVA void setMaxUnsuccessfulAdaptions(std::size_t);
	 /** @brief Retrieves the maximum number of adaption attempts that may pass without actual modifications */
	 G_API_GENEVA std::size_t getMaxUnsuccessfulAdaptions() const;

	 /** @brief Set maximum number of retries until a valid individual was found  */
	 G_API_GENEVA void setMaxRetriesUntilValid(std::size_t maxRetriesUntilValid);
	 /** Retrieves the maximum number of retries until a valid individual was found. */
	 G_API_GENEVA std::size_t getMaxRetriesUntilValid() const;

	 /** @brief Retrieves the number of adaptions performed during the last call to adapt() */
	 G_API_GENEVA std::size_t getNAdaptions() const;

	 /** @brief Allows to set the current iteration of the parent optimization algorithm. */
	 G_API_GENEVA void setAssignedIteration(const std::uint32_t&);
	 /** @brief Gives access to the parent optimization algorithm's iteration */
	 G_API_GENEVA std::uint32_t getAssignedIteration() const;

	 /** @brief Allows to specify the number of optimization cycles without improvement of the primary fitness criterion */
	 G_API_GENEVA void setNStalls(const std::uint32_t&);
	 /** @brief Allows to retrieve the number of optimization cycles without improvement of the primary fitness criterion */
	 G_API_GENEVA std::uint32_t getNStalls() const;

	 /** @brief Retrieves an identifier for the current personality of this object */
	 G_API_GENEVA std::string getPersonality() const;

	 /***************************************************************************/
	 /**
	  * Retrieves a parameter of a given type at the specified position.
	  */
	 template <typename val_type>
	 val_type getVarVal(
		 const std::tuple<std::size_t, std::string, std::size_t>& target
	 ) {
		 val_type result = val_type(0);

		 if(typeid(val_type) == typeid(double)) {
			 return boost::numeric_cast<val_type>(boost::any_cast<double>(this->getVarVal("d", target)));
		 } else if(typeid(val_type) == typeid(float)) {
			 return boost::numeric_cast<val_type>(boost::any_cast<float>(this->getVarVal("f", target)));
		 } if(typeid(val_type) == typeid(std::int32_t)) {
			 return boost::numeric_cast<val_type>(boost::any_cast<std::int32_t>(this->getVarVal("i", target)));
		 } if(typeid(val_type) == typeid(bool)) {
			 return boost::numeric_cast<val_type>(boost::any_cast<bool>(this->getVarVal("b", target)));
		 } else {
			 throw gemfony_exception(
				 g_error_streamer(DO_LOG, time_and_place)
					 << "In GParameterSet::getVarVal<>(): Error!" << std::endl
					 << "Received invalid type descriptor " << std::endl
			 );
		 }

		 return result;
	 }

	 /***************************************************************************/
	 /**
	  * The function converts the local personality base pointer to the desired type
	  * and returns it for modification by the corresponding optimization algorithm.
	  * The base algorithms have been declared "friend" of GParameterSet and
	  * can thus access this function. External entities have no need to do so. Note
	  * that this function will only be accessible to the compiler if personality_type
	  * is a derivative of GPersonalityTraits, thanks to the magic of std::enable_if
	  * and type_traits.
	  *
	  * @return A std::shared_ptr converted to the desired target type
	  */
	 template <typename personality_type>
	 std::shared_ptr<personality_type> getPersonalityTraits(
		 typename std::enable_if<std::is_base_of<GPersonalityTraits, personality_type>::value>::type *dummy = nullptr
	 ) {
#ifdef DEBUG
		 // Check that m_pt_ptr actually points somewhere
		 if(!m_pt_ptr) {
			 throw gemfony_exception(
				 g_error_streamer(DO_LOG, time_and_place)
					 << "In GParameterSet::getPersonalityTraits<personality_type>() : Empty personality pointer found" << std::endl
					 << "This should not happen." << std::endl
			 );

			 // Make the compiler happy
			 return std::shared_ptr<personality_type>();
		 }
#endif /* DEBUG */

		 // Does error checks on the conversion internally
		 return Gem::Common::convertSmartPointer<GPersonalityTraits, personality_type>(m_pt_ptr);
	 }

	 /* ----------------------------------------------------------------------------------
	  * Tested in GParameterSet::specificTestsNoFailureExpected_GUnitTests()
	  * Tested in GParameterSet::specificTestsFailureExpected_GUnitTests()
	  * ----------------------------------------------------------------------------------
	  */

	 /***************************************************************************/
	 /** @brief This function returns the current personality traits base pointer */
	 G_API_GENEVA std::shared_ptr<GPersonalityTraits> getPersonalityTraits();

	 /** @brief Sets the current personality of this individual */
	 G_API_GENEVA void setPersonality(
		 std::shared_ptr<GPersonalityTraits>
	 );
	 /** @brief Resets the current personality to PERSONALITY_NONE */
	 G_API_GENEVA void resetPersonality();
	 /** @brief Retrieves the mnemonic used for the optimization of this object */
	 G_API_GENEVA std::string getMnemonic() const;

	 /** @brief Adds local configuration options to a GParserBuilder object */
	 virtual G_API_GENEVA void addConfigurationOptions(
		 Gem::Common::GParserBuilder&
	 ) override;

	 /** @brief Emits a name for this class / object */
	 G_API_GENEVA std::string name() const override;

	 /** @brief Check how valid a given solution is */
	 G_API_GENEVA double getValidityLevel() const;
	 /** @brief Checks whether all constraints were fulfilled */
	 G_API_GENEVA bool constraintsFulfilled() const;
	 /** @brief Allows to register a constraint with this individual */
	 G_API_GENEVA void registerConstraint(std::shared_ptr<GPreEvaluationValidityCheckT<GParameterSet>>);

	 /** @brief Allows to set the policy to use in case this individual represents an invalid solution */
	 G_API_GENEVA void setEvaluationPolicy(evaluationPolicy evalPolicy);
	 /** @brief Allows to retrieve the current policy in case this individual represents an invalid solution */
	 G_API_GENEVA evaluationPolicy getEvaluationPolicy() const;

	 /** @brief Checks whether this is a valid solution; meant to be called for "clean" individuals only */
	 G_API_GENEVA bool isValid() const;
	 /** @brief Checks whether this solution is invalid */
	 G_API_GENEVA bool isInValid() const;

	 /** @brief Allows an optimization algorithm to set the worst known valid evaluation up to the current iteration */
	 G_API_GENEVA void setWorstKnownValid(const std::vector<std::tuple<double, double>>&);
	 /** @brief Allows to retrieve the worst known valid evaluation up to the current iteration, as set by an external optimization algorithm */
	 G_API_GENEVA std::tuple<double, double> getWorstKnownValid(const std::uint32_t&) const;
	 /** @brief Allows to retrieve all worst known valid evaluations up to the current iteration, as set by an external optimization algorithm */
	 G_API_GENEVA std::vector<std::tuple<double, double>> getWorstKnownValids() const;
	 /** @brief Fills the worstKnownValid-vector with best values */
	 G_API_GENEVA void populateWorstKnownValid();

	 /** @brief Triggers an update of the internal evaluation, if necessary */
	 G_API_GENEVA void postEvaluationUpdate();

	 /** @brief Allows to set the globally best known primary fitness */
	 G_API_GENEVA void setBestKnownPrimaryFitness(const std::tuple<double, double>&);
	 /** @brief Retrieves the value of the globally best known primary fitness */
	 G_API_GENEVA std::tuple<double, double> getBestKnownPrimaryFitness() const;

	 /** @brief Retrieve the id assigned to the current evaluation */
	 G_API_GENEVA std::string getCurrentEvaluationID() const;

	 /** @brief Checks whether a new solution is worse then an older solution, depending on the maxMode */
	 virtual G_API_GENEVA bool isWorse(double, const double&) const BASE;
	 /** @brief Checks whether a new solution is better then an older solution, depending on the maxMode */
	 virtual G_API_GENEVA bool isBetter(double, const double&) const BASE;

	 /** @brief Checks whether this object is better than the argument, depending on the maxMode */
	 G_API_GENEVA bool isBetterThan(std::shared_ptr<GParameterSet>) const;
	 /** @brief Checks whether this object is worse than the argument, depending on the maxMode */
	 G_API_GENEVA bool isWorseThan(std::shared_ptr<GParameterSet>) const;

	 /***************************************************************************/
	 /**
	  * Checks if a given position of a std::tuple is better then another,
	  * depending on our maximization mode
	  */
	 template <std::size_t pos>
	 bool isWorse(
		 std::tuple<double, double> newValue
		 , std::tuple<double, double> oldValue
	 ) const {
		 if(this->getMaxMode()) {
			 return (std::get<pos>(newValue) < std::get<pos>(oldValue));
		 } else { // minimization
			 return (std::get<pos>(newValue) > std::get<pos>(oldValue));
		 }
	 }

	 /***************************************************************************/
	 /**
	  * Checks if a given position of a std::tuple is better then another,
	  * depending on our maximization mode
	  */
	 template <std::size_t pos>
	 bool isBetter(
		 std::tuple<double, double> newValue
		 , std::tuple<double, double> oldValue
	 ) const {
		 if(this->getMaxMode()) {
			 return (std::get<pos>(newValue) > std::get<pos>(oldValue));
		 } else { // minimization
			 return (std::get<pos>(newValue) < std::get<pos>(oldValue));
		 }
	 }

	 /***************************************************************************/
	 /**
	  * This function returns a parameter set at a given position of the data set.
	  * Note that this function will only be accessible to the compiler if par_type
	  * is a derivative of GParameterBase, thanks to the magic of std::enable_if and
	  * type_traits.
	  *
	  * @param pos The position in our data array that shall be converted
	  * @return A converted version of the GParameterBase object, as required by the user
	  */
	 template <typename par_type>
	 const std::shared_ptr<par_type> at(
		 const std::size_t& pos
		 , typename std::enable_if<std::is_base_of<GParameterBase, par_type>::value>::type *dummy = nullptr
	 )  const {
		 // Does error checks on the conversion internally
		 return Gem::Common::convertSmartPointer<GParameterBase, par_type>(data.at(pos));
	 }

	 /* ----------------------------------------------------------------------------------
	  * So far untested. See also the second version of the at() function.
	  * ----------------------------------------------------------------------------------
	  */

	 /******************************************************************************/
	 /**
	  * Allows to retrieve a list of all variable names registered with the parameter set
	  */
	 template <typename par_type>
	 std::vector<std::string> getVariableNames() const {
		 std::vector<std::string> varNames;
		 std::map<std::string, std::vector<par_type>> pMap;
		 this->streamline<par_type>(pMap);

		 for(const auto& name: pMap) {
			 varNames.push_back(name.first);
		 }

		 return varNames;
	 }

	 /***************************************************************************/
	 /**
	  * Retrieves an item according to a description provided by the target tuple
	  */
	 template <typename par_type>
	 boost::any getVarItem(
		 const std::tuple<std::size_t, std::string, std::size_t> &target
	 ) {
		 boost::any result;

		 switch(std::get<0>(target)) {
			 //---------------------------------------------------------------------
			 case 0:
			 {
				 std::vector<par_type> vars;
				 this->streamline<par_type>(vars);
				 result = vars.at(std::get<2>(target));
			 }
				 break;

				 //---------------------------------------------------------------------
			 case 1: // var[3]
			 case 2: // var    --> treated as var[0]
			 {
				 std::map<std::string, std::vector<par_type>> varMap;
				 this->streamline<par_type>(varMap);
				 result = (Gem::Common::getMapItem<std::vector<par_type>>(varMap, std::get<1>(target))).at(std::get<2>(target));
			 }
				 break;

				 //---------------------------------------------------------------------
			 default:
			 {
				 throw gemfony_exception(
					 g_error_streamer(DO_LOG, time_and_place)
						 << "In GParameterSet::getVarVal(): Error!" << std::endl
						 << "Got invalid mode setting: " << std::get<0>(target) << std::endl
				 );
			 }
				 break;

				 //---------------------------------------------------------------------
		 }

		 return result;
	 }

	 /***************************************************************************/
	 /**
	  * Retrieve information about the total number of parameters of type
	  * par_type in the individual. Note that the GParameterBase-template
	  * function will throw if this function is called for an unsupported type.
	  *
	  * @param am An enum indicating whether only information about active, inactive or all parameters of this type should be extracted
	  */
	 template <typename par_type>
	 std::size_t countParameters(
		 const activityMode& am = activityMode::DEFAULTACTIVITYMODE
	 ) const {
		 std::size_t result = 0;

		 // Loop over all GParameterBase objects. Each object
		 // will contribute the amount of its parameters of this type
		 // to the result.
		 for(const auto& parm_ptr: *this) {
			 result += parm_ptr->countParameters<par_type>(am);
		 }

		 return result;
	 }

	 /* ----------------------------------------------------------------------------------
	  * So far untested.
	  * ----------------------------------------------------------------------------------
	  */

	 /***************************************************************************/
	 /**
	  * Loops over all GParameterBase objects. Each object will add the
	  * values of its parameters to the vector, if they comply with the
	  * type of the parameters to be stored in the vector.
	  *
	  * @param parVec The vector to which the parameters will be added
	  * @param am An enum indicating whether only information about active, inactive or all parameters of this type should be extracted
	  */
	 template <typename par_type>
	 void streamline(
		 std::vector<par_type>& parVec
		 , const activityMode& am = activityMode::DEFAULTACTIVITYMODE
	 ) const {
		 // Make sure the vector is clean
		 parVec.clear();

		 // Loop over all GParameterBase objects.
		 for(const auto& parm_ptr: *this) {
			 parm_ptr->streamline<par_type>(parVec, am);
		 }
	 }

	 /* ----------------------------------------------------------------------------------
	  * So far untested.
	  * ----------------------------------------------------------------------------------
	  */

	 /***************************************************************************/
	 /**
	  * Loops over all GParameterBase objects. Each object will add its name
	  * and the values of its parameters to the map, if they comply with the
	  * type of the parameters to be stored in the vector.
	  *
	  * @param parVec The map to which the parameters will be added
	  * @param am An enum indicating whether only information about active, inactive or all parameters of this type should be extracted
	  */
	 template <typename par_type>
	 void streamline(
		 std::map<std::string, std::vector<par_type>>& parVec
		 , const activityMode& am = activityMode::DEFAULTACTIVITYMODE
	 ) const {
		 // Make sure the vector is clean
		 parVec.clear();

		 // Loop over all GParameterBase objects.
		 for(const auto& parm_ptr: *this) {
			 parm_ptr->streamline<par_type>(parVec, am);
		 }
	 }

	 /* ----------------------------------------------------------------------------------
	  * So far untested.
	  * ----------------------------------------------------------------------------------
	  */

	 /***************************************************************************/
	 /**
	  * Assigns values from a std::vector to the parameters in the collection
	  *
	  * @param parVec A vector of values, to be assigned to be added to GParameterBase derivatives
	  * @param am An enum indicating whether only information about active, inactive or all parameters of this type should be assigned
	  */
	 template <typename par_type>
	 void assignValueVector(
		 const std::vector<par_type>& parVec
		 , const activityMode& am = activityMode::DEFAULTACTIVITYMODE
	 ) {
#ifdef DEBUG
		 if(countParameters<par_type>() != parVec.size()) {
			 throw gemfony_exception(
				 g_error_streamer(DO_LOG, time_and_place)
					 << "In GParameterSet::assignValueVector(const std::vector<pat_type>&):" << std::endl
					 << "Sizes don't match: " <<  countParameters<par_type>() << " / " << parVec.size() << std::endl
			 );
		 }
#endif /* DEBUG */

		 // Start assignment at the beginning of parVec
		 std::size_t pos = 0;

		 // Loop over all GParameterBase objects. Each object will extract the relevant
		 // parameters and increment the position counter as required.
		 for(const auto& parm_ptr: *this) {
			 parm_ptr->assignValueVector<par_type>(parVec, pos, am);
		 }

		 // As we have modified our internal data sets, make sure the dirty flag is set
		 GParameterSet::setDirtyFlag();
	 }

	 /***************************************************************************/
	 /**
	  * Assigns values from a std::map<std::string, std::vector<par_type>> to the parameters in the collection
	  *
	  * @param parMap A map of values, to be assigned to be added to GParameterBase derivatives
	  * @param am An enum indicating whether only information about active, inactive or all parameters of this type should be assigned
	  */
	 template <typename par_type>
	 void assignValueVectors(
		 const std::map<std::string, std::vector<par_type>>& parMap
		 , const activityMode& am = activityMode::DEFAULTACTIVITYMODE
	 ) {
		 // Loop over all GParameterBase objects. Each object will extract the relevant parameters
		 for(const auto& parm_ptr: *this) {
			 parm_ptr->assignValueVectors<par_type>(parMap, am);
		 }

		 // As we have modified our internal data sets, make sure the dirty flag is set
		 GParameterSet::setDirtyFlag();
	 }

	 /***************************************************************************/
	 /**
	  * Loops over all GParameterBase objects. Each object will add the
	  * lower and upper boundaries of its parameters to the vector, if
	  * they comply with the type of the parameters to be stored in the
	  * vector.
	  *
	  * @param lBndVec The vector to which the lower boundaries will be added
	  * @param uBndVec The vector to which the upper boundaries will be added
	  * @param am An enum indicating whether only information about active, inactive or all parameters of this type should be extracted
	  */
	 template <typename par_type>
	 void boundaries(
		 std::vector<par_type>& lBndVec
		 , std::vector<par_type>& uBndVec
		 , const activityMode& am = activityMode::DEFAULTACTIVITYMODE
	 ) const {
		 // Make sure the vectors are clean
		 lBndVec.clear();
		 uBndVec.clear();

		 // Loop over all GParameterBase objects.
		 for(const auto& parm_ptr: *this) {
			 parm_ptr->boundaries<par_type>(lBndVec, uBndVec, am);
		 }
	 }

	 /* ----------------------------------------------------------------------------------
	  * So far untested.
	  * ----------------------------------------------------------------------------------
	  */

	 /***************************************************************************/
	 /**
	  * Multiplication with a random value in a given range
	  */
	 template <typename par_type>
	 void multiplyByRandom(
		 const par_type& min
		 , const par_type& max
		 , const activityMode& am
	 ) {
		 // Loop over all GParameterBase objects.
		 for(auto& parm_ptr: *this) {
			 parm_ptr->multiplyByRandom<par_type>(min, max, am, m_gr);
		 }
	 }

	 /***************************************************************************/
	 /**
	  * Multiplication with a random value in the range [0, 1[
	  */
	 template <typename par_type>
	 void multiplyByRandom(
		 const activityMode& am
	 ) {
		 // Loop over all GParameterBase objects.
		 for(auto& parm_ptr: *this) {
			 parm_ptr->multiplyByRandom<par_type>(am, m_gr);
		 }
	 }

	 /***************************************************************************/
	 /**
	  * Multiplication with a constant value
	  */
	 template <typename par_type>
	 void multiplyBy(
		 const par_type& val
		 , const activityMode& am
	 ) {
		 // Loop over all GParameterBase objects.
		 for(auto& parm_ptr: *this) {
			 parm_ptr->multiplyBy<par_type>(val, am);
		 }
	 }

	 /***************************************************************************/
	 /**
	  * Initializes all parameters of a given type with a constant value
	  */
	 template <typename par_type>
	 void fixedValueInit(
		 const par_type& val
		 , const activityMode& am
	 ) {
		 // Loop over all GParameterBase objects.
		 GParameterSet::iterator it;
		 for(it=this->begin(); it!=this->end(); ++it) {
			 (*it)->fixedValueInit<par_type>(val, am);
		 }
	 }

	 /***************************************************************************/
	 /**
	  * Adds the parameters of another GParameterSet object to this one
	  */
	 template <typename par_type>
	 void add(
		 const std::shared_ptr<GParameterSet>& p
		 , const activityMode& am
	 ) {
		 GParameterSet::iterator it;
		 GParameterSet::const_iterator cit;

		 // Note that the GParameterBase objects need to accept a
		 // std::shared_ptr<GParameterBase>, contrary to the calling conventions
		 // of this function.
		 for(it=this->begin(), cit=p->begin(); it!=this->end(); ++it, ++cit) {
			 (*it)->add<par_type>(*cit, am);
		 }
	 }

	 /***************************************************************************/
	 /**
	  * Subtracts the parameters of another GParameterSet object from this one
	  */
	 template <typename par_type>
	 void subtract(
		 const std::shared_ptr<GParameterSet>& p
		 , const activityMode& am
	 ) {
		 GParameterSet::iterator it;
		 GParameterSet::const_iterator cit;

		 // Note that the GParameterBase objects need to accept a
		 // std::shared_ptr<GParameterBase>, contrary to the calling conventions
		 // of this function.
		 for(it=this->begin(), cit=p->begin(); it!=this->end(); ++it, ++cit) {
			 (*it)->subtract<par_type>(*cit, am);
		 }
	 }

	 /***************************************************************************/
	 // Deleted functions

	 explicit G_API_GENEVA GParameterSet(const float&) = delete; ///< Intentionally undefined
	 explicit G_API_GENEVA GParameterSet(const double&) = delete; ///< Intentionally undefined

protected:
	 /***************************************************************************/
	 /**
	  * A random number generator. Note that the actual calculation is
	  * done in a random number proxy / factory
	  */
	 Gem::Hap::GRandomT<Gem::Hap::RANDFLAVOURS::RANDOMPROXY> m_gr;

/***************************************************************************/
	 /**
	  * Re-implementation of a corresponding function in GStdPtrVectorInterface.
	  * Make the vector wrapper purely virtual allows the compiler to perform
	  * further optimizations.
	  */
	 void dummyFunction() override { /* nothing */ }

	 /***************************************************************************/
	 /** @brief Do the required processing for this object */
	 G_API_GENEVA void process_() override;

	 /** @brief Allows to give an indication of the processing result (if any); may not throw. */
	 G_API_GENEVA double get_processing_result() const noexcept override;

	 /** @brief Loads the data of another GObject */
	 G_API_GENEVA void load_(const GObject*) override;

	 /** @brief Random initialization */
	 virtual G_API_GENEVA bool randomInit_(const activityMode&) BASE;

	 /* @brief The actual adaption operations. */
	 virtual G_API_GENEVA std::size_t customAdaptions() BASE ;

	 /** @brief The fitness calculation for the main quality criterion takes place here */
	 virtual G_API_GENEVA double fitnessCalculation() BASE = 0;
	 /** @brief Sets the fitness to a given set of values and clears the dirty flag */
	 G_API_GENEVA void setFitness_(const std::vector<double>&);

	 /** @brief Sets the dirtyFlag_ to any desired value */
	 G_API_GENEVA boost::logic::tribool setDirtyFlag(const boost::logic::tribool&) ;

	 /** @brief Combines secondary evaluation results by adding the individual results */
	 G_API_GENEVA double sumCombiner() const;
	 /** @brief Combines secondary evaluation results by adding the absolute values of individual results */
	 G_API_GENEVA double fabsSumCombiner() const;
	 /** @brief Combines secondary evaluation results by calculating the square root of the squared sum */
	 G_API_GENEVA double squaredSumCombiner() const;
	 /** @brief Combines secondary evaluation results by calculation the square root of the weighed squared sum */
	 G_API_GENEVA double weighedSquaredSumCombiner(const std::vector<double>&) const;

	 /** @brief Allows users to mark this solution as invalid in derived classes (usually from within the evaluation function) */
	 G_API_GENEVA void markAsInvalid();
	 /** @brief Allows to check whether this solution was marked as invalid */
	 G_API_GENEVA bool markedAsInvalidByUser() const;

	 /** @brief Checks whether this solution has been rated to be valid; meant to be called by internal functions only */
	 G_API_GENEVA bool parameterSetFulfillsConstraints(double&) const;

private:
	 /***************************************************************************/
	 /** @brief Retrieves a parameter of a given type at the specified position */
	 G_API_GENEVA boost::any getVarVal(
		 const std::string&
		 , const std::tuple<std::size_t, std::string, std::size_t>& target
	 );

	 /***************************************************************************/
	 /** @brief Creates a deep clone of this object */
	 G_API_GENEVA  GObject* clone_() const override = 0;

	 /** @brief Checks whether all results are at the worst possible value */
	 bool allRawResultsAtWorst() const;

	 /***************************************************************************/
	 // Data

	 /** @brief Uniformly distributed integer random numbers */
	 std::uniform_int_distribution<std::size_t> m_uniform_int;

	 double m_perItemCrossOverProbability = DEFAULTPERITEMEXCHANGELIKELIHOOD; ///< A likelihood for "per item" cross-over operations to be performed

	 /** @brief The total number of fitness criteria */
	 std::size_t m_n_fitness_criteria = 1;
	 /** @brief Holds this object's internal, raw and transformed fitness */
	 std::vector<std::tuple<double, double>> m_current_fitness_vec = std::vector<std::tuple<double, double>>(m_n_fitness_criteria);

	 /** @brief The worst known evaluation up to the current iteration */
	 std::vector<std::tuple<double, double>> m_worst_known_valids_vec = std::vector<std::tuple<double, double>>(m_n_fitness_criteria);
	 /** @brief Indicates whether the user has marked this solution as invalid inside of the evaluation function */
	 Gem::Common::GLockVarT<bool> m_marked_as_invalid_by_user{OE_NOT_MARKED_AS_INVALID};

	 /** @brief Holds the globally best known primary fitness of all individuals */
	 std::tuple<double, double> m_best_past_primary_fitness{std::make_tuple(0., 0.)};
	 /** @brief The number of stalls of the primary fitness criterion in the entire set of individuals */
	 std::uint32_t m_n_stalls = 0;
	 /** @brief Internal representation of the adaption status of this object */
	 boost::logic::tribool m_dirty_flag = true; // boost::logic::indeterminate refers to "delayed evaluation"
	 /** @brief Indicates whether we are using maximization or minimization mode */
	 maxMode m_maxmode = maxMode::MINIMIZE;
	 /** @brief The iteration of the parent algorithm's optimization cycle */
	 std::uint32_t m_assigned_iteration = 0;
	 /** @brief Indicates how valid a given solution is */
	 double m_validity_level = 0.;
	 /** @brief Holds the actual personality information */
	 std::shared_ptr<GPersonalityTraits> m_pt_ptr;

	 /** @brief Specifies what to do when the individual is marked as invalid */
	 evaluationPolicy m_eval_policy = Gem::Geneva::evaluationPolicy::USESIMPLEEVALUATION;
	 /** @brief Determines the "steepness" of a sigmoid function used by optimization algorithms */
	 double m_sigmoid_steepness = Gem::Geneva::FITNESSSIGMOIDSTEEPNESS;
	 /** @brief Determines the extreme values of a sigmoid function used by optimization algorithms */
	 double m_sigmoid_extremes = Gem::Geneva::WORSTALLOWEDVALIDFITNESS;

	 /** @brief A constraint-check to be applied to one or more components of this individual */
	 std::shared_ptr<GPreEvaluationValidityCheckT<GParameterSet>> m_individual_constraint_ptr;

	 std::size_t m_max_unsuccessful_adaptions = Gem::Geneva::DEFMAXUNSUCCESSFULADAPTIONS; ///< The maximum number of calls to customAdaptions() in a row without actual modifications
	 std::size_t m_max_retries_until_valid = Gem::Geneva::DEFMAXRETRIESUNTILVALID; ///< The maximum number an adaption of an individual should be performed until a valid parameter set was found
	 std::size_t m_n_adaptions = 0; ///< Stores the actual number of adaptions after a call to "adapt()"

	 /** @brief A unique id that is assigned to an evaluation */
	 std::string m_evaluation_id = "empty";

public:
	 /***************************************************************************/
	 /** @brief Applies modifications to this object. This is needed for testing purposes */
	 G_API_GENEVA bool modify_GUnitTests() override;
	 /** @brief Performs self tests that are expected to succeed. This is needed for testing purposes */
	 G_API_GENEVA void specificTestsNoFailureExpected_GUnitTests() override;
	 /** @brief Performs self tests that are expected to fail. This is needed for testing purposes */
	 G_API_GENEVA void specificTestsFailuresExpected_GUnitTests() override;
	 /***************************************************************************/
};

} /* namespace Geneva */
} /* namespace Gem */

/******************************************************************************/
/**
 * @brief Needed for Boost.Serialization
 */
BOOST_CLASS_EXPORT_KEY(Gem::Geneva::GParameterSet)

/******************************************************************************/

#endif /* GPARAMETERSET_HPP_ */
