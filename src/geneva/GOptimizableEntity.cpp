/**
 * @file GOptimizableEntity.cpp
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
 * but WITHOUT ANY WARRANTY without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with the Geneva library. If not, see <http://www.gnu.org/licenses/>.
 *
 * For further information on Gemfony scientific and Geneva, visit
 * http://www.gemfony.eu .
 */

#include "geneva/GOptimizableEntity.hpp"

namespace Gem {
namespace Geneva {

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/**
 * The default constructor. Note that the usage of this constructor implies
 * that only a single fitness criterion is available! There is no way to
 * reset that number later externally. Some variables are initialized in the
 * class body.
 */
GOptimizableEntity::GOptimizableEntity()
	: GOptimizableEntity(1 /* nFitnessCriteria_ */)
{ /* nothing */ }

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/**
 * Initialization with the number of fitness criteria. Note that individuals
 * wishing to use this functionality need to directly or indirectly call this
 * constructor, as this is the only way to set the number of criteria. Some
 * variables are initialized in the class body.
 */
GOptimizableEntity::GOptimizableEntity(const std::size_t &nFitnessCriteria)
	: GMutableI()
	, GRateableI()
	, GObject()
	, nFitnessCriteria_(nFitnessCriteria ? nFitnessCriteria : 1) // Note: Thus function will silently assign a minimum of 1 to nFitnessCriteria_
	, currentFitnessVec_(nFitnessCriteria_)
	, worstKnownValids_(nFitnessCriteria_)
	, markedAsInvalidByUser_(OE_NOT_MARKED_AS_INVALID) // Means: the object has not been marked as invalid by the user in the evaluation function
	, bestPastPrimaryFitness_(std::make_tuple(0., 0.))
{ /* nothing */ }

/******************************************************************************/
/**
 * The standard copy constructor.
 *
 * @param cp A copy of another GOptimizableEntity object
 */
GOptimizableEntity::GOptimizableEntity(const GOptimizableEntity &cp)
	: GMutableI(cp)
	, GRateableI(cp)
	, GObject(cp)
	, nFitnessCriteria_(cp.nFitnessCriteria_)
	, currentFitnessVec_(cp.currentFitnessVec_)
	, worstKnownValids_(cp.worstKnownValids_)
	, markedAsInvalidByUser_(cp.markedAsInvalidByUser_)
	, bestPastPrimaryFitness_(cp.bestPastPrimaryFitness_)
	, nStalls_(cp.nStalls_)
	, dirtyFlag_(cp.dirtyFlag_)
	, maximize_(cp.maximize_)
	, assignedIteration_(cp.assignedIteration_)
	, validityLevel_(cp.validityLevel_)
	, evalPolicy_(cp.evalPolicy_)
	, steepness_(cp.steepness_)
	, barrier_(cp.barrier_)
	, maxUnsuccessfulAdaptions_(cp.maxUnsuccessfulAdaptions_)
	, maxRetriesUntilValid_(cp.maxRetriesUntilValid_)
	, nAdaptions_(cp.nAdaptions_)
	, evaluationID_(cp.evaluationID_)
{
	// Copy the personality pointer over
	Gem::Common::copyCloneableSmartPointer(cp.pt_ptr_, pt_ptr_);

	// Make sure any constraints are copied over
	Gem::Common::copyCloneableSmartPointer(cp.individualConstraint_, individualConstraint_);
}

/******************************************************************************/
/**
 * The standard destructor.
 */
GOptimizableEntity::~GOptimizableEntity() { /* nothing */ }

/***************************************************************************/
/**
 * The standard assignment operator
 */
const GOptimizableEntity &GOptimizableEntity::operator=(const GOptimizableEntity &cp) {
	this->load_(&cp);
	return *this;
}

/******************************************************************************/
/**
 * Checks for equality with another GOptimizableEntity object
 *
 * @param  cp A constant reference to another GOptimizableEntity object
 * @return A boolean indicating whether both objects are equal
 */
bool GOptimizableEntity::operator==(const GOptimizableEntity &cp) const {
	using namespace Gem::Common;
	try {
		this->compare(cp, Gem::Common::expectation::CE_EQUALITY, CE_DEF_SIMILARITY_DIFFERENCE);
		return true;
	} catch (g_expectation_violation &) {
		return false;
	}
}

/******************************************************************************/
/**
 * Checks for inequality with another GOptimizableEntity object
 *
 * @param  cp A constant reference to another GOptimizableEntity object
 * @return A boolean indicating whether both objects are inequal
 */
bool GOptimizableEntity::operator!=(const GOptimizableEntity &cp) const {
	using namespace Gem::Common;
	try {
		this->compare(cp, Gem::Common::expectation::CE_INEQUALITY, CE_DEF_SIMILARITY_DIFFERENCE);
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
void GOptimizableEntity::compare(
	const GObject &cp, const Gem::Common::expectation &e, const double &limit
) const {
	using namespace Gem::Common;

	// Check that we are dealing with a GOptimizableEntity reference independent of this object and convert the pointer
	const GOptimizableEntity *p_load = Gem::Common::g_convert_and_compare<GObject, GOptimizableEntity>(cp, this);

	GToken token("GOptimizableEntity", e);

	// Compare our parent data ...
	Gem::Common::compare_base<GObject>(IDENTITY(*this, *p_load), token);

	// ... and then the local data
	compare_t(IDENTITY(nFitnessCriteria_, p_load->nFitnessCriteria_), token);
	compare_t(IDENTITY(currentFitnessVec_, p_load->currentFitnessVec_), token);
	compare_t(IDENTITY(worstKnownValids_, p_load->worstKnownValids_), token);
	compare_t(IDENTITY(markedAsInvalidByUser_, p_load->markedAsInvalidByUser_), token);
	compare_t(IDENTITY(bestPastPrimaryFitness_, p_load->bestPastPrimaryFitness_), token);
	compare_t(IDENTITY(nStalls_, p_load->nStalls_), token);
	compare_t(IDENTITY(dirtyFlag_, p_load->dirtyFlag_), token);
	compare_t(IDENTITY(maximize_, p_load->maximize_), token);
	compare_t(IDENTITY(assignedIteration_, p_load->assignedIteration_), token);
	compare_t(IDENTITY(validityLevel_, p_load->validityLevel_), token);
	compare_t(IDENTITY(evalPolicy_, p_load->evalPolicy_), token);
	compare_t(IDENTITY(pt_ptr_, p_load->pt_ptr_), token);
	compare_t(IDENTITY(individualConstraint_, p_load->individualConstraint_), token);
	compare_t(IDENTITY(steepness_, p_load->steepness_), token);
	compare_t(IDENTITY(barrier_, p_load->barrier_), token);
	compare_t(IDENTITY(maxUnsuccessfulAdaptions_, p_load->maxUnsuccessfulAdaptions_), token);
	compare_t(IDENTITY(maxRetriesUntilValid_, p_load->maxRetriesUntilValid_), token);
	compare_t(IDENTITY(nAdaptions_, p_load->nAdaptions_), token);
	compare_t(IDENTITY(evaluationID_, p_load->evaluationID_), token);

	// React on deviations from the expectation
	token.evaluate();
}

/******************************************************************************/
/**
 * Emits a name for this class / object
 */
std::string GOptimizableEntity::name() const {
	return std::string("GOptimizableEntity");
}

/******************************************************************************/
/**
 * Allows to register a constraint with this individual. Note that the constraint
 * object will be cloned.
 */
void GOptimizableEntity::registerConstraint(
	std::shared_ptr < GPreEvaluationValidityCheckT<GOptimizableEntity>> c_ptr
) {
	if (!c_ptr) {
		glogger
		<< "In GOptimizableEntity::registerConstraint(): Error!" << std::endl
		<< "Tried to register empty constraint object" << std::endl
		<< GEXCEPTION;
	}

	// We store clones, so individual objects do not share the same object
	individualConstraint_ = c_ptr->GObject::clone<GPreEvaluationValidityCheckT<GOptimizableEntity>>();
}

/******************************************************************************/
/**
 * Allows to set the policy to use in case this individual represents an invalid solution
 */
void GOptimizableEntity::setEvaluationPolicy(evaluationPolicy evalPolicy) {
	evalPolicy_ = evalPolicy;
}

/******************************************************************************/
/**
 * Allows to retrieve the current policy in case this individual represents an invalid solution
 */
evaluationPolicy GOptimizableEntity::getEvaluationPolicy() const {
	return evalPolicy_;
}

/******************************************************************************/
/**
 * Loads the data of another GOptimizableEntity, camouflaged as a GObject
 *
 * @param cp A copy of another GOptimizableEntity object, camouflaged as a GObject
 */
void GOptimizableEntity::load_(const GObject *cp) {
	// Check that we are dealing with a GOptimizableEntity reference independent of this object and convert the pointer
	const GOptimizableEntity *p_load = Gem::Common::g_convert_and_compare<GObject, GOptimizableEntity>(cp, this);

	// Load the parent class'es data
	GObject::load_(cp);

	// Then load our local data
	nFitnessCriteria_ = p_load->nFitnessCriteria_;
	currentFitnessVec_ = p_load->currentFitnessVec_;
	worstKnownValids_ = p_load->worstKnownValids_;
	markedAsInvalidByUser_.setValue((p_load->markedAsInvalidByUser_).value());
	bestPastPrimaryFitness_ = p_load->bestPastPrimaryFitness_;
	nStalls_ = p_load->nStalls_;
	dirtyFlag_ = p_load->dirtyFlag_;
	maximize_ = p_load->maximize_;
	assignedIteration_ = p_load->assignedIteration_;
	validityLevel_ = p_load->validityLevel_;
	evalPolicy_ = p_load->evalPolicy_;
	steepness_ = p_load->steepness_;
	barrier_ = p_load->barrier_;
	maxUnsuccessfulAdaptions_ = p_load->maxUnsuccessfulAdaptions_;
	maxRetriesUntilValid_ = p_load->maxRetriesUntilValid_;
	nAdaptions_ = p_load->nAdaptions_;
	evaluationID_ = p_load->evaluationID_;

	Gem::Common::copyCloneableSmartPointer(p_load->pt_ptr_, pt_ptr_);
	Gem::Common::copyCloneableSmartPointer(p_load->individualConstraint_, individualConstraint_);
}

/******************************************************************************/
/**
 * The adaption interface. Triggers adaption of the individual, using each parameter object's
 * adaptor. Sets the dirty flag, as the parameters have been changed. This facility is mostly
 * used in Evolutionary Algorithms and Simulated Annealing. Other algorithms, such as
 * PSO and Gradient Descents, may choose to change parameters directly. Adaptions will be performed
 * until actual changes were done to the object AND a valid parameter set was found.
 */
std::size_t GOptimizableEntity::adapt() {
	std::size_t nAdaptionAttempts = 0;
	std::size_t nAdaptions = 0; // This is a measure of the "effective" adaption probability
	std::size_t nInvalidAdaptions = 0;
	double validity = 0;

	// Perform adaptions until a valid solution was find. In the context
	// of evolutionary algorithms, this process is indeed equivalent to
	// a larger population, if invalid solutions were produced. The downside
	// may be, that the algorithm moves closer to MUPLUSNU. Thus, if you find
	// yourself stuck in local optima too often, consider setting maxRetriesUntilValid_
	// to 0, using the appropriate function.
	while (true) {
		// Make sure at least one modification is performed. E.g., for low
		// adaption probabilities combined with few parameters, it may happen
		// otherwise that individuals remain unchanged after a call to adapt()
		while (true) { // Try again if no adaption has taken place
			// Perform the actual adaption
			nAdaptions = this->customAdaptions();

			// Terminate, if at least one adaption was performed
			if (nAdaptions > 0) {
				break;
			}

			// Terminate, if the maximum number of adaptions has been exceeded
			if (maxUnsuccessfulAdaptions_ > 0 && ++nAdaptionAttempts > maxUnsuccessfulAdaptions_) {
				break;
			}
		}

		if (
			this->parameterSetFulfillsConstraints(validity)
			|| ++nInvalidAdaptions > maxRetriesUntilValid_
			) {
			break;
		}
	}

	// Make sure the individual is re-evaluated when fitness(...) is called next time
	GOptimizableEntity::setDirtyFlag();

	// Store the number of adaptions for later use
	nAdaptions_ = nAdaptions;

	return nAdaptions;
}

/* ----------------------------------------------------------------------------------
 * Tested in GTestIndividual1::specificTestsNoFailureExpected_GUnitTests()
 * ----------------------------------------------------------------------------------
 */

/******************************************************************************/
/**
 * Returns the cached result of the fitness function with id 0. This function
 * will always return the raw fitness, as it is likely the one called by users
 * directly -- they will expect untransformed values. This is the const version
 */
double GOptimizableEntity::fitness() const {
	return fitness(0, PREVENTREEVALUATION, USERAWFITNESS);
}

/******************************************************************************/
/**
 * Calculate or returns the result of a fitness function with a given id.This
 * function will always return the raw fitness, as it is likely the one called by users
 * directly -- they will expect untransformed values. This is the const version
 */
double GOptimizableEntity::fitness(const std::size_t &id) const {
	return fitness(id, PREVENTREEVALUATION, USERAWFITNESS);
}

/******************************************************************************/
/**
 * Returns the transformed result of the fitness function with id 0
 */
double GOptimizableEntity::transformedFitness() const {
	return this->transformedFitness(0);
}

/******************************************************************************/
/**
 * Returns the transformed result of the fitness function with id 0
 */
double GOptimizableEntity::transformedFitness(const std::size_t &id) const {
	return fitness(id, PREVENTREEVALUATION, USETRANSFORMEDFITNESS);
}

/******************************************************************************/
/**
 * Returns a fitness targetted at optimization algorithms, taking into account maximization and minimization
 */
double GOptimizableEntity::minOnly_fitness() const {
	return this->minOnly_fitness(0);
}

/******************************************************************************/
/**
 * Returns a fitness targetted at optimization algorithms, taking into account maximization and minimization
 */
double GOptimizableEntity::minOnly_fitness(const std::size_t &id) const {
	double f = fitness(id, PREVENTREEVALUATION, USETRANSFORMEDFITNESS);

	if (true == this->getMaxMode()) { // Negation will transform maximization problems into minimization problems
		if (boost::numeric::bounds<double>::highest() == f) {
			return boost::numeric::bounds<double>::lowest();
		} else if (boost::numeric::bounds<double>::lowest() == f) {
			return boost::numeric::bounds<double>::highest();
		} else {
			return -f;
		}
	} else {
		return f;
	}
}

/******************************************************************************/
/**
 * Returns all raw fitness results in a std::vector
 */
std::vector<double> GOptimizableEntity::fitnessVec() const {
	std::size_t nFitnessCriteria = this->getNumberOfFitnessCriteria();
	std::vector<double> resultVec;

	for (std::size_t i = 0; i < nFitnessCriteria; i++) {
		resultVec.push_back(this->fitness(i));
	}

	return resultVec;
}

/******************************************************************************/
/**
 * Returns all raw or transformed fitness results in a std::vector
 */
std::vector<double> GOptimizableEntity::fitnessVec(bool useRawFitness) const {
	std::size_t nFitnessCriteria = this->getNumberOfFitnessCriteria();
	std::vector<double> resultVec;

	for (std::size_t i = 0; i < nFitnessCriteria; i++) {
		if (useRawFitness) {
			resultVec.push_back(this->fitness(i));
		} else {
			resultVec.push_back(this->transformedFitness(i));
		}
	}

	return resultVec;
}

/******************************************************************************/
/**
 * Returns all transformed fitness results in a std::vector
 */
std::vector<double> GOptimizableEntity::transformedFitnessVec() const {
	std::size_t nFitnessCriteria = this->getNumberOfFitnessCriteria();
	std::vector<double> resultVec;

	for (std::size_t i = 0; i < nFitnessCriteria; i++) {
		resultVec.push_back(this->transformedFitness(i));
	}

	return resultVec;
}

/******************************************************************************/
/**
 * A wrapper for the non-const fitness function, so we can bind to it. It is
 * needed as bind cannot distinguish between the non-const and const
 * overload of the fitness() function.
 */
double GOptimizableEntity::nonConstFitness(
	const std::size_t &id, bool reevaluationAllowed, bool useTransformedFitness
) {
	return this->fitness(id, reevaluationAllowed, useTransformedFitness);
}

/******************************************************************************/
/**
 * A wrapper for the const fitness function, so we can bind to it. It is
 * needed as bind cannot distinguish between the non-const and const
 * overload of the fitness() function.
 */
double GOptimizableEntity::constFitness(
	const std::size_t &id, bool reevaluationAllowed, bool useTransformedFitness
) const {
	return this->fitness(id, reevaluationAllowed, useTransformedFitness);
}

/******************************************************************************/
/**
 * Returns the last known fitness calculations of this object. Re-calculation
 * of the fitness is triggered, if the "dirty flag" is set, unless this is the server mode.
 * By means of supplying an id it is possible to distinguish between different target functions.
 * 0 denotes the main fitness criterion. The user can specify whether he/she is interested
 * in the transformed or the raw fitness value.
 *
 * @param id The id of the fitness criterion
 * @param reevaluationAllowed Explicit permission to re-evaluate the individual
 * @param useTransformedFitness Whether the transformed or the raw fitness should be returned
 * @return The fitness of this individual
 */
double GOptimizableEntity::fitness(
	const std::size_t &id, bool reevaluationAllowed, bool useTransformedFitness
) {
	// Check whether we need to recalculate the fitness
	if (true == dirtyFlag_) {
		if (reevaluationAllowed) {
			this->enforceFitnessUpdate();

#ifdef DEBUG
         // Check if the dirty flag is still set. This should only happen in special cases
         if(true==dirtyFlag_) { // Note that dirtyFlag_ may also assume the state boost::logic::indeterminate, if evaluation was delayed
            glogger
            << "In GOptimizableEntity::fitness(...): Error!" << std::endl
            << "Dirty flag is still set in a location where it shouldn't be" << std::endl
            << GEXCEPTION;
         }
#endif /* DEBUG */
		} else {
			glogger
			<< "In GOptimizableEntity::fitness():" << std::endl
			<< "Tried to perform re-evaluation when this action was not allowed" << std::endl
			<< GEXCEPTION;
		}
	}

	// Return the desired result -- there should be no situation where the dirtyFlag is still set
	return getCachedFitness(id, useTransformedFitness);
}

/* ----------------------------------------------------------------------------------
 * Tested in GTestIndividual1::specificTestsNoFailureExpected_GUnitTests()
 * Test for throw in serverMode tested in GTestIndividual1::specificTestsFailuresExpected_GUnitTests()
 * ----------------------------------------------------------------------------------
 */

/******************************************************************************/
/**
 * Returns the last known fitness calculations of this object. This is the
 * const version of the general fitness() function, which consequently cannot
 * trigger re-evaluation, if the individual is dirty. Hence the function will
 * throw, when it is called on a dirty individual (unless we use the
 * USEWORSTKNOWNVALIDFORINVALID policy)
 *
 * @param id The id of the fitness criterion
 * @param reevaluationAllowed Explicit permission to re-evaluate the individual
 * @param useTransformedFitness Whether the transformed or the raw fitness should be returned
 * @return The fitness of this individual
 */
double GOptimizableEntity::fitness(
	const std::size_t &id, bool reevaluationAllowed, bool useTransformedFitness
) const {
#ifdef DEBUG
   // This function should only be called for clean (i.e. evaluated) individuals
   if(!this->isClean()) {
      glogger
      << "In GOptimizableEntity::fitness(...) const: Error!" << std::endl
      << "Dirty flag is still set in a location where it shouldn't be" << std::endl
      << GEXCEPTION;
   }
#endif /* DEBUG */

	// Return the desired result -- there should be no situation where the dirtyFlag is still set
	return getCachedFitness(id, useTransformedFitness);
}

/******************************************************************************/
/**
 * Adapts and evaluates the individual in one go
 *
 * @return The main fitness result
 */
void GOptimizableEntity::adaptAndEvaluate() {
	adapt();
	enforceFitnessUpdate();
}

/******************************************************************************/
/**
 * Retrieve the current (not necessarily up-to-date) fitness
 */
double GOptimizableEntity::getCachedFitness(
	const std::size_t &id, const bool &useTransformedFitness
) const {
	if (useTransformedFitness) {
		return std::get<G_TRANSFORMED_FITNESS>(currentFitnessVec_.at(id));
	} else {
		return std::get<G_RAW_FITNESS>(currentFitnessVec_.at(id));
	}
}

/******************************************************************************/
/**
 * Registers a new, "raw" secondary result value of the custom fitness calculation. This is used in
 * multi-criterion optimization. fitnessCalculation() returns the main fitness value, but may also add further,
 * secondary results. Note that, whether these are actually used, depends on the optimization algorithm being
 * used. Transformation for the second fitness value will be done in the enforceFitnessUpdate function.
 *
 * @param id The position of the fitness criterion (must be > 0 !)
 * @param secondaryValue The secondary fitness value to be registered
 */
void GOptimizableEntity::registerSecondaryResult(
	const std::size_t &id, const double &secondaryValue
) {
#ifdef DEBUG
   if(currentFitnessVec_.size() <= id || 0==id) {
      glogger
      << "In GOptimizableEntity::registerSecondaryResult(...): Error!" << std::endl
      << "Invalid position in vector: " << id << " (expected min 1 and max " <<  currentFitnessVec_.size()-1 << ")" << std::endl
      << GEXCEPTION;
   }
#endif /* DEBUG */

	std::get<G_RAW_FITNESS>(currentFitnessVec_.at(id)) = secondaryValue;
}

/******************************************************************************/
/**
 * Enforces re-calculation of the fitness values.
 */
void GOptimizableEntity::enforceFitnessUpdate(std::function<std::vector<double>()> f) {
	// Assign a new evaluation id
	evaluationID_ = std::string("eval_") + boost::lexical_cast<std::string>(boost::uuids::random_generator()());

	// We start a new evaluation. Make sure the object isn't marked as invalid,
	// and that this state cannot be changed.
	markedAsInvalidByUser_.reset();

	// Find out, whether this is a valid solution
	if (
		this->parameterSetFulfillsConstraints(
			validityLevel_) // Needs to be called first, or else the validityLevel_ will not be filled
		|| evaluationPolicy::USESIMPLEEVALUATION == evalPolicy_
		) {
		// Marking individuals as invalid happens inside of the user-supplied fitness
		// calculation (if at all) so we want to reset the corresponding "invalid" flag
		markedAsInvalidByUser_.unlockWithValue(OE_NOT_MARKED_AS_INVALID);

		if (f) {
			std::vector<double> fitnessVec = f();

			// Use the external evaluation function (needed e.g. when using a GPGPU for the evaluation step)
#ifdef DEBUG
         if(fitnessVec.size() != getNumberOfFitnessCriteria()) {
            glogger
            << "In GOptimizableEntity::enforceFitnessUpdate(): Error!" << std::endl
            << "Invalid size of external evaluation criteria: " << fitnessVec.size() << " (expected " << getNumberOfFitnessCriteria() << ")" << std::endl
            <<  GEXCEPTION;
         }
#endif

			// Assign the actual fitness values
			for (std::size_t i = 0; i < getNumberOfFitnessCriteria(); i++) {
				std::get<G_RAW_FITNESS>(currentFitnessVec_.at(0)) = fitnessVec.at(i);
			}


		} else {
			// Trigger actual fitness calculation using the user-supplied function. This will
			// also register secondary "raw" fitness values used in multi-criterion optimization.
			// Transformed values are taken care of below
			std::get<G_RAW_FITNESS>(currentFitnessVec_.at(0)) = fitnessCalculation();
		}

		// Lock setting of the variable again, so the invalidity state can only be changed
		// upon re-calculation of the object's values
		markedAsInvalidByUser_.lock();

		if (markedAsInvalidByUser_ || this->allRawResultsAtWorst()) { // Is this an invalid result ?
			// Fill the raw and transformed vectors with the worst case scenario. It is assumed
			// here that marking entire solutions as invalid after the evaluation happens relatively
			// rarely so that a flat "worst" quality surface for such solutions does not hinder
			// progress of the optimization procedure too much
			for (std::size_t i = 0; i < getNumberOfFitnessCriteria(); i++) {
				std::get<G_RAW_FITNESS>(currentFitnessVec_.at(i)) = this->getWorstCase();
				std::get<G_TRANSFORMED_FITNESS>(currentFitnessVec_.at(i)) = this->getWorstCase();
			}
		} else { // This is a valid solution nonetheless
			for (std::size_t i = 0; i < this->getNumberOfFitnessCriteria(); i++) {
				if (evaluationPolicy::USESIGMOID == evalPolicy_) { // Update the fitness value to use sigmoidal values
					std::get<G_TRANSFORMED_FITNESS>(currentFitnessVec_.at(i)) =
						Gem::Common::gsigmoid(std::get<G_RAW_FITNESS>(currentFitnessVec_.at(i)), barrier_, steepness_);
				} else { // All other transformation policies leave valid solutions intact
					std::get<G_TRANSFORMED_FITNESS>(currentFitnessVec_.at(i)) = std::get<G_RAW_FITNESS>(
						currentFitnessVec_.at(i));
				}
			}
		}

		// Clear the dirty flag -- all possible evaluation work was done
		//--------------
		setDirtyFlag(false);
		//--------------
	} else { // Some constraints were violated. Act on the chosen policy
		if (evaluationPolicy::USEWORSTCASEFORINVALID == evalPolicy_) {
			for (std::size_t i = 0; i < this->getNumberOfFitnessCriteria(); i++) {
				std::get<G_RAW_FITNESS>(currentFitnessVec_.at(i)) = this->getWorstCase();
				std::get<G_TRANSFORMED_FITNESS>(currentFitnessVec_.at(i)) = this->getWorstCase();
			}

			// Clear the dirty flag -- all possible evaluation work was done
			//--------------
			setDirtyFlag(false);
			//--------------
		} else if (evaluationPolicy::USESIGMOID == evalPolicy_) {
			double uniformFitnessValue = 0.;
			if (true == this->getMaxMode()) { // maximize
				if (boost::numeric::bounds<double>::highest() == validityLevel_) {
					uniformFitnessValue = this->getWorstCase();
				} else {
					uniformFitnessValue = -validityLevel_ * barrier_;
				}
			} else { // minimize
				if (boost::numeric::bounds<double>::highest() == validityLevel_) {
					uniformFitnessValue = this->getWorstCase();
				} else {
					uniformFitnessValue = validityLevel_ * barrier_;
				}
			}

			for (std::size_t i = 0; i < getNumberOfFitnessCriteria(); i++) {
				std::get<G_RAW_FITNESS>(currentFitnessVec_.at(i)) = this->getWorstCase();
				std::get<G_TRANSFORMED_FITNESS>(currentFitnessVec_.at(i)) = uniformFitnessValue;
			}

			// Clear the dirty flag -- all possible evaluation work was done
			//--------------
			setDirtyFlag(false);
			//--------------
		} else if (evaluationPolicy::USEWORSTKNOWNVALIDFORINVALID == evalPolicy_) {
			// Some of this will be reset later, in  GOptimizableEntity::postEvaluationUpdate().
			// The caller needs to tell us about the worst solution known up to now. It is only
			// known once all individuals of this iteration have been evaluated, i.e. not at this
			// place.
			for (std::size_t i = 0; i < getNumberOfFitnessCriteria(); i++) {
				std::get<G_RAW_FITNESS>(currentFitnessVec_.at(i)) = this->getWorstCase();
				std::get<G_TRANSFORMED_FITNESS>(currentFitnessVec_.at(i)) = this->getWorstCase();
			}

			// As only place-holders have been stored in the fitness criteria, the individual
			// is not clean. However, we can tell the audience that evaluation was delayed
			//--------------
			setDirtyFlag(boost::logic::indeterminate);
			//--------------
		}
	}
}

/* ----------------------------------------------------------------------------------
 * untested
 * ----------------------------------------------------------------------------------
 */

/******************************************************************************/
/**
 * Determines the number of fitness criteria present for the individual.
 *
 * @return The number of fitness criteria registered with this individual
 */
std::size_t GOptimizableEntity::getNumberOfFitnessCriteria() const {
	return nFitnessCriteria_;
}

/******************************************************************************/
/**
 * Allows to reset the number of fitness criteria. Note that this should only
 * be done before the first evaluation takes place. One valid use-case for this
 * function is a factory class associated with an individual. Calling this function
 * will likely result in resized worstKnownValids_ and currentFitnessVec_ vectors.
 * This will result in a need to add best- and worst-case values or the removal
 * of existing values.
 */
void GOptimizableEntity::setNumberOfFitnessCriteria(std::size_t nFitnessCriteria) {
	if (nFitnessCriteria < nFitnessCriteria_) {
		currentFitnessVec_.resize(nFitnessCriteria);
		worstKnownValids_.resize(nFitnessCriteria);
	} else if (nFitnessCriteria > nFitnessCriteria_) {
		std::tuple<double, double> worstVal, bestVal;

		std::get<G_RAW_FITNESS>(worstVal) = this->getWorstCase();
		std::get<G_TRANSFORMED_FITNESS>(worstVal) = this->getWorstCase();

		std::get<G_RAW_FITNESS>(bestVal) = this->getBestCase();
		std::get<G_TRANSFORMED_FITNESS>(bestVal) = this->getBestCase();

		currentFitnessVec_.resize(nFitnessCriteria, worstVal);
		worstKnownValids_.resize(nFitnessCriteria, bestVal);
	} // else do nothing

	nFitnessCriteria_ = nFitnessCriteria;
}

/******************************************************************************/
/**
 * Determines whether more than one fitness criterion is present for this individual
 *
 * @return A boolean indicating whether more than one target function is present
 */
bool GOptimizableEntity::hasMultipleFitnessCriteria() const {
	return (getNumberOfFitnessCriteria() > 1 ? true : false);
}

/******************************************************************************/
/**
 * Checks the worst valid fitness and updates it when needed
 */
void GOptimizableEntity::challengeWorstValidFitness(
	std::tuple<double, double> &worstCandidate, const std::size_t &id
) {
#ifdef DEBUG
   if(id >= this->getNumberOfFitnessCriteria()) {
      glogger
      << "In GOptimizableEntity::challengeWorstValidFitness(): Error!" << std::endl
      << "Requested fitness id " << id << " exceeds allowed range " << this->getNumberOfFitnessCriteria() << std::endl
      << GEXCEPTION;
   }

   if(!this->isClean()) {
      glogger
      << "In GOptimizableEntity::challengeWorstValidFitness(): Error!" << std::endl
      << "Function called for dirty individual or with delayed evaluation" << std::endl
      << GEXCEPTION;
   }

   if(!this->isValid()) {
      glogger
      << "In GOptimizableEntity::challengeWorstValidFitness(): Error!" << std::endl
      << "Function called for invalid individual" << std::endl
      << GEXCEPTION;
   }
#endif /* DEBUG */

	double rawFitness = this->fitness(id, PREVENTREEVALUATION, USERAWFITNESS);
	double transformedFitness = this->fitness(id, PREVENTREEVALUATION, USETRANSFORMEDFITNESS);

	// This rather verbose way of creating eval is done so we do not make mistakes later
	// if raw and transformed fitness change their order
	std::tuple<double, double> eval;
	std::get<G_RAW_FITNESS>(eval) = rawFitness;
	std::get<G_TRANSFORMED_FITNESS>(eval) = transformedFitness;

	if (isWorse(std::get<G_TRANSFORMED_FITNESS>(worstCandidate), std::get<G_TRANSFORMED_FITNESS>(eval))) {
		worstCandidate = eval;
	}
}

/******************************************************************************/
/**
 * Retrieve the fitness tuple at a given evaluation position.
 */
std::tuple<double, double> GOptimizableEntity::getFitnessTuple(const std::uint32_t &id) const {
	return currentFitnessVec_.at(id);
}

/******************************************************************************/
/**
 * Sets the fitness to a given set of values and clears the dirty flag. This is meant
 * to be used by external methods of performing the actual evaluation, such as the
 * OpenCL-Consumer. Note that this function assumes that the individual and solution
 * is valid, so it does not currently try to take into account situations where for
 * example constraints are violated. The fitness vector is interpreted as raw fitness
 * values. Hence only SIGMOIDAL transformations are taken into account.
 *
 * @param f_vec A vector of fitness values
 */
void GOptimizableEntity::setFitness_(const std::vector<double> &f_vec) {
#ifdef DEBUG
	if(
      f_vec.size() != this->getNumberOfFitnessCriteria()
	   || currentFitnessVec_.size() != this->getNumberOfFitnessCriteria()
	) {
	   glogger
	   << "In GOptimizableEntity::setFitness_(...): Error!" << std::endl
      << "Invalid size of fitness vector: " << std::endl
      << f_vec.size() << " / " << currentFitnessVec_.size() << " / expected: " << this->getNumberOfFitnessCriteria() << std::endl
      << GEXCEPTION;
	}
#endif /* DEBUG */

	for (std::size_t i = 0; i < this->getNumberOfFitnessCriteria(); i++) {
		std::get<G_RAW_FITNESS>(currentFitnessVec_.at(i)) = f_vec.at(i);

		// We need to update the transformed fitness for the USESIGMOID
		// case. All other transformations are handled elsewhere.
		std::get<G_TRANSFORMED_FITNESS>(currentFitnessVec_.at(i)) =
			evaluationPolicy::USESIGMOID == evalPolicy_
			? Gem::Common::gsigmoid(f_vec.at(i), barrier_, steepness_)
			: f_vec.at(i);
	}

	// Clear the dirty flag
	setDirtyFlag(false);
}

/* ----------------------------------------------------------------------------------
 * Tested in GExternalSetterIndividual::specificTestsNoFailureExpected_GUnitTests()
 * and GExternalSetterIndividual::specificTestsFailuresExpected_GUnitTests()
 * ----------------------------------------------------------------------------------
 */

/******************************************************************************/
/**
 * Checks whether this individual is "clean", i.e neither "dirty" nor has a delayed evaluation
 */
bool GOptimizableEntity::isClean() const {
	if (true == dirtyFlag_ || boost::logic::indeterminate(dirtyFlag_)) {
		return false;
	} else {
		return true;
	}
}

/******************************************************************************/
/**
 * Checks whether the dirty flag is set
 *
 * @return The value of the dirtyFlag_ variable
 */
bool GOptimizableEntity::isDirty() const {
	return true == dirtyFlag_;
}

/* ----------------------------------------------------------------------------------
 * Tested in GTestIndividual1::specificTestsNoFailureExpected_GUnitTests()
 * ----------------------------------------------------------------------------------
 */

/******************************************************************************/
/**
 * Specify whether we want to work in maximization (true) or minimization
 * (false) mode. This function is protected. The idea is that GParameterSet provides a public
 * wrapper for this function, so that a user can specify whether he wants to maximize or
 * minimize a given evaluation function. Optimization algorithms, in turn, only check the
 * maximization-mode of the individuals stored in them and set their own maximization mode
 * internally accordingly, using the protected, overloaded function.
 *
 * @param mode A boolean which indicates whether we want to work in maximization or minimization mode
 */
void GOptimizableEntity::setMaxMode_(const bool &mode) {
	maximize_ = mode;
}

/* ----------------------------------------------------------------------------------
 * Setting is tested in GOptimizableEntity::specificTestsNoFailureExpected_GUnitTests()
 * ----------------------------------------------------------------------------------
 */

/******************************************************************************/
/**
 * Allows to retrieve the maximize_ parameter
 *
 * @return The current value of the maximize_ parameter
 */
bool GOptimizableEntity::getMaxMode() const {
	return maximize_;
}

/* ----------------------------------------------------------------------------------
 * Retrieval is tested in GOptimizableEntity::specificTestsNoFailureExpected_GUnitTests()
 * ----------------------------------------------------------------------------------
 */

/***************************************************************************/
/**
 * Helper function that emits the worst case value depending on whether maximization
 * or minimization is performed.
 *
 * @return The worst case value, depending on maximization or minimization
 */
double GOptimizableEntity::getWorstCase() const {
	return (this->getMaxMode() ? boost::numeric::bounds<double>::lowest() : boost::numeric::bounds<double>::highest());
}

/******************************************************************************/
/**
 * Retrieves the best possible evaluation result, depending on whether we are in
 * maximization or minimization mode
 */
double GOptimizableEntity::getBestCase() const {
	return (this->getMaxMode() ? boost::numeric::bounds<double>::highest() : boost::numeric::bounds<double>::lowest());
}

/******************************************************************************/
/**
 * Retrieves the steepness_ variable (used for the sigmoid transformation)
 */
double GOptimizableEntity::getSteepness() const {
	return steepness_;
}

/******************************************************************************/
/**
 * Sets the steepness variable (used for the sigmoid transformation)
 */
void GOptimizableEntity::setSteepness(double steepness) {
	if (steepness <= 0.) {
		glogger
		<< "In GOptimizableEntity::setSteepness(double steepness): Error!" << std::endl
		<< "Invalid value of steepness parameter: " << steepness << std::endl
		<< GEXCEPTION;
	}

	steepness_ = steepness;
}

/******************************************************************************/
/**
 * Retrieves the barrier_ variable (used for the sigmoid transformation)
 */
double GOptimizableEntity::getBarrier() const {
	return barrier_;
}

/******************************************************************************/
/**
 * Sets the barrier variable (used for the sigmoid transformation)
 */
void GOptimizableEntity::setBarrier(double barrier) {
	if (barrier <= 0.) {
		glogger
		<< "In GOptimizableEntity::setBarrier(double barrier): Error!" << std::endl
		<< "Invalid value of barrier parameter: " << barrier << std::endl
		<< GEXCEPTION;
	}

	barrier_ = barrier;
}

/******************************************************************************/
/**
 * Sets the maximum number of calls to customAdaptions() that may pass without
 * actual modifications. Setting this to 0 disables this check. You should only
 * do this if you are sure that an adaption will eventually happen. Otherwise
 * you would get an endless loop.
 */
void GOptimizableEntity::setMaxUnsuccessfulAdaptions(std::size_t maxUnsuccessfulAdaptions) {
	maxUnsuccessfulAdaptions_ = maxUnsuccessfulAdaptions;
}

/******************************************************************************/
/**
 * Retrieves the maximum number of calls to customAdaptions that may pass without
 * actual modifications
 */
std::size_t GOptimizableEntity::getMaxUnsuccessfulAdaptions() const {
	return maxUnsuccessfulAdaptions_;
}

/******************************************************************************/
/**
 * Allows to set the maximum number of retries during the adaption of individuals
 * until a valid individual was found. Setting this value to 0 will disable retries.
 */
void GOptimizableEntity::setMaxRetriesUntilValid(
	std::size_t maxRetriesUntilValid
) {
	maxRetriesUntilValid_ = maxRetriesUntilValid;
}

/******************************************************************************/
/**
 * Allows to retrieve the current maximum number of retries during the adaption of
 * individuals until a valid individual was found.
 */
std::size_t GOptimizableEntity::getMaxRetriesUntilValid() const {
	return maxRetriesUntilValid_;
}

/******************************************************************************/
/**
 * Retrieves the number of adaptions performed during the last call to adapt()
 * (or 0, if no adaptions were performed so far).
 */
std::size_t GOptimizableEntity::getNAdaptions() const {
	return nAdaptions_;
}

/******************************************************************************/
/**
 * Sets the dirtyFlag_. This is a "one way" function, accessible to derived classes. Once the dirty flag
 * has been set, the only way to reset it is to calculate the fitness of this object.
 */
void GOptimizableEntity::setDirtyFlag() {
	dirtyFlag_ = true;
}

/* ----------------------------------------------------------------------------------
 * Tested in GTestIndividual1::specificTestsNoFailureExpected_GUnitTests()
 * Tested in GOptimizableEntity::specificTestsNoFailureExpected_GUnitTests()
 * ----------------------------------------------------------------------------------
 */

/******************************************************************************/
/**
 * Checks whether evaluation was delayed
 */
bool GOptimizableEntity::evaluationDelayed() const {
	if (boost::logic::indeterminate(dirtyFlag_)) {
		return true;
	} else {
		return false;
	}
}

/******************************************************************************/
/**
 * Sets the dirtyFlag_ to any desired value
 *
 * @param dirtyFlag The new value for the dirtyFlag_ variable
 * @return The previous value of the dirtyFlag_ variable
 */
boost::logic::tribool GOptimizableEntity::setDirtyFlag(
	const boost::logic::tribool &dirtyFlag
) {
	bool previous = dirtyFlag_;
	dirtyFlag_ = dirtyFlag;
	return previous;
}

/* ----------------------------------------------------------------------------------
 * Tested in GOptimizableEntity::specificTestsNoFailureExpected_GUnitTests()
 * ----------------------------------------------------------------------------------
 */

/******************************************************************************/
/**
 * Checks whether this solution is valid. This function is meant to be called
 * for "clean" individuals only and will throw when called for individuals, whose
 * dirty flag is set. Note that it is well possible to call the function if
 * evaluation was delayed.
 */
bool GOptimizableEntity::isValid() const {
	if (this->isDirty()) {
		glogger
		<< "In GOptimizableEntity::isValid(): Error!" << std::endl
		<< "Function was called while dirty flag was set" << std::endl
		<< GEXCEPTION;
	}

	if (validityLevel_ <= 1. && !markedAsInvalidByUser_ && !this->allRawResultsAtWorst()) {
		return true;
	} else {
		return false;
	}
}

/******************************************************************************/
/**
 * Checks whether this solution is invalid
 */
bool GOptimizableEntity::isInValid() const {
	if (validityLevel_ > 1. || markedAsInvalidByUser_ || this->allRawResultsAtWorst()) {
		return true;
	} else {
		return false;
	}
}

/******************************************************************************/
/**
 * Checks whether this solution fulfills the set of constraints. Note that this
 * function may be called prior to evaluation in order to check
 */
bool GOptimizableEntity::parameterSetFulfillsConstraints(double &validityLevel) const {
	if (individualConstraint_) {
		return individualConstraint_->isValid(this, validityLevel);
	} else { // Always valid, if no constraint object has been registered
		validityLevel = 0.;
		return true;
	}

	// Make the compiler happy
	return false;
}

/******************************************************************************/
/**
 * Checks whether all results are at the worst possible value
 */
bool GOptimizableEntity::allRawResultsAtWorst() const {
	for (std::size_t i = 0; i < getNumberOfFitnessCriteria(); i++) {
		if (this->getWorstCase() != std::get<G_RAW_FITNESS>(currentFitnessVec_.at(i))) return false;
	}

	// O.k., so all results are at their worst value
	return true;
}

/******************************************************************************/
/**
 * Check how valid a given solution is
 */
double GOptimizableEntity::getValidityLevel() const {
	return validityLevel_;
}

/******************************************************************************/
/**
 * Checks whether all constraints were fulfilled
 */
bool GOptimizableEntity::constraintsFulfilled() const {
	if (validityLevel_ <= 1.) return true;
	else return false;
}

/******************************************************************************/
/**
 * Allows to set the globally best known primary fitness so far
 *
 * @param bnf The best known primary fitness so far
 */
void GOptimizableEntity::setBestKnownPrimaryFitness(
	const std::tuple<double, double> &bnf
) {
	bestPastPrimaryFitness_ = bnf;
}

/* ----------------------------------------------------------------------------------
 * Tested in GOptimizableEntity::specificTestsNoFailureExpected_GUnitTests()
 * ----------------------------------------------------------------------------------
 */

/******************************************************************************/
/**
 * Retrieves the value of the globally best known primary fitness so far
 *
 * @return The best known primary fitness so far
 */
std::tuple<double, double> GOptimizableEntity::getBestKnownPrimaryFitness() const {
	return bestPastPrimaryFitness_;
}

/* ----------------------------------------------------------------------------------
 * Tested in GOptimizableEntity::specificTestsNoFailureExpected_GUnitTests()
 * ----------------------------------------------------------------------------------
 */

/******************************************************************************/
/**
 * Retrieve the id assigned to the current evaluation
 */
std::string GOptimizableEntity::getCurrentEvaluationID() const {
	return evaluationID_;
}

/******************************************************************************/
/**
 * Allows an optimization algorithm to set the worst known valid (primary and secondary
 * evaluation up to the current iteration. Note that these are not the best evaluations
 * for a single evaluation criterion, but the worst evaluations for all individuals that
 * were visited so far. Of the std::tuple, the first value signifies the untransformed
 * value, the second value the (possibly transformed) evaluation.
 */
void GOptimizableEntity::setWorstKnownValid(
	const std::vector<std::tuple<double, double>> &worstKnownValid
) {
	worstKnownValids_ = worstKnownValid;
}

/******************************************************************************/
/**
 * Allows to retrieve the worst known valid evaluation up to the current iteration,
 * as set by an external optimization algorithm, at a given position.
 */
std::tuple<double, double> GOptimizableEntity::getWorstKnownValid(
	const std::uint32_t &id
) const {
#ifdef DEBUG
   glogger
   << "In GOptimizableEntity::getWorstKnownValid(" << id << "): Error!" << std::endl
   << "Expected id of max " << worstKnownValids_.size() - 1 << std::endl
   << GEXCEPTION;
#endif /* DEBUG */

	return worstKnownValids_.at(id);
}

/******************************************************************************/
/**
 * Allows to retrieve all worst known valid evaluations up to the
 * current iteration, as set by an external optimization algorithm
 */
std::vector<std::tuple<double, double>> GOptimizableEntity::getWorstKnownValids() const {
	return worstKnownValids_;
}

/******************************************************************************/
/**
 * Fills the worstKnownValid-vector with best values. This function assumes all
 * fitness criteria have been made known already.
 */
void GOptimizableEntity::populateWorstKnownValid() {
#ifdef DEBUG
   if(worstKnownValids_.size() != nFitnessCriteria_) {
      glogger
      << "In GOptimizableEntity::populateWorstKnownValid(): Error!" << std::endl
      << "Invalid size of worstKnownValids_: " << worstKnownValids_.size() << " (expected " << nFitnessCriteria_ << ")" << std::endl
      << GEXCEPTION;
   }
#endif /* DEBUG */

	for (std::size_t i = 0; i < nFitnessCriteria_; i++) {
		worstKnownValids_.at(i) = std::tuple<double, double>(this->getBestCase(), this->getBestCase());
	}
}

/******************************************************************************/
/**
 * Triggers an update of the internal evaluation, if necessary.
 */
void GOptimizableEntity::postEvaluationUpdate() {
#ifdef DEBUG
   if((nFitnessCriteria_) != currentFitnessVec_.size()) {
      glogger
      << "In GOptimizableEntity::postEvaluationUpdate(): Error!" << std::endl
      << "Number of expected fitness criteria " << nFitnessCriteria_ << " does not match actual number " << currentFitnessVec_.size() << std::endl
      << GEXCEPTION;
   }

   if(worstKnownValids_.empty()) {
      glogger
      << "In GOptimizableEntity::postEvaluationUpdate(): Error!" << std::endl
      << "worstKnownValids_ does not seem to be initialized" << std::endl
      << GEXCEPTION;
   }
#endif /* DEBUG */

	if (evaluationPolicy::USEWORSTKNOWNVALIDFORINVALID == evalPolicy_ && this->isInValid()) {
		if (true == maximize_) {
			for (std::size_t i = 0; i < nFitnessCriteria_; i++) {
				if (boost::numeric::bounds<double>::highest() == validityLevel_ ||
					 boost::numeric::bounds<double>::lowest() == validityLevel_) {
					std::get<G_TRANSFORMED_FITNESS>(currentFitnessVec_.at(i)) = this->getWorstCase();
				} else {
					std::get<G_TRANSFORMED_FITNESS>(currentFitnessVec_.at(i))
						= -(std::max)(std::get<G_TRANSFORMED_FITNESS>(worstKnownValids_.at(i)), (std::max)(barrier_, 1.)) *
						  validityLevel_;
				}
			}
		} else {
			for (std::size_t i = 1; i < nFitnessCriteria_; i++) {
				if (boost::numeric::bounds<double>::highest() == validityLevel_ ||
					 boost::numeric::bounds<double>::lowest() == validityLevel_) {
					std::get<G_TRANSFORMED_FITNESS>(currentFitnessVec_.at(i)) = this->getWorstCase();
				} else {
					std::get<G_TRANSFORMED_FITNESS>(currentFitnessVec_.at(i))
						= (std::max)(std::get<G_TRANSFORMED_FITNESS>(worstKnownValids_.at(i)), (std::max)(barrier_, 1.)) *
						  validityLevel_;
				}
			}
		}

		// Note: the "rawFitness" values have already been set in enforceFitnessUpdate() to the worst known evaluation

		// Make sure the dirty flag is set to false, so our results do not get overwritten
		this->setDirtyFlag(false);
	}
}

/******************************************************************************/
/**
 * Combines evaluation results by adding the individual results
 *
 *  @return The result of the combination
 */
double GOptimizableEntity::sumCombiner() const {
	double result = 0.;
	std::vector<std::tuple<double, double>>::const_iterator cit;
	for (cit = currentFitnessVec_.begin(); cit != currentFitnessVec_.end(); ++cit) {
		result += std::get<G_TRANSFORMED_FITNESS>(*cit);
	}
	return result;
}

/******************************************************************************/
/**
 * Combines evaluation results by adding the absolute values of individual results
 *
 *  @return The result of the combination
 */
double GOptimizableEntity::fabsSumCombiner() const {
	double result = 0.;
	std::vector<std::tuple<double, double>>::const_iterator cit;
	for (cit = currentFitnessVec_.begin(); cit != currentFitnessVec_.end(); ++cit) {
		result += fabs(std::get<G_TRANSFORMED_FITNESS>(*cit));
	}
	return result;
}

/******************************************************************************/
/**
 * Combines evaluation results by calculating the square root of the squared sum.
 * It is assumed that the result of this function is returned as
 * the main result of the fitnessCalculation() function.
 *
 * @return The result of the combination
 */
double GOptimizableEntity::squaredSumCombiner() const {
	double result = 0.;
	std::vector<std::tuple<double, double>>::const_iterator cit;
	for (cit = currentFitnessVec_.begin(); cit != currentFitnessVec_.end(); ++cit) {
		result += GSQUARED(std::get<G_TRANSFORMED_FITNESS>(*cit));
	}
	return sqrt(result);
}

/******************************************************************************/
/**
 * Combines evaluation results by calculating the square root of the weighed squared sum. Note that we
 * only evaluate the secondary results here. It is assumed that the result of this function is returned as
 * the main result of the fitnessCalculation() function.
 *
 * @param weights The weights to be multiplied with the cached results
 * @return The result of the combination
 */
double GOptimizableEntity::weighedSquaredSumCombiner(const std::vector<double> &weights) const {
	double result = 0.;
	std::vector<std::tuple<double, double>>::const_iterator cit_eval;
	std::vector<double>::const_iterator cit_weights;

	if (currentFitnessVec_.size() != weights.size()) {
		glogger
		<< "In GOptimizableEntity::weighedSquaredSumCombine(): Error!" << std::endl
		<< "Sizes of transformedCurrentFitnessVec_ and the weights vector don't match: " << currentFitnessVec_.size() <<
		" / " << weights.size() << std::endl
		<< GEXCEPTION;
	}

	for (cit_eval = currentFitnessVec_.begin(), cit_weights = weights.begin();
		  cit_eval != currentFitnessVec_.end();
		  ++cit_eval, ++cit_weights
		) {
		result += GSQUARED((*cit_weights) * (std::get<G_TRANSFORMED_FITNESS>(*cit_eval)));
	}

	return sqrt(result);
}

/******************************************************************************/
/**
 * Allows users to mark this solution as invalid in derived classes (usually
 * from within the evaluation function)
 */
void GOptimizableEntity::markAsInvalid() {
	if (!markedAsInvalidByUser_.isLocked()) {
		markedAsInvalidByUser_ = true;
	} else {
		glogger
		<< "In GOptimizableEntity::markAsInvalid(): Error!" << std::endl
		<< "Tried to mark individual as invalid while changes to this property" << std::endl
		<< "were not allowed. This function may only be called from inside the" << std::endl
		<< "fitness evaluation!" << std::endl
		<< GEXCEPTION;
	}
}

/******************************************************************************/
/**
 * Allows to check whether this solution was marked as invalid
 */
bool GOptimizableEntity::markedAsInvalidByUser() const {
	return markedAsInvalidByUser_;
}

/******************************************************************************/
/**
 * Helps to determine whether a given value is strictly better (i.e. better than equal)
 * than another one. As "better" means something different for maximization and minimization,
 * this function helps to make the code easier to understand.
 *
 * @param newValue The new value
 * @param oldValue The old value
 * @return true if newValue is better than oldValue, otherwise false.
 */
bool GOptimizableEntity::isBetter(double newValue, const double &oldValue) const {
	if (this->getMaxMode()) {
		if (newValue > oldValue) return true;
		else return false;
	} else { // minimization
		if (newValue < oldValue) return true;
		else return false;
	}
}

/******************************************************************************/
/**
 * Helps to determine whether a given value is strictly worse (i.e. worse than equal)
 * than another one. As "worse" means something different for maximization and minimization,
 * this function helps to make the code easier to understand.
 *
 * @param newValue The new value
 * @param oldValue The old value
 * @return true of newValue is worse than oldValue, otherwise false.
 */
bool GOptimizableEntity::isWorse(double newValue, const double &oldValue) const {
	if (this->getMaxMode()) {
		if (newValue < oldValue) return true;
		else return false;
	} else { // minimization
		if (newValue > oldValue) return true;
		else return false;
	}
}

/******************************************************************************/
/**
 * Checks whether this object is better than the argument, depending on the maxMode
 */
bool GOptimizableEntity::isBetterThan(std::shared_ptr < GOptimizableEntity > p) const {
	if (this->getMaxMode()) {
		if (this->transformedFitness() > p->transformedFitness()) return true;
		else return false;
	} else { // minimization
		if (this->transformedFitness() < p->transformedFitness()) return true;
		else return false;
	}
}

/******************************************************************************/
/**
 * Checks whether this object is worse than the argument, depending on the maxMode
 */
bool GOptimizableEntity::isWorseThan(std::shared_ptr < GOptimizableEntity > p) const {
	if (this->getMaxMode()) {
		if (this->transformedFitness() < p->transformedFitness()) return true;
		else return false;
	} else { // minimization
		if (this->transformedFitness() > p->transformedFitness()) return true;
		else return false;
	}
}

/******************************************************************************/
/**
 * Adds local configuration options to a GParserBuilder object
 *
 * @param gpb The GParserBuilder object to which configuration options should be added
 */
void GOptimizableEntity::addConfigurationOptions(
	Gem::Common::GParserBuilder &gpb
) {
	// Call our parent class'es function
	GObject::addConfigurationOptions(gpb);

	// Add local data
	gpb.registerFileParameter<evaluationPolicy>(
		"evalPolicy" // The name of the variable
		, Gem::Geneva::evaluationPolicy::USESIMPLEEVALUATION // The default value
		, [this](evaluationPolicy ep) { this->setEvaluationPolicy(ep); }
	)
	<< "Specifies which strategy should be used to calculate the evaluation:" << std::endl
	<< "0 (a.k.a. USESIMPLEEVALUATION): Always call the evaluation function, even for invalid solutions" << std::endl
	<<
	"1 (a.k.a. USEWORSTCASEFORINVALID) : Assign the worst possible value to our fitness and evaluate only valid solutions" <<
	std::endl
	<<
	"2 (a.k.a. USESIGMOID): Assign a multiple of validityLevel_ and sigmoid barrier to invalid solutions, apply a sigmoid function to valid evaluations" <<
	std::endl
	<<
	"3 (a.k.a. USEWORSTKNOWNVALIDFORINVALID): Assign \"invalidityLevel*worstKnownValid\" to invalid individuals, using normal evaluation otherwise";

	gpb.registerFileParameter<double>(
		"steepness" // The name of the variable
		, Gem::Geneva::FITNESSSIGMOIDSTEEPNESS // The default value
		, [this](double ss) { this->setSteepness(ss); }
	)
	<< "When using a sigmoid function to transform the individual's fitness," << std::endl
	<< "this parameter influences the steepness of the function at the center of the sigmoid." << std::endl
	<< "The parameter must have a value > 0.";

	gpb.registerFileParameter<double>(
		"barrier" // The name of the variable
		, Gem::Geneva::WORSTALLOWEDVALIDFITNESS // The default value
		, [this](double barrier) { this->setBarrier(barrier); }
	)
	<< "When using a sigmoid function to transform the individual's fitness," << std::endl
	<< "this parameter sets the upper/lower boundary of the sigmoid." << std::endl
	<< "The parameter must have a value > 0.;";

	gpb.registerFileParameter<std::size_t>(
		"maxUnsuccessfulAdaptions" // The name of the variable
		, DEFMAXUNSUCCESSFULADAPTIONS // The default value
		, [this](std::size_t mua) { this->setMaxUnsuccessfulAdaptions(mua); }
	)
	<< "The maximum number of unsuccessful adaptions in a row for one call to adapt()";

	gpb.registerFileParameter<std::size_t>(
		"maxRetriesUntilValid" // The name of the variable
		, DEFMAXRETRIESUNTILVALID // The default value
		, [this](std::size_t mruv) { this->setMaxRetriesUntilValid(mruv); }
	)
	<< "The maximum allowed number of retries during the" << std::endl
	<< "adaption of individuals until a valid solution was found" << std::endl
	<< "A parameter set is considered to be \"valid\" if" << std::endl
	<< "it passes all validity checks;";



	// maximize_ will be set in GParameterSet, as it has a different
	// meaning for optimization algorithms that also derive indirectly
	// from this class.
}

/******************************************************************************/
/**
 * Sets the current personality of this individual
 *
 * @param gpt A pointer to an object representing the new personality of this object
 */
void GOptimizableEntity::setPersonality(
	std::shared_ptr < GPersonalityTraits > gpt
) {
	// Make sure we haven't been given an empty pointer
	if (!gpt) {
		glogger
		<< "In GOptimizableEntity::setPersonality(): Error!" << std::endl
		<< "Received empty personality traits pointer" << std::endl
		<< GEXCEPTION;
	}

	// Add the personality traits object to our local pointer
	pt_ptr_ = gpt;
}


/* ----------------------------------------------------------------------------------
 * Tested in GOptimizableEntity::specificTestsNoFailureExpected_GUnitTests()
 * ----------------------------------------------------------------------------------
 */

/******************************************************************************/
/**
 * Resets the current personality to PERSONALITY_NONE
 */
void GOptimizableEntity::resetPersonality() {
	pt_ptr_.reset();
}

/* ----------------------------------------------------------------------------------
 * Tested in GOptimizableEntity::specificTestsNoFailureExpected_GUnitTests()
 * ----------------------------------------------------------------------------------
 */

/******************************************************************************/
/**
 * Retrieves the mnemonic used for the optimization of this object
 */
std::string GOptimizableEntity::getMnemonic() const {
	if(pt_ptr_) {
		return pt_ptr_->getMnemonic();
	} else {
		glogger
			<< "In GOptimizableEntity::getMnemonic():" << std::endl
			<< "Pointer to personality traits object is empty." << std::endl
			<< GEXCEPTION;
	}
}


/******************************************************************************/
/**
 * Retrieves the current personality of this individual
 *
 * @return An identifier for the current personality of this object
 */
std::string GOptimizableEntity::getPersonality() const {
	if (pt_ptr_) {
		return pt_ptr_->name();
	} else {
		return std::string("PERSONALITY_NONE");
	}
}

/* ----------------------------------------------------------------------------------
 * Tested in GOptimizableEntity::specificTestsNoFailureExpected_GUnitTests()
 * ----------------------------------------------------------------------------------
 */

/******************************************************************************/
/**
 * This function returns the current personality traits base pointer. Note that there
 * is another version of the same command that does on-the-fly conversion of the
 * personality traits to the derived class.
 *
 * @return A shared pointer to the personality traits base class
 */
std::shared_ptr <GPersonalityTraits> GOptimizableEntity::getPersonalityTraits() {
#ifdef DEBUG
	// Do some error checking
	if(!pt_ptr_) {
	   glogger
	   << "In GOptimizableEntity::getPersonalityTraits():" << std::endl
      << "Pointer to personality traits object is empty." << std::endl
      << GEXCEPTION;
	}
#endif

	return pt_ptr_;
}

/* ----------------------------------------------------------------------------------
 * Tested in GOptimizableEntity::specificTestsNoFailureExpected_GUnitTests()
 * Tested in GOptimizableEntity::specificTestsFailuresExpected_GUnitTests()
 * ----------------------------------------------------------------------------------
 */

/******************************************************************************/
/**
 * Actions to be performed when adapting this object. This function will be overloaded particularly
 * for the GParameterSet class.
 */
std::size_t GOptimizableEntity::customAdaptions() BASE {
	return 0;
}

/******************************************************************************/
/**
 * Allows to set the current iteration of the parent optimization algorithm.
 *
 * @param parentAlgIteration The current iteration of the optimization algorithm
 */
void GOptimizableEntity::setAssignedIteration(const std::uint32_t &parentAlgIteration) {
	assignedIteration_ = parentAlgIteration;
}

/* ----------------------------------------------------------------------------------
 * Tested in GOptimizableEntity::specificTestsNoFailureExpected_GUnitTests()
 * ----------------------------------------------------------------------------------
 */

/******************************************************************************/
/**
 * Gives access to the parent optimization algorithm's iteration
 *
 * @return The parent optimization algorithm's current iteration
 */
std::uint32_t GOptimizableEntity::getAssignedIteration() const {
	return assignedIteration_;
}

/* ----------------------------------------------------------------------------------
 * Tested in GOptimizableEntity::specificTestsNoFailureExpected_GUnitTests()
 * ----------------------------------------------------------------------------------
 */

/******************************************************************************/
/**
 * Allows to specify the number of optimization cycles without improvement of the primary fitness criterion
 *
 * @param nStalls The number of optimization cycles without improvement in the parent algorithm
 */
void GOptimizableEntity::setNStalls(const std::uint32_t &nStalls) {
	nStalls_ = nStalls;
}

/* ----------------------------------------------------------------------------------
 * Tested in GOptimizableEntity::specificTestsNoFailureExpected_GUnitTests()
 * ----------------------------------------------------------------------------------
 */

/******************************************************************************/
/**
 * Allows to retrieve the number of optimization cycles without improvement of the primary fitness criterion
 *
 * @return The number of optimization cycles without improvement in the parent algorithm
 */
std::uint32_t GOptimizableEntity::getNStalls() const {
	return nStalls_;
}

/* ----------------------------------------------------------------------------------
 * Tested in GOptimizableEntity::specificTestsNoFailureExpected_GUnitTests()
 * ----------------------------------------------------------------------------------
 */

/******************************************************************************/
/**
 * Applies modifications to this object. This is needed for testing purposes
 *
 * @return A boolean which indicates whether modifications were made
 */
bool GOptimizableEntity::modify_GUnitTests() {
#ifdef GEM_TESTING

	using boost::unit_test_framework::test_suite;
	using boost::unit_test_framework::test_case;

	bool result = false;

	// Call the parent class'es function
	if (GObject::modify_GUnitTests()) result = true;

	// Adaption of parameters through the adapt() call is done in GTestIndividual1.
	// We do not know what is stored in this individual, hence we cannot modify
	// parameters directly here in this class.

	// A relatively harmless change
	nStalls_++;

	return result;

#else /* GEM_TESTING */  // If this function is called when GEM_TESTING isn't set, throw
   condnotset("GOptimizableEntity::modify_GUnitTests", "GEM_TESTING");
   return false;
#endif /* GEM_TESTING */
}

/******************************************************************************/
/**
 * Performs self tests that are expected to succeed. This is needed for testing purposes
 */
void GOptimizableEntity::specificTestsNoFailureExpected_GUnitTests() {
#ifdef GEM_TESTING
	using boost::unit_test_framework::test_suite;
	using boost::unit_test_framework::test_case;

	// Call the parent class'es function
	GObject::specificTestsNoFailureExpected_GUnitTests();

	// --------------------------------------------------------------------------

	{ // Test setting and retrieval of the maximization mode flag
		std::shared_ptr <GOptimizableEntity> p_test = this->clone<GOptimizableEntity>();

		BOOST_CHECK_NO_THROW(p_test->setMaxMode_(true));
		BOOST_CHECK(p_test->getMaxMode() == true);
		BOOST_CHECK_NO_THROW(p_test->setMaxMode_(false));
		BOOST_CHECK(p_test->getMaxMode() == false);
	}

	// --------------------------------------------------------------------------

	{ // Check setting of the dirty flag
		std::shared_ptr <GOptimizableEntity> p_test = this->clone<GOptimizableEntity>();

		BOOST_CHECK_NO_THROW(p_test->setDirtyFlag(true));
		BOOST_CHECK(p_test->isDirty() == true);
		BOOST_CHECK_NO_THROW(p_test->setDirtyFlag(false));
		BOOST_CHECK(p_test->isDirty() == false);
		BOOST_CHECK_NO_THROW(p_test->setDirtyFlag());
		BOOST_CHECK(p_test->isDirty() == true); // Note the missing argument -- this is a different function
		BOOST_CHECK_NO_THROW(p_test->setDirtyFlag(false));
		BOOST_CHECK(p_test->isDirty() == false);
		BOOST_CHECK_NO_THROW(p_test->setDirtyFlag(boost::logic::indeterminate));
		BOOST_CHECK(p_test->evaluationDelayed() == true);
		BOOST_CHECK(p_test->isDirty() == false);
		BOOST_CHECK(p_test->isClean() == false);
	}

	// --------------------------------------------------------------------------

	{ // Test setting and retrieval of the surrounding optimization algorithm's current iteration
		std::shared_ptr <GOptimizableEntity> p_test = this->clone<GOptimizableEntity>();

		for (std::uint32_t i = 1; i < 10; i++) {
			BOOST_CHECK_NO_THROW(p_test->setAssignedIteration(i));
			BOOST_CHECK_MESSAGE(
				p_test->getAssignedIteration() == i, "\n"
																 << "p_test->getAssignedIteration() = " <<
																 p_test->getAssignedIteration() << "\n"
																 << "i = " << i << "\n"
			);
		}
	}

	// --------------------------------------------------------------------------

	{ // Test setting and retrieval of the best known fitness so far
		std::shared_ptr <GOptimizableEntity> p_test = this->clone<GOptimizableEntity>();

		for (double d = 0.; d < 1.; d += 0.1) {
			BOOST_CHECK_NO_THROW(p_test->setBestKnownPrimaryFitness(std::make_tuple(d, d)));
			BOOST_CHECK_MESSAGE(
				p_test->getBestKnownPrimaryFitness() == std::make_tuple(d, d), "\n"
																  << "p_test->getBestKnownPrimaryFitness() = " <<
																  Gem::Common::g_to_string(p_test->getBestKnownPrimaryFitness()) <<
																  "\n"
																  << "d = " << d << "\n"
			);
		}
	}

	// --------------------------------------------------------------------------

	{ // Test setting and retrieval of the number of consecutive stalls
		std::shared_ptr <GOptimizableEntity> p_test = this->clone<GOptimizableEntity>();

		for (std::uint32_t i = 1; i < 10; i++) {
			BOOST_CHECK_NO_THROW(p_test->setNStalls(i));
			BOOST_CHECK_MESSAGE(
				p_test->getNStalls() == i, "\n"
													<< "p_test->getNStalls() = " << p_test->getNStalls() << "\n"
													<< "i = " << i << "\n"
			);
		}
	}

	// --------------------------------------------------------------------------

#else /* GEM_TESTING */ // If this function is called when GEM_TESTING isn't set, throw
   condnotset("GOptimizableEntity::specificTestsNoFailureExpected_GUnitTests", "GEM_TESTING");
#endif /* GEM_TESTING */
}

/******************************************************************************/
/**
 * Performs self tests that are expected to fail. This is needed for testing purposes
 */
void GOptimizableEntity::specificTestsFailuresExpected_GUnitTests() {
#ifdef GEM_TESTING
	using boost::unit_test_framework::test_suite;
	using boost::unit_test_framework::test_case;

	// Call the parent class'es function
	GObject::specificTestsFailuresExpected_GUnitTests();

	// --------------------------------------------------------------------------
	// --------------------------------------------------------------------------

#else /* GEM_TESTING */ // If this function is called when GEM_TESTING isn't set, throw
   condnotset("GOptimizableEntity::specificTestsFailuresExpected_GUnitTests", "GEM_TESTING");
#endif /* GEM_TESTING */
}

/******************************************************************************/

} /* namespace Geneva */
} /* namespace Gem */
