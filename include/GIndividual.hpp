/**
 * @file GIndividual.hpp
 */

/* Copyright (C) 2004-2008 Dr. Ruediger Berlich
 * Copyright (C) 2007-2008 Forschungszentrum Karlsruhe GmbH
 *
 * This file is part of Geneva, Gemfony scientific's optimization library.
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
 */

// Standard header files go here
#include <sstream>
#include <string>
#include <map>

// Boost header files go here

#include <boost/version.hpp>

#if BOOST_VERSION < 103600
#error "Error: Boost should at least have version 1.36 !"
#endif /* BOOST_VERSION */

#include <boost/cstdint.hpp>
#include <boost/serialization/map.hpp>


#ifndef GINDIVIDUAL_HPP_
#define GINDIVIDUAL_HPP_

// Geneva header files go here

#include "GMutableI.hpp"
#include "GRateableI.hpp"
#include "GObject.hpp"
#include "GLogger.hpp"
#include "GenevaExceptions.hpp"

namespace Gem {
namespace GenEvA {

/**************************************************************************************************/
/**
 * This class acts as an interface class for all objects that can take part
 * in an evolutionary improvement. Such items must possess mutation functionality
 * and must know how to calculate their fitness. They also need the basic GObject
 * interface. In particular, they absolutely need to be serializable. As this library
 * was designed with particularly expensive evaluation calculations in mind, this
 * class also contains a framework for lazy evaluation, so not all evaluations take
 * place at the same time.
 */
class GIndividual
	:public GMutableI,
	 public GRateableI,
	 public GObject
{
	///////////////////////////////////////////////////////////////////////
	friend class boost::serialization::access;

	template<class Archive>
	void serialize(Archive & ar, const unsigned int version){
	  using boost::serialization::make_nvp;
	  ar & make_nvp("GObject", boost::serialization::base_object<GObject>(*this));
	  ar & make_nvp("GRateableI", boost::serialization::base_object<GRateableI>(*this));
	  ar & make_nvp("GMutableI", boost::serialization::base_object<GMutableI>(*this));
	  ar & make_nvp("currentFitness_",currentFitness_);
	  ar & make_nvp("dirtyFlag_",dirtyFlag_);
	  ar & make_nvp("allowLazyEvaluation_",allowLazyEvaluation_);
	  ar & make_nvp("parentPopGeneration_",parentPopGeneration_);
	  ar & make_nvp("parentCounter_",parentCounter_);
	  ar & make_nvp("attributeTable_",attributeTable_);
	}
	///////////////////////////////////////////////////////////////////////

public:
	/** @brief The default constructor */
	GIndividual();
	/** @brief The copy constructor */
	GIndividual(const GIndividual&);
	/** @brief The destructor */
	virtual ~GIndividual();

	/** @brief Creates a deep clone of this object */
	virtual GObject* clone() = 0;
	/** @brief Loads the data of another GObject */
	virtual void load(const GObject*);

	/** @brief The mutate interface */
	virtual void mutate();
	/** @brief Calculate the fitness of this object */
	virtual double fitness();
	/** @brief Do the required processing for this object */
	void process();
	/** @brief Do the required processing for this object and catch all exceptions */
	void checkedProcess();

	/** @brief Retrieve a value for this class and check for exceptions. Useful for threads */
	virtual double checkedFitness();
	/** @brief Retrieve the current (not necessarily up-to-date) fitness */
	double getCurrentFitness(bool&) const throw();
	/** @brief Enforce fitness calculation */
	double doFitnessCalculation();

	/** @brief Indicate whether lazy evaluation is allowed */
	bool setAllowLazyEvaluation(const bool&) throw();
	/** @brief Retrieve the allowLazyEvaluation_ parameter */
	bool getAllowLazyEvaluation(void) const throw();

	/** @brief Check whether the dirty flag is set */
	bool isDirty() const throw();

	/** @brief Sets the parentPopGeneration_ parameter */
	void setParentPopGeneration(const boost::uint32_t&) throw();
	/** @brief Retrieve the parentPopGeneration_ parameter */
	boost::uint32_t getParentPopGeneration() const throw();

	/** @brief Checks whether this is a parent individual */
	bool isParent() const throw();

	/** @brief Marks an individual as a parent*/
	bool setIsParent(void);
	/** @brief Marks an individual as a child */
	bool setIsChild(void);

	/** @brief Retrieves the current value of the parentCounter_ variable */
	boost::uint32_t getParentCounter() const throw();

	/** @brief Adds an attribute to the individual */
	std::string setAttribute(const std::string&, const std::string&);
	/** @brief Retrieves an attribute from the individual */
	std::string getAttribute(const std::string&);
	/** @brief Removes an attribute from the individual */
	std::string delAttribute(const std::string&);
	/** @brief Clears the attribute table */
	void clearAttributes();

protected:
	/** @brief The actual fitness calculation takes place here */
	virtual double fitnessCalculation();
	/** @brief The actual mutation operations */
	virtual void customMutations();

	/** @brief Sets the dirtyFlag_ */
	void setDirtyFlag() throw();

private:
	/** @brief Sets the parentCounter_ parameter */
	bool setIsParent(const bool&) throw();

	/** @brief Sets the dirtyFlag_ to any desired value */
	bool setDirtyFlag(const bool&) throw();

	/** @brief Holds this object's internal fitness */
    double currentFitness_;
    /** @brief Internal representation of the mutation status of this object */
    bool dirtyFlag_;
    /** @brief Steers whether lazy evaluation is allowed */
    bool allowLazyEvaluation_;
    /** @brief Holds the parent population's current generation */
    boost::uint32_t parentPopGeneration_;
    /** @brief Allows populations to mark members as parents or children */
    boost::uint32_t parentCounter_;
    /** @brief Holds string attributes assigned to this class */
    std::map<std::string, std::string> attributeTable_;
};

} /* namespace GenEvA */
} /* namespace Gem */


/**************************************************************************************************/
/**
 * @brief Needed for Boost.Serialization
 */
BOOST_SERIALIZATION_ASSUME_ABSTRACT(Gem::GenEvA::GIndividual)

/**************************************************************************************************/

#endif /* GINDIVIDUAL_HPP_ */
