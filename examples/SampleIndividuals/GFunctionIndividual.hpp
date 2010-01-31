/**
 * @file GFunctionIndividual.hpp
 */

/* Copyright (C) Dr. Ruediger Berlich and Karlsruhe Institute of Technology
 * (University of the State of Baden-Wuerttemberg and National Laboratory
 * of the Helmholtz Association)
 *
 * Contact: info [at] gemfony (dot) com
 *
 * This file is part of the Geneva library, Gemfony scientific's optimization
 * library.
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


// Standard header files go here
#include <iostream>
#include <cmath>
#include <sstream>
#include <vector>

// Boost header files go here
#include <boost/shared_ptr.hpp>

#ifndef GFUNCTIONINDIVIDUAL_HPP_
#define GFUNCTIONINDIVIDUAL_HPP_

// For Microsoft-compatible compilers
#if defined(_MSC_VER)  &&  (_MSC_VER >= 1020)
#pragma once
#endif

// GenEvA header files go here
#include "GDoubleCollection.hpp"
#include "GParameterSet.hpp"
#include "GDoubleGaussAdaptor.hpp"
#include "GFunctionIndividualDefines.hpp"

namespace Gem
{
namespace GenEvA
{

/************************************************************************************************/
/**
 * This individual searches for a minimum of a number of predefined functions, each capable
 * of processing their input in multiple dimensions. Currently implemented are:
 * - A simple parabola
 * - A "noisy" parabola, featuring an immense number of local optima
 * - The generalized Rosenbrock function
 *
 * Note that the free variables of this example are not equipped with boundaries.
 * See the GBoundedParabola example for ways of specifying boundaries for variables.
 * This class is purely meant for demonstration purposes and in order to check the
 * performance of the Geneva library.
 */
class GFunctionIndividual: public GParameterSet
{
	///////////////////////////////////////////////////////////////////////
	friend class boost::serialization::access;

    template<class Archive>
    void save(Archive & ar, const unsigned int version) const
    {
		using boost::serialization::make_nvp;

		ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(GParameterSet)
		   & BOOST_SERIALIZATION_NVP(demoFunction_);


		// add all local variables here, if you want them to be serialized. E.g.:
		// ar & make_nvp("myLocalVar_",myLocalVar_);
		// or
		// ar & BOOST_SERIALIZATION_NVP(myLocalVar);
		// This also works with objects, if they have a corresponding serialize() function.
		// The first function can be necessary when dealing with templates
    }

    template<class Archive>
    void load(Archive & ar, const unsigned int version)
    {
		using boost::serialization::make_nvp;

		ar & make_nvp("ParameterSet", boost::serialization::base_object<GParameterSet>(*this));

		demoFunction tmpDemoFunction;

		ar & make_nvp("demoFunction_", tmpDemoFunction);
		this->setDemoFunction(tmpDemoFunction);

		// add other local variables here, if you want them to be de-serialized. E.g.:
		// ar & make_nvp("myLocalVar_",myLocalVar_);
		// This also works with objects, if they have a corresponding function.
    }

    BOOST_SERIALIZATION_SPLIT_MEMBER()

	///////////////////////////////////////////////////////////////////////

public:
	/********************************************************************************************/
	/**
	 * The default constructor.
	 */
	GFunctionIndividual()
		: demoFunction_(PARABOLA)
	{ /* nothing */ }

	/********************************************************************************************/
	/**
	 * A standard copy constructor
	 *
	 * @param cp A copy of another GFunctionIndidivual
	 */
	GFunctionIndividual(const GFunctionIndividual& cp)
		: GParameterSet(cp)
		, demoFunction_(cp.demoFunction_)
	{ /* nothing */	}

	/********************************************************************************************/
	/**
	 * The standard destructor
	 */
	~GFunctionIndividual()
	{ /* nothing */	}

	/********************************************************************************************/
	/**
	 * A standard assignment operator
	 *
 	 * @param cp A copy of another GFunctionIndividual
	 */
	const GFunctionIndividual& operator=(const GFunctionIndividual& cp){
		GFunctionIndividual::load(&cp);
		return *this;
	}

	/********************************************************************************************/
	/**
	 * Loads the data of another GFunctionIndividual, camouflaged as a GObject
	 *
	 * @param cp A copy of another GFunctionIndividual, camouflaged as a GObject
	 */
	virtual void load(const GObject* cp){
		const GFunctionIndividual *p_load = conversion_cast(cp, this);

		// Load our parent class'es data ...
		GParameterSet::load(cp);

		// ... and then our own
		demoFunction_ = p_load->demoFunction_;
	}

	/*******************************************************************************************/
	/**
	 * Checks for equality with another GFunctionIndividual object
	 *
	 * @param  cp A constant reference to another GFunctionIndividual object
	 * @return A boolean indicating whether both objects are equal
	 */
	bool operator==(const GFunctionIndividual& cp) const {
		using namespace Gem::Util;
		// Means: The expectation of equality was fulfilled, if no error text was emitted (which converts to "true")
		return !checkRelationshipWith(cp, CE_EQUALITY, 0.,"GFunctionIndividual::operator==","cp", CE_SILENT);
	}

	/*******************************************************************************************/
	/**
	 * Checks for inequality with another GFunctionIndividual object
	 *
	 * @param  cp A constant reference to another GFunctionIndividual object
	 * @return A boolean indicating whether both objects are inequal
	 */
	bool operator!=(const GFunctionIndividual& cp) const {
		using namespace Gem::Util;
		// Means: The expectation of inequality was fulfilled, if no error text was emitted (which converts to "true")
		return !checkRelationshipWith(cp, CE_INEQUALITY, 0.,"GFunctionIndividual::operator!=","cp", CE_SILENT);
	}

	/********************************************************************************************/
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
			const Gem::Util::expectation& e,
			const double& limit,
			const std::string& caller,
			const std::string& y_name,
			const bool& withMessages) const
	{
	    using namespace Gem::Util;
	    using namespace Gem::Util::POD;

		// Check that we are indeed dealing with a GParamterBase reference
		const GFunctionIndividual *p_load = GObject::conversion_cast(&cp,  this);

		// Will hold possible deviations from the expectation, including explanations
	    std::vector<boost::optional<std::string> > deviations;

		// Check our parent class'es data ...
		deviations.push_back(GParameterSet::checkRelationshipWith(cp, e, limit, "GFunctionIndividual", y_name, withMessages));

		// ... and then our local data
		deviations.push_back(checkExpectation(withMessages, "GFunctionIndividual", demoFunction_, p_load->demoFunction_, "demoFunction_", "p_load->demoFunction_", e , limit));

		return evaluateDiscrepancies("GFunctionIndividual", caller, deviations, e);
	}

	/*******************************************************************************************/
	/**
	 * Specifies that a given function should be used for the evaluation step
	 *
	 * @param df The id of the demo function
	 */
	void setDemoFunction(const demoFunction& df) {
		demoFunction_ = df;
	}

	/*******************************************************************************************/
	/**
	 * Allows to retrieve the demo function
	 *
	 * @return The current value of the demoFunction_ variable
	 */
	demoFunction getDemoFunction() const {
		return demoFunction_;
	}

protected:
	/********************************************************************************************/
	/**
	 * Creates a deep clone of this object
	 *
	 * @return A deep clone of this object, camouflaged as a GObject
	 */
	virtual GObject* clone_() const {
		return new GFunctionIndividual(*this);
	}

	/********************************************************************************************/
	/**
	 * The actual fitness calculation takes place here.
	 *
	 * @return The value of this object
	 */
	virtual double fitnessCalculation(){
		// Extract the GDoubleCollection object
		boost::shared_ptr<GDoubleCollection> x = pc_at<GDoubleCollection>(0);

		// Register a suitable transfer function, depending on the value of transferMode_
		switch(demoFunction_){
		case PARABOLA:
			return parabola(*x);
			break;
		case NOISYPARABOLA:
			return noisyParabola(*x);
			break;
		case ROSENBROCK:
			return rosenbrock(*x);
			break;
		}
	}

	/********************************************************************************************/
private:
	/**
	 * This function object gives access to the actual evaluation function
	 */
	demoFunction demoFunction_; ///< Specifies which demo function should be used

	/********************************************************************************************/
	/**
	 * A simple, multi-dimensional parabola
	 *
	 * @param x The input parameters for the function
	 * @return The result of the calculation
	 */
	double parabola(const GDoubleCollection& x) {
		std::size_t parameterSize = x.size();
		double result = 0.;
		for(std::size_t i=0; i<parameterSize; i++) result += GSQUARED(x[i]);
		return result;
	}

	/********************************************************************************************/
	/**
	 * A "noisy" parabola, i.e. a parabola with a very large number of local optima
	 *
	 * @param x The input parameters for the function
	 * @return The result of the calculation
	 */
	double noisyParabola(const GDoubleCollection& x) {
		std::size_t parameterSize = x.size();
		double result = 0.;

		// Great - now we can do the actual calculations. We do this the fancy way ...
		for(std::size_t i=0; i<parameterSize; i++){
			double xsquared = GSQUARED(x[i]);
			result += (cos(xsquared) + 2)*xsquared;
		}

		return result;
	}

	/********************************************************************************************/
	/**
	 * The generalized Rosenbrock function (see e.g. http://en.wikipedia.org/wiki/Rosenbrock_function)
	 *
	 * @param x The input parameters for the function
	 * @return The result of the calculation
	 */
	double rosenbrock(const GDoubleCollection x) {
		std::size_t parameterSize = x.size();
		double result = 0.;

#ifdef DEBUG
		// Check the size of the parameter vector -- must be at least 2
		if(parameterSize < 2) {
			std::ostringstream error;
			error << "In GFunctionIndividual::rosenbrock(): Error!" << std::endl
				  << "Need to use at least two input dimensions, but got " << parameterSize << std::endl;
			throw(Gem::GenEvA::geneva_error_condition(error.str()));
		}
#endif /* DEBUG */

		for(std::size_t i=0; i<(parameterSize-1); i++) {
			double firstTerm = 1.-x[i]; firstTerm *= firstTerm;
			double secondTerm = 100.*GSQUARED(x[i+1]-GSQUARED(x[i]));
			result += firstTerm + secondTerm;
		}

		return result;
	}

	/********************************************************************************************/
};


} /* namespace GenEvA */
} /* namespace Gem */

#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT(Gem::GenEvA::GFunctionIndividual)

#endif /* GFUNCTIONINDIVIDUAL_HPP_ */
