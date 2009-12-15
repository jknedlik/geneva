/**
 * @file GTestIndividual1.hpp
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
#include <cstdlib>
#include <sstream>
#include <vector>
#include <list>
#include <algorithm> // for std::sort
#include <utility> // For std::pair

// Boost header files go here
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/cast.hpp>

#ifndef GTESTINDIVIDUAL1_HPP_
#define GTESTINDIVIDUAL1_HPP_

// For Microsoft-compatible compilers
#if defined(_MSC_VER) && (_MSC_VER >= 1020)
#pragma once
#endif

// GenEvA header files go here
#include "GParameterSet.hpp"
#include "GDoubleGaussAdaptor.hpp"
#include "GBoundedDoubleCollection.hpp"
#include "GBoundedDouble.hpp"
#include "GenevaExceptions.hpp"
#include "GGlobalOptionsT.hpp"
#include "GEnums.hpp"

namespace Gem
{
namespace GenEvA
{
/************************************************************************************************/
/**
 * This individual serves as the basis for unit tests of the individual hierarchy. At the time
 * of writing, it was included in order to be able to set the individual's personality without
 * weakening data protection.
 */
class GTestIndividual1 :public GParameterSet
{
	///////////////////////////////////////////////////////////////////////
	friend class boost::serialization::access;

	template<typename Archive>
	void serialize(Archive & ar, const unsigned int) {
		using boost::serialization::make_nvp;

		ar & make_nvp("GParameterSet", boost::serialization::base_object<GParameterSet>(*this));

		/* Add your own class-variables here in the following way:
		 * ar & make_nvp("myVar", myVar);
		 */
	}
	///////////////////////////////////////////////////////////////////////

public:
	/********************************************************************************************/
	/**
	 * The default constructor.
	 */
	GTestIndividual1() :GParameterSet()
	{	/* nothing */ }

	/********************************************************************************************/
	/**
	 * A simple constructor that initializes this object with a collection of bounded
	 * double variables.
	 *
	 * @param dim The amount of variables
	 * @param min The lower boundary of the variables
	 * @param max The upper boundary of the variables
	 */
	GTestIndividual1(const std::size_t& dim,
					 const double& min,
					 const double& max) :GParameterSet()
	 {
		// Set up a GBoundedDoubleCollection
		boost::shared_ptr<GBoundedDoubleCollection> gbdc_ptr(new GBoundedDoubleCollection());

		// Create a suitable adaptor (sigma=0.1, sigma-adaption=0.5, min sigma=0, max sigma=0,5)
		boost::shared_ptr<GDoubleGaussAdaptor> gdga_ptr(new GDoubleGaussAdaptor(0.1, 0.5, 0., 0.5));
		gdga_ptr->setAdaptionThreshold(1); // Mutation parameters are adapted after each mutation
		gdga_ptr->setRnrGenerationMode(Gem::Util::RNRFACTORY); // Random number generation in the factory
		gdga_ptr->setMutationProbability(0.05); // The likelihood for a parameter to be mutated

		// Register the adaptor with the collection. You could also add individual adaptors
		// to the GBoundedDouble objects below.
		gbdc_ptr->addAdaptor(gdga_ptr);

		// Add bounded double objects
		for(std::size_t i=0; i<dim; i++) {
			// GBoundedDouble will start with random values in the range [min:max]
			boost::shared_ptr<GBoundedDouble> gbd_ptr(new GBoundedDouble(min, max) );

			// Add a GBoundedDouble object to the collection
			gbdc_ptr->push_back(gbd_ptr);
		}

		// Add the collection to this object
		this->push_back(gbdc_ptr);
	 }

	/********************************************************************************************/
	/**
	 * The standard destructor
	 */
	virtual ~GTestIndividual1()
	{ /* nothing */	}

	/********************************************************************************************/
	/**
	 * A standard assignment operator
	 *
	 * @param cp A copy of another GTestIndividual1 object
	 * @return A constant reference to this object
	 */
	const GTestIndividual1& operator=(const GTestIndividual1& cp){
		GTestIndividual1::load(&cp);
		return *this;
	}

	/********************************************************************************************/
	/**
	 * Creates a deep clone of this object
	 *
	 * @return A deep clone of this object, camouflaged as a GObject
	 */
	virtual GObject* clone() const {
		return new GTestIndividual1(*this);
	}

	/********************************************************************************************/
	/**
	 * Loads the data of another GTestIndividual1, camouflaged as a GObject.
	 *
	 * @param cp A copy of another GTestIndividual1, camouflaged as a GObject
	 */
	virtual void load(const GObject* cp)
	{
		// Check that we are indeed dealing with a GBoundedNumT<T> reference
		const GTestIndividual1 *gsi_load = GObject::conversion_cast(cp,  this);

		// Load our parent's data
		GParameterSet::load(cp);

		// Load local data here like this:
		// myVar = gsi_load->myVar;
	}

	/********************************************************************************************/
	/**
	 * Allows to set the individual's personality. Note that this is not a normal feature of
	 * individuals but has been added in this particular individual in order to allow unit tests.
	 */
	void setPersonalityType(const Gem::GenEvA::personality& pers) {
		this->setPersonality(pers);
	}

	/********************************************************************************************/
	/**
	 * Allows to retrieve the individual's personality. Note that this is not a normal feature of
	 * individuals but has been added in this particular individual in order to allow unit tests.
	 */
	personality getPersonalityType() const {
		return this->getPersonality();
	}

protected:
	/********************************************************************************************/
	/**
	 * The actual fitness calculation takes place here.
	 *
	 * @return The value of this object
	 */
	virtual double fitnessCalculation(){
		double result = 0.;

		// Extract the GDoubleCollection object. In a realistic scenario, you might want
		// to add error checks here upon first invocation.
		boost::shared_ptr<GBoundedDoubleCollection> vC = pc_at<GBoundedDoubleCollection>(0);

		// Calculate the value of the parabola
		for(std::size_t i=0; i<vC->size(); i++)
			result += pow(vC->at(i)->value(), 2);

		return result;
	}

private:
	/********************************************************************************************/
	// You can add other variables here. Do not forget to serialize them if necessary
	// int myVar;
};

} /* namespace GenEvA */
} /* namespace Gem */

// Needed for serialization purposes
#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT(Gem::GenEvA::GTestIndividual1)

#endif /* GTESTINDIVIDUAL1_HPP_ */
