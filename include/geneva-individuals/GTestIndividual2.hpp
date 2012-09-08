/**
 * @file GTestIndividual2.hpp
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
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <vector>
#include <list>
#include <algorithm> // for std::sort

// Boost header files go here
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/cast.hpp>

#ifndef GTESTINDIVIDUAL2_HPP_
#define GTESTINDIVIDUAL2_HPP_

// For Microsoft-compatible compilers
#if defined(_MSC_VER) && (_MSC_VER >= 1020)
#pragma once
#endif

// Geneva header files go here
#include "geneva/GParameterSet.hpp"
#include "geneva/GDoubleCollection.hpp"
#include "geneva/GDoubleObject.hpp"
#include "geneva/GDoubleGaussAdaptor.hpp"
#include "geneva/GConstrainedDoubleCollection.hpp"
#include "geneva/GConstrainedDoubleObjectCollection.hpp"
#include "geneva/GConstrainedDoubleObject.hpp"
#include "geneva/GOptimizationEnums.hpp"
#include "geneva/GEAPersonalityTraits.hpp"
#include "geneva/GGDPersonalityTraits.hpp"
#include "geneva/GSwarmPersonalityTraits.hpp"
#include "common/GExceptions.hpp"
#include "common/GGlobalOptionsT.hpp"
#include "common/GCommonEnums.hpp"

namespace Gem
{
namespace Tests
{

/**
 * The types of objects to be tested in this class
 */
enum PERFOBJECTTYPE {
	PERFGDOUBLEOBJECT=0
	, PERFGCONSTRDOUBLEOBJECT=1
	, PERFGCONSTRAINEDDOUBLEOBJECTCOLLECTION=2
	, PERFGDOUBLECOLLECTION=3
	, PERFGCONSTRAINEDDOUBLECOLLECTION=4
};

const PERFOBJECTTYPE POTMIN=PERFGDOUBLEOBJECT;
const PERFOBJECTTYPE POTMAX=PERFGCONSTRAINEDDOUBLEOBJECTCOLLECTION;
const std::size_t NPERFOBJECTTYPES = 5;

/************************************************************************************************/
/**
 * This individual serves as the basis for unit tests of the individual hierarchy. At the time
 * of writing, it was included in order to be able to set the individual's personality without
 * weakening data protection.
 */
class GTestIndividual2 :public Gem::Geneva::GParameterSet
{
	///////////////////////////////////////////////////////////////////////
	friend class boost::serialization::access;

	template<typename Archive>
	void serialize(Archive & ar, const unsigned int) {
		using boost::serialization::make_nvp;

		ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(GParameterSet);
	}
	///////////////////////////////////////////////////////////////////////

public:
	/** @brief The default constructor */
	GTestIndividual2(const std::size_t&, const PERFOBJECTTYPE&);
	/** @brief The copy constructor */
	GTestIndividual2(const GTestIndividual2&);
	/** @brief The standard destructor */
	virtual ~GTestIndividual2();

	/** @brief A standard assignment operator */
	const GTestIndividual2& operator=(const GTestIndividual2&);

	/** @brief Checks for equality with another GTestIndividual2 object */
	bool operator==(const GTestIndividual2& cp) const;
	/** @brief Checks for inequality with another GTestIndividual2 object */
	bool operator!=(const GTestIndividual2& cp) const;

	/** @brief Checks whether a given expectation for the relationship between this object and another object is fulfilled */
	virtual boost::optional<std::string> checkRelationshipWith(
			const GObject&,
			const Gem::Common::expectation&,
			const double&,
			const std::string&,
			const std::string&,
			const bool&
	) const;

protected:
	/** @brief Loads the data of another GTestIndividual2 */
	virtual void load_(const GObject*);
	/** @brief Creates a deep clone of this object */
	virtual GObject* clone_() const;

	/** @brief The actual fitness calculation takes place here. */
	virtual double fitnessCalculation();


private:
	/** @brief The default constructor -- protected, as it is only needed for (de-)serialization purposes */
	GTestIndividual2();

#ifdef GEM_TESTING
public:
	/** @brief Applies modifications to this object. */
	virtual bool modify_GUnitTests();
	/** @brief Performs self tests that are expected to succeed. */
	virtual void specificTestsNoFailureExpected_GUnitTests();
	/** @brief Performs self tests that are expected to fail. */
	virtual void specificTestsFailuresExpected_GUnitTests();

#endif /* GEM_TESTING */
};

} /* namespace Tests */
} /* namespace Gem */

BOOST_CLASS_EXPORT_KEY(Gem::Tests::GTestIndividual2)

#endif /* GTESTINDIVIDUAL2_HPP_ */