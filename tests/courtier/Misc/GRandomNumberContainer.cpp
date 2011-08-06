/**
 * @file GRandomNumberContainer.cpp
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

#include "GRandomNumberContainer.hpp"

BOOST_CLASS_EXPORT_IMPLEMENT(Gem::Courtier::Tests::GRandomNumberContainer);

namespace Gem {
namespace Courtier {
namespace Tests {

/********************************************************************************************/
/**
 * The default constructor -- only needed for de-serialization purposes.
 */
GRandomNumberContainer::GRandomNumberContainer()
{ /* nothing */ }

/********************************************************************************************/
/**
 * The standard constructor -- Initialization with an amount of random numbers
 *
 * @param nrnr The desired amount of random numbers to be added to the randomNumbers_ vector
 */
GRandomNumberContainer::GRandomNumberContainer(const std::size_t& nrnr) {
	Gem::Hap::GRandomT<Gem::Hap::RANDOMPROXY> gr;
	for(std::size_t i=0; i<nrnr; i++) {
		randomNumbers_.push_back(gr.uniform_01<double>());
	}
}

/********************************************************************************************/
/**
 * The copy constructor
 *
 * @param cp A copy of another GRandomNumberContainer object
 */
GRandomNumberContainer::GRandomNumberContainer(const GRandomNumberContainer& cp)
	: Gem::Courtier::GSubmissionContainerT<GRandomNumberContainer>(cp)
	, randomNumbers_(cp.randomNumbers_)
{ /* nothing */ }

/********************************************************************************************/
/**
 * The destructor
 */
GRandomNumberContainer::~GRandomNumberContainer() {
	randomNumbers_.clear();
}

/********************************************************************************************/
/**
 * Allows to specify the tasks to be performed for this object. We simply sort the array of
 * random numbers.
 *
 * @return A boolean which indicates whether a useful result was obtained
 */
bool GRandomNumberContainer::process() {
	std::sort(randomNumbers_.begin(), randomNumbers_.end());
	return true;
}

/********************************************************************************************/
/**
 * Prints out this functions random number container
 */
void GRandomNumberContainer::print() {
	for(std::size_t i=0; i<randomNumbers_.size(); i++) {
		std::cout << i << ": " << randomNumbers_[i] << std::endl;
	}
}

/********************************************************************************************/

} /* namespace Tests */
} /* namespace Courtier */
} /* namespace Gem */