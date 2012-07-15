/**
 * @file GOptimizableI.cpp
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

#include "geneva/GOptimizableI.hpp"

namespace Gem {
namespace Geneva {

/**********************************************************************************/
/**
 * The default constructor
 */
GOptimizableI::GOptimizableI()
{ /* nothing */ }

/**********************************************************************************/
/**
 * The destructor
 */
GOptimizableI::~GOptimizableI()
{ /* nothing */ }

/**********************************************************************************/
/**
 * This is a simple wrapper function that forces the class to start with offset 0
 */
void GOptimizableI::optimize() {
	optimize(0);
}

/**********************************************************************************/
/**
 * Returns information about the type of optimization algorithm. This function needs
 * to be overloaded by the actual algorithms to return the correct type.
 *
 * @return The type of optimization algorithm
 */
personality_oa GOptimizableI::getOptimizationAlgorithm() const {
	return PERSONALITY_NONE;
}

/**********************************************************************************/
/**
 * Checks whether a given algorithm type likes to communicate via the broker
 *
 * @return A boolean indicating whether the algorithm likes to communicate via the broker
 */
bool GOptimizableI::usesBroker() const {
	return false;
}

/**********************************************************************************/

} /* namespace Geneva */
} /* namespace Gem */