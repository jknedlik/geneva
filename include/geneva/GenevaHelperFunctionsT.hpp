/**
 * @file GenevaHelperFunctionsT.hpp
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

// Standard headers go here
#include <vector>
#include <sstream>

// Includes check for correct Boost version(s)
#include "common/GGlobalDefines.hpp"

// Boost headers go here
#include <boost/shared_ptr.hpp>
#include <boost/utility/enable_if.hpp>
#include <boost/type_traits.hpp>
#include <boost/cstdint.hpp>

#ifndef GENEVAHELPERFUNCTIONST_HPP_
#define GENEVAHELPERFUNCTIONST_HPP_

// For Microsoft-compatible compilers
#if defined(_MSC_VER)  &&  (_MSC_VER >= 1020)
#pragma once
#endif

// Our own headers go here
#include "geneva/GOptimizationEnums.hpp"
#include "geneva/GObject.hpp"
#include "geneva/GAdaptorT.hpp"
#include "common/GExceptions.hpp"

namespace Gem
{
namespace Geneva
{

/**************************************************************************************************/
/**
 * This function takes two smart Geneva pointers and copies their contents (if any) with the
 * load / clone functions. boost::enable_if makes sure that this function can only be called
 * for GObject-derivatives (which are then required to have to load/clone interface.
 *
 * @param from The source smart pointer
 * @param to The target smart pointer
 */
template <typename T>
void copyGenevaSmartPointer (
	const boost::shared_ptr<T>& from
	, boost::shared_ptr<T>& to
	, typename boost::enable_if<boost::is_base_of<Gem::Geneva::GObject, T> >::type* dummy = 0
) {
	// Make sure to is empty when from is empty
	if(!from) {
		to.reset();
	} else {
		if(!to) {
			to = from->GObject::clone<T>();
		} else {
			to->load(from);
		}
	}
}

/**************************************************************************************************/
/**
 * This factory function returns default adaptors for a given base type. This function is a trap.
 * Specializations are responsible for the actual implementation.
 *
 * @return The default adaptor for a given base type
 */
template <typename T>
boost::shared_ptr<GAdaptorT<T> > getDefaultAdaptor() {
	raiseException(
			"In getDefaultAdaptor():" << std::endl
			<< "Function called with invalid type."
	);
}

// Specializations for double, boost::int32_t and bool
template <> boost::shared_ptr<GAdaptorT<double> > getDefaultAdaptor<double>();
template <> boost::shared_ptr<GAdaptorT<boost::int32_t> > getDefaultAdaptor<boost::int32_t>();
template <> boost::shared_ptr<GAdaptorT<bool> > getDefaultAdaptor<bool>();

/**************************************************************************************************/

} /* namespace Geneva */
} /* namespace Gem */

#endif /* GENEVAHELPERFUNCTIONST_HPP_ */
