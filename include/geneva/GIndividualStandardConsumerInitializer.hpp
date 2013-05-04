/**
 * @file GIndividualStandardConsumerInitializer.hpp
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

// Boost header files go here
#include <boost/shared_ptr.hpp>
#include <boost/type_traits.hpp>
#include <boost/mpl/assert.hpp>


#ifndef GCONSUMERINITIALIZERT_HPP_
#define GCONSUMERINITIALIZERT_HPP_

// For Microsoft-compatible compilers
#if defined(_MSC_VER)  &&  (_MSC_VER >= 1020)
#pragma once
#endif

// Geneva headers go here
#include "common/GLogger.hpp"
#include "common/GGlobalOptionsT.hpp"
#include "courtier/GBaseConsumerT.hpp"
#include "geneva/GIndividual.hpp"
#include "geneva/GConsumerStore.hpp"

namespace Gem {
namespace Geneva {

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/**
 * This base class takes care of adding GIndividual-based consumer objects
 * to a global store
 */
template <typename pl_type, typename c_type>
class GIndividualStandardConsumerInitializer {
public:
   /** @brief The initializing constructor */
   inline GIndividualStandardConsumerInitializer() {
      // Create a smart pointer holding the algorithm
      boost::shared_ptr<GBaseConsumerT<pl_type> > p(new c_type());

      // We require that the algorithm has a default
      // constructor and the static "nickname" data member
      if(!GConsumerStore->setOnce(c_type::nickname, p)) { // Algorithm factory already exists in the store
         glogger
         << "In GIndividualStandardConsumerInitializer<pl_type, c_type>::GIndividualStandardConsumerInitializer(): Error!" << std::endl
         << "Identifier " << c_type::nickname << " already exists in store." << std::endl
         << GTERMINATION;
      } else {
         std::cout << "Registered factory for algorithm \"" << c_type::nickname << "\" in the store." << std::endl;
      }
   }
   /** @brief An empty destructor */
   virtual inline ~GIndividualStandardConsumerInitializer() { /* nothing */ }
};

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/

} /* namespace Geneva */
} /* namespace Gem */

#endif /* GCONSUMERINITIALIZERT_HPP_ */
