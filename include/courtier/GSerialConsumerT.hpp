/**
 * @file GSerialConsumerT.hpp
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

// Includes check for correct Boost version(s)
#include "common/GGlobalDefines.hpp"

// Boost headers go here

#ifndef GSERIALCONSUMERT_HPP_
#define GSERIALCONSUMERT_HPP_

// For Microsoft-compatible compilers
#if defined(_MSC_VER)  &&  (_MSC_VER >= 1020)
#pragma once
#endif

// Geneva headers go here
#include "courtier/GBrokerT.hpp"
#include "courtier/GConsumer.hpp"

namespace Gem {
namespace Courtier {

/***************************************************************/
/**
 * This class adds a serial consumer to the collection of consumers.
 * This allows to use a single implementation of the available
 * optimization algorithms with all available execution modes instead
 * of different implementations of the algorithms for each mode.
 */
template <class processable_type>
class GSerialConsumerT
	:public Gem::Courtier::GConsumer
{
public:
	/***************************************************************/
	/**
	 * The default constructor. Nothing special here.
	 */
	GSerialConsumerT()
		: Gem::Courtier::GConsumer()
		, stop_(false)
		, broker_(GBROKER(processable_type))
	{ /* nothing */ }

	/***************************************************************/
	/**
	 * Standard destructor
	 */
	~GSerialConsumerT()
	{ /* nothing */ }

	/***************************************************************/
	/**
	 * The function that gets new items from the broker, processes them
	 * and returns them when finished. As this function is the main
	 * execution point of a thread started by the broker, we need to
	 * catch all exceptions.
	 */
	void process() {
		try{
			boost::shared_ptr<processable_type> p;
			Gem::Common::PORTIDTYPE id;
			boost::posix_time::time_duration timeout(boost::posix_time::milliseconds(10));

			while(true){
				// Have we been asked to stop ?
				{
					boost::shared_lock<boost::shared_mutex> lock(stopMutex_);
					if(stop_) break;
				} // explicit scope -- the lock will be destroyed here

				try{
					id = broker_->get(p, timeout);
				}
				catch(Gem::Common::condition_time_out &) { continue; }

				if(p){
					p->process();
					broker_->put(id, p, timeout);
				}
			}
		}
		catch(boost::thread_interrupted&){
			// Terminate
			return;
		}
		catch(std::exception& e) {
			std::ostringstream error;
			error << "In GSerialConsumerT::processItems(): Caught std::exception with message" << std::endl
			      << e.what() << std::endl;
			std::cerr << error.str();
			std::terminate();
		}
		catch(boost::exception& e){
			std::ostringstream error;
		    error << "In GSerialConsumerT::processItems(): Caught boost::exception with message" << std::endl;
		    std::cerr << error.str();
		    std::terminate();
		}
		catch(...) {
			std::ostringstream error;
			error << "In GSerialConsumerT::processItems(): Caught unknown exception." << std::endl;
			std::cerr << error.str();
			std::terminate();
		}
	}

	/***************************************************************/
	/**
	* Finalization code. Sends all threads an interrupt signal.
	* process() then waits for them to join.
	*/
	void shutdown() {
		boost::unique_lock<boost::shared_mutex> lock(stopMutex_);
		stop_=true;
	}


private:
	GSerialConsumerT(const GSerialConsumerT<processable_type>&); ///< Intentionally left undefined
	const GSerialConsumerT<processable_type>& operator=(const GSerialConsumerT<processable_type>&); ///< Intentionally left undefined

	mutable boost::shared_mutex stopMutex_;
	mutable bool stop_; ///< Set to true if we are expected to stop
	boost::shared_ptr<GBrokerT<processable_type> > broker_; ///< A shortcut to the broker so we do not have to go through the singleton
};

/***************************************************************/

} /* namespace Courtier */
} /* namespace Gem */

#endif /* GSERIALCONSUMERT_HPP_ */