/**
 * @file GSerialConsumerT.hpp
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
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with the Geneva library. If not, see <http://www.gnu.org/licenses/>.
 *
 * For further information on Gemfony scientific and Geneva, visit
 * http://www.gemfony.eu .
 */

// Global checks, defines and includes needed for all of Geneva
#include "common/GGlobalDefines.hpp"

// Standard headers go here

// Boost headers go here

#ifndef GSERIALCONSUMERT_HPP_
#define GSERIALCONSUMERT_HPP_

// Geneva headers go here
#include "common/GLogger.hpp"
#include "common/GExceptions.hpp"
#include "common/GErrorStreamer.hpp"
#include "courtier/GBrokerT.hpp"
#include "courtier/GBaseConsumerT.hpp"

namespace Gem {
namespace Courtier {

/******************************************************************************/
/**
 * This class adds a serial consumer to the collection of consumers.
 * This allows to use a single implementation of the available
 * optimization algorithms with all available execution modes instead
 * of different implementations of the algorithms for each mode.
 */
template<class processable_type>
class GSerialConsumerT
	: public Gem::Courtier::GBaseConsumerT<processable_type> {
public:
	/***************************************************************************/
	/**
	 * The default constructor. Nothing special here.
	 */
	GSerialConsumerT()
		: Gem::Courtier::GBaseConsumerT<processable_type>(), m_broker_ptr(GBROKER(processable_type))
	{ /* nothing */ }

	/***************************************************************************/
	/**
	 * Standard destructor
	 */
	virtual  ~GSerialConsumerT()
	{ /* nothing */ }

	/***************************************************************************/
	/**
	 * Starts a single worker thread. Termination of the thread is
	 * triggered by a call to GBaseConsumerT<processable_type>::shutdown().
	 */
	virtual void async_startProcessing() override {
		m_processingThread = std::move(std::thread(
			[this]() { this->processItems(); }
		));
	}

	/***************************************************************************/
	/**
	* Finalization code. Sends all threads an interrupt signal.
	*/
	virtual void shutdown() override {
		// This will set the GBaseConsumerT<processable_type>::stop_ flag
		GBaseConsumerT<processable_type>::shutdown();
		// Wait for our local threads to join
		m_processingThread.join();
	}

	/***************************************************************************/
	/**
	* A unique identifier for a given consumer
	*
	* @return A unique identifier for a given consumer
	*/
	virtual std::string getConsumerName() const override {
		return std::string("GSerialConsumerT");
	}

	/***************************************************************************/
	/**
	 * Returns a short identifier for this consumer
	 */
	virtual std::string getMnemonic() const override {
		return std::string("sc");
	}

	/***************************************************************************/
	/**
	 * Returns an indication whether full return can be expected from this
	 * consumer. Since evaluation is performed serially, we assume that this
	 * is possible and return true.
	 */
	virtual bool capableOfFullReturn() const override {
		return true;
	}

 	/***************************************************************************/
 	/**
  	 * Returns the (possibly estimated) number of concurrent processing units.
  	 * A return value of 0 means "unknown".
  	 */
 	virtual std::size_t getNProcessingUnitsEstimate(bool& exact) const override {
		// Mark the answer as exact
		exact=true;
		// Return the result
	 	return boost::numeric_cast<std::size_t>(1);
 	}

private:
	/***************************************************************************/

	GSerialConsumerT(const GSerialConsumerT<processable_type> &); ///< Intentionally left undefined
	const GSerialConsumerT<processable_type> &operator=(
		const GSerialConsumerT<processable_type> &); ///< Intentionally left undefined

	/***************************************************************************/
	/**
	* The function that gets new items from the broker, processes them
	* and returns them when finished. As this function is the main
	* execution point of a thread, we need to catch all exceptions.
	*/
	void processItems() {
		try {
			std::shared_ptr<processable_type> p;
			std::chrono::milliseconds timeout(200);

			while (true) {
				// Have we been asked to stop ?
				if (GBaseConsumerT<processable_type>::stopped()) break;

				// If we didn't get a (valid) item, start again with the while loop
				if (!m_broker_ptr->get(p, timeout)) {
					continue;
				}

#ifdef DEBUG
				// Check that we indeed got a valid item
				if(!p) { // We didn't get a valid item after all
					throw gemfony_exception(
						g_error_streamer(DO_LOG,  time_and_place)
							<< "In GSerialConsumerT<T>::startProcessing(): Error!" << std::endl
							<< "Got empty item when it shouldn't be empty!" << std::endl
					);
				}
#endif /* DEBUG */

				// Do the actual work. We check for processing exceptions here. When
				// found, processing will continue, but we make the user aware of the
				// fact in a warning. Decisions on the fate of the work item should be
				// taken by the recipient of the work item.
				try {
					p->process();
				} catch(const g_processing_exception& e) {
					glogger
						<< "The work item has flagged a processing exception with the message" << std::endl
					   << e << std::endl
					   << "GSerialConsumerT will continue with other work items. It is up to" << std::endl
					   << "the recipient of the work item to decide on the fate of the work item" << std::endl
						<< GWARNING;
				}

				// Return the item to the broker. The item will be discarded
				// if the requested target queue cannot be found.
				try {
					while (!m_broker_ptr->put(p, timeout)) { // Items can get lost here
						// Terminate if we have been asked to stop
						if (GBaseConsumerT<processable_type>::stopped()) {
							glogger
								<< "In GSerialConsimerT::processItems():" << std::endl
								<< "The consumer has been instructed to stop." << std::endl
							   << "The current item will be discarded" << std::endl
							   << GWARNING;

							break;
						}
					}
				} catch (Gem::Courtier::buffer_not_present &) {
					glogger
						<< "In GSerialConsimerT::processItems():" << std::endl
						<< "Target buffer could not be found. The item" << std::endl
						<< "will be discarded" << std::endl
						<< GWARNING;

					continue;
				}
			}
		} catch(gemfony_exception& e) { // Exceptions caught here will terminate the loop
			throw gemfony_exception(
				g_error_streamer(DO_LOG,  time_and_place)
					<< "In GSerialConsumerT::processItems(): Caught gemfony_exception with message"
					<< std::endl
					<< e.what() << std::endl
			);
		} catch (std::exception &e) {
			throw gemfony_exception(
				g_error_streamer(DO_LOG,  time_and_place)
					<< "In GSerialConsumerT::processItems(): Caught std::exception with message" << std::endl
					<< e.what() << std::endl
			);
		} catch (boost::exception &e) {
			throw gemfony_exception(
				g_error_streamer(DO_LOG,  time_and_place)
					<< "In GSerialConsumerT::processItems(): Caught boost::exception with information" << std::endl
					<< boost::diagnostic_information(e) << std::endl
			);
		} catch (...) {
			throw gemfony_exception(
				g_error_streamer(DO_LOG,  time_and_place)
					<< "In GSerialConsumerT::processItems(): Caught unknown exception." << std::endl
			);
		}
	}

	/***************************************************************************/

	std::thread m_processingThread;
	std::shared_ptr<GBrokerT<processable_type>> m_broker_ptr; ///< A shortcut to the broker so we do not have to go through the singleton
};

/******************************************************************************/

} /* namespace Courtier */
} /* namespace Gem */

#endif /* GSERIALCONSUMERT_HPP_ */
