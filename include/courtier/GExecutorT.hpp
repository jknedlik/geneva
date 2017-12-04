/**
 * @file GExecutorT.hpp
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

#include <cmath>
#include <sstream>
#include <algorithm>
#include <vector>
#include <utility>
#include <functional>
#include <memory>
#include <type_traits>
#include <tuple>
#include <chrono>
#include <exception>
#include <thread>
#include <mutex>

// Boost headers go here
#include <boost/lexical_cast.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/tracking.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/exception/diagnostic_information.hpp>

#ifndef GEXECUTOR_HPP_
#define GEXECUTOR_HPP_


// Geneva headers go here
#include "common/GCommonInterfaceT.hpp"
#include "common/GExceptions.hpp"
#include "common/GLogger.hpp"
#include "common/GExpectationChecksT.hpp"
#include "common/GCommonMathHelperFunctionsT.hpp"
#include "common/GCommonHelperFunctionsT.hpp"
#include "common/GParserBuilder.hpp"
#include "common/GPlotDesigner.hpp"
#include "common/GSerializationHelperFunctionsT.hpp"
#include "common/GThreadPool.hpp"
#include "courtier/GBufferPortT.hpp"
#include "courtier/GBrokerT.hpp"
#include "courtier/GCourtierEnums.hpp"
#include "courtier/GProcessingContainerT.hpp"
#include "courtier/GCourtierHelperFunctions.hpp"

namespace Gem {
namespace Courtier {

/**
 * Invariants of this hierarchy:
 * - for consumers indicating that they are capable of full return, the process()
 *   function must always return a valid result (even if the user-code crashes)
 */

// TODO: Even in the case "capable of full return" we need to take into account
// that work items may return unprocessed. E.g. the processing code itself
// might throw.
// TODO: In G_OptimizationAlgorithm_Base(cpp/hpp): serialize-exports instantiations with processable_type == GParameterSet
// TODO: Some variables are still not listet in all Gemfony-API functions
// TODO: Do we need to serialize further items ?

/******************************************************************************/
/**
 * Status information for the workOn function and helper functions
 */
struct executor_status_t {
	 bool is_complete = false; ///< Indicates whether a complete set of current items was obtained
	 bool has_errors  = false; ///< Indicates whether there were errors during processing of current items
};

/******************************************************************************/
/**
 * This class centralizes some functionality and data that is needed to perform
 * serial, parallel local or networked execution for a set of work items. Its main
 * purpose is to avoid duplication of code. Derived classes may deal with different
 * types of parallel execution, including connection to a broker and multi-threaded
 * execution. The serial mode is meant for debugging purposes only. The class
 * follows the Gemfony conventions for loading / cloning and comparison with
 * other objects. The main function "workOn" returns a struct which indicates whether
 * all submitted items have returned ("is_complete") and whether there were errors
 * ("has_errors"). Return items may have errors, i.e. it is possible that a set
 * of work items is complate, but has errors.
 */
template<typename processable_type>
class GBaseExecutorT
	: public Gem::Common::GCommonInterfaceT<GBaseExecutorT<processable_type>>
{
	 // Make sure processable_type adheres to the GProcessingContainerT interface
	 static_assert(
		 std::is_base_of<Gem::Courtier::GProcessingContainerT<processable_type, typename processable_type::result_type>, processable_type>::value
		 , "GBaseExecutorT: processable_type does not adhere to the GProcessingContainerT<> interface"
	 );

public:
	 /////////////////////////////////////////////////////////////////////////////

	 friend class boost::serialization::access;

	 template<typename Archive>
	 void serialize(Archive &ar, const unsigned int) {
		 using boost::serialization::make_nvp;

		 ar
		 & make_nvp("GCommonInterfaceT_GBaseExecutotT"
						, boost::serialization::base_object<Gem::Common::GCommonInterfaceT<GBaseExecutorT<processable_type>>>(*this))
		 & BOOST_SERIALIZATION_NVP(m_maxResubmissions);
	 }

	 /////////////////////////////////////////////////////////////////////////////

	 /***************************************************************************/
	 /** @brief The default constructor */
	 GBaseExecutorT() = default;

	 /***************************************************************************/
	 /**
	  * The copy constructor. No local data to be copied.
	  * m_submission_counter is just a temporary which always
	  * starts counting at 0.
	  *
	  * @param cp A copy of another GBrokerConnector object
	  */
	 GBaseExecutorT(const GBaseExecutorT<processable_type> &cp)
		 : Gem::Common::GCommonInterfaceT<GBaseExecutorT<processable_type>>(cp)
		 , m_maxResubmissions(cp.m_maxResubmissions)
	 { /* nothing */ }

	 /***************************************************************************/
	 /** @brief The standard destructor */
	 virtual ~GBaseExecutorT() = default;

	 /***************************************************************************/
	 /**
	  * Checks for equality with another GBaseExecutorT<processable_type> object
	  *
	  * @param  cp A constant reference to another GBaseExecutorT<processable_type> object
	  * @return A boolean indicating whether both objects are equal
	  */
	 bool operator==(const GBaseExecutorT<processable_type>& cp) const {
		 using namespace Gem::Common;
		 try {
			 this->compare(cp, Gem::Common::expectation::CE_EQUALITY, CE_DEF_SIMILARITY_DIFFERENCE);
			 return true;
		 } catch (g_expectation_violation &) {
			 return false;
		 }
	 }

	 /***************************************************************************/
	 /**
	  * Checks for inequality with another GBaseExecutorT<processable_type> object
	  *
	  * @param  cp A constant reference to another GBaseExecutorT<processable_type> object
	  * @return A boolean indicating whether both objects are inequal
	  */
	 bool operator!=(const GBaseExecutorT<processable_type>& cp) const {
		 using namespace Gem::Common;
		 try {
			 this->compare(cp, Gem::Common::expectation::CE_INEQUALITY, CE_DEF_SIMILARITY_DIFFERENCE);
			 return true;
		 } catch (g_expectation_violation &) {
			 return false;
		 }
	 }

	 /***************************************************************************/
	 /**
	  * A standard assignment operator for GBaseExecutorT<processable_type> objects,
	  *
	  * @param cp A copy of another GBaseExecutorT<processable_type> object
	  * @return A constant reference to this object
	  */
	 GBaseExecutorT<processable_type>& operator=(const GBaseExecutorT<processable_type> &cp) {
		 this->load_(&cp);
		 return *this;
	 }

	 /***************************************************************************/
	 /**
	  * Returns the name of this class
	  */
	 std::string name() const override {
		 return std::string("GBaseExecutorT<processable_type>");
	 }

	 /***************************************************************************/
	 /**
	  * General initialization function to be called prior to the first submission
	  */
	 void init() {
		 this->init_();
	 }

	 /***************************************************************************/
	 /**
	  * General finalization function to be called after the last submission
	  */
	 void finalize() {
		 this->finalize_();
	 }

	 /***************************************************************************/
	 /**
	  * Submits and retrieves a set of work items in cycles / iterations. Each
	  * iteration represents a cycle of work item submissions and (posssibly full) retrieval.
	  * Iterations may not overlap, i.e. this function may not be called on the same
	  * GExecutorT object while another call is still running. The function aims to
	  * prevent this situation by setting and releasing a lock during its run-time.
	  * Work items need to be derived from the GProcessingContainerT, and need to have been marked with
	  * either IGNORE (these will not be processed) or DO_PROCESS. After processing,
	  * they will have one of the flags IGNORE (if they were not due to be processed),
	  * EXCEPTION_CAUGHT (if some exception was raised during processing) or ERROR_FLAGGED
	  * (if the user code has marked a solution as unusable). After a timeout, items may
	  * still have the DO_PROCESS flag set (if items did not return in time). Note that
	  * items still marked as DO_PROCESS may well return in later iterations. It is thus
	  * also possible that returned items do not belong to the current submission cycle.
	  * They will be appended to the m_old_work_items_vec vector. You might thus have to
	  * post-process such "old" work items and decide what to do with them. This vector
	  * will be cleared for each new iteration. Hence, if you are interested in old work
	  * items, you need to retrieve them before a new submission of work items. The return
	  * code "is_complete" means that there were responses for each submitted work item
	  * of the current iteration. The return code "has_errors" means that some or all of
	  * them have had errors during processing.
	  *
	  * @param workItems A vector with work items to be evaluated beyond the broker
	  * @param resubmitUnprocessed Indicates whether unprocessed items should be resubmitted
	  * @param caller Optionally holds information on the caller
	  * @return A struct of booleans indicating whether all items were processed successfully and whether there were errors
	  */
	 executor_status_t workOn(
		 std::vector<std::shared_ptr<processable_type>>& workItems
		 , bool resubmitUnprocessed = false
		 , const std::string &caller = std::string()
	 ) {
		 //------------------------------------------------------------------------------------------
		 // Make sure only one instance of this function may be called at the same time. If locking
		 // the mutex fails, some other call to this function is still alive, which is a severe error
		 std::unique_lock<std::mutex> workon_lock(m_concurrent_workon_mutex, std::defer_lock);
		 if(!workon_lock.try_lock()) {
			 throw gemfony_exception(
				 g_error_streamer(DO_LOG, time_and_place)
					 << "In GBaseExeuctorT<processable_type>::workOn(): Another call to this function still seems" << std::endl
					 << "to be active which is a severe error."
			 );
		 }

		 //------------------------------------------------------------------------------------------
		 // The main business logic of item submission

		 // Perform necessary setup work for an iteration
		 this->iterationInit(workItems);

		 std::size_t nResubmissions = 0;
		 executor_status_t status {false /* is_complete */, false /* has_errors */};
		 do {
			 //-----------------------
			 // Submission and retrieval

			 // Submit all work items.
			 m_expectedNumber = this->submitAllWorkItems(workItems);

			 // Wait for work items to complete. This function needs to
			 // be re-implemented in derived classes.
			 auto current_status = waitForReturn(
				 workItems
				 , m_old_work_items_vec
			 );

			 // There may not be errors during resubmission, so we need to save the "error state"
			 if (current_status.is_complete) status.is_complete = true;
			 if (current_status.has_errors) status.has_errors = true;

			 //-----------------------
			 // Normal operation -- all current items have returned

			 // We can always break if we are complete
			 if (status.is_complete) break;

			 //-----------------------
			 // Not complete. We might nevertheless have to leave

			 // Leave if we haven't been asked to resubmit unprocessed items,
			 // even if we do not have a complete set of work items. Also leave
			 // if we have reached the maximum number of resubmissions or
			 // m_maxResubmissions was explicitly set to 0.
			 if (
				 !resubmitUnprocessed
				 || (resubmitUnprocessed && m_maxResubmissions == 0)
			    || (++nResubmissions >= m_maxResubmissions)
			 ) break;
			 //-----------------------
		 } while(true);

		 // Perform necessary cleanup work for an iteration
		 this->iterationFinalize(workItems);

		 //------------------------------------------------------------------------------------------

		 // Give feedback to the audience (may be overloaded in derived classes)
		 this->visualize_performance();

		 // Update the iteration counter
		 m_iteration_counter++;

		 // Note: unprocessed items will be returned to the caller who needs to deal with them
		 return status;
	 }

	 /***************************************************************************/
	 /**
	  * Retrieves a copy of the old work items vector. Calling this function will
	  * clear the vector!
	  *
	  * @return A copy of the old work items vector
	  */
	 std::vector<std::shared_ptr<processable_type>> getOldWorkItems() {
		 return std::move(m_old_work_items_vec);
	 }

	 /***************************************************************************/
	 /**
	  * Adds local configuration options to a GParserBuilder object
	  *
	  * @param gpb The GParserBuilder object to which configuration options should be added
	  */
	 virtual void addConfigurationOptions(
		 Gem::Common::GParserBuilder &gpb
	 ) BASE {
		 gpb.registerFileParameter<std::size_t>(
			 "maxResubmissions" // The name of the variable
			 , DEFAULTMAXRESUBMISSIONS // The default value
			 , [this](std::size_t r) {
				 this->setMaxResubmissions(r);
			 }
		 )
			 << "The amount of resubmissions allowed if a full return of work" << std::endl
			 << "items was expected but only a subset has returned";
	 }

	 /***************************************************************************/
	 /**
	  * Specifies how often work items should be resubmitted in the case a full return
	  * of work items is expected.
	  *
	  * @param maxResubmissions The maximum number of allowed resubmissions
	  */
	 void setMaxResubmissions(std::size_t maxResubmissions) {
		 m_maxResubmissions = maxResubmissions;
	 }

	 /***************************************************************************/
	 /**
	  * Returns the maximum number of allowed resubmissions
	  *
	  * @return The maximum number of allowed resubmissions
	  */
	 std::size_t getMaxResubmissions() const {
		 return m_maxResubmissions;
	 }

	 /***************************************************************************/
	 /**
	  * Retrieve the number of individuals returned during the last iteration
	  */
	 std::size_t getNReturnedLast() const noexcept {
		 return m_n_returnedLast;
	 }

	 /***************************************************************************/
	 /**
	  * Retrieve the number of individuals NOT returned during the last iteration
	  */
	 std::size_t getNNotReturnedLast() const noexcept {
		 return m_n_notReturnedLast;
	 }

	 /***************************************************************************/
	 /**
 	  * Retrieves the current number of old work items in this iteration
 	  */
	 std::size_t getNOldWorkItems() const noexcept {
		 return m_n_oldWorkItems;
	 }

	 /***************************************************************************/
	 /**
	  * Retrieves the number of work items with errors in this iteration
	  */
	 std::size_t getNErroneousWorkItems() const noexcept {
		 return m_n_erroneousItems;
	 }

	 /***************************************************************************/
	 /**
	  * Searches for compliance with expectations with respect to another object
	  * of the same type
	  */
	 void compare(
		 const GBaseExecutorT<processable_type>& cp // the other object
		 , const Gem::Common::expectation& e // the expectation for this object, e.g. equality
		 , const double& limit // the limit for allowed deviations of floating point types
	 ) const override {
		 using namespace Gem::Common;

		 // Check that we are dealing with a GDecorator reference independent of this object and convert the pointer
		 const GBaseExecutorT<processable_type> *p_load = Gem::Common::g_convert_and_compare(cp, this);

		 GToken token("GBaseExecutorT<processable_type>", e);

		 // Compare our parent data ...
		 Gem::Common::compare_base<GCommonInterfaceT<GBaseExecutorT<processable_type>>>(IDENTITY(*this, *p_load), token);

		 // ... and then our local data
		 compare_t(IDENTITY(this->m_maxResubmissions,  p_load->m_maxResubmissions), token);

		 // React on deviations from the expectation
		 token.evaluate();
	 }

protected:
	 /***************************************************************************/
	 /**
	  * Loads the data of another object
	  */
	 void load_(const GBaseExecutorT<processable_type>* cp) override {
		 // Check that we are dealing with a GDecorator reference independent of this object and convert the pointer
		 const auto p_load_ptr = Gem::Common::g_convert_and_compare(cp, this);

		 // No parent class with loadable data

		 // Copy local data
		 m_maxResubmissions = p_load_ptr->m_maxResubmissions;
	 }

	 /***************************************************************************/
	 /**
	  * General initialization function to be called prior to the first submission
	  */
	 virtual void init_() BASE { /* nothing */ }

	 /***************************************************************************/
	 /**
	  * General finalization function to be called after the last submission
	  */
	 virtual void finalize_() BASE { /* nothing */ }

	 /***************************************************************************/
	 /**
	  * Allow to perform necessary setup work for an iteration. Derived classes
	  * should make sure this base function is called first when they overload
	  * this function.
	  */
	 virtual void iterationInit_(
		 std::vector<std::shared_ptr<processable_type>>& workItems
	 ) BASE {
		 //------------------------------------------------------------------------------------------
		 // Reset some counters

		 m_n_returnedLast = 0;
		 m_n_notReturnedLast = 0;
		 m_n_oldWorkItems = 0;
		 m_n_erroneousItems = 0;

		 // Clear old work items not cleared by the caller after the last iteration
		 m_old_work_items_vec.clear();
	 }

	 /***************************************************************************/
	 /**
	  * Non-virtual wrapper for iterationInit_()
	  */
	 void iterationInit(
		 std::vector<std::shared_ptr<processable_type>>& workItems
	 ) {
		 this->iterationInit_(workItems);
	 }

	 /***************************************************************************/
	 /**
	  * Allows to perform necessary cleanup work for an iteration. Derived classes
	  * should make sure this base function is called last when they overload
	  * this function.
	  */
	 virtual void iterationFinalize_(
		 std::vector<std::shared_ptr<processable_type>>& workItems
	 ) BASE {
		 // Find out about the number of items that have not returned (yet) from the last submission.
		 // These are equivalent to the items that still have the DO_PROCESS flag set.
		 m_n_notReturnedLast = this->countItemsWithStatus(workItems, processingStatus::DO_PROCESS);
		 // The number of actually returned items is equal to the number of expected items minus the number of not returned items
		 m_n_returnedLast    = m_expectedNumber - m_n_notReturnedLast;
		 // Count the number of work items with errors. We need to count two flags
		 m_n_erroneousItems  = this->countItemsWithStatus(workItems, processingStatus::ERROR_FLAGGED);
		 m_n_erroneousItems += this->countItemsWithStatus(workItems, processingStatus::EXCEPTION_CAUGHT);
		 // Remove unprocessed items from the list of old work items
		 m_n_oldWorkItems    = this->cleanItemVectorWithoutFlag(m_old_work_items_vec, processingStatus::PROCESSED);

		 // Sort remaining old work items according to their position so they can be readily used by the caller.
		 std::sort(
			 m_old_work_items_vec.begin()
			 , m_old_work_items_vec.end()
			 , [](std::shared_ptr<processable_type> x_ptr, std::shared_ptr<processable_type> y_ptr) -> bool {
				 using namespace boost;
				 return x_ptr->getSubmissionPosition() < y_ptr->getSubmissionPosition();
			 }
		 );
	 }

	 /***************************************************************************/
	 /**
	  * Non-virtual wrapper for iterationFinalize_()
	  */
	 void iterationFinalize(
		 std::vector<std::shared_ptr<processable_type>>& workItems
	 ) {
		 this->iterationFinalize_(workItems);
	 }

	 /***************************************************************************/
	 /**
	  * Submission of all work items in the list
	  *
	  * TODO: Take care of situations where submission may block
	  */
	 std::size_t submitAllWorkItems(std::vector<std::shared_ptr<processable_type>>& workItems) {
		 // Submit work items
		 SUBMISSION_POSITION_TYPE pos_cnt = 0;
		 std::size_t nSubmittedItems = 0;
		 for(auto w_ptr: workItems) { // std::shared_ptr may be copied
#ifdef DEBUG
			 if(!w_ptr) {
				 throw gemfony_exception(
					 g_error_streamer(DO_LOG, time_and_place)
						 << "In GBaseExecutorT<processable_type>::submitAllWorkItems(): Error" << std::endl
						 << "Received empty work item in position "  << pos_cnt << std::endl
						 << "m_iteration_counter = " << m_iteration_counter << std::endl
				 );
			 }
#endif

			 // Is the item due to be submitted ?
			 processingStatus ps = w_ptr->getProcessingStatus();
			 if(processingStatus::DO_PROCESS == ps) {
				 // Update some internal variables
				 w_ptr->setSubmissionCounter(m_iteration_counter);
				 w_ptr->setSubmissionPosition(pos_cnt);

				 // Do the actual submission
				 this->submit(w_ptr);

				 // Update the submission counter
				 nSubmittedItems++;
			 } else if(processingStatus::IGNORE != ps) {
				 throw gemfony_exception(
					 g_error_streamer(DO_LOG, time_and_place)
						 << "In GBaseExecutorT<processable_type>::submitAllWorkItems(): Error" << std::endl
						 << "processing status is neither DO_PROCESS nor IGNORE. We got " << ps << std::endl
				 );
			 }

			 pos_cnt++;
		 }

		 // Set the start time of the new iteration. How this time is determined depends
		 // on the actual executor. m_executionStartTime will be equal to the first submission
		 // time for serial and multi-threaded executors, but equal to the first retrieval time
		 // from the GBufferPortT class for the broker executor. NOTE that the following call
		 // may block, if a start time cannot yet be determined.
		 m_executionStartTime = this->determineExecutionStartTime();

		 return nSubmittedItems;
	 }

	 /***************************************************************************/
	 /**
	  * Returns the current iteration as used for the tagging of work items
	  * @return The id of the current submission cycle
	  */
	 ITERATION_COUNTER_TYPE get_iteration_counter() const noexcept {
		 return m_iteration_counter;
	 }

	 /***************************************************************************/
	 /**
	  * Retrieves the expected number of work items in the current iteration
	  */
	 std::size_t getExpectedNumber() const noexcept {
		 return m_expectedNumber;
	 }

	 /***************************************************************************/
	 /**
	  * Retrieves the time when execution has started
	  */
	 std::chrono::high_resolution_clock::time_point getExecutionStartTime() const {
		 return this->m_executionStartTime;
	 }

	 /***************************************************************************/
	 /**
	  * A little helper function to make the code easier to read
	  */
	 std::chrono::high_resolution_clock::time_point now() const {
		 return std::chrono::high_resolution_clock::now();
	 }

	 /***************************************************************************/

	 /** @brief Submits a single work item */
	 virtual void submit(
		 std::shared_ptr<processable_type>
	 ) BASE = 0;

	 /***************************************************************************/
	 /**
	  * Waits for work items to return and checks for completeness
	  * (i.e. all items have returned and there were no exceptions).
	  *
	  * @param workItems A vector with work items to be evaluated beyond the broker
	  * @param oldWorkItems A vector with work items that have returned after the threshold
	  * @return A struct of booleans indicating whether all items were processed successfully and whether there were errors
	  */
	 virtual executor_status_t waitForReturn(
		 std::vector<std::shared_ptr<processable_type>>& workItems
		 , std::vector<std::shared_ptr<processable_type>>&
	 ) BASE = 0;


	 /***************************************************************************/
	 /**
	  * Count the number of work items in a batch with a specific flag
	  */
	 std::size_t countItemsWithStatus(
		 const std::vector<std::shared_ptr<processable_type>>& workItems
		 , const processingStatus& ps
	 ) {
		 return boost::numeric_cast<std::size_t>(
			 std::count_if(
				 workItems.begin()
				 , workItems.end()
				 , [ps](std::shared_ptr<processable_type> p) {
					 if(ps == p->getProcessingStatus()) {
						 return true;
					 }
					 return false;
				 }
			 )
		 );
	 }

	 /***************************************************************************/
	 /**
	  * Checks for the status with respect to errors and processing status
	  * of a collecion of work items
	  */
	 executor_status_t checkExecutionState(
		 const std::vector<std::shared_ptr<processable_type>>& workItems
	 ) {
		 bool is_complete = true;
		 bool has_errors = false;

		 // First check that there are no remaining items with the DO_PROCESS status
		 auto nUnprocessed = this->countItemsWithStatus(
			 workItems
			 , processingStatus::DO_PROCESS
		 );
		 if(nUnprocessed > 0) is_complete = false;

		 // Now check for error states
		 auto nErrorState  = this->countItemsWithStatus(
			 workItems
			 , processingStatus::ERROR_FLAGGED
		 );
		 nErrorState      += this->countItemsWithStatus(
			 workItems
			 , processingStatus::EXCEPTION_CAUGHT
		 );
		 if(nErrorState > 0) has_errors = true;

		 return executor_status_t{is_complete, has_errors};
	 }

	 /***************************************************************************/
	 /**
	  * Removes work items without a given flag from the vector and return
	  * the number of remaining items
	  */
	 std::size_t cleanItemVectorWithoutFlag(
		 std::vector<std::shared_ptr<processable_type>>& items_vec
		 , const processingStatus& desired_ps
	 ) {
		 // Remove unprocessed items from the list of old work items
		 Gem::Common::erase_if(
			 items_vec
			 , [this, desired_ps](std::shared_ptr<processable_type> item_ptr) {
				 auto ps = item_ptr->getProcessingStatus();
				 if(ps != desired_ps) {
#ifdef DEBUG
					 // Some logging, as this condition should be very rare and might indicate a more general problem.
					 glogger
						 << "In GBaseExeuctorT<processable_type>::cleanItemVectorWithoutFlag():" << std::endl
						 << "Removing work item in submission " << this->get_iteration_counter() << std::endl
						 << "because it does not have the desired status " << desired_ps << std::endl
						 << "Found status " << ps << " instead." << std::endl
						 << GLOGGING;
#endif

					 // Erase
					 return true;
				 }

				 // Do not erase
				 return false;
			 }
		 );

		 // Return the number of remaining items
		 return items_vec.size();
	 }

private:
	 /***************************************************************************/
	 /** @brief Determination of the time when execution has started */
	 virtual std::chrono::high_resolution_clock::time_point determineExecutionStartTime() const BASE = 0;

	 /***************************************************************************/
	 /** @brief Graphical progress feedback */
	 virtual void visualize_performance() BASE = 0;

	 /***************************************************************************/
	 // Data

	 /* @brief Counts the number of submissions initiated for this object; Note: not serialized! */
	 ITERATION_COUNTER_TYPE m_iteration_counter = ITERATION_COUNTER_TYPE(0);

	 std::size_t m_expectedNumber = 0; ///< The number of work items to be submitted (and expected back)

	 /** @brief Temporary that holds the start time for the retrieval of items in a given iteration */
	 std::chrono::high_resolution_clock::time_point m_executionStartTime = std::chrono::high_resolution_clock::now();

	 /** @brief The maximum number of re-submissions allowed if a full return of submitted items is attempted*/
	 std::size_t m_maxResubmissions = DEFAULTMAXRESUBMISSIONS;

	 std::size_t m_n_returnedLast = 0; ///< The number of individuals returned in the last iteration cycle
	 std::size_t m_n_notReturnedLast = 0; ///< The number of individuals NOT returned in the last iteration cycle
	 std::size_t m_n_oldWorkItems = 0; ///< The number of old work items returned in a given iteration
	 std::size_t m_n_erroneousItems = 0; ///< The number of work items with errors in the current iteration

	 std::vector<std::shared_ptr<processable_type>> m_old_work_items_vec; ///< Temporarily holds old work items of the current iteration

	 std::mutex m_concurrent_workon_mutex; ///< Makes sure the workOn function is only called once at the same time on this object
};

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/**
 * This class executes work items serially. It is mostly meant for debugging
 * purposes
 */
template<typename processable_type>
class GSerialExecutorT
	: public GBaseExecutorT<processable_type>
{
	 ///////////////////////////////////////////////////////////////////////

	 friend class boost::serialization::access;

	 template<typename Archive>
	 void serialize(Archive &ar, const unsigned int) {
		 using boost::serialization::make_nvp;

		 ar & make_nvp("GBaseExecutorT", boost::serialization::base_object<GBaseExecutorT<processable_type>>(*this));
	 }

	 ///////////////////////////////////////////////////////////////////////

public:
	 /***************************************************************************/
	 /** @brief The default constructor */
	 GSerialExecutorT() = default;

	 /***************************************************************************/
	 /**
	  * The copy constructor
	  *
	  * @param cp A copy of another GBrokerConnector object
	  */
	 GSerialExecutorT(const GSerialExecutorT<processable_type> &cp)
		 : GBaseExecutorT<processable_type>(cp)
	 { /* nothing */ }

	 /***************************************************************************/
	 /** @brief The destructor */
	 virtual ~GSerialExecutorT() = default;

	 /***************************************************************************/
	 /**
	  * Checks for equality with another GSerialExecutorT<processable_type> object
	  *
	  * @param  cp A constant reference to another GSerialExecutorT<processable_type> object
	  * @return A boolean indicating whether both objects are equal
	  */
	 bool operator==(const GSerialExecutorT<processable_type>& cp) const {
		 using namespace Gem::Common;
		 try {
			 this->compare(cp, Gem::Common::expectation::CE_EQUALITY, CE_DEF_SIMILARITY_DIFFERENCE);
			 return true;
		 } catch (g_expectation_violation &) {
			 return false;
		 }
	 }

	 /***************************************************************************/
	 /**
	  * Checks for inequality with another GSerialExecutorT<processable_type> object
	  *
	  * @param  cp A constant reference to another GSerialExecutorT<processable_type> object
	  * @return A boolean indicating whether both objects are inequal
	  */
	 bool operator!=(const GSerialExecutorT<processable_type>& cp) const {
		 using namespace Gem::Common;
		 try {
			 this->compare(cp, Gem::Common::expectation::CE_INEQUALITY, CE_DEF_SIMILARITY_DIFFERENCE);
			 return true;
		 } catch (g_expectation_violation &) {
			 return false;
		 }
	 }

	 /***************************************************************************/
	 /**
	  * Returns the name of this class
	  */
	 std::string name() const override {
		 return std::string("GSerialExecutorT<processable_type>");
	 }

	 /***************************************************************************/
	 /**
	  * A standard assignment operator for GSerialExecutorT<processable_type> objects,
	  *
	  * @param cp A copy of another GSerialExecutorT<processable_type> object
	  * @return A constant reference to this object
	  */
	 GSerialExecutorT<processable_type>& operator=(const GSerialExecutorT<processable_type> &cp) {
		 GSerialExecutorT<processable_type>::load_(&cp);
		 return *this;
	 }

	 /***************************************************************************/
	 /**
	  * Adds local configuration options to a GParserBuilder object
	  *
	  * @param gpb The GParserBuilder object to which configuration options should be added
	  */
	 void addConfigurationOptions(
		 Gem::Common::GParserBuilder &gpb
	 ) override {
		 // Call our parent class's function
		 GBaseExecutorT<processable_type>::addConfigurationOptions(gpb);

		 // No local data
	 }

	 /***************************************************************************/
	 /**
	  * Searches for compliance with expectations with respect to another object
	  * of the same type
	  */
	 void compare(
		 const GBaseExecutorT<processable_type>& cp // the other object
		 , const Gem::Common::expectation& e // the expectation for this object, e.g. equality
		 , const double& limit // the limit for allowed deviations of floating point types
	 ) const override {
		 using namespace Gem::Common;

		 // Check that we are dealing with a GSerialExecutorT reference independent of this object and convert the pointer
		 const GSerialExecutorT<processable_type> *p_load = Gem::Common::g_convert_and_compare(cp, this);

		 GToken token("GSerialExecutorT<processable_type>", e);

		 // Compare our parent data ...
		 Gem::Common::compare_base<GBaseExecutorT<processable_type>>(IDENTITY(*this, *p_load), token);

		 // ... no local data

		 // React on deviations from the expectation
		 token.evaluate();
	 }

protected:
	 /***************************************************************************/
	 /**
	  * Loads the data of another GSerialExecutorT object
	  *
	  * @param cp A constant pointer to another GSerialExecutorT object
	  */
	 void load_(const GBaseExecutorT<processable_type> *cp) override {
		 const auto p_load_ptr = dynamic_cast<GSerialExecutorT<processable_type> const *const>(cp);

		 if (!cp) { // nullptr
			 throw gemfony_exception(
				 g_error_streamer(DO_LOG, time_and_place)
					 << "In GSerialExecutorT<processable_type>::load_(): Conversion error!" << std::endl
			 );
		 }

		 // Load our parent classes data
		 GBaseExecutorT<processable_type>::load_(p_load_ptr);
	 }

	 /***************************************************************************/
	 /**
	  * Submits a single work item. In the case of serial execution, all work
	  * is done inside of this function. We rely on the process() function which
	  * is guaranteed to be part of the processable_type interface (note that
	  * our base class stipulates that processable_type is a derivative of
	  * GProcessingContainerT<processable_type> .
	  *
	  * @param w_ptr The work item to be processed
	  */
	 void submit(
		 std::shared_ptr<processable_type> w_ptr
	 ) override {
		 try {
			 // Set the first submission start time, if necessary
			 if(!m_first_submission_measured) {
				 m_first_submission_time = this->now();
				 m_first_submission_measured = true;
			 }

			 // process may throw ...
			 w_ptr->process();
		 } catch(const g_processing_exception& e) {
			 // This is an expected exception if processing has failed. We do nothing,
			 // it is up to the caller to decide what to do with processing errors, and
			 // these are also stored in the processing item. We do try to create a sort
			 // of stack trace by emitting a warning, though. Processing errors should be rare,
			 // so might hint at some problem.
			 glogger
				 << "In GSerialExecutorT<processable_type>::submit():" << std::endl
				 << "Caught a g_processing_exception exception while processing the work item" << std::endl
				 << "with the error message" << std::endl
				 << e.what() << std::endl
				 << "Exception information should have been stored in the" << std::endl
				 << "work item itself. Processing should have been marked as" << std::endl
				 << "unsuccessful in the work item. We leave it to the" << std::endl
				 << "submitter to deal with this." << std::endl
				 << GWARNING;
		 } catch(const std::exception& e) {
			 // All exceptions should be caught inside of the process() call. It is a
			 // severe error if we nevertheless catch an error here. We throw a corresponding
			 // gemfony exception.
			 throw gemfony_exception(
				 g_error_streamer(DO_LOG, time_and_place)
					 << "In GSerialExecutorT<processable_type>::submit(): Caught a" << std::endl
					 << "std::exception in a place where we didn't expect any exceptions." << std::endl
				 	 << "Got message" << std::endl
				 	 << e.what() << std::endl
			 );
		 } catch(...) {
			 // All exceptions should be caught inside of the process() call. It is a
			 // severe error if we nevertheless catch an error here. We throw a corresponding
			 // gemfony exception.
			 throw gemfony_exception(
				 g_error_streamer(DO_LOG, time_and_place)
					 << "In GSerialExecutorT<processable_type>::submit(): Caught an" << std::endl
					 << "unknown exception in a place where we didn't expect any exceptions" << std::endl
			 );
		 }
	 }

	 /***************************************************************************/
	 /**
	  * Waits for work items to return and checks for completeness
	  * (i.e. all items have returned and there were no exceptions).
	  *
	  * @param workItems A vector with work items to be evaluated beyond the broker
	  * @param oldWorkItems A vector with work items that have returned after the threshold
	  * @return A struct of booleans indicating whether all items were processed successfully and whether there were errors
	  */
	 executor_status_t waitForReturn(
		 std::vector<std::shared_ptr<processable_type>>& workItems
		 , std::vector<std::shared_ptr<processable_type>>& oldWorkItems
	 ) override {
		 // Note: Old work items are cleared in the "workOn" function

		 // We are dealing with serial execution on the local computer, so we
		 // do not need to wait for return. All we need to do is to check whether
		 // we had a "complete return" and/or errors.
		 return this->checkExecutionState(workItems);
	 }

private:
	 /***************************************************************************/
	 /**
	  * Creates a deep clone of this object.
	  */
	 GBaseExecutorT<processable_type>* clone_() const override {
		 return new GSerialExecutorT<processable_type>(*this);
	 }

	 /***************************************************************************/
	 /**
	  * Retrieval of the start time
	  */
	 std::chrono::high_resolution_clock::time_point determineExecutionStartTime() const override {
		return m_first_submission_time;
	 }

	 /***************************************************************************/
	 /** @brief Graphical progress feedback */
	 void visualize_performance() override { /* nothing */ }

	 /***************************************************************************/
	 // Data

	 std::chrono::high_resolution_clock::time_point m_first_submission_time = std::chrono::high_resolution_clock::now(); ///< The timepoint of the first submission
	 std::atomic<bool> m_first_submission_measured{false}; ///< Indicates whether the first submission time was already set
};

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/**
 * This class executes a collection of work items in multiple threads
 */
template<typename processable_type>
class GMTExecutorT
	: public GBaseExecutorT<processable_type>
{
	 ///////////////////////////////////////////////////////////////////////

	 friend class boost::serialization::access;

	 template<typename Archive>
	 void serialize(Archive &ar, const unsigned int) {
		 using boost::serialization::make_nvp;

		 ar
		 & make_nvp("GBaseExecutorT", boost::serialization::base_object<GBaseExecutorT<processable_type>>(*this))
		 & BOOST_SERIALIZATION_NVP(m_n_threads);
	 }

	 ///////////////////////////////////////////////////////////////////////

public:
	 /***************************************************************************/
	 /**
	  * The default constructor
	  */
	 GMTExecutorT()
		 : GBaseExecutorT<processable_type>()
		 , m_n_threads(boost::numeric_cast<std::uint16_t>(Gem::Common::getNHardwareThreads()))
	 { /* nothing */ }

	 /***************************************************************************/
	 /**
	  * Initialization with the number of threads
	  */
	 explicit GMTExecutorT(std::uint16_t nThreads)
		 : GBaseExecutorT<processable_type>()
		 , m_n_threads(nThreads)
	 { /* nothing */ }

	 /***************************************************************************/
	 /**
	  * The copy constructor
	  *
	  * @param cp A copy of another GBrokerConnector object
	  */
	 GMTExecutorT(const GMTExecutorT<processable_type> &cp)
		 : GBaseExecutorT<processable_type>(cp)
		 , m_n_threads(cp.m_n_threads)
	 { /* nothing */ }

	 /***************************************************************************/
	 /** @brief The destructor */
	 virtual ~GMTExecutorT() = default;

	 /***************************************************************************/
	 /**
	  * Checks for equality with another GMTExecutorT<processable_type> object
	  *
	  * @param  cp A constant reference to another GMTExecutorT<processable_type> object
	  * @return A boolean indicating whether both objects are equal
	  */
	 bool operator==(const GMTExecutorT<processable_type>& cp) const {
		 using namespace Gem::Common;
		 try {
			 this->compare(cp, Gem::Common::expectation::CE_EQUALITY, CE_DEF_SIMILARITY_DIFFERENCE);
			 return true;
		 } catch (g_expectation_violation &) {
			 return false;
		 }
	 }

	 /***************************************************************************/
	 /**
	  * Checks for inequality with another GMTExecutorT<processable_type> object
	  *
	  * @param  cp A constant reference to another GMTExecutorT<processable_type> object
	  * @return A boolean indicating whether both objects are inequal
	  */
	 bool operator!=(const GMTExecutorT<processable_type>& cp) const {
		 using namespace Gem::Common;
		 try {
			 this->compare(cp, Gem::Common::expectation::CE_INEQUALITY, CE_DEF_SIMILARITY_DIFFERENCE);
			 return true;
		 } catch (g_expectation_violation &) {
			 return false;
		 }
	 }

	 /***************************************************************************/
	 /**
	  * Returns the name of this class
	  */
	 std::string name() const override {
		 return std::string("GMTExecutorT<processable_type>");
	 }

	 /***************************************************************************/
	 /**
	  * A standard assignment operator for GMTExecutorT<processable_type> objects,
	  *
	  * @param cp A copy of another GMTExecutorT<processable_type> object
	  * @return A constant reference to this object
	  */
	 GMTExecutorT<processable_type> &operator=(const GMTExecutorT<processable_type> &cp) {
		 GMTExecutorT<processable_type>::load_(&cp);
		 return *this;
	 }

	 /***************************************************************************/
	 /**
	  * Adds local configuration options to a GParserBuilder object
	  *
	  * @param gpb The GParserBuilder object to which configuration options should be added
	  */
	 void addConfigurationOptions(
		 Gem::Common::GParserBuilder &gpb
	 ) override {
		 // Call our parent class's function ...
		 GBaseExecutorT<processable_type>::addConfigurationOptions(gpb);

		 // ... then add local dara
		 gpb.registerFileParameter<std::uint16_t>(
			 "nProcessingThreads" // The name of the variable in the configuration file
			 , Gem::Courtier::DEFAULTNSTDTHREADS // The default value
			 , [this](std::uint16_t nt) { this->setNThreads(nt); }
		 )
			 << "The number of threads used to simultaneously process work items" << std::endl
			 << "0 means \"automatic\"";
	 }

	 /***************************************************************************/
	 /**
	  * Sets the number of threads for the thread pool. If nThreads is set
	  * to 0, an attempt will be made to set the number of threads to the
	  * number of hardware threading units (e.g. number of cores or hyperthreading
	  * units).
	  *
	  * @param nThreads The number of threads the threadpool should use
	  */
	 void setNThreads(std::uint16_t nThreads) {
		 if (nThreads == 0) {
			 m_n_threads = boost::numeric_cast<std::uint16_t>(Gem::Common::getNHardwareThreads(Gem::Courtier::DEFAULTNSTDTHREADS));
		 }
		 else {
			 m_n_threads = nThreads;
		 }
	 }

	 /***************************************************************************/
	 /**
	  * Retrieves the number of threads this population uses.
	  *
	  * @return The maximum number of allowed threads
	  */
	 std::uint16_t getNThreads() const {
		 return m_n_threads;
	 }

	 /***************************************************************************/
	 /**
	  * Searches for compliance with expectations with respect to another object
	  * of the same type
	  */
	 void compare(
		 const GBaseExecutorT<processable_type>& cp // the other object
		 , const Gem::Common::expectation& e // the expectation for this object, e.g. equality
		 , const double& limit // the limit for allowed deviations of floating point types
	 ) const override {
		 using namespace Gem::Common;

		 // Check that we are dealing with a GSerialExecutorT reference independent of this object and convert the pointer
		 const GMTExecutorT<processable_type> *p_load = Gem::Common::g_convert_and_compare(cp, this);

		 GToken token("GMTExecutorT<processable_type>", e);

		 // Compare our parent data ...
		 Gem::Common::compare_base<GBaseExecutorT<processable_type>>(IDENTITY(*this, *p_load), token);

		 // ... and then our local data
		 compare_t(IDENTITY(m_n_threads, p_load->m_n_threads), token);

		 // React on deviations from the expectation
		 token.evaluate();
	 }

protected:
	 /***************************************************************************/
	 /**
	  * Loads the data of another GMTExecutorT object
	  *
	  * @param cp A constant pointer to another GMTExecutorT object
	  */
	 void load_(const GBaseExecutorT<processable_type> *cp) override {
		 const auto p_load_ptr = dynamic_cast<const GMTExecutorT<processable_type> *>(cp);

		 if (!cp) { // nullptr
			 throw gemfony_exception(
				 g_error_streamer(DO_LOG, time_and_place)
					 << "In GMTExecutorT<processable_type>::load_(): Conversion error!" << std::endl
			 );
		 }

		 // Load our parent classes data
		 GBaseExecutorT<processable_type>::load_(cp);

		 // Load our local data
		 m_n_threads = p_load_ptr->m_n_threads;
	 }

	 /***************************************************************************/
	 /**
	  * General initialization function to be called prior to the first submission
	  */
	 void init_() override {
		 // GBaseExecutorT<processable_type> sees exactly the environment it would when called from its own class
		 GBaseExecutorT<processable_type>::init_();

		 // Initialize our thread pool
		 m_gtp_ptr.reset(new Gem::Common::GThreadPool(m_n_threads));
	 }

	 /***************************************************************************/
	 /**
	  * General finalization function to be called after the last submission
	  */
	 void finalize_() override {
		 // Terminate our thread pool
		 m_gtp_ptr.reset();

		 // GBaseExecutorT<processable_type> sees exactly the environment it would when called from its own class
		 GBaseExecutorT<processable_type>::finalize_();
	 }

	 /***************************************************************************/
	 /**
	  * Allow to perform necessary setup work for an iteration.
	  */
	 void iterationInit_(
		 std::vector<std::shared_ptr<processable_type>>& workItems
	 ) override {
		 // Make sure the parent classes iterationInit function is executed first
		 // This function will also update the iteration start time
		 GBaseExecutorT<processable_type>::iterationInit_(workItems);

		 // We want an empty futures vector for a new submission cycle,
		 // so we do not deal with old errors.
		 m_future_vec.clear();
	 }

	 /***************************************************************************/
	 /**
	  * Submits a single work item. As we are dealing with multi-threaded
	  * execution, we simply let a thread pool executed a lambda function
	  * which takes care of the processing.
	  *
	  * @param w_ptr The work item to be processed
	  */
	 void submit(
		 std::shared_ptr<processable_type> w_ptr
	 ) override {
		 using result_type = typename processable_type::result_type;

		 if (m_gtp_ptr && w_ptr) { // Do we have a valid thread pool and a valid work item ?
			 // Set the first submission start time, if necessary
			 if(!m_first_submission_measured) {
				 m_first_submission_time = this->now();
				 m_first_submission_measured = true;
			 }

			 // async_schedule emits a future, which is std::moved into the std::vector
			 m_future_vec.push_back(
				 m_gtp_ptr->async_schedule( [w_ptr](){ return w_ptr->process(); })
			 );
		 } else {
			 if (!m_gtp_ptr) {
				 throw gemfony_exception(
					 g_error_streamer(DO_LOG, time_and_place)
						 << "In In GMTExecutorT<processable_type>::submit(): Error!" << std::endl
						 << "Threadpool pointer is empty" << std::endl
				 );
			 } else if(!w_ptr) {
				 throw gemfony_exception(
					 g_error_streamer(DO_LOG, time_and_place)
						 << "In In GMTExecutorT<processable_type>::submit(): Error!" << std::endl
						 << "work item pointer is empty" << std::endl
				 );
			 }
		 }
	 }

	 /***************************************************************************/
	 /**
	  * Waits for the thread pool to run empty and checks for completeness
	  * (i.e. all items have returned and there were no exceptions).
	  *
	  * @param workItems A vector with work items to be evaluated beyond the broker
	  * @param oldWorkItems A vector with work items that have returned after the threshold
	  * @return A struct of booleans indicating whether all items were processed successfully and whether there were errors
	  */
	 executor_status_t waitForReturn(
		 std::vector<std::shared_ptr < processable_type>>& workItems
		 , std::vector<std::shared_ptr<processable_type>>& oldWorkItems
	 ) override {
		 using result_type = typename processable_type::result_type;

		 // Note: Old work items are cleared in the "workOn" function

		 // Find out about the errors that were found
		 for(auto &f: m_future_vec) {
			 // Retrieve the future and check for errors
			 try {
				 result_type r = f.get();
			 } catch(const g_processing_exception& e) {
				 // This is an expected exception if processing has failed. We do nothing,
				 // it is up to the caller to decide what to do with processing errors, and
				 // these are also stored in the processing item. We do try to create a sort
				 // of stack trace by emitting a warning, though. Processing errors should be rare,
				 // so might hint at some problem.
				 glogger
					 << "In GMTExecutorT<processable_type>::waitForReturn():" << std::endl
					 << "Caught a g_processing_exception exception while retrieving a future" << std::endl
					 << "with the error message" << std::endl
					 << e.what() << std::endl
					 << "Exception information should have been stored in the" << std::endl
					 << "work item itself. Processing should have been marked as" << std::endl
					 << "unsuccessful in the work item. We leave it to the" << std::endl
					 << "caller to deal with this." << std::endl
					 << GWARNING;
			 } catch(const std::exception& e) {
				 throw gemfony_exception(
					 g_error_streamer(DO_LOG, time_and_place)
						 << "In GMTExecutorT<processable_type>::waitForReturn(): Caught an" << std::endl
						 << "caught std::exception in a place where we didn't expect any exceptions" << std::endl
					 	 << "Got error message:" << std::endl
					    << e.what() << std::endl
				 );
			 } catch(...) {
				 // All exceptions should be caught inside of the process() call (i.e. emanate
				 // from future.get() . It is a severe error if we nevertheless catch an error here.
				 // We throw a corresponding gemfony exception.
				 throw gemfony_exception(
					 g_error_streamer(DO_LOG, time_and_place)
						 << "In GMTExecutorT<processable_type>::waitForReturn(): Caught an" << std::endl
						 << "unknown exception in a place where we didn't expect any exceptions" << std::endl
				 );
			 }
		 }

		 // We are dealing with multi-threaded execution on the local computer, so we
		 // do not need to wait for return. All we need to do is to check whether
		 // we had a "complete return" and/or errors.
		 return this->checkExecutionState(workItems);
	 }

private:
	 /***************************************************************************/
	 /**
	  * Creates a deep clone of this object.
	  */
	 GBaseExecutorT<processable_type>* clone_() const override {
		 return new GMTExecutorT<processable_type>(*this);
	 }

	 /***************************************************************************/
	 /**
	  * Retrieval of the start time
	  */
	 std::chrono::high_resolution_clock::time_point determineExecutionStartTime() const override {
		 return m_first_submission_time;
	 }

	 /***************************************************************************/
	 /** @brief Graphical progress feedback */
	 void visualize_performance() override { /* nothing */ }

	 /***************************************************************************/

	 std::uint16_t m_n_threads; ///< The number of threads
	 std::shared_ptr<Gem::Common::GThreadPool> m_gtp_ptr; ///< Temporarily holds a thread pool

	 std::vector<std::future<typename processable_type::result_type>> m_future_vec; ///< Holds futures stored during the submit call

	 std::chrono::high_resolution_clock::time_point m_first_submission_time = std::chrono::high_resolution_clock::now(); ///< The timepoint of the first submission
	 std::atomic<bool> m_first_submission_measured{false}; ///< Indicates whether the first submission time was already set
};

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/**
 * This class relays execution of work items to a broker, to which several
 * different consumers may be connected.
 */
template<typename processable_type>
class GBrokerExecutorT :
	public GBaseExecutorT<processable_type>
{
	 ///////////////////////////////////////////////////////////////////////

	 friend class boost::serialization::access;

	 template<typename Archive>
	 void serialize(Archive &ar, const unsigned int) {
		 using boost::serialization::make_nvp;

		 ar
		 & make_nvp("GBaseExecutorT", boost::serialization::base_object<GBaseExecutorT<processable_type>>(*this))
		 & BOOST_SERIALIZATION_NVP(m_waitFactor)
		 & BOOST_SERIALIZATION_NVP(m_initialWaitFactor)
		 & BOOST_SERIALIZATION_NVP(m_minPartialReturnPercentage)
		 & BOOST_SERIALIZATION_NVP(m_capable_of_full_return)
		 & BOOST_SERIALIZATION_NVP(m_gpd)
		 & BOOST_SERIALIZATION_NVP(m_waiting_times_graph)
		 & BOOST_SERIALIZATION_NVP(m_returned_items_graph)
		 & BOOST_SERIALIZATION_NVP(m_waitFactorWarningEmitted)
		 & BOOST_SERIALIZATION_NVP(m_lastReturnTime)
		 & BOOST_SERIALIZATION_NVP(m_lastAverage)
		 & BOOST_SERIALIZATION_NVP(m_remainingTime)
		 & BOOST_SERIALIZATION_NVP(m_maxTimeout);
	 }

	 ///////////////////////////////////////////////////////////////////////

	 using GBufferPortT_ptr = std::shared_ptr<Gem::Courtier::GBufferPortT<processable_type>>;
	 using GBroker_ptr = std::shared_ptr<Gem::Courtier::GBrokerT<processable_type>>;

public:
	 /***************************************************************************/
	 /**
	  * The default constructor
	  */
	 GBrokerExecutorT()
		 : GBaseExecutorT<processable_type>()
	 	 , m_gpd("Maximum waiting times and returned items", 1, 2)
	 {
		 m_gpd.setCanvasDimensions(std::make_tuple<std::uint32_t,std::uint32_t>(1200,1600));

		 m_waiting_times_graph = std::shared_ptr<Gem::Common::GGraph2D>(new Gem::Common::GGraph2D());
		 m_waiting_times_graph->setXAxisLabel("Iteration");
		 m_waiting_times_graph->setYAxisLabel("Maximum waiting time [s]");
		 m_waiting_times_graph->setPlotMode(Gem::Common::graphPlotMode::CURVE);

		 m_returned_items_graph = std::shared_ptr<Gem::Common::GGraph2D>(new Gem::Common::GGraph2D());
		 m_returned_items_graph->setXAxisLabel("Iteration");
		 m_returned_items_graph->setYAxisLabel("Number of returned items");
		 m_returned_items_graph->setPlotMode(Gem::Common::graphPlotMode::CURVE);
	 }

	 /***************************************************************************/
	 /**
	  * The copy constructor
	  *
	  * @param cp A copy of another GBrokerConnector object
	  */
	 GBrokerExecutorT(const GBrokerExecutorT<processable_type> &cp)
		 : GBaseExecutorT<processable_type>(cp)
			, m_waitFactor(cp.m_waitFactor)
			, m_initialWaitFactor(cp.m_initialWaitFactor)
			, m_minPartialReturnPercentage(cp.m_minPartialReturnPercentage)
			, m_capable_of_full_return(cp.m_capable_of_full_return)
			, m_gpd("Maximum waiting times and returned items", 1, 2) // Intentionally not copied
			, m_waitFactorWarningEmitted(cp.m_waitFactorWarningEmitted)
			, m_lastReturnTime(cp.m_lastReturnTime)
			, m_lastAverage(cp.m_lastAverage)
			, m_remainingTime(cp.m_remainingTime)
			, m_maxTimeout(cp.m_maxTimeout)
	 {
		 m_gpd.setCanvasDimensions(std::make_tuple<std::uint32_t,std::uint32_t>(1200,1600));

		 m_waiting_times_graph = std::shared_ptr<Gem::Common::GGraph2D>(new Gem::Common::GGraph2D());
		 m_waiting_times_graph->setXAxisLabel("Iteration");
		 m_waiting_times_graph->setYAxisLabel("Maximum waiting time [s]");
		 m_waiting_times_graph->setPlotMode(Gem::Common::graphPlotMode::CURVE);

		 m_returned_items_graph = std::shared_ptr<Gem::Common::GGraph2D>(new Gem::Common::GGraph2D());
		 m_returned_items_graph->setXAxisLabel("Iteration");
		 m_returned_items_graph->setYAxisLabel("Number of returned items");
		 m_returned_items_graph->setPlotMode(Gem::Common::graphPlotMode::CURVE);
	 }

	 /***************************************************************************/
	 /**
	  * The destructor
	  *
	  * TODO: This is a hack. GBrokerExecutorT from factory will otherwise
	  * overwrite the file.
	  */
	 virtual ~GBrokerExecutorT()
	 {
		 // Register the plotter
		 m_gpd.registerPlotter(m_waiting_times_graph);
		 m_gpd.registerPlotter(m_returned_items_graph);

		 // Write out the result. This is a hack.
		 if(m_waiting_times_graph->currentSize() > 0.) {
			 m_gpd.writeToFile("maximumWaitingTimes.C");
		 }
	 }

	 /***************************************************************************/
	 /**
	  * Checks for equality with another GBrokerExecutorT<processable_type> object
	  *
	  * @param  cp A constant reference to another GBrokerExecutorT<processable_type> object
	  * @return A boolean indicating whether both objects are equal
	  */
	 bool operator==(const GBrokerExecutorT<processable_type>& cp) const {
		 using namespace Gem::Common;
		 try {
			 this->compare(cp, Gem::Common::expectation::CE_EQUALITY, CE_DEF_SIMILARITY_DIFFERENCE);
			 return true;
		 } catch (g_expectation_violation &) {
			 return false;
		 }
	 }

	 /***************************************************************************/
	 /**
	  * Checks for inequality with another GBrokerExecutorT<processable_type> object
	  *
	  * @param  cp A constant reference to another GBrokerExecutorT<processable_type> object
	  * @return A boolean indicating whether both objects are inequal
	  */
	 bool operator!=(const GBrokerExecutorT<processable_type>& cp) const {
		 using namespace Gem::Common;
		 try {
			 this->compare(cp, Gem::Common::expectation::CE_INEQUALITY, CE_DEF_SIMILARITY_DIFFERENCE);
			 return true;
		 } catch (g_expectation_violation &) {
			 return false;
		 }
	 }

	 /***************************************************************************/
	 /**
	  * Returns the name of this class
	  */
	 std::string name() const override {
		 return std::string("GBrokerExecutorT<processable_type>");
	 }


	 /***************************************************************************/
	 /**
	  * A standard assignment operator for GBrokerExecutorT<processable_type> objects,
	  *
	  * @param cp A copy of another GBrokerExecutorT<processable_type> object
	  * @return A constant reference to this object
	  */
	 GBrokerExecutorT<processable_type> &operator=(const GBrokerExecutorT<processable_type> &cp) {
		 GBrokerExecutorT<processable_type>::load_(&cp);
		 return *this;
	 }

	 /***************************************************************************/
	 /**
	  * Adds local configuration options to a GParserBuilder object
	  *
	  * @param gpb The GParserBuilder object to which configuration options should be added
	  */
	 void addConfigurationOptions(
		 Gem::Common::GParserBuilder &gpb
	 ) override {
		 // Call our parent class's function
		 GBaseExecutorT<processable_type>::addConfigurationOptions(gpb);

		 // Add local data

		 gpb.registerFileParameter<double>(
			 "waitFactor" // The name of the variable
			 , DEFAULTBROKERWAITFACTOR2 // The default value
			 , [this](double w) {
				 this->setWaitFactor(w);
			 }
		 )
			 << "A static double factor for timeouts" << std::endl
			 << "A wait factor <= 0 means \"no timeout\"." << std::endl
			 << "It is suggested to use values >= 1.";

		 gpb.registerFileParameter<double>(
			 "initialWaitFactor" // The name of the variable
			 , DEFAULTINITIALBROKERWAITFACTOR2 // The default value
			 , [this](double w) {
				 this->setInitialWaitFactor(w);
			 }
		 )
			 << "A static double factor for timeouts in the first iteration." << std::endl
			 << "Set this to the inverse of the number of parallel processing" << std::endl
			 << "units being used.";

		 gpb.registerFileParameter<std::uint16_t>(
			 "minPartialReturnPercentage" // The name of the variable
			 , DEFAULTEXECUTORPARTIALRETURNPERCENTAGE // The default value
			 , [this](std::uint16_t percentage) {
				 this->setMinPartialReturnPercentage(percentage);
			 }
		 )
			 << "Set to a value < 100 to allow execution to continue when" << std::endl
			 << "minPartialReturnPercentage percent of the expected work items"  << std::endl
			 << "have returned. Set to 0 to disable this option.";
	 }

	 /***************************************************************************/
	 /**
	  * Allows to set the wait factor to be applied to timeouts. A wait factor
	  * <= 0 indicates an indefinite waiting time.
	  */
	 void setWaitFactor(double waitFactor) {
		 m_waitFactor = waitFactor;
	 }

	 /***************************************************************************/
	 /**
	  * Allows to retrieve the wait factor variable
	  */
	 double getWaitFactor() const {
		 return m_waitFactor;
	 }

	 /***************************************************************************/
	 /**
	  * Allows to set the initial wait factor to be applied to timeouts. A wait factor
	  * <= 0 is not allowed.
	  */
	 void setInitialWaitFactor(double initialWaitFactor) {
		 if(initialWaitFactor <= 0.) {
			 throw gemfony_exception(
				 g_error_streamer(DO_LOG, time_and_place)
					 << "In GBrokerExecutorT<processable_type>::setInitialWaitFactor(): Error!" << std::endl
					 << "Invalid wait factor " << initialWaitFactor << " supplied. Must be > 0."
			 );
		 }
		 m_initialWaitFactor = initialWaitFactor;
	 }

	 /***************************************************************************/
	 /**
	  * Allows to retrieve the wait factor variable
	  */
	 double getInitialWaitFactor() const noexcept {
		 return m_initialWaitFactor;
	 }

	 /***************************************************************************/
	 /**
	  * Allows to retrieve the percentage of items that must have returned
	  * before execution continues. 0 means: Option is disabled.
	  */
	 std::uint16_t getMinPartialReturnPercentage() const noexcept {
		 return m_minPartialReturnPercentage;
	 }

	 /***************************************************************************/
	 /**
	  * Allows to set the percentage of items that must have returned
	  * before execution continues. Set to 0 to disable this option.
	  */
	 void setMinPartialReturnPercentage(std::uint16_t minPartialReturnPercentage) {
		 Gem::Common::checkRangeCompliance<std::uint16_t>(
			 minPartialReturnPercentage
			 , 0, 100
			 , "GBrokerExecutorT<>::setMinPartialReturnPercentage()"
		 );
		 m_minPartialReturnPercentage = minPartialReturnPercentage;
	 }

	 /***************************************************************************/
	 /**
	  * Searches for compliance with expectations with respect to another object
	  * of the same type
	  */
	 void compare(
		 const GBaseExecutorT<processable_type>& cp // the other object
		 , const Gem::Common::expectation& e // the expectation for this object, e.g. equality
		 , const double& limit // the limit for allowed deviations of floating point types
	 ) const override {
		 using namespace Gem::Common;

		 // Check that we are dealing with a GSerialExecutorT reference independent of this object and convert the pointer
		 const GBrokerExecutorT<processable_type> *p_load = Gem::Common::g_convert_and_compare(cp, this);

		 GToken token("GBrokerExecutorT<processable_type>", e);

		 // Compare our parent data ...
		 Gem::Common::compare_base<GBaseExecutorT<processable_type>>(IDENTITY(*this, *p_load), token);

		 // ... and then our local data
		 compare_t(IDENTITY(m_waitFactor, p_load->m_waitFactor), token);
		 compare_t(IDENTITY(m_initialWaitFactor, p_load->m_initialWaitFactor), token);
		 compare_t(IDENTITY(m_minPartialReturnPercentage, p_load->m_minPartialReturnPercentage), token);
		 compare_t(IDENTITY(m_capable_of_full_return, p_load->m_capable_of_full_return), token);
		 compare_t(IDENTITY(m_waitFactorWarningEmitted, p_load->m_waitFactorWarningEmitted), token);
		 compare_t(IDENTITY(m_lastReturnTime, p_load->m_lastReturnTime), token);
		 compare_t(IDENTITY(m_lastAverage, p_load->m_lastAverage), token);
		 compare_t(IDENTITY(m_remainingTime, p_load->m_remainingTime), token);
		 compare_t(IDENTITY(m_maxTimeout, p_load->m_maxTimeout), token);

		 // React on deviations from the expectation
		 token.evaluate();
	 }

protected:
	 /***************************************************************************/
	 /**
	  * Loads the data of another GBrokerExecutorT object
	  *
	  * @param cp A constant pointer to another GBrokerExecutorT object
	  */
	 void load_(const GBaseExecutorT<processable_type> * cp) override {
		 const auto p_load_ptr = dynamic_cast<const GBrokerExecutorT<processable_type> *>(cp);

		 if (!p_load_ptr) { // nullptr
			 throw gemfony_exception(
				 g_error_streamer(DO_LOG, time_and_place)
					 << "In GBrokerExecutorT<processable_type>::load(): Conversion error!" << std::endl
			 );
		 }

		 // Load our parent classes data
		 GBaseExecutorT<processable_type>::load_(cp);

		 // Local data
		 m_waitFactor = p_load_ptr->m_waitFactor;
		 m_initialWaitFactor = p_load_ptr->m_initialWaitFactor;
		 m_minPartialReturnPercentage = p_load_ptr->m_minPartialReturnPercentage;
		 m_capable_of_full_return = p_load_ptr->m_capable_of_full_return;
		 m_waitFactorWarningEmitted = p_load_ptr->m_waitFactorWarningEmitted;
		 m_lastReturnTime = p_load_ptr->m_lastReturnTime;
		 m_lastAverage = p_load_ptr->m_lastAverage;
		 m_remainingTime = p_load_ptr->m_remainingTime;
		 m_maxTimeout = p_load_ptr->m_maxTimeout;
	 }

	 /***************************************************************************/
	 /**
	  * General initialization function to be called prior to the first submission
	  */
	 void init_() override {
		 // To be called prior to all other initialization code
		 GBaseExecutorT<processable_type>::init_();

		 // Make sure we have a valid buffer port
		 if (!m_CurrentBufferPort) {
			 m_CurrentBufferPort
				 = GBufferPortT_ptr(new Gem::Courtier::GBufferPortT<processable_type>());
		 }

		 // Retrieve a connection to the broker
		 m_broker_ptr = GBROKER(processable_type);

		 // Add the buffer port to the broker
		 m_broker_ptr->enrol(m_CurrentBufferPort);

		 // Check the capabilities of consumsers enrolled with the broker.
		 // Note that this call may block until consumers have actually been enrolled.
		 m_capable_of_full_return = m_broker_ptr->capableOfFullReturn();

#ifdef DEBUG
		 if(m_capable_of_full_return) {
			 glogger
				 << "In GBrokerExecutorT<>::init():" << std::endl
				 << "Assuming that all consumers are capable of full return" << std::endl
				 << GLOGGING;
		 } else {
			 glogger
				 << "In GBrokerExecutorT<>::init():" << std::endl
				 << "At least one consumer is not capable of full return" << std::endl
				 << GLOGGING;
		 }
#endif
	 }

	 /***************************************************************************/
	 /**
	  * General finalization function to be called after the last submission
	  */
	 void finalize_() override {
		 // Get rid of the buffer port
		 m_CurrentBufferPort.reset();

		 // Likely unnecessary cleanup
		 m_capable_of_full_return = false;

		 // Disconnect from the broker
		 m_broker_ptr.reset();

		 // To be called after all other finalization code
		 GBaseExecutorT<processable_type>::finalize_();
	 }

	 /***************************************************************************/
	 /**
	  * Allows to perform necessary setup work for an iteration
	  */
	 void iterationInit_(
		 std::vector<std::shared_ptr<processable_type>>& workItems
	 ) override {
		 // Make sure the parent classes iterationInit function is executed first
		 // This function will also update the iteration start time
		 GBaseExecutorT<processable_type>::iterationInit_(workItems);

#ifdef DEBUG
		 // Check that the waitFactor has a suitable size
		 if(!m_waitFactorWarningEmitted) {
			 if(m_waitFactor > 0. && m_waitFactor < 1.) {
				 glogger
					 << "In GBrokerExecutorT::reviseMaxTime(): Warning" << std::endl
					 << "It is suggested not to use a wait time < 1. Current value: " << m_waitFactor << std::endl
					 << GWARNING;
			 }
		 }
#endif
	 }

	 /***************************************************************************/
	 /**
	  * Allows to perform necessary cleanup work for an iteration or do calculations
	  * for the next iteration.
	  */
	 void iterationFinalize_(
		 std::vector<std::shared_ptr<processable_type>>& workItems
	 ) override {
		 // Calculate average return times of work items.
		 m_lastAverage =
			 (this->getNReturnedLast() > 0)
			 ? (m_lastReturnTime - this->getExecutionStartTime())/this->getNReturnedLast()
			 : (this->now() - this->getExecutionStartTime())/this->getExpectedNumber(); // this is an artificial number, as no items have returned

		 m_maxTimeout =
			 m_lastAverage
			 * boost::numeric_cast<double>(this->getExpectedNumber())
			 * m_waitFactor;

		 // Make sure the parent classes iterationFinalize_ function is executed last
		 GBaseExecutorT<processable_type>::iterationFinalize_(workItems);
	 }

	 /***************************************************************************/
	 /**
	  * Submits a single work item.
	  *
	  * TODO: Deal with exceptions in this class, e.g. during pushs
	  *
	  * @param w The work item to be processed
	  */
	 void submit(
		 std::shared_ptr<processable_type> w_ptr
	 ) override {
		 if(!w_ptr) {
			 throw gemfony_exception(
				 g_error_streamer(DO_LOG, time_and_place)
					 << "In GBrokerExecutorT::submit(): Error!" << std::endl
					 << "Work item is empty" << std::endl
			 );
		 }

		 if(!m_CurrentBufferPort) {
			 throw gemfony_exception(
				 g_error_streamer(DO_LOG, time_and_place)
					 << "In GBrokerExecutorT::submit(): Error!" << std::endl
					 << "Current buffer port is empty when it shouldn't be" << std::endl
			 );
		 }

		 // Store the id of the buffer port in the item
		 w_ptr->setBufferId(m_CurrentBufferPort->getUniqueTag());

		 // Perform the actual submission
		 m_CurrentBufferPort->push_raw(w_ptr);
	 }

	 /***************************************************************************/
	 /**
	  * Waits for all items to return or possibly until a timeout has been reached.
	  * Checks for completeness (i.e. all items have returned and there were no exceptions).
	  *
	  * @param workItems A vector with work items to be evaluated beyond the broker
	  * @param oldWorkItems A vector with work items that have returned after the threshold
	  * @return A struct of booleans indicating whether all items were processed successfully and whether there were errors
	  */
	 executor_status_t waitForReturn(
		 std::vector<std::shared_ptr<processable_type>>& workItems
		 , std::vector<std::shared_ptr<processable_type>>& oldWorkItems
	 ) override {
		 // Act depending on the capabilities of the consumer or user-preferences
	    return
			 (m_capable_of_full_return || m_waitFactor == 0.)?
			 this->waitForFullReturn(workItems, oldWorkItems):
			 this->waitForTimeOut(workItems, oldWorkItems);
	 }

private:
	 /***************************************************************************/
	 /**
	  * Creates a deep clone of this object.
	  */
	 GBaseExecutorT<processable_type>* clone_() const override {
		 return new GBrokerExecutorT<processable_type>(*this);
	 }

	 /***************************************************************************/
	 /**
	  * Retrieves an item from the broker, waiting indefinitely for returns
	  *
	  * @return The obtained processed work item
	  */
	 std::shared_ptr<processable_type> retrieve() {
		 // Holds the retrieved item
		 std::shared_ptr<processable_type> w;
		 m_CurrentBufferPort->pop_processed(w);

		 return w;
	 }

	 /***************************************************************************/
	 /**
	  * Retrieves an item from the broker, waiting up to a given amount of time.
	  * The call will return earlier, if an item could already be retrieved.
	  *
	  * @return The obtained processed work item
	  */
	 std::shared_ptr<processable_type> retrieve(
		 const std::chrono::duration<double> &timeout
	 ) {
		 // Holds the retrieved item, if any
		 std::shared_ptr<processable_type> w_ptr;
		 m_CurrentBufferPort->pop_processed(w_ptr, timeout);
		 return w_ptr;
	 }

	 /***************************************************************************/
	 /**
	  * Updates the maximum allowed timeframe for calculations in the first iteration
	  */
	 void reviseMaxTime(std::size_t nReturnedCurrent) {
		 // Are we called for the first time in the first iteration)
		 if(nReturnedCurrent==1) {
			 std::chrono::duration<double> currentElapsed
				 = this->now() - this->getExecutionStartTime();

			 if (this->get_iteration_counter() == ITERATION_COUNTER_TYPE(0)) {
				 // Calculate a timeout for subsequent retrievals in this iteration. In the first iteration and for the first item,
				 // this timeout is the number of remaining items times the return time needed for the first item times a custom
				 // wait factor for the first submission. This may be very long, but takes care of a situation where there is only
				 // a single worker.
				 m_maxTimeout =
					 currentElapsed
					 * boost::numeric_cast<double>(this->getExpectedNumber())
					 * m_initialWaitFactor;
			 } else { // Not the first work item
				 std::chrono::duration<double> currentAverage = currentElapsed / ((std::max)(nReturnedCurrent, std::size_t(1))); // Avoid division by 0
				 m_maxTimeout =
					 currentAverage
					 * this->getExpectedNumber()
					 * m_waitFactor;
			 }
		 }
	 }

	 /***************************************************************************/
	 /**
	  * Checks whether we have passed the maximum time frame. The function will
	  * also update the remaining time.
	  *
	  * @return A boolean indicating whether the maximum allowed time was passed
	  */
	 bool passedMaxTime(std::size_t nReturnedCurrent) {
		 std::chrono::duration<double> currentElapsed
			 = this->now() - this->getExecutionStartTime();

		 if (currentElapsed > m_maxTimeout) {
			 m_remainingTime = std::chrono::duration<double>(0.);
			 return true;
		 } else {
			 m_remainingTime = m_maxTimeout - currentElapsed;
			 return false;
		 }
	 }

	 /***************************************************************************/
	 /**
	  * Returns a complete set of items or waits until a timeout occurs.
	  * Returns two status flags "is_complete" (== all items from the current
	  * submission have returned) and "has_errors" (== some or all of the submitted
	  * items had errors).
	  *
	  * @param workItems The work items to be processed
	  * @param oldWorkItems A collection of old work items that have returned after a timeout
	  * @return A struct of booleans indicating whether all items were processed successfully and whether there were errors
	  */
	 executor_status_t waitForTimeOut(
		 std::vector<std::shared_ptr<processable_type>>& workItems
		 , std::vector<std::shared_ptr<processable_type>>& oldWorkItems
	 ) {
		 std::shared_ptr<processable_type> w;
		 std::size_t nReturnedCurrent = 0;
		 executor_status_t status;

		 // Note: Old work items are cleared in the "workOn" function

		 // Start to retrieve individuals and sort them into our vectors,
		 // until a halt criterion is reached.
		 do {
			 // Get the next individual. If we didn't receive a valid
			 // item, go to the timeout check
			 if(!(w = this->getNextIndividual(nReturnedCurrent))) continue;

			 // Try to add the work item to the list and check for completeness
			 status = this->addWorkItemAndCheckCompleteness(
				 w
				 , nReturnedCurrent
				 , workItems
				 , oldWorkItems
			 );

			 // Update the internal timeout variables,
			 // so we know how much longer this cycle should run
			 this->updateTimeout(nReturnedCurrent);

			 // No need to continue if all currently submitted work items have returned
			 if(status.is_complete) break;
		 } while(!halt(nReturnedCurrent));

		 // Check for the processing flags and derive the is_complete and has_errors states
		 return this->checkExecutionState(workItems);
	 };

	 /***************************************************************************/
	 /**
	  * Waits (possibly indefinitely) until all items have returned. Note that this
	  * function may stall, if for whatever reason a work item does not return. If this
	  * is not acceptable, use waitForTimeout() instead of this function. It is recommended
	  * to only use this function in environments that are considered safe in the sense
	  * that work items will always return. Local cluster environments may fall into this
	  * category (or the risk is considered low enough by the user to use this simpler
	  * function). It is possible to allow a partial return by setting the percentage
	  * of required returned items to an appropriate number. If all iterations run a
	  * "full return" strategy without this option, the oldWorkItems vector will remain empty.
	  * Otherwise there may be old returning items in the next iterations.
	  *
	  * @param workItems The work items to be processed
	  * @param oldWorkItems A collection of old work items that have returned after a timeout
	  * @return A struct of booleans indicating whether all items were processed successfully and whether there were errors
	  */
	 executor_status_t waitForFullReturn(
		 std::vector<std::shared_ptr<processable_type>>& workItems
		 , std::vector<std::shared_ptr<processable_type>>& oldWorkItems
	 ) {
		 std::size_t nReturnedCurrent = 0;
		 executor_status_t status;

		 do {
			 status = this->addWorkItemAndCheckCompleteness(
				 this->retrieve()
				 , nReturnedCurrent
				 , workItems
				 , oldWorkItems
			 );
		 }
		 // Break the loop if all items (or at least the minimum percentage) were received
		 while(!status.is_complete && !this->minPartialReturnRateReached(nReturnedCurrent));

		 // Check for the processing flags and derive the is_complete and has_errors states
		 return this->checkExecutionState(workItems);
	 }

	 /***************************************************************************/
	 /**
	  * Updates the remaining time for this iteration
	  *
	  * @param nReturnedCurrent The number of work items from the current submission cycle that has returned so far
	  */
	 void updateTimeout(const std::size_t& nReturnedCurrent) {

	 }

	 /***************************************************************************/
	 /**
	  * Checks whether a timeout was encountered. Also updates the internal
	  * timeout measurements.
	  */
	 bool timeout() {
		 std::chrono::duration<double> currentElapsed
			 = this->now() - this->getExecutionStartTime();

		 if (currentElapsed > m_maxTimeout) {
			 m_remainingTime = std::chrono::duration<double>(0.);
			 return true;
		 } else {
			 m_remainingTime = m_maxTimeout - currentElapsed;
			 return false;
		 }
	 }

	 /***************************************************************************/
	 /**
	  * Checks for halt criteria
	  */
	 bool halt(const std::size_t& nReturnedCurrent) {
		 // Timeout checks and update of timeout variables
		 if(this->timeout()) return true;

		 // For some algorithms, a partial return rate suffices
		 if(this->minPartialReturnRateReached(nReturnedCurrent)) return true;

		 // We want to continue
		 return false;
	 }

	 /***************************************************************************/
	 /**
	  * Checks whether the minimum return rate was reached
	  */
	 bool minPartialReturnRateReached(const std::size_t& nReturnedCurrent) {
		 // Leave if this check is disabled
		 if(0 == this->getMinPartialReturnPercentage()) return false;

		 // Avoid problems related to floating point accuracy
		 std::size_t expectedNumber = this->getExpectedNumber();
		 if(nReturnedCurrent == expectedNumber) return true;

		 // Check if we have reached the minimum percentage
		 double realPercentage = boost::numeric_cast<double>(nReturnedCurrent) / boost::numeric_cast<double>(expectedNumber);
		 if(realPercentage >= boost::numeric_cast<double>(this->getMinPartialReturnPercentage())) {
			 return true;
		 } else {
			 return false;
		 }
	 }

	 /***************************************************************************/
	 /**
	  * Get the remaining time for this iteration
	  */
	 std::chrono::duration<double> remainingTime() const {
		 return this->m_remainingTime;
	 }

	 /***************************************************************************/
	 /**
	  * Retrieves the next individual
	  */
	 std::shared_ptr<processable_type> getNextIndividual(const std::size_t& nReturnedCurrent) {
		 std::shared_ptr<processable_type> w;

		 if(this->firstIndInFirstIteration(nReturnedCurrent)) {
			 // Wait indefinitely for the very first individual
			 w = this->retrieve();

			 // It is a severe error if we didn't get an individual here
			 if(!w) {
				 throw gemfony_exception(
					 g_error_streamer(DO_LOG, time_and_place)
						 << "In GBaseExeuctorT<processable_type>::getNextIndividual(): Received empty first individual" << std::endl
				 );
			 }
		 } else { // not the first individual in the first iteration
			 // Obtain the next item, observing a timeout
			 w = retrieve(this->remainingTime());
		 }

		 return w;
	 }

	 /***************************************************************************/
	 /**
	  * Checks whether we are dealing with the first returned individual in the
	  * first iteration
	  */
	 bool firstIndInFirstIteration(const std::size_t& nReturnedCurrent) const {
		 return (0 == this->get_iteration_counter() && 0 == nReturnedCurrent);
	 }

	 /***************************************************************************/
	 /**
	  * Adds a work item to the corresponding vectors. This function assumes that
	  * the work item is valid, i.e. points to a valid object.
	  *
	  * @param w_ptr The item to be added to the (old?) work items
	  * @param nReturnedCurrent The number of returned work items from the current iteration
	  * @param workItems The list of work items from the current iteration
	  * @param oldWorkItems The list of wirk items from previous iterations
	  * @return A struct indicating whether all work items of the current iteration were received and whether there were errors
	  */
	 executor_status_t addWorkItemAndCheckCompleteness(
		 std::shared_ptr <processable_type> w_ptr
		 , std::size_t &nReturnedCurrent
		 , std::vector<std::shared_ptr<processable_type>>& workItems
		 , std::vector<std::shared_ptr<processable_type>>& oldWorkItems
	 ) {
		 // If we have been passed an empty item, simply continue the outer loop
		 if(!w_ptr) {
			 return executor_status_t{false /* is_complete */, false /* has_errors */ };
		 } else {
			 // Make the return time of the last item known
			 m_lastReturnTime = this->now();
		 }

		 bool complete = false;
		 bool has_errors = false;

		 auto current_submission_id = this->get_iteration_counter();
		 auto worker_submission_id = w_ptr->getSubmissionCounter();

		 // Did this work item originate in the current submission cycle ?
		 if (current_submission_id == worker_submission_id) {
			 // Extract the original position of the work item and cross-check
			 std::size_t worker_position = w_ptr->getSubmissionPosition();

			 // Add the work item to the list in the desired position. Re-submitted items
			 // might return twice, so we only add them if a processed item hasn't been added yet.
			 // We also take care of the situation that w_ptr points to the same object as the one
			 // stored in the workItems vector. This may happen in the case of local submission,
			 // such as with multi-threaded or serial consumers. DO_PROCESS will have been replaced
			 // by PROCESSED in this case.
			 if (
				 workItems.at(worker_position) == w_ptr // Same item
				 || processingStatus::DO_PROCESS == workItems.at(worker_position)->getProcessingStatus()
			 ) {
				 // Note that also items with errors may be added here. It is up to
				 // the caller to decide what to do with such work items.
				 if(workItems.at(worker_position) != w_ptr) workItems.at(worker_position) = w_ptr;
				 if (++nReturnedCurrent==this->getExpectedNumber()) complete=true;
				 if (w_ptr->has_errors()) has_errors=true;
			 } // no else
		 } else { // Not a work item from the current submission cycle.
			 // Ignore old work items with errors
			 if (processingStatus::PROCESSED == w_ptr->getProcessingStatus()) {
				 oldWorkItems.push_back(w_ptr);
			 } else {
				 // This should be rare. As we throw away items here, we want to
				 // make a record as a frequent occurrance might indicate a problem
				 glogger
					<< "In GBrokerExecutorT<>::addWorkItemAndCheckCompleteness():" << std::endl
					<< "Received old work item from submission cycle " << worker_submission_id << " (now " << current_submission_id << ")" << std::endl
					<< "We will throw the item away as it has the status id " << w_ptr->getProcessingStatus() << std::endl
					<< "(expected processingStatus::PROCESSED / " << processingStatus::PROCESSED << ")" << std::endl
					<< (w_ptr->has_errors()?w_ptr->getStoredErrorDescriptions():"") << std::endl
					<< GLOGGING;
			 }
		 }

		 return executor_status_t{complete, has_errors};
	 }

	 /***************************************************************************/
	 /**
 	  * Allows to emit information at the end of an iteration
 	  * TODO: The content of this function is a hack. Submitted for current debugging purposes
 	  */
	 void visualize_performance() override {
		 // Now do our own reporting
		 std::chrono::duration<double> currentElapsed
			 = this->now() - this->getExecutionStartTime();
		 auto current_iteration = this->get_iteration_counter();

		 m_waiting_times_graph->add(boost::numeric_cast<double>(current_iteration), m_maxTimeout.count());
		 m_returned_items_graph->add(boost::numeric_cast<double>(current_iteration), boost::numeric_cast<double>(this->getNReturnedLast()));

#ifdef DEBUG
		 if (0 == current_iteration) {
			 glogger
				 << "Maximum waiting time in iteration " << current_iteration << ": " << m_maxTimeout.count()
				 << " s (" << currentElapsed.count() << ", "
				 << this->getNReturnedLast() << " / " << this->getExpectedNumber() << ", " << m_initialWaitFactor << ")" << std::endl
				 << GLOGGING;
		 } else {
			 glogger
				 << "Maximum waiting time in iteration " << current_iteration << ": " << m_maxTimeout.count()
				 << " s (" << m_lastAverage.count() << ", "
				 << this->getNReturnedLast() << " / " << this->getExpectedNumber() << ", " << m_waitFactor << ")" << std::endl
				 << GLOGGING;
		 }
#endif
	 }

	 /***************************************************************************/
	 /**
	  * Retrieval of the start time. For brokered execution this is defined as the
	  * time of the first retrieval of an item from the buffer port, as it may take
	  * some time until (networked) clients ask for work.
	  *
	  * @return The time of the first retrieval of a work item from the buffer port
	  */
	 std::chrono::high_resolution_clock::time_point determineExecutionStartTime() const override {
#ifdef DEBUG
		 // Check if we have a valid buffer port
		 if(!m_CurrentBufferPort) {
			 throw gemfony_exception(
				 g_error_streamer(DO_LOG, time_and_place)
					 << "In GBrokerExecutorT<processable_type>::determineExecutionStartTime():" << std::endl
				 	 << "No valid buffer port found" << std::endl
			 );
		 }
#endif

		 return m_CurrentBufferPort->getFirstRetrievalTime();
	 }

	 /***************************************************************************/
	 // Local data
	 double m_waitFactor = DEFAULTBROKERWAITFACTOR2; ///< A static factor to be applied to timeouts
	 double m_initialWaitFactor = DEFAULTINITIALBROKERWAITFACTOR2; ///< A static factor to be applied to timeouts in the first iteration

	 std::uint16_t m_minPartialReturnPercentage = DEFAULTEXECUTORPARTIALRETURNPERCENTAGE; ///< Minimum percentage of returned items after which execution continues

	 GBufferPortT_ptr m_CurrentBufferPort; ///< Holds a GBufferPortT object during the calculation. Note: It is neither serialized nor copied
	 GBroker_ptr m_broker_ptr; ///< A (possibly empty) pointer to a global broker

	 bool m_capable_of_full_return = false; ///< Indicates whether the broker may return results without losses

	 Gem::Common::GPlotDesigner m_gpd; ///< A wrapper for the plots
	 std::shared_ptr<Gem::Common::GGraph2D> m_waiting_times_graph;  ///< The maximum waiting time resulting from the wait factor
	 std::shared_ptr<Gem::Common::GGraph2D> m_returned_items_graph;  ///< The maximum waiting time resulting from the wait factor

	 bool m_waitFactorWarningEmitted = false; ///< Specifies whether a warning about a small waitFactor has already been emitted

	 std::chrono::high_resolution_clock::time_point m_lastReturnTime = std::chrono::high_resolution_clock::now(); ///< Temporary that holds the time of the return of the last item of an iteration
	 std::chrono::duration<double> m_lastAverage = std::chrono::duration<double>(0.); ///< The average time needed for the last submission
	 std::chrono::duration<double> m_remainingTime = std::chrono::duration<double>(0.); ///< The remaining time in the current iteration
	 std::chrono::duration<double> m_maxTimeout = std::chrono::duration<double>(0.); ///< The maximum amount of time allowed for the entire calculation
};


/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/

} /* namespace Courtier */
} /* namespace Gem */

/******************************************************************************/
// Some code for Boost.Serialization

/******************************************************************************/
// Mark GBaseExecutorT<> as abstract. This is the content of BOOST_SERIALIZATION_ASSUME_ABSTRACT(T)

namespace boost {
namespace serialization {
template<typename processable_type>
struct is_abstract<Gem::Courtier::GBaseExecutorT<processable_type>> : public boost::true_type {};
template<typename processable_type>
struct is_abstract<const Gem::Courtier::GBaseExecutorT<processable_type>> : public boost::true_type {};
}
}

/******************************************************************************/

#endif /* GEXECUTOR_HPP_ */
