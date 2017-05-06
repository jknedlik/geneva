/**
 * @file GThreadSafeQueueT.hpp
 */

/*
 * This file is part of the Geneva library collection.
 *
 * Note: this class was adapted from an example provided by Anthony
 * Williams along with his (highly recommended) book "Concurrency
 * in Action" / Manning . The code is covered by the Boost Software
 * License 1.0 . The original code and all remaining portions in the
 * code below are Copyright Anthony Williams.
 *
 * As allowed by the license, modifications were applied to the code.
 * These are also covered by the Boost Software License, Version 1.0, and are
 * Copyright (C) Gemfony scientific UG (haftungsbeschraenkt)
 *
 * NOTE THAT THE BOOST-LICENSE DOES NOT APPLY TO ANY OTHER FILES OF THE
 * GENEVA LIBRARY, UNLESS THIS IS EXPLICITLY STATED IN THE CORRESPONDING FILE!
 * See the AUTHORS file in the top-level directory for a list of authors.
 *
 * Contact: contact [at] gemfony (dot) eu
 *
 * Geneva was developed with kind support from Karlsruhe Institute of
 * Technology (KIT) and Steinbuch Centre for Computing (SCC). Further
 * information about KIT and SCC can be found at http://www.kit.edu/english
 * and http://scc.kit.edu .
 *
 * Geneva is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * Boost Software License for more details.
 *
 * For further information on Gemfony scientific and Geneva, visit
 * http://www.gemfony.eu .
 */

/*
 * The following license applies to the code in this file:
 *
 * ***************************************************************************
 *
 * Boost Software License - Version 1.0 - August 17th, 2003
 *
 * Permission is hereby granted, free of charge, to any person or organization
 * obtaining a copy of the software and accompanying documentation covered by
 * this license (the "Software") to use, reproduce, display, distribute,
 * execute, and transmit the Software, and to prepare derivative works of the
 * Software, and to permit third-parties to whom the Software is furnished to
 * do so, all subject to the following:
 *
 * The copyright notices in the Software and this entire statement, including
 * the above license grant, this restriction and the following disclaimer,
 * must be included in all copies of the Software, in whole or in part, and
 * all derivative works of the Software, unless such copies or derivative
 * works are solely in the form of machine-executable object code generated by
 * a source language processor.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
 * FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * ***************************************************************************
 */

// Global checks, defines and includes needed for all of Geneva
#include "common/GGlobalDefines.hpp"

// Standard headers go here
#include <memory>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <type_traits>
#include <chrono>

// Boost headers go here

#ifndef GTHREADSAFEQUEUET_HPP_
#define GTHREADSAFEQUEUET_HPP_

// Geneva headers go here
#include "common/GCommonEnums.hpp"

namespace Gem {
namespace Common {

/******************************************************************************/
/**
 * A queue-like structure featuring thread-safe access and fine-grained locking.
 * Modelled after an implementation provided by Anthony Williams in his book
 * "C++ Concurrency in Action" (Manning).
 *
 * @tparam T The type of the data stored in this class
 * @tparam t_capacity The maximum number of data items to be stored in this class (0 means unlimited)
 */
template<
	typename T
	, std::size_t t_capacity = DEFAULTBUFFERSIZE
>
class GThreadSafeQueueT {

	 // Forward declaration
	 struct node;

public:
	 /***************************************************************************/
	 /**
	  * The default constructor. All initialization work is done in the class body.
	  */
	 GThreadSafeQueueT() { /* nothing */ }

	 /***************************************************************************/
    // Various deleted functions

	 GThreadSafeQueueT(const GThreadSafeQueueT& other)=delete; ///< Disabled copy constructor
	 GThreadSafeQueueT(const GThreadSafeQueueT&& other)=delete; ///< Disabled move constructor
	 GThreadSafeQueueT& operator=(const GThreadSafeQueueT& other)=delete; ///< Disabled assignment operator
	 GThreadSafeQueueT& operator=(const GThreadSafeQueueT&& other)=delete; ///< Disabled move-assignment operator

	 /***************************************************************************/
	 /**
	  * Tries to add an item to the queue. Will return true if this was successful,
	  * false otherwise. This overload of try_push is used for normal copyable/movable
	  * types. There are additional overloads for std::shared_ptr and std::unique_ptr.
	  */
	 bool try_push(
		 T new_value
	 ) {
		 bool space_is_available = false;
		 {
			 std::unique_lock<std::mutex> tail_lock(check_space_is_available(space_is_available));

			 if (space_is_available) {
				 // We only copy the data when space is actually available -- this function is non-blocking
				 std::shared_ptr<T> new_data(std::make_shared<T>(std::move(new_value)));
				 std::unique_ptr<node> p(new node);

				 m_tail_ptr->data = new_data;
				 node *const new_tail = p.get();
				 m_tail_ptr->next = std::move(p);
				 m_tail_ptr = new_tail;
				 m_n_data_sets++;
			 }
		 }

		 if(space_is_available) {
#ifdef GENEVA_COMMON_BOUNDED_BUFFER_USE_NOTIFY_ALL
			 m_not_empty.notify_all();
#else
			 m_not_empty.notify_one();
#endif
		 }

		 return space_is_available;
	 }

	 /***************************************************************************/
	 /**
	  * Tries to add an item to the queue. Will return true if this was successful,
	  * false otherwise. This overload of try_push is used for items wrapped into
	  * a std::shared_ptr.
	  */
	 bool try_push(
		 std::shared_ptr<T> new_value_ptr
	 ) {
		 bool space_is_available = false;
		 {
			 std::unique_lock<std::mutex> tail_lock(check_space_is_available(space_is_available));

			 if (space_is_available) {
				 // We only copy the data when space is actually available -- this function is non-blocking
				 std::unique_ptr<node> p(new node);

				 m_tail_ptr->data = new_value_ptr;
				 node *const new_tail = p.get();
				 m_tail_ptr->next = std::move(p);
				 m_tail_ptr = new_tail;
				 m_n_data_sets++;
			 }
		 }

		 if(space_is_available) {
#ifdef GENEVA_COMMON_BOUNDED_BUFFER_USE_NOTIFY_ALL
			 m_not_empty.notify_all();
#else
			 m_not_empty.notify_one();
#endif
		 }

		 return space_is_available;
	 }

	 /***************************************************************************/
	 /**
	  * Tries to add an item to the queue. Will return true if this was successful,
	  * false otherwise. This overload of try_push is used for items wrapped into
	  * a std::unique_ptr. The argument-pointer will be invalidated, if space was
	  * available in the queue.
	  */
	 bool try_push(
		 std::unique_ptr<T> &new_value_ptr
	 ) {
		 bool space_is_available = false;
		 {
			 std::unique_lock<std::mutex> tail_lock(check_space_is_available(space_is_available));

			 if (space_is_available) {
				 // We only copy the data when space is actually available -- this function is non-blocking
				 std::unique_ptr<node> p(new node);

				 m_tail_ptr->data = std::shared_ptr<T>(new_value_ptr.release());
				 node *const new_tail = p.get();
				 m_tail_ptr->next = std::move(p);
				 m_tail_ptr = new_tail;
				 m_n_data_sets++;
			 }
		 }

		 if(space_is_available) {
#ifdef GENEVA_COMMON_BOUNDED_BUFFER_USE_NOTIFY_ALL
			 m_not_empty.notify_all();
#else
			 m_not_empty.notify_one();
#endif
		 }

		 return space_is_available;
	 }

	 /***************************************************************************/

	 void push_and_block(
		 T new_value
	 ) {
		 std::shared_ptr<T> new_data(std::make_shared<T>(std::move(new_value)));
		 std::unique_ptr<node> p(new node);
		 {
			 std::unique_lock<std::mutex> tail_lock(wait_for_space());
			 m_tail_ptr->data=new_data;
			 node* const new_tail=p.get();
			 m_tail_ptr->next=std::move(p);
			 m_tail_ptr=new_tail;
			 m_n_data_sets++;
		 }
#ifdef GENEVA_COMMON_BOUNDED_BUFFER_USE_NOTIFY_ALL
		 m_not_empty.notify_all();
#else
		 m_not_empty.notify_one();
#endif
	 }

	 /***************************************************************************/

	 void push_and_block(
		 std::shared_ptr<T> new_value_ptr
	 ) {
		 std::unique_ptr<node> p(new node);
		 {
			 std::unique_lock<std::mutex> tail_lock(wait_for_space());
			 m_tail_ptr->data=new_value_ptr;
			 node* const new_tail=p.get();
			 m_tail_ptr->next=std::move(p);
			 m_tail_ptr=new_tail;
			 m_n_data_sets++;
		 }
#ifdef GENEVA_COMMON_BOUNDED_BUFFER_USE_NOTIFY_ALL
		 m_not_empty.notify_all();
#else
		 m_not_empty.notify_one();
#endif
	 }

	 /***************************************************************************/

	 void push_and_block(
		 std::unique_ptr<T> &new_value_ptr
	 ) {
		 std::unique_ptr<node> p(new node);
		 {
			 std::unique_lock<std::mutex> tail_lock(wait_for_space());
			 m_tail_ptr->data=std::shared_ptr<T>(new_value_ptr.release());
			 node* const new_tail=p.get();
			 m_tail_ptr->next=std::move(p);
			 m_tail_ptr=new_tail;
			 m_n_data_sets++;
		 }
#ifdef GENEVA_COMMON_BOUNDED_BUFFER_USE_NOTIFY_ALL
		 m_not_empty.notify_all();
#else
		 m_not_empty.notify_one();
#endif
	 }

	 /***************************************************************************/

	 bool push_and_wait(
		 T new_value
		 , const std::chrono::duration<double> &timeout
	 ) {
		 bool space_is_available = false;
		 {
			 std::unique_lock<std::mutex> tail_lock(wait_for_space(timeout, space_is_available));

			 if (space_is_available) {
				 // We only copy the data when space is actually available -- this function is non-blocking
				 std::shared_ptr<T> new_data(std::make_shared<T>(std::move(new_value)));
				 std::unique_ptr<node> p(new node);

				 m_tail_ptr->data = new_data;
				 node *const new_tail = p.get();
				 m_tail_ptr->next = std::move(p);
				 m_tail_ptr = new_tail;
				 m_n_data_sets++;
			 }
		 }

		 if(space_is_available) {
#ifdef GENEVA_COMMON_BOUNDED_BUFFER_USE_NOTIFY_ALL
			 m_not_empty.notify_all();
#else
			 m_not_empty.notify_one();
#endif
		 }

		 return space_is_available;
	 }

	 /***************************************************************************/

	 bool push_and_wait(
		 std::shared_ptr<T> new_value_ptr
		 , const std::chrono::duration<double> &timeout
	 ) {
		 bool space_is_available = false;
		 {
			 std::unique_lock<std::mutex> tail_lock(wait_for_space(timeout, space_is_available));

			 if (space_is_available) {
				 // We only copy the data when space is actually available -- this function is non-blocking
				 std::unique_ptr<node> p(new node);

				 m_tail_ptr->data = new_value_ptr;
				 node *const new_tail = p.get();
				 m_tail_ptr->next = std::move(p);
				 m_tail_ptr = new_tail;
				 m_n_data_sets++;
			 }
		 }

		 if(space_is_available) {
#ifdef GENEVA_COMMON_BOUNDED_BUFFER_USE_NOTIFY_ALL
			 m_not_empty.notify_all();
#else
			 m_not_empty.notify_one();
#endif
		 }

		 return space_is_available;
	 }

	 /***************************************************************************/

	 bool push_and_wait(
		 std::unique_ptr<T> &new_value_ptr
		 , const std::chrono::duration<double> &timeout
	 ) {
		 bool space_is_available = false;
		 {
			 std::unique_lock<std::mutex> tail_lock(wait_for_space(timeout, space_is_available));

			 if (space_is_available) {
				 // We only copy the data when space is actually available -- this function is non-blocking
				 std::unique_ptr<node> p(new node);

				 m_tail_ptr->data=std::shared_ptr<T>(new_value_ptr.release());
				 node *const new_tail = p.get();
				 m_tail_ptr->next = std::move(p);
				 m_tail_ptr = new_tail;
				 m_n_data_sets++;
			 }
		 }

		 if(space_is_available) {
#ifdef GENEVA_COMMON_BOUNDED_BUFFER_USE_NOTIFY_ALL
			 m_not_empty.notify_all();
#else
			 m_not_empty.notify_one();
#endif
		 }

		 return space_is_available;
	 }

	 /***************************************************************************/

	 bool try_pop(T &item) {
		 std::unique_ptr<node> popped_node;
		 {
			 std::unique_lock<std::mutex> head_lock(m_head_mutex);
			 if (m_head_ptr.get() == get_tail()) {
				 return false;
			 }

			 popped_node = unprotected_pop_head();
			 item = *popped_node->data;

			 m_n_data_sets--;
		 }

#ifdef GENEVA_COMMON_BOUNDED_BUFFER_USE_NOTIFY_ALL
			 m_not_full.notify_all();
#else
			 m_not_full.notify_one();
#endif

		 return true;
	 }

	 /***************************************************************************/

	 std::shared_ptr<T> try_pop(bool &item_is_available) {
		 std::unique_ptr<node> popped_node;
		 {
			 std::unique_lock<std::mutex> head_lock(m_head_mutex);
			 if (m_head_ptr.get() == get_tail()) {
				 item_is_available = false;
				 return std::shared_ptr<T>();
			 }

			 popped_node = unprotected_pop_head();
			 item_is_available = true;

			 m_n_data_sets--;
		 }

#ifdef GENEVA_COMMON_BOUNDED_BUFFER_USE_NOTIFY_ALL
		 m_not_full.notify_all();
#else
		 m_not_full.notify_one();
#endif

		 return popped_node->data;
	 }

	 /***************************************************************************/

	 std::shared_ptr<T> try_pop() {
		 std::unique_ptr<node> popped_node;
		 {
			 std::unique_lock<std::mutex> head_lock(m_head_mutex);
			 if (m_head_ptr.get() == get_tail()) {
				 return std::shared_ptr<T>();
			 }

			 popped_node = unprotected_pop_head();

			 m_n_data_sets--;
		 }

#ifdef GENEVA_COMMON_BOUNDED_BUFFER_USE_NOTIFY_ALL
		 m_not_full.notify_all();
#else
		 m_not_full.notify_one();
#endif

		 return popped_node->data;
	 }

	 /***************************************************************************/

	 void pop_and_block(T &item) {
		 {
			 std::unique_lock<std::mutex> head_lock(wait_for_data());
			 item = *unprotected_pop_head()->data;
			 m_n_data_sets--;
		 }

#ifdef GENEVA_COMMON_BOUNDED_BUFFER_USE_NOTIFY_ALL
		 m_not_full.notify_all();
#else
		 m_not_full.notify_one();
#endif
	 }

	 /***************************************************************************/

	 std::shared_ptr<T> pop_and_block() {
		 std::shared_ptr<T> popped_item;
		 {
			 std::unique_lock<std::mutex> head_lock(wait_for_data());
			 popped_item = unprotected_pop_head()->data;
			 m_n_data_sets--;
		 }

#ifdef GENEVA_COMMON_BOUNDED_BUFFER_USE_NOTIFY_ALL
		 m_not_full.notify_all();
#else
		 m_not_full.notify_one();
#endif

		 return popped_item;
	 }

	 /***************************************************************************/

	 bool pop_and_wait(
		 T& item
		 , const std::chrono::duration<double> &timeout
	 ) {
		 bool data_is_available = false;
		 {
			 std::unique_lock<std::mutex> head_lock(
				 wait_for_data(
					 timeout
					 , data_is_available
				 ));
			 if (!data_is_available) {
				 return false;
			 }

			 item = *unprotected_pop_head()->data;
			 m_n_data_sets--;
		 }

#ifdef GENEVA_COMMON_BOUNDED_BUFFER_USE_NOTIFY_ALL
		 m_not_full.notify_all();
#else
		 m_not_full.notify_one();
#endif

		 return true;
	 }

	 /***************************************************************************/

	 std::shared_ptr<T> pop_and_wait(
		 bool &item_is_available
		 , const std::chrono::duration<double> &timeout
	 ) {
		 bool data_is_available = false;
		 std::shared_ptr<T> popped_item;
		 {
			 std::unique_lock<std::mutex> head_lock(
				 wait_for_data(
					 timeout
					 , data_is_available
				 ));
			 if (!data_is_available) {
				 item_is_available = false;
				 return std::shared_ptr<T>();
			 }

			 popped_item = unprotected_pop_head()->data;
			 m_n_data_sets--;
		 }

#ifdef GENEVA_COMMON_BOUNDED_BUFFER_USE_NOTIFY_ALL
		 m_not_full.notify_all();
#else
		 m_not_full.notify_one();
#endif

		 item_is_available = true;
		 return popped_item;
	 }

	 /***************************************************************************/

	 std::shared_ptr<T> pop_and_wait(
		 const std::chrono::duration<double> &timeout
	 ) {
		 bool data_is_available = false;
		 std::shared_ptr<T> popped_item;
		 {
			 std::unique_lock<std::mutex> head_lock(
				 wait_for_data(
					 timeout
					 , data_is_available
				 ));
			 if (!data_is_available) {
				 return std::shared_ptr<T>();
			 }

			 popped_item = unprotected_pop_head()->data;
			 m_n_data_sets--;
		 }

#ifdef GENEVA_COMMON_BOUNDED_BUFFER_USE_NOTIFY_ALL
		 m_not_full.notify_all();
#else
		 m_not_full.notify_one();
#endif

		 return popped_item;
	 }

	 /***************************************************************************/

	 bool empty() {
		 std::unique_lock<std::mutex> head_lock(m_head_mutex);
		 return (m_head_ptr==get_tail());
	 }

private:
	 /***************************************************************************/
	 /**
	  * A struct to be instantiated for each new data set stored in the queue
	  */
	 struct node
	 {
		  std::shared_ptr<T> data;
		  std::unique_ptr<node> next;
	 };

	 /***************************************************************************/

	 std::unique_lock<std::mutex> wait_for_data() {
		 std::unique_lock<std::mutex> head_lock(m_head_mutex);
		 m_not_empty.wait(
			 head_lock
			 , [&]{ return m_head_ptr!=get_tail(); }
		 );
		 return std::move(head_lock);
	 }


	 /***************************************************************************/

	 std::unique_lock<std::mutex> wait_for_data(
		 const std::chrono::duration<double> &timeout
		 , bool &data_is_available
	 ) {
		 std::unique_lock<std::mutex> head_lock(m_head_mutex);
		 data_is_available = m_not_empty.wait_for(
			 head_lock
			 , timeout
			 , [&]{ return m_head_ptr.get()!=get_tail(); }
		 );
		 return std::move(head_lock);
	 }

	 /***************************************************************************/
	 /**
	  * Blocks until the number of work items in the queue has fallen below t_capacity.
	  *
	  * @return The lock used in this function
	  */
	 template <typename std::size_t u_capacity = t_capacity>
	 std::unique_lock<std::mutex> wait_for_space(
		 typename std::enable_if<(u_capacity>0)>::type* = 0
	 ) {
		 static_assert(
			 t_capacity==u_capacity
			 , "t_capacity != u_capacity"
		 );

		 std::unique_lock<std::mutex> tail_lock(m_tail_mutex);
		 m_not_full.wait(
			 tail_lock
			 , [&] { return m_n_data_sets.load() < u_capacity; }
		 );
		 return std::move(tail_lock);
	 }

	 /***************************************************************************/
	 /**
	  * This function was added for consistency, so we can use wait_for_space
	  * regardless of whether the queue is limited or not.
	  *
	  * @return The lock used in this function
	  */
	 template <typename std::size_t u_capacity = t_capacity>
	 std::unique_lock<std::mutex> wait_for_space(
		 typename std::enable_if<(u_capacity==0)>::type* = 0
	 ) {
		 static_assert(
			 t_capacity==u_capacity
			 , "t_capacity != u_capacity"
		 );

		 std::unique_lock<std::mutex> tail_lock(m_tail_mutex);
		 return std::move(tail_lock);
	 }

	 /***************************************************************************/
	 /**
	  * Blocks until the number of work items in the queue has fallen below t_capacity
	  * or a given amount of time has passed.
	  *
	  * @return The lock used in this function
	  */
	 template <typename std::size_t u_capacity = t_capacity>
	 std::unique_lock<std::mutex> wait_for_space(
		 const std::chrono::duration<double> &timeout
		 , bool &space_is_available
		 , typename std::enable_if<(u_capacity>0)>::type* = 0
	 ) {
		 static_assert(
			 t_capacity==u_capacity
			 , "t_capacity != u_capacity"
		 );

		 std::unique_lock<std::mutex> tail_lock(m_tail_mutex);
		 space_is_available = m_not_full.wait_for(
			 tail_lock
			 , std::chrono::duration_cast<std::chrono::milliseconds>(timeout)
			 , [&] { return m_n_data_sets.load() < u_capacity; }
		 );
		 return std::move(tail_lock);
	 }

	 /***************************************************************************/
	 /**
	  * This function was added for consistency, so we can use wait_for_space
	  * regardless of whether the queue is limited or not.
	  *
	  * @return The lock used in this function
	  */
	 template <typename std::size_t u_capacity = t_capacity>
	 std::unique_lock<std::mutex> wait_for_space(
		 const std::chrono::duration<double> &timeout
		 , bool &space_is_available
		 , typename std::enable_if<(u_capacity==0)>::type* = 0
	 ) {
		 static_assert(
			 t_capacity==u_capacity
			 , "t_capacity != u_capacity"
		 );

		 std::unique_lock<std::mutex> tail_lock(m_tail_mutex);
		 space_is_available = true;
		 return std::move(tail_lock);
	 }

	 /***************************************************************************/
	 /**
	  * Checks whether space is available in the queue.
	  *
	  * @return The lock used in this function
	  */
	 template <typename std::size_t u_capacity = t_capacity>
	 std::unique_lock<std::mutex> check_space_is_available(
		 bool &space_is_available
		 , typename std::enable_if<(u_capacity>0)>::type* = 0
	 ) {
		 static_assert(
			 t_capacity==u_capacity
			 , "t_capacity != u_capacity"
		 );

		 std::unique_lock<std::mutex> tail_lock(m_tail_mutex);
		 space_is_available = (m_n_data_sets.load() < u_capacity);
		 return std::move(tail_lock);
	 }

	 /***************************************************************************/
	 /**
	  * This function was added for consistency, so we can use check_space_is_available
	  * regardless of whether the queue is limited or not. It will always return true,
	  * as this is the unlimited case of the function.
	  *
	  * @return The lock used in this function
	  */
	 template <typename std::size_t u_capacity = t_capacity>
	 std::unique_lock<std::mutex> check_space_is_available(
		 bool &space_is_available
		 , typename std::enable_if<(u_capacity==0)>::type* = 0
	 ) {
		 static_assert(
			 t_capacity==u_capacity
			 , "t_capacity != u_capacity"
		 );

		 std::unique_lock<std::mutex> tail_lock(m_tail_mutex);
		 space_is_available = true;
		 return std::move(tail_lock);
	 }


	 /***************************************************************************/

	 node* get_tail() {
		 std::unique_lock<std::mutex> tail_lock(m_tail_mutex);
		 return m_tail_ptr;
	 }

	 /***************************************************************************/

	 std::unique_ptr<node> unprotected_pop_head() {
		 std::unique_ptr<node> old_head=std::move(m_head_ptr);
		 m_head_ptr=std::move(old_head->next);
		 return old_head;
	 }

	 /***************************************************************************/
	 // Data items

	 std::mutex m_head_mutex; ///< Protects access to the head node
	 std::mutex m_tail_mutex; ///< Protects access to the tail node

	 std::unique_ptr<node> m_head_ptr = std::make_unique<node>(); ///< Points to the head of the linked list
	 node* m_tail_ptr = m_head_ptr.get(); ///< Points to the tail of the linked list

	 std::condition_variable m_not_empty; ///< Allows to wait until new nodes have appeared
	 std::condition_variable m_not_full;  ///< Allows to wait until space becomes available in the data structure

	 std::atomic<std::size_t> m_n_data_sets {0}; ///< The number of data sets stored in this class

	 /***************************************************************************/
};

/******************************************************************************/

} /* namespace Common */
} /* namespace Gem */

#endif /* GTHREADSAFEQUEUET_HPP_ */
