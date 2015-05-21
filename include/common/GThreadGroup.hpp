/**
 * @file GThreadGroup.hpp
 */

/*
 * This file is part of the Geneva library collection.
 *
 * Note: this class was adapted from an earlier Boost 1.36 version of the
 * thread_group class. The original code contained the following text:
 *
 * ***
 * Distributed under the Boost Software License, Version 1.0. (See
 * accompanying file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
 * (C) Copyright 2007-8 Anthony Williams
 * ***
 *
 * As allowed by the license, modifications were applied to the code.
 * These are also covered by the Boost Software License, Version 1.0, and are
 * Copyright (C) Gemfony scientific UG (haftungsbeschraenkt)
 *
 * NOTE THAT THE BOOST-LICENSE DOES NOT APPLY TO ANY OTHER FILES OF THE
 * GENEVA LIBRARY, UNLESS THIS IS EXPLICITLY STATED IN THE CORRESPONDING FILE.
 *
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
 */

// Global checks, defines and includes needed for all of Geneva
#include "common/GGlobalDefines.hpp"

// Standard headers go here
#include <vector>

// Boost headers go here
#include <boost/cstdint.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/utility.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/cstdint.hpp>

#ifndef GTHREADGROUP_HPP_
#define GTHREADGROUP_HPP_

namespace Gem {
namespace Common {

// Forward declaration
class GThreadPool;

/******************************************************************************/
/**
 * A simple thread group that extends the standard Boost thread group by the
 * ability to address the last threads in order to remove threads without
 * the need for a pointer to them. This class was adapted from a version by
 * Anthony Williams, as offered as part of the Boost 1.36 release
 */
class GThreadGroup
	: private boost::noncopyable {
	friend class GThreadPool;

	typedef std::shared_ptr <boost::thread> thread_ptr;
	typedef std::vector<thread_ptr> thread_vector;

public:
	/** @brief The destructor */
	G_API_COMMON ~GThreadGroup();

	/** @brief Adds an already created thread to the group */
	G_API_COMMON void add_thread(thread_ptr);

	/** @brief Remove a thread from the group. Does nothing if the thread is empty. */
	G_API_COMMON void remove_thread(thread_ptr);

	/** @brief Requests all threads to join */
	G_API_COMMON void join_all();

	/** @brief Sends all threads the interrupt signal */
	G_API_COMMON void interrupt_all();

	/** @brief Interrupts, joins and finally removes the last thread in the group. */
	G_API_COMMON void remove_last();

	/** @brief Interrupts, joins and finally removes the last nThreads threads in the group */
	G_API_COMMON void remove_last(const std::size_t &);

	/** @brief Returns the size of the current thread group */
	G_API_COMMON std::size_t size() const;

	/***************************************************************************/
	/**
	 * Creates a new thread and adds it to the group
	 *
	 * @param f The function to be run by the thread
	 * @return A pointer to the newly created thread
	 */
	template<typename F>
	std::shared_ptr <boost::thread> create_thread(F f) {
		boost::lock_guard<boost::mutex> guard(m_);
		thread_ptr new_thread(new boost::thread(f));
		threads_.push_back(new_thread);
		return new_thread;
	}

	/***************************************************************************/
	/**
	 * Creates nThreads new threads with the same function
	 * and adds them to the group
	 *
	 * @param f The function to be run by the thread
	 * @param nThreads The number of threads to add to the group
	 * @return A pointer to the newly created thread
	 */
	template<typename F>
	void create_threads(F f, const std::size_t &nThreads) {
		for (std::size_t i = 0; i < nThreads; i++) create_thread(f);
	}

	/***************************************************************************/

private:
	/** @brief Clears the thread vector */
	void clearThreads();

	thread_vector threads_; ///< Holds the actual threads
	mutable boost::mutex m_; ///< Needed to synchronize access to the vector
};

/******************************************************************************/

} /* namespace Common*/
} /* namespace Gem */

#endif /* GTHREADGROUP_HPP_ */
