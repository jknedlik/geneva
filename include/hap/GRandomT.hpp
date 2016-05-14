/**
 * @file GRandomT.hpp
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

#include <cstdlib>
#include <iomanip>
#include <ctime>
#include <cmath>
#include <iostream>
#include <sstream>
#include <cassert>

// Boost headers go here
#include <boost/thread/thread.hpp>

#ifndef GRANDOMT_HPP_
#define GRANDOMT_HPP_

// Geneva headers go here
#include "hap/GRandomBase.hpp"
#include "hap/GRandomDefines.hpp"
#include "common/GLogger.hpp"

namespace Gem {
namespace Hap {

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/**
 * Access to different random number distributions, whose "raw" material is
 * produced in different ways. We only define the interface here. The actual
 * implementation can be found in the (partial) specializations of this class.
 */
template<Gem::Hap::RANDFLAVOURS s = Gem::Hap::RANDFLAVOURS::RANDOMPROXY>
class GRandomT
	: public Gem::Hap::GRandomBase {
public:
	/** @brief The default constructor */
	GRandomT();

	/** @brief The destructor */
	virtual ~GRandomT();

protected:
	/** @brief Uniformly distributed integer random numbers */
	virtual GRandomBase::result_type int_random();
};

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/**
 * This specialization of the general GRandomT<> class retrieves random numbers
 * in batches from a global random number factory. The functions provided by
 * GRandomBase then produce different types of random numbers from this raw material.
 * As the class derives from boost::noncopyable, it is not possible to assign other
 * objects or use copy constructors.
 */
template<>
class GRandomT<Gem::Hap::RANDFLAVOURS::RANDOMPROXY>
	: public Gem::Hap::GRandomBase
{
public:
	/***************************************************************************/
	/**
	 * The standard constructor
	 */
	GRandomT()
		: Gem::Hap::GRandomBase()
	  	, p_( /* empty */ )
	  	, grf_(GRANDOMFACTORY) // Make sure we have a local pointer to the factory
	{
		// Make sure we have a first random number package available
		getNewRandomContainer();
	}

	/***************************************************************************/
	/**
	 * The standard destructor
	 */
	virtual ~GRandomT() {
		if (p_) {
			grf_->returnUsedPackage(p_);
		}
		grf_.reset();
	}

protected:
	/***************************************************************************/
	/**
	 * This function retrieves random number packages from a global
	 * factory and emits them one by one. Once a package has been fully
	 * used, it is discarded and a new package is obtained from the factory.
	 * Essentially this class thus acts as a random number proxy -- to the
	 * caller it appears as if random numbers are created locally. This function
	 * assumes that a valid container is already available.
	 */
	virtual GRandomBase::result_type int_random() {
		if (p_->empty()) {
			// Get rid of the old container ...
			grf_->returnUsedPackage(p_);
			// ... then get a new one
			getNewRandomContainer();
		}
		return p_->next();
	}

private:
	/***************************************************************************/
	/**
	 * (Re-)Initialization of p_. Checks that a valid GRandomFactory still
	 * exists, then retrieves a new container.
	 */
	void getNewRandomContainer() {
		// Make sure we get rid of the old container
		p_.reset();

#ifdef DEBUG
		if(!grf_) {
		   glogger
		   << "In GRandomT<RANDOMPROXY>::getNewRandomContainer(): Error!" << std::endl
         << "No connection to GRandomFactory object." << std::endl
         << GEXCEPTION;
		}
#endif /* DEBUG */

#ifdef DEBUG
		std::uint32_t nRetries = 0;
#endif /* DEBUG */

		// Try until a valid container has been received. new01Container has
		// a timeout of DEFAULTFACTORYGETWAIT internally.
		while (!(p_ = grf_->getNewRandomContainer())) {
#ifdef DEBUG
		   nRetries++;
#endif /* DEBUG */
		}

#ifdef DEBUG
		if(nRetries>1) {
		   std::cout << "Info: Had to try " << nRetries << " times to retrieve a valid random number container." << std::endl;
		}
#endif /* DEBUG */
	}


	/***************************************************************************/
	/** @brief Holds the container of uniform random numbers */
	std::shared_ptr <random_container> p_;
	/** @brief A local copy of the global GRandomFactory */
	std::shared_ptr <Gem::Hap::GRandomFactory> grf_;
};

/** @brief Convenience typedef */
typedef GRandomT<Gem::Hap::RANDFLAVOURS::RANDOMPROXY> GRandom;

/***************************************************************************/
/** @brief Central access to a random number generator through thread-local storage */
boost::thread_specific_ptr<Gem::Hap::GRandom>& gr_tls_ptr();

// Syntactic sugar
#define GRANDOM_TLS (*(Gem::Hap::gr_tls_ptr()))
#define GRANDOM_TLS_PTR Gem::Hap::gr_tls_ptr()

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/**
 * This specialization of the general GRandomT<> class produces random numbers
 * locally. The functions provided by GRandomBase<> then produce different types
 * of random numbers from this raw material. A seed can be provided either to
 * the constructor, or is taken from the global seed manager (recommended) in
 * case the default constructor is used.
 */
template<>
class GRandomT<Gem::Hap::RANDFLAVOURS::RANDOMLOCAL>
	: public Gem::Hap::GRandomBase
{
public:
	/***************************************************************************/
	/**
	 * The standard constructor
	 */
	GRandomT()
		: Gem::Hap::GRandomBase()
	  	, rng_(GRANDOMFACTORY->getSeed())
	{ /* nothing */ }

	/***************************************************************************/
	/**
	 * The standard destructor
	 */
	virtual ~GRandomT()
	{ /* nothing */ }

protected:
	/***************************************************************************/
	/**
	 * This function produces uniform random numbers locally.
	 */
	virtual GRandomBase::result_type int_random() {
		return rng_();
	}

private:
	/***************************************************************************/
	/** @brief The actual generator for local random number creation */
	G_BASE_GENERATOR rng_;
};

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/**
 * This class produces integer random numbers, using GRandomT<RAMDONPROXY> as
 * the random number engine.
 */
template <typename int_type>
class g_uniform_int {
	 // Make sure this class can only be instantiated if int_type is an integer type
	 static_assert(
		 std::is_integral<int_type>::value
		 , "int_type should be an integer type"
	 );

public:
	 /**
	  * The default constructor
	  */
	 g_uniform_int()
		 : m_uniform_int_distribution(0,std::numeric_limits<int_type>::max())
	 { /* nothing */ }

	 /**
	  * Initialization with an upper limit
	  */
	 explicit g_uniform_int(int_type r)
		 : m_uniform_int_distribution(0,r)
	 { /* nothing */ }

	 /**
	  * Initialization with a lower and upper limit
	  */
	 g_uniform_int(int_type l, int_type r)
	 	: m_uniform_int_distribution(l,r)
	 { /* nothing */ }

	 /**
	  * Initialization through a std::uniform_int_distribution<int_type>::param_type object
	  */
	 explicit g_uniform_int(const typename std::uniform_int_distribution<int_type>::param_type& params)
	 	: m_uniform_int_distribution(params)
	 { /* nothing */ }

	 /**
	  * Returns uniformly distributed random numbers, using
	  * the boundaries specified in the constructor
	  */
	 inline int_type operator()() const {
		 return m_uniform_int_distribution(GRANDOM_TLS);
	 }

	 /**
     * Returns uniformly distributed random numbers between 0
     * and an upper boundary
     */
	 inline int_type operator()(int_type r) const {
		 return m_uniform_int_distribution (
			 GRANDOM_TLS
			 , typename std::uniform_int_distribution<int_type>::param_type(0,r)
		 );
	 }

	 /**
	  * Returns uniformly distributed random numbers using
	  * a new set of boundaries.
	  */
	 inline int_type operator()(int_type l, int_type r) const {
		 return m_uniform_int_distribution (
			 GRANDOM_TLS
			 , typename std::uniform_int_distribution<int_type>::param_type(l,r)
		 );
	 }

	 /**
 	  * Returns uniformly distributed random numbers using a param_type object
 	  */
	 inline int_type operator()(const typename std::uniform_int_distribution<int_type>::param_type& params) const {
		 return m_uniform_int_distribution (
			 GRANDOM_TLS, params
		 );
	 }

private:
	 /** @brief Uniformly distributed integer random numbers */
	 mutable std::uniform_int_distribution<int_type> m_uniform_int_distribution;

	 // Deleted copy and assignment operators. Copy-construction is possible
	 // through the std::uniform_int_distribution<int_type>::param_type object
	 g_uniform_int(const g_uniform_int<int_type>&) = delete;
	 g_uniform_int<int_type>& operator=(const g_uniform_int<int_type>&) = delete;
};

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/**
 * This class produces evenly distributed floating point random numbers, using
 * GRandomT<RAMDONPROXY> as the random number engine.
 */
template <typename fp_type>
class g_uniform_real {
	 // Make sure this class can only be instantiated if fp_type is a floating point type
	 static_assert(
		 std::is_floating_point<fp_type>::value
		 , "fp_type should be a floating point type"
	 );

public:
	 /**
	  * The default constructor
	  */
	 g_uniform_real()
		 : m_uniform_real_distribution(0.,std::numeric_limits<fp_type>::max())
	 { /* nothing */ }

	 /**
	  * Initialization with an upper limit
	  */
	 explicit g_uniform_real(fp_type r)
		 : m_uniform_real_distribution(0.,r)
	 { /* nothing */ }

	 /**
	  * Initialization with a lower and upper limit
	  */
	 g_uniform_real(fp_type l, fp_type r)
		 : m_uniform_real_distribution(l,r)
	 { /* nothing */ }

	 /**
 	  * Initialization through a std::uniform_real_distribution<fp_type>::param_type object
 	  */
	 explicit g_uniform_real(const typename std::uniform_real_distribution<fp_type>::param_type& params)
		 : m_uniform_real_distribution(params)
	 { /* nothing */ }

	 /**
	  * Returns uniformly distributed random numbers, using
	  * the boundaries specified in the constructor
	  */
	 inline fp_type operator()() const {
		 return m_uniform_real_distribution(GRANDOM_TLS);
	 }

	 /**
     * Returns uniformly distributed random numbers between 0
     * and an upper boundary
     */
	 inline fp_type operator()(fp_type r) const {
		 return m_uniform_real_distribution (
			 GRANDOM_TLS
			 , typename std::uniform_real_distribution<fp_type>::param_type(0.,r)
		 );
	 }

	 /**
	  * Returns uniformly distributed random numbers using
	  * a new set of boundaries.
	  */
	 inline fp_type operator()(fp_type l, fp_type r) const {
		 return m_uniform_real_distribution (
			 GRANDOM_TLS
			 , typename std::uniform_real_distribution<fp_type>::param_type(l,r)
		 );
	 }

	 /**
     * Returns uniformly distributed floating point random numbers using the specifications found in
     * a param_type object
     */
	 inline fp_type operator()(const typename std::uniform_real_distribution<fp_type>::param_type& params) const {
		 return m_uniform_real_distribution (
			 GRANDOM_TLS, params
		 );
	 }

private:
	 /** @brief Uniformly distributed floating point random numbers */
	 mutable std::uniform_real_distribution<fp_type> m_uniform_real_distribution;

	 // Deleted copy and assignment operators. Copy-construction is possible
	 // through the std::uniform_real_distribution<fp_type>::param_type object
	 g_uniform_real(const g_uniform_real<fp_type>&) = delete;
	 g_uniform_real<fp_type>& operator=(const g_uniform_real<fp_type>&) = delete;
};

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/**
 * This class produces boolean random numbers, using GRandomT<RAMDONPROXY> as the
 * random number engine. The probability for true vs. false may be passed
 * either to the contructor or to the operator() .
 */
class g_boolean_distribution
{
public:
	 /**
	  * The default constructor. Implicit probability of 50% for "true".
	  */
	 g_boolean_distribution() : m_bernoulli_distribution()
	 { /* nothing */ }

	 /**
	  * Initialization with a probability for "true".
	  */
	 explicit g_boolean_distribution(double p) : m_bernoulli_distribution(p)
	 { /* nothing */ }

	 /**
 	  * Initialization through a std::uniform_real_distribution<fp_type>::param_type object
 	  */
	 explicit g_boolean_distribution(const std::bernoulli_distribution::param_type& params)
		 : m_bernoulli_distribution(params)
	 { /* nothing */ }

	 /**
	  * Returns boolean random numbers with a probability of 50% for true vs. false
	  */
	 inline bool operator()() const {
		 return m_bernoulli_distribution(GRANDOM_TLS);
	 }

	 /**
     * Returns boolean random numbers with a likelihood of "p" for "true" values
     */
	 inline bool operator()(double p) const {
		 return m_bernoulli_distribution (
			 GRANDOM_TLS
			 , std::bernoulli_distribution::param_type(p)
		 );
	 }

	 /**
 	  * Returns uniformly distributed floating point random numbers using the specifications found
 	  * in a param_type object
 	  */
	 inline bool operator()(const std::bernoulli_distribution::param_type& params) const {
		 return m_bernoulli_distribution (
			 GRANDOM_TLS, params
		 );
	 }
private:
	 /** @brief Boolean random numbers with configurable probability structure */
	 mutable std::bernoulli_distribution m_bernoulli_distribution;

	 // Deleted copy and assignment operators. Copy-construction is possible
	 // through the std::bernoulli_distribution::param_type object
	 g_boolean_distribution(const g_boolean_distribution&) = delete;
	 g_boolean_distribution& operator=(const g_boolean_distribution&) = delete;
};

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/

} /* namespace Hap */
} /* namespace Gem */


#endif /* GRANDOMT_HPP_ */
