/**
 * @file Go2.hpp
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

// Boost header files go here
#include <boost/algorithm/string.hpp>
#include <boost/thread/once.hpp>

#ifndef GO2_HPP_
#define GO2_HPP_

// For Microsoft-compatible compilers
#if defined(_MSC_VER)  &&  (_MSC_VER >= 1020)
#pragma once
#endif


// Geneva headers go here
#include "common/GFactoryT.hpp"
#include "common/GExceptions.hpp"
#include "common/GParserBuilder.hpp"
#include "hap/GRandomFactory.hpp"
#include "hap/GRandomT.hpp"
#include "courtier/GAsioHelperFunctions.hpp"
#include "courtier/GAsioTCPConsumerT.hpp"
#include "courtier/GBoostThreadConsumerT.hpp"
#include "courtier/GBrokerT.hpp"
#include "geneva/GenevaHelperFunctionsT.hpp"
#include "geneva/GIndividual.hpp"
#include "geneva/GMutableSetT.hpp"
#include "geneva/GOptimizableI.hpp"
#include "geneva/GOptimizationAlgorithmT.hpp"
#include "geneva/GOptimizationEnums.hpp"
#include "geneva/GParameterObjectCollection.hpp"
#include "geneva/GParameterSet.hpp"
#include "geneva/GOAFactoryStore.hpp"
#include "geneva/GOAMonitorStore.hpp"
#include "geneva/GConsumerStore.hpp"

#include "geneva/GBaseEA.hpp"
#include "geneva/GBasePS.hpp"
#include "geneva/GBaseSwarm.hpp"
#include "geneva/GBrokerEA.hpp"
#include "geneva/GBrokerGD.hpp"
#include "geneva/GBrokerPS.hpp"
#include "geneva/GBrokerSA.hpp"
#include "geneva/GBrokerSwarm.hpp"
#include "geneva/GSerialEA.hpp"
#include "geneva/GSerialGD.hpp"
#include "geneva/GSerialPS.hpp"
#include "geneva/GSerialSA.hpp"
#include "geneva/GSerialSwarm.hpp"
#include "geneva/GMultiThreadedEA.hpp"
#include "geneva/GMultiThreadedGD.hpp"
#include "geneva/GMultiThreadedPS.hpp"
#include "geneva/GMultiThreadedSA.hpp"
#include "geneva/GMultiThreadedSwarm.hpp"
#include "geneva/GEvolutionaryAlgorithmFactory.hpp"
#include "geneva/GGradientDescentFactory.hpp"
#include "geneva/GParameterScanFactory.hpp"
#include "geneva/GSimulatedAnnealingFactory.hpp"
#include "geneva/GSwarmAlgorithmFactory.hpp"

namespace Gem {
namespace Geneva {

/******************************************************************************/
// Default values for the variables used by the optimizer
const std::string GO2_DEF_DEFAULTCONFIGFILE="config/Go2.json";
const bool GO2_DEF_CLIENTMODE=false;
const execMode GO2_DEF_DEFAULPARALLELIZATIONMODE=EXECMODE_MULTITHREADED;
const Gem::Common::serializationMode GO2_DEF_SERIALIZATIONMODE=Gem::Common::SERIALIZATIONMODE_BINARY;
const std::string GO2_DEF_IP="localhost";
const unsigned int GO2_DEF_PORT=10000;
const bool GO2_DEF_DEFAULTVERBOSE=false;
const bool GO2_DEF_COPYBESTONLY=true;
const boost::uint16_t GO2_DEF_MAXSTALLED=0;
const boost::uint16_t GO2_DEF_MAXCONNATT=100;
const bool GO2_DEF_RETURNREGARDLESS=true;
const boost::uint16_t GO2_DEF_NPRODUCERTHREADS=0;
const boost::uint32_t GO2_DEF_OFFSET=0;
const std::string GO2_DEF_OPTALGS="";

/******************************************************************************/
/** @brief Set a number of parameters of the random number factory */
void setRNFParameters(const boost::uint16_t&);

/******************************************************************************/
/** Syntactic sugar -- make the code easier to read */
typedef Gem::Geneva::GOptimizationAlgorithmT<Gem::Geneva::GParameterSet> GOABase;

/******************************************************************************/
/**
 * This class allows to "chain" a number of optimization algorithms so that a given
 * set of individuals can be optimized using more than one algorithm in sequence. The
 * class also hides the details of client/server mode, consumer initialization, etc.
 * While it is derived from GOptimizableI, it is not currently meant to be used as an
 * individual. Hence the ability to serialize the class has been removed.
 *
 * Command line examples:
 * Geneva -p 2 -c --consumer="net // ip=localhost, port=10000" // networked client for consumer "net", connecting to localhost:10000
 * Geneva -p 2    --consumer="net // port=10000"               // networked server with consumer, listening at port 10000
 * Geneva -p 2    --consumer="mt  // nthreads=4"               // multithreaded consumer with 4 threads
 * Geneva -p 2    --consumer="mt  // nthreads=0"               // multithreaded consumer with as many threads as compute units
 * Geneva -p 1                                                 // multithreaded execution without consumer
 * Geneva -p 0                                                 // serial execution without consumer
 * Geneva                                                      // multithreaded execution without consumer
 */
class Go2
	: public GMutableSetT<GParameterSet>
	, public GOptimizableI
{
public:
	/** @brief The default constructor */
	Go2();
	/** @brief A constructor that first parses the command line for relevant parameters */
	Go2(int, char **);
	/** @brief A constructor that first parses the command line for relevant parameters and allows to specify a default config file name */
	Go2(int, char **, const std::string&);
	/** @brief A constructor that is given the usual command line parameters, then loads the rest of the data from a config file */
	Go2(
		const bool&
		, const Gem::Common::serializationMode&
		, const std::string&
		, const unsigned short&
		, const std::string&
		, const execMode&
		, const bool&
	);
	/** @brief A copy constructor */
	Go2(const Go2&);

	/** @brief The destructor */
	virtual ~Go2();

	/** @brief Standard assignment operator */
	const Go2& operator=(const Go2&);

	/** @brief Checks for equality with another Go2 object */
	bool operator==(const Go2&) const;
	/** @brief Checks for inequality with another Go2 object */
	bool operator!=(const Go2&) const;

	/** @brief Checks whether this object fulfills a given expectation in relation to another object */
	virtual boost::optional<std::string> checkRelationshipWith(
			const GObject&
			, const Gem::Common::expectation&
			, const double&
			, const std::string&
			, const std::string&
			, const bool&
	) const;

	/** @brief Triggers execution of the client loop */
	int clientRun();

	/** @brief Checks whether this object is running in client mode */
	bool clientMode() const;

	void setParallelizationMode(const execMode&);
	execMode getParallelizationMode() const;

	/* @brief Specifies whether only the best individuals of a population should be copied */
	void setCopyBestIndividualsOnly(const bool&);
	/** @brief Checks whether only the best individuals are copied */
	bool onlyBestIndividualsAreCopied() const;

	/** @brief Allows to randomly initialize parameter members. Unused in this wrapper object */
	virtual void randomInit();
	/** @brief Triggers fitness calculation (i.e. optimization) for this object */
	virtual double fitnessCalculation();

	/** @brief Allows to add an optimization algorithm to the chain */
	void addAlgorithm(boost::shared_ptr<GOABase>);
	/** @brief Makes it easier to add algorithms */
	Go2& operator&(boost::shared_ptr<GOABase>);
	/** @brief Allows to add an optimization algorithm through its mnemomic */
   void addAlgorithm(const std::string&);
   /** @brief Makes it easier to add algorithms */
   Go2& operator&(const std::string&);

   /** @brief Allows to register a content creator */
   void registerContentCreator(
         boost::shared_ptr<Gem::Common::GFactoryT<GParameterSet> >
   );
	/** @brief Perform the actual optimization cycle */
	virtual void optimize(const boost::uint32_t& = 0);

	/***************************************************************************/
	// The following is a trivial list of getters and setters
	void setClientMode(bool);
	bool getClientMode() const;

	void setSerializationMode(const Gem::Common::serializationMode&);
	Gem::Common::serializationMode getSerializationMode() const;

	void setServerIp(const std::string&);
	std::string getServerIp() const;

	void setServerPort(const unsigned short&);
	unsigned short getServerPort() const;

	void setVerbosity(const bool&);
	bool getVerbosity() const;

	void setMaxStalledDataTransfers(const boost::uint32_t&);
	boost::uint32_t getMaxStalledDataTransfers() const;

	void setMaxConnectionAttempts(const boost::uint32_t&);
	boost::uint32_t getMaxConnectionAttempts() const;

	void setReturnRegardless(const bool&);
	bool getReturnRegardless() const;

	void setNProducerThreads(const boost::uint16_t&);
	boost::uint16_t getNProducerThreads() const;

	void setOffset(const boost::uint32_t&);
	boost::uint32_t getIterationOffset() const;

	/** @brief Retrieval of the current iteration */
	virtual uint32_t getIteration() const;

	/** @brief Returns the name of this optimization algorithm */
	virtual std::string getAlgorithmName() const;

	/** @brief Loads some configuration data from arguments passed on the command line (or another char ** that is presented to it) */
	void parseCommandLine(int, char **);

	/** @brief Adds local configuration options to a GParserBuilder object */
	virtual void addConfigurationOptions(Gem::Common::GParserBuilder& , const bool&);

	/** @brief Allows to assign a name to the role of this individual(-derivative) */
	virtual std::string getIndividualCharacteristic() const;

	/***************************************************************************/
	/**
	 * Starts the optimization cycle and returns the best individual found, converted to
	 * the desired target type. This is a convenience overload of the corresponding
	 * GOptimizableI function.
	 *
	 * @return The best individual found during the optimization process, converted to the desired type
	 */
	template <typename individual_type>
	boost::shared_ptr<individual_type> optimize() {
		return GOptimizableI::optimize<individual_type>();
	}

	/***************************************************************************/
	/**
	 * Starts the optimization cycle and returns the best individual found, converted to
	 * the desired target type. This function uses a configurable offset for the iteration
	 * counter. This is a convenience overload of the corresponding
	 * GOptimizableI function.
	 *
	 * @param offset An offset for the iteration counter
	 * @return The best individual found during the optimization process, converted to the desired type
	 */
	template <typename individual_type>
	boost::shared_ptr<individual_type> optimize(
			const boost::uint32_t& offset
	) {
		return GOptimizableI::optimize<individual_type>(offset);
	}

   /***************************************************************************/
   /** @brief Emits a name for this class / object */
   virtual std::string name() const;

   /** @brief Allows to register a default algorithm. */
   void registerDefaultAlgorithm(boost::shared_ptr<GOABase>);
   /** @brief Allows to register a default algorithm. */
   void registerDefaultAlgorithm(const std::string& default_algorithm);

protected:
	/***************************************************************************/
	/** @brief Loads the data of another Go2 object */
	virtual void load_(const GObject *);
	/** @brief Creates a deep clone of this object */
	virtual GObject *clone_() const;

	/** @brief Retrieves the best individual found */
	virtual boost::shared_ptr<GIndividual> getBestIndividual();
	/** @brief Retrieves a list of the best individuals found */
	std::vector<boost::shared_ptr<GIndividual> > getBestIndividuals();

private:
   /***************************************************************************/
   /**
    * A termination handler
    */
   static void GTerminateImproperBoostTermination() {
      std::cout
      << "***********************************************************" << std::endl
      << "* Note that there seems to be a bug in some Boost         *" << std::endl
      << "* versions that prevents proper termination of Geneva.    *" << std::endl
      << "* If you see this message it means that you are using     *" << std::endl
      << "* one of the affected releases, so we have to force       *" << std::endl
      << "* termination. Since this happens when all work has       *" << std::endl
      << "* already been done, this will very likely have no effect *" << std::endl
      << "* on your results. So you can safely ignore this message. *" << std::endl
      << "***********************************************************" << std::endl;
   }

	/***************************************************************************/
	// These parameters can enter the object through the constructor
	bool clientMode_; ///< Specifies whether this object represents a network client
	Gem::Common::serializationMode serializationMode_; ///< Indicates whether serialization should be done in Text, XML or Binary form
   std::string ip_; ///< Where the server can be reached
   unsigned short port_; ///< The port on which the server answers
	std::string configFilename_; ///< Indicates where the configuration file is stored
	execMode parMode_; ///< The desired parallelization mode for free-form algorithms
	bool verbose_; ///< Whether additional information should be emitted, e.g. when parsing configuration files

	//----------------------------------------------------------------------------------------------------------------
	// General parameters
    boost::uint32_t maxStalledDataTransfers_; ///< Specifies how often a client may try to unsuccessfully retrieve data from the server (0 means endless)
    boost::uint32_t maxConnectionAttempts_; ///< Specifies how often a client may try to connect unsuccessfully to the server (0 means endless)
    bool returnRegardless_; ///< Specifies whether unsuccessful processing attempts should be returned to the server

    //----------------------------------------------------------------------------------------------------------------
    // Parameters for the random number generator
    boost::uint16_t nProducerThreads_; ///< The number of threads that will simultaneously produce random numbers

    //----------------------------------------------------------------------------------------------------------------
    // Internal parameters
    boost::uint32_t offset_; ///< The offset to be used when starting a new optimization run
    bool sorted_; ///< Indicates whether local individuals have been sorted
    boost::uint32_t iterationsConsumed_; ///< The number of successive iterations performed by this object so far

    //----------------------------------------------------------------------------------------------------------------
    boost::shared_ptr<GParameterSet> bestIndividual_; ///< The best individual found during an optimization run

    //----------------------------------------------------------------------------------------------------------------
    // The list of "chained" optimization algorithms
    std::vector<boost::shared_ptr<GOABase> > algorithms_;
    // Algorithms that were specified on the command line
    std::vector<boost::shared_ptr<GOABase> > cl_algorithms_;
    // The default algorithm (if any)
    boost::shared_ptr<GOABase> default_algorithm_;
    // Holds an object capable of producing objects of the desired type
    boost::shared_ptr<Gem::Common::GFactoryT<GParameterSet> > contentCreatorPtr_;
};

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/**
 * This class performs some necessary initialization and finalization work, so
 * the user does not need to do this manually in main()
 */
class GenevaInitializer {
public:
	/** @brief The default constructor */
   inline GenevaInitializer();
   /** @brief The destructor */
   inline ~GenevaInitializer();

private:
   // Local copies of the factory and broker pointers. Needed so we are
   // sure the corresponding objects still exist when the destructor is called.
   boost::shared_ptr<Gem::Hap::GRandomFactory> grf_;
   boost::shared_ptr<Gem::Courtier::GBrokerT<GIndividual> > gbr_;
};

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/

} /* namespace Geneva */
} /* namespace Gem */

#endif /* GO2_HPP_ */
