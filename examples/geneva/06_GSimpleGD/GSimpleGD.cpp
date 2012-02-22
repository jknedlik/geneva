/**
 * @file GSimpleGD.cpp
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
#include <cmath>
#include <sstream>

// Boost header files go here
#include <boost/lexical_cast.hpp>

// Geneva header files go here
#include <geneva/Geneva.hpp>

// The individual that should be optimized
#include "geneva-individuals/GFunctionIndividual.hpp"

// Declares a function to parse the command line
#include "GArgumentParser.hpp"

// Holds the optimization monitor
// #include "GInfoFunction.hpp"

using namespace Gem::Geneva;
using namespace Gem::Courtier;
using namespace Gem::Hap;


/************************************************************************************************/
/**
 * The main function.
 */
int main(int argc, char **argv){
  std::string configFile;		  
  boost::uint16_t parallelizationMode;
  bool serverMode;
  std::string ip;
  unsigned short port;
  boost::uint16_t nProducerThreads;
  boost::uint16_t nEvaluationThreads;
  std::size_t populationSize;
  std::size_t nParents;
  boost::uint32_t maxIterations;
  long maxMinutes;
  boost::uint32_t reportIteration;
  duplicationScheme rScheme;
  std::size_t arraySize;
  std::size_t parDim;
  double minVar;
  double maxVar;
  sortingMode smode;
  boost::uint32_t processingCycles;
  boost::uint32_t nProcessingUnits;
  solverFunction df;
  boost::uint32_t adaptionThreshold;
  double sigma;
  double sigmaSigma;
  double minSigma;
  double maxSigma;
  double adProb;
  bool returnRegardless;
  Gem::Common::serializationMode serMode;
  boost::uint16_t xDim;
  boost::uint16_t yDim;
  bool followProgress;
  bool trackParentRelations;
  bool drawArrows;
  std::size_t nStartingPoints;
  float finiteStep;
  float stepSize;

  if(
   !parseCommandLine(
		  argc
		  , argv
		  , configFile
		  , parallelizationMode
		  , serverMode
		  , ip
		  , port
		  , serMode
  	)
     ||
   !parseConfigFile(
   		configFile
   		, nProducerThreads
   		, nEvaluationThreads
   		, nStartingPoints
   		, finiteStep
   		, stepSize
   		, maxIterations
   		, maxMinutes
   		, reportIteration
   		, arraySize
   		, processingCycles
   		, returnRegardless
   		, nProcessingUnits
   		, parDim
   		, minVar
   		, maxVar
   		, df
	)
  )
  { exit(1); }

  //***************************************************************************
  // Initialize Geneva
  Geneva::init();

  // Random numbers are our most valuable good. Set the number of threads
  GRANDOMFACTORY->setNProducerThreads(nProducerThreads);
  GRANDOMFACTORY->setArraySize(arraySize);

  //***************************************************************************
  // If this is a client in networked mode, we can just start the listener and
  // return when it has finished
  if(parallelizationMode==2 && !serverMode) {
    boost::shared_ptr<GAsioTCPClientT<GIndividual> > p(new GAsioTCPClientT<GIndividual>(ip, boost::lexical_cast<std::string>(port)));

    p->setMaxStalls(0); // An infinite number of stalled data retrievals
    p->setMaxConnectionAttempts(100); // Up to 100 failed connection attempts

    // Prevent return of unsuccessful adaption attempts to the server
    p->returnResultIfUnsuccessful(returnRegardless);

    // Start the actual processing loop
    p->run();

    return 0;
  }

  //***************************************************************************
  // Create a factory for GFunctionIndividual objects and perform
  // any necessary initial work.
  GFunctionIndividualFactory gfi("./GFunctionIndividual.cfg");
  gfi.init();

  // Create the first set of parent individuals. Initialization of parameters is done randomly.
  std::vector<boost::shared_ptr<GParameterSet> > parentIndividuals;
  for(std::size_t p = 0 ; p<nStartingPoints; p++) {
	  boost::shared_ptr<GParameterSet> functionIndividual_ptr = gfi();
	  functionIndividual_ptr->randomInit();
	  parentIndividuals.push_back(functionIndividual_ptr);
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // We can now start creating populations. We refer to them through the base class

  // This smart pointer will hold the different population types
  boost::shared_ptr<GBaseGD> pop_ptr;

  // Create the actual populations
  switch (parallelizationMode) {
  //-----------------------------------------------------------------------------------------------------
  case 0: // Serial execution
	  // Create an empty population
	  pop_ptr = boost::shared_ptr<GSerialGD>(new GSerialGD(nStartingPoints, finiteStep, stepSize));
	  break;

	  //-----------------------------------------------------------------------------------------------------
  case 1: // Multi-threaded execution
  {
	  // Create the multi-threaded population
	  boost::shared_ptr<GMultiThreadedGD> popPar_ptr(new GMultiThreadedGD(nStartingPoints, finiteStep, stepSize));

	  // Population-specific settings
	  popPar_ptr->setNThreads(nEvaluationThreads);

	  // Assignment to the base pointer
	  pop_ptr = popPar_ptr;
  }
  break;

  //-----------------------------------------------------------------------------------------------------
  case 2: // Execution with networked consumer
  {
	  // Create a network consumer and enrol it with the broker
	  boost::shared_ptr<GAsioTCPConsumerT<GIndividual> > gatc(new GAsioTCPConsumerT<GIndividual>(port, 0, serMode));
	  GBROKER(Gem::Geneva::GIndividual)->enrol(gatc);

	  // Create the actual broker population
	  boost::shared_ptr<GBrokerGD> popBroker_ptr(new GBrokerGD(nStartingPoints, finiteStep, stepSize));

	  // Assignment to the base pointer
	  pop_ptr = popBroker_ptr;
  }
  break;
  }


  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Now we have suitable populations and can fill them with data

  // Add individuals to the population
  for(std::size_t p = 0 ; p<nStartingPoints; p++) {
    pop_ptr->push_back(parentIndividuals[p]);
  }
 
  // Specify some general population settings
  pop_ptr->setMaxIteration(maxIterations);
  pop_ptr->setMaxTime(boost::posix_time::minutes(maxMinutes));
  pop_ptr->setReportIteration(reportIteration);
  // pop_ptr->registerInfoFunction(boost::bind(&optimizationMonitor::informationFunction, om_ptr, _1, _2));
  
  // Do the actual optimization
  pop_ptr->optimize();

  //--------------------------------------------------------------------------------------------
  // Terminate
  Geneva::finalize();

  std::cout << "Done ..." << std::endl;
  return(0);
}