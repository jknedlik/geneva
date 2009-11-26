/**
 * @file GStartProject.cpp
 */

/* Copyright (C) 2009 Dr. Ruediger Berlich
 *
 * This file is part of Geneva, Gemfony scientific's optimization library.
 *
 * Geneva is free software: you can redistribute it and/or modify
 * it under the terms of version 3 of the GNU Affero General Public License
 * as published by the Free Software Foundation.
 *
 * Geneva is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with the Geneva library.  If not, see <http://www.gnu.org/licenses/>.
 */

// Standard header files go here
#include <iostream>
#include <cmath>
#include <sstream>

// Boost header files go here
#include <boost/lexical_cast.hpp>

// GenEvA header files go here
#include "GRandom.hpp"
#include "GBasePopulation.hpp"
#include "GBoostThreadPopulation.hpp"
#include "GBrokerPopulation.hpp"
#include "GIndividualBroker.hpp"
#include "GAsioTCPConsumer.hpp"
#include "GAsioTCPClient.hpp"
#include "GAsioHelperFunctions.hpp"

// The individual that should be optimized
#include "GStartIndividual.hpp"

// Declares a function to parse the command line
#include "GArgumentParser.hpp"

// Information retrieval and printing
#include "GInfoFunction.hpp"

using namespace Gem::GenEvA;
using namespace Gem::Util;

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
  boost::uint32_t maxGenerations;
  long maxMinutes;
  boost::uint32_t reportGeneration;
  recoScheme rScheme;
  std::size_t arraySize;
  std::size_t parDim;
  double minVar;
  double maxVar;
  boost::uint16_t infoInterval;
  sortingMode smode;
  boost::uint32_t processingCycles;
  bool returnRegardless;
  boost::uint32_t waitFactor;

  if(!parseCommandLine(argc, argv,
		       configFile,			  
		       parallelizationMode,
		       serverMode,
		       ip,
		       port)
     ||
     !parseConfigFile(configFile,
		      nProducerThreads,
		      nEvaluationThreads,
		      populationSize,
		      nParents,
		      maxGenerations,
		      maxMinutes,
		      reportGeneration,
		      rScheme,
		      smode,
		      arraySize,
		      infoInterval,
		      processingCycles,
		      returnRegardless,
		      waitFactor,
		      parDim,
		      minVar,
		      maxVar))
    { exit(1); }

  // Random numbers are our most valuable good. Set the number of threads
  GRANDOMFACTORY->setNProducerThreads(nProducerThreads);
  GRANDOMFACTORY->setArraySize(arraySize);
  
  //***************************************************************************
  // If this is a client in networked mode, we can just start the listener and
  // return when it has finished
  if(parallelizationMode==2 && !serverMode) {
    boost::shared_ptr<GAsioTCPClient> p(new GAsioTCPClient(ip, boost::lexical_cast<std::string>(port)));

    p->setMaxStalls(0); // An infinite number of stalled data retrievals
    p->setMaxConnectionAttempts(100); // Up to 100 failed connection attempts

    // Prevent return of unsuccessful mutation attempts to the server
    p->returnResultIfUnsuccessful(returnRegardless);

    // Start the actual processing loop
    p->run();

    return 0;
  }
  //***************************************************************************

  // Create the first set of parent individuals. Initialization of parameters is done randomly.
  std::vector<boost::shared_ptr<GStartIndividual> > parentIndividuals;
  for(std::size_t p = 0 ; p<nParents; p++) {
    boost::shared_ptr<GStartIndividual> gdii_ptr(new GStartIndividual(parDim, minVar, maxVar));
    gdii_ptr->setProcessingCycles(processingCycles);

    parentIndividuals.push_back(gdii_ptr);
  }

  // Create an instance of our optimization monitor, telling it to output information in given intervals
  std::ofstream resultSummary("./result.C");
  boost::shared_ptr<optimizationMonitor> om(new optimizationMonitor(infoInterval, nParents, resultSummary));

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // We can now start creating populations. We refer to them through the base class

  // This smart pointer will hold the different population types
  boost::shared_ptr<GBasePopulation> pop_ptr;

  // Create the actual populations
  switch (parallelizationMode) {
    //-----------------------------------------------------------------------------------------------------
  case 0: // Serial execution
    // Create an empty population
    pop_ptr = boost::shared_ptr<GBasePopulation>(new GBasePopulation());
    break;

    //-----------------------------------------------------------------------------------------------------
  case 1: // Multi-threaded execution
    {
      // Create the multi-threaded population
      boost::shared_ptr<GBoostThreadPopulation> popPar_ptr(new GBoostThreadPopulation());

      // Population-specific settings
      popPar_ptr->setNThreads(nEvaluationThreads);

      // Assignment to the base pointer
      pop_ptr = popPar_ptr;
    }
    break;

    //-----------------------------------------------------------------------------------------------------
  case 2: // Networked execution (server-side)
    {
      // Create a network consumer and enrol it with the broker
      boost::shared_ptr<GAsioTCPConsumer> gatc(new GAsioTCPConsumer(port));
      GINDIVIDUALBROKER->enrol(gatc);

      // Create the actual broker population
      boost::shared_ptr<GBrokerPopulation> popBroker_ptr(new GBrokerPopulation());
      popBroker_ptr->setWaitFactor(waitFactor);

      // Assignment to the base pointer
      pop_ptr = popBroker_ptr;
    }
    break;
  }


  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Now we have suitbale populations and can fill them with data

  // Add individuals to the population
  for(std::size_t p = 0 ; p<nParents; p++) {
    pop_ptr->push_back(parentIndividuals[p]);
  }
 
  // Specify some general population settings
  pop_ptr->setPopulationSize(populationSize,nParents);
  pop_ptr->setMaxGeneration(maxGenerations);
  pop_ptr->setMaxTime(boost::posix_time::minutes(maxMinutes)); // Calculation should be finished after 5 minutes
  pop_ptr->setReportGeneration(reportGeneration); // Emit information during every generation
  pop_ptr->setRecombinationMethod(rScheme); // The best parents have higher chances of survival
  pop_ptr->setSortingScheme(smode); // Determines the sorting mode used (MUPLUSNU, MUCOMMANU, MUNU1PRETAIN)
  pop_ptr->registerInfoFunction(boost::bind(&optimizationMonitor::informationFunction, om, _1, _2));
  
  // Do the actual optimization
  pop_ptr->optimize();

  //--------------------------------------------------------------------------------------------

  // Make sure we close the result file
  resultSummary.close();

  std::cout << "Done ..." << std::endl;
  return 0;
}
