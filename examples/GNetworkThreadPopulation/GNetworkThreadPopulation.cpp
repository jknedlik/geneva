/**
 * @file GSimpleBasePopulation.cpp
 */

/* Copyright (C) 2004-2008 Dr. Ruediger Berlich
 * Copyright (C) 2007-2008 Forschungszentrum Karlsruhe GmbH
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

// GenEvA header files go here
#include "GRandom.hpp"
#include "GBasePopulation.hpp"
#include "GDoubleCollection.hpp"
#include "GParameterSet.hpp"
#include "GDoubleGaussAdaptor.hpp"
#include "GLogger.hpp"
#include "GLogTargets.hpp"
#include "GBoostThreadPopulation.hpp"

// The individual that should be optimized
// This is a neural network.
#include "GNeuralNetworkIndividual.hpp"

// Declares a function to parse the command line
#include "GCommandLineParser.hpp"

using namespace Gem::GenEvA;
using namespace Gem::Util;
using namespace Gem::GLogFramework;

/************************************************************************************************/
/**
 * The main function. We search for the minimum of a parabola. This example demonstrates the use
 * of the GBasePopulation class or (at your choice) of the GBoostThreadPopulation class. Note that
 * a number of command line options are available. Call the executable with the "-h" switch
 * to get an overview.
 */
int main(int argc, char **argv){
	 std::size_t nPopThreads;
	 std::size_t populationSize, nParents;
	 std::size_t nData, nDim;
	 std::size_t nHiddenLayer1Nodes, nHiddenLayer2Nodes;
	 double radius, randMin, randMax;
	 boost::uint16_t nProducerThreads;
	 boost::uint32_t maxGenerations, reportGeneration;
	 long maxMinutes;
	 bool verbose;
	 std::string resultFile;
	 recoScheme rScheme;

	// Parse the command line
	if(!parseCommandLine(argc, argv,
		  			     nData,
						 nDim,
						 radius,
						 randMin,
						 randMax,
						 nHiddenLayer1Nodes,
						 nHiddenLayer2Nodes,
						 nProducerThreads,
						 nPopThreads,
						 populationSize,
						 nParents,
						 maxGenerations,
						 maxMinutes,
						 reportGeneration,
						 rScheme,
						 resultFile,
						 verbose))
	{ exit(1); }

	// Add some log levels to the logger
	LOGGER->addLogLevel(Gem::GLogFramework::CRITICAL);
	LOGGER->addLogLevel(Gem::GLogFramework::WARNING);
	LOGGER->addLogLevel(Gem::GLogFramework::INFORMATIONAL);
	LOGGER->addLogLevel(Gem::GLogFramework::PROGRESS);

	// Add log targets to the system
	LOGGER->addTarget(boost::shared_ptr<GBaseLogTarget>(new GDiskLogger("GSimpleBasePopulation.log")));
	LOGGER->addTarget(boost::shared_ptr<GBaseLogTarget>(new GConsoleLogger()));

	// Random numbers are our most valuable good. Set the number of threads
	// that simultaneously produce random numbers.
	GRANDOMFACTORY->setNProducerThreads(nProducerThreads);

	// Create training data for the individual
	// boost::shared_ptr<trainingData> p = GNeuralNetworkIndividual::createHyperSphereTrainingData("", nData, nDim, radius);
	boost::shared_ptr<trainingData> p = GNeuralNetworkIndividual::createHyperCubeTrainingData("", nData, nDim, radius);

	// The neural network architecture is currently hard-wired to two hidden layers
	// Note that this is a restriction of this main function only and not of the network individual.
	std::vector<std::size_t> architecture;
	architecture.push_back(nDim); // Input layer needs exactly the same number of nodes as the dimension of the training data
	architecture.push_back(nHiddenLayer1Nodes);
	architecture.push_back(nHiddenLayer2Nodes);
	architecture.push_back(1); // Output layer has just one node (yes/no decision)

	// Set up a single parabola individual
	boost::shared_ptr<GNeuralNetworkIndividual>
		networkIndividual(new GNeuralNetworkIndividual(p, architecture, randMin, randMax));

	// Now we've got our first individual and can create a population.
	// We choose a multi-threaded population here.

	GBoostThreadPopulation pop;
	pop.setNThreads(nPopThreads);

	pop.push_back(networkIndividual);

	// Specify some population settings
	pop.setPopulationSize(populationSize,nParents);
	pop.setMaxGeneration(maxGenerations);
	pop.setMaxTime(boost::posix_time::minutes(maxMinutes)); // Calculation should be finished after 5 minutes
	pop.setReportGeneration(reportGeneration); // Emit information during every generation
	pop.setRecombinationMethod(rScheme); // The best parents have higher chances of survival

	// Do the actual optimization
	pop.optimize();

	// Save the network
	std::cout << "Saving network ..." << std::endl;
	boost::shared_ptr<GNeuralNetworkIndividual> bestIndividual = pop.individual_cast<GNeuralNetworkIndividual>(0);
    bestIndividual->writeTrainedNetwork("trainingResult.hpp", "testNetwork.cpp");

	std::cout << "Done ..." << std::endl;

	return 0;
}
