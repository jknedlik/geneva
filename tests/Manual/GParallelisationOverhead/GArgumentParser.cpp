/**
 * @file GArgumentParser.cpp
 */

/* Copyright (C) 2009 Dr. Ruediger Berlich
 *
 * This file is part of the Geneva library.
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

#include "GArgumentParser.hpp"

namespace Gem
{
namespace GenEvA
{
namespace bf = boost::filesystem; // Needed for check whether the reference image exists

/************************************************************************************************/
/**
 * A function that parses the command line for all required parameters
 */
bool parseCommandLine(int argc, char **argv,
		std::string& configFile,
		boost::uint16_t& parallelizationMode,
		bool& serverMode,
		std::string& ip,
		unsigned short& port,
		boost::uint32_t& startGeneration)
{
	try{
		// Check the command line options. Uses the Boost program options library.
		po::options_description desc("Usage: evaluator [options]");
		desc.add_options()
			  ("help,h", "emit help message")
			  ("configFile,c", po::value<std::string>(&configFile)->default_value(DEFAULTCONFIGFILE),
					  "The name of the configuration file holding further configuration options")
			  ("parallelizationMode,p", po::value<boost::uint16_t>(&parallelizationMode)->default_value(DEFAULTPARALLELIZATIONMODE),
					  "Whether or not to run this optimization in serial mode (0), multi-threaded (1) or networked (2) mode")
			  ("serverMode,s","Whether to run networked execution in server or client mode. The option only gets evaluated if \"--parallelizationMode=2\"")
			  ("ip",po::value<std::string>(&ip)->default_value(DEFAULTIP), "The ip of the server")
			  ("port",po::value<unsigned short>(&port)->default_value(DEFAULTPORT), "The port of the server")
			  ("startGeneration", po::value<boost::uint32_t>(&startGeneration)->default_value(DEFAULTSTARTGENERATION),
					  "The start value for the generation counter. Set this higher than the last value when starting from a checkpoint file")
		;

		po::variables_map vm;
		po::store(po::parse_command_line(argc, argv, desc), vm);
		po::notify(vm);

		// Emit a help message, if necessary
		if (vm.count("help")) {
			std::cerr << desc << std::endl;
			return false;
		}

		serverMode=false;
		if (vm.count("parallelizationMode")) {
			if(parallelizationMode > 2) {
				std::cout << "Error: the \"-p\" or \"--parallelizationMode\" option may only assume the"<< std::endl
						<< "values 0 (serial), 1 (multi-threaded) or 2 (networked). Leaving ..." << std::endl;
				return false;
			}

			if(parallelizationMode == 2) if(vm.count("serverMode")) serverMode = true;
		}

		if(parallelizationMode != DEFAULTPARALLELIZATIONMODE ||  ip != DEFAULTIP  ||  port != DEFAULTPORT){
			std::string parModeString;
			switch(parallelizationMode) {
			case 0:
				parModeString = "serial";
				break;
			case 1:
				parModeString = "multi-threaded";
				break;
			case 2:
				parModeString = "networked";
				break;
			};

			std::cout << std::endl
					<< "Running with the following command line options:" << std::endl
					<< "configFile = " << configFile << std::endl
					<< "startGeneration = " << startGeneration << std::endl
					<< "parallelizationMode = " << parModeString << std::endl
					<< "serverMode = " << (serverMode?"true":"false") << std::endl
					<< "ip = " << ip << std::endl
					<< "port = " << port << std::endl
					<< std::endl;
		}
	}
	catch(...){
		std::cout << "Error parsing the command line" << std::endl;
		return false;
	}

	return true;

}

/************************************************************************************************/
/**
 * A function that parses a config file for further parameters
 */
bool parseConfigFile(const std::string& configFile,
		boost::uint16_t& nProducerThreads,
		boost::uint16_t& nEvaluationThreads,
		std::size_t& populationSize,
		std::size_t& nParents,
		boost::uint32_t& maxGenerations,
		boost::uint32_t& processingCycles,
		std::size_t& nBoostThreadConsumerThreads,
		boost::uint32_t& waitFactor,
		std::size_t& nVariables)
{
	bool verbose;

	// Check the name of the configuation file
	if(configFile.empty() || configFile == "empty" || configFile == "unknown") {
		std::cerr << "Error: Invalid configuration file name given: \"" << configFile << "\"" << std::endl;
		return false;
	}

	try{
		// Check the configuration file line options. Uses the Boost program options library.
		po::options_description config("Allowed options");
		config.add_options()
  	    ("nProducerThreads",po::value<boost::uint16_t>(&nProducerThreads)->default_value(DEFAULTNPRODUCERTHREADS),
  	    		"The amount of random number producer threads")
  	    ("nEvaluationThreads",po::value<boost::uint16_t>(&nEvaluationThreads)->default_value(DEFAULTNEVALUATIONTHREADS),
  	    		"The amount of threads processing individuals simultaneously")
  	    ("populationSize",po::value<std::size_t>(&populationSize)->default_value(DEFAULTPOPULATIONSIZE),
  	    		"The size of the super-population")
  	    ("nParents",po::value<std::size_t>(&nParents)->default_value(DEFAULTNPARENTS),
  	    		"The number of parents in the population") // Needs to be treated separately
  	    ("maxGenerations", po::value<boost::uint32_t>(&maxGenerations)->default_value(DEFAULTMAXGENERATIONS),
  	    		"Maximum number of generations in the population")
  	    ("verbose",po::value<bool>(&verbose)->default_value(DEFAULTVERBOSE),
  	    		"Whether additional information should be emitted")
  	    ("processingCycles", po::value<boost::uint32_t>(&processingCycles)->default_value(DEFAULTPROCESSINGCYCLES),
  	    		"The maximum number of cycles a client should perform mutations before it returns without success")
  	    ("nBoostThreadConsumerThreads", po::value<std::size_t>(&nBoostThreadConsumerThreads)->default_value(DEFAULTNBTCONSUMERTHREADS),
  	    		"The number of threads used in the GBoostThreadConsumer to process individuals")
  	    ("waitFactor", po::value<boost::uint32_t>(&waitFactor)->default_value(DEFAULTGBTCWAITFACTOR),
  	    		"Influences the maximum waiting time of the GBrokerPopulation after the arrival of the first evaluated individuum")
		("nVariables", po::value<std::size_t>(&nVariables)->default_value(DEFAULTNVARIABLES),
				"The amount of variables in each individual")
  	    ;

		po::variables_map vm;
		std::ifstream ifs(configFile.c_str());
		if(!ifs.good()) {
			std::cerr << "Error accessing configuration file " << configFile;
			return false;
		}

		store(po::parse_config_file(ifs, config), vm);
		notify(vm);

		// Emit a help message, if necessary
		if (vm.count("help")) {
			std::cout << config << std::endl;
			return false;
		}

		// Check the number of parents in the super-population
		if(2*nParents > populationSize){
			std::cout << "Error: Invalid number of parents inpopulation" << std::endl
					<< "nParents       = " << nParents << std::endl
					<< "populationSize = " << populationSize << std::endl;

			return false;
		}

		if(waitFactor == 0) waitFactor = DEFAULTGBTCWAITFACTOR;

		if(sleepSeconds < 0 || sleepMilliSeconds < 0) {
			std::cout << "Error: invalid sleep time:" << sleepSeconds << " " << sleepMilliSeconds << std::endl;
			return false;
		}

		if(verbose){
			std::cout << std::endl
					<< "Running with the following options from " << configFile << ":" << std::endl
					<< "nProducerThreads = " << (boost::uint16_t)nProducerThreads << std::endl // boost::uint8_t not printable on gcc ???
					<< "populationSize = " << populationSize << std::endl
					<< "nParents = " << nParents << std::endl
					<< "maxGenerations = " << maxGenerations << std::endl
					<< "processingCycles = " << processingCycles << std::endl
					<< "nBoostThreadConsumerThreads = " << nBoostThreadConsumerThreads << std::endl
					<< "waitFactor = " << waitFactor << std::endl
					<< "sleepSeconds = " << sleepSeconds << std::endl
					<< "sleepMilliSeconds = " << sleepMilliSeconds << std::endl
					<< std::endl;
		}
	}
	catch(...){
		std::cout << "Error parsing the configuration file " << configFile << std::endl;
		return false;
	}

	return true;
}

/************************************************************************************************/

} /* namespace GenEvA */
} /* namespace Gem */
