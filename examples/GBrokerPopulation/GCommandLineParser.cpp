/**
 * @file GCommandLineParser.cpp
 */

/* Copyright (C) 2004-2008 Dr. Ruediger Berlich
 * Copyright (C) 2008 Forschungszentrum Karlsruhe GmbH
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

#include "GCommandLineParser.hpp"

namespace Gem
{
namespace GenEvA
{

/************************************************************************************************/
/**
 * A function that parses the command line for all required parameters
 */
bool parseCommandLine(int argc, char **argv,
					  std::string& ip,
					  std::string& mode,
					  unsigned short& port,
					  std::size_t& parabolaDimension,
					  double& parabolaMin,
					  double& parabolaMax,
					  boost::uint16_t& nProducerThreads,
					  std::size_t& populationSize,
					  std::size_t& nParents,
					  boost::uint32_t& maxGenerations,
					  long& maxMinutes,
					  boost::uint32_t& reportGeneration,
					  recoScheme& rScheme,
					  bool& verbose)
{
	boost::uint16_t recombinationScheme;

	try{
		// Check the command line options. Uses the Boost program options library.
		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "emit help message")
			("ip",po::value<std::string>(&ip)->default_value(DEFAULTIP),
					"The ip of the server - required by the client")
			("mode",po::value<std::string>(&mode)->default_value(DEFAULTMODE),
					"server or client, dependent on the mode")
			("port",po::value<unsigned short>(&port)->default_value(DEFAULTPORT),
					"The port of the server")
			("parabolaDimension,d", po::value<std::size_t>(&parabolaDimension)->default_value(DEFAULTPARABOLADIMENSION),
					"number of dimensions in the parabola")
			("parabolaMin,m", po::value<double>(&parabolaMin)->default_value(DEFAULTPARABOLAMIN),
					"Lower boundary for random numbers")
			("parabolaMax,M", po::value<double>(&parabolaMax)->default_value(DEFAULTPARABOLAMAX),
					"Upper boundary for random numbers")
			("nProducerThreads,p",po::value<boost::uint16_t>(&nProducerThreads)->default_value(DEFAULTNPRODUCERTHREADS),
					"The amount of random number producer threads")
			("populationSize,s",po::value<std::size_t>(&populationSize)->default_value(DEFAULTPOPULATIONSIZE),
					"The size of the population")
			("nParents,P",po::value<std::size_t>(&nParents)->default_value(DEFAULTNPARENTS),
					"The number of parents in the population") // Needs to be treated separately
			("maxGenerations,g", po::value<boost::uint32_t>(&maxGenerations)->default_value(DEFAULTMAXGENERATIONS),
					"maximum number of generations")
			("maxMinutes,x", po::value<long>(&maxMinutes)->default_value(DEFAULTMAXMINUTES),
					"The maximum number of minutes the optimization should run")
			("reportGeneration,G",po::value<boost::uint32_t>(&reportGeneration)->default_value(DEFAULTREPORTGENERATION),
					"The number of generations after which information should be emitted")
			("rScheme,r",po::value<boost::uint16_t>(&recombinationScheme)->default_value(DEFAULTRSCHEME),
					"The recombination scheme")
			("verbose,v",po::value<bool>(&verbose)->default_value(DEFAULTVERBOSE),
					"Whether additional information should be emitted")
		;

		po::variables_map vm;
		po::store(po::parse_command_line(argc, argv, desc), vm);
		po::notify(vm);

		// Emit a help message, if necessary
		if (vm.count("help")) {
			 std::cout << desc << std::endl;
			 return false;
		}

		// Check the number of parents
		if(2*nParents > populationSize){
			std::cout << "Error: Invalid number of parents" << std::endl
				      << "nParents       = " << nParents << std::endl
				      << "populationSize = " << populationSize << std::endl;

			return false;
		}

		// Check the parabolaMin/Max parameters
		if(parabolaMin >= parabolaMax){
			std::cout << "Error: Invalid parabolaMin/Max parameters" << std::endl
				      << "parabolaMin = " << parabolaMin << std::endl
				      << "parabolaMax = " << parabolaMax << std::endl;

			return false;;
		}

		// Workaround for assigment problem with recoScheme
		if(recombinationScheme==(boost::uint16_t)VALUERECOMBINE)
			rScheme=VALUERECOMBINE;
		else if(recombinationScheme==(boost::uint16_t)RANDOMRECOMBINE)
			rScheme=RANDOMRECOMBINE;
		else if(recombinationScheme==(boost::uint16_t)DEFAULTRECOMBINE)
			rScheme=DEFAULTRECOMBINE;
		else {
			std::cout << "Error: Invalid recombination scheme: " << recombinationScheme << std::endl;
			return false;
		}

		if(verbose){
			std::cout << std::endl
				      << "Running with the following options:" << std::endl
				      << "ip = " << ip << std::endl
				      << "mode = " << mode << std::endl
				      << "port = " << port << std::endl
					  << "parabolaDimension = " << parabolaDimension << std::endl
					  << "parabolaMin = " << parabolaMin << std::endl
					  << "parabolaMax = " << parabolaMax << std::endl
					  << "nProducerThreads = " << (boost::uint16_t)nProducerThreads << std::endl // boost::uint8_t not printable on gcc ???
					  << "populationSize = " << populationSize << std::endl
					  << "nParents = " << nParents << std::endl
					  << "maxGenerations = " << maxGenerations << std::endl
					  << "maxMinutes = " << maxMinutes << std::endl
					  << "reportGeneration = " << reportGeneration << std::endl
					  << "rScheme = " << (boost::uint16_t)rScheme << std::endl
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

} /* namespace GenEvA */
} /* namespace Gem */
