/**
 * @file GRandomUsage.cpp
 */

/********************************************************************************
 *
 * This file is part of the Geneva library collection. The following license
 * applies to this file:
 *
 * ------------------------------------------------------------------------------
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * ------------------------------------------------------------------------------
 *
 * Note that other files in the Geneva library collection may use a different
 * license. Please see the licensing information in each file.
 *
 ********************************************************************************
 *
 * Geneva was started by Dr. Rüdiger Berlich and was later maintained together
 * with Dr. Ariel Garcia under the auspices of Gemfony scientific. For further
 * information on Gemfony scientific, see http://www.gemfomy.eu .
 *
 * The majority of files in Geneva was released under the Apache license v2.0
 * in February 2020.
 *
 * See the NOTICE file in the top-level directory of the Geneva library
 * collection for a list of contributors and copyright information.
 *
 ********************************************************************************/
/**
 * This test creates NENTRIES random numbers each for numbers and items with
 * different characteristics. Each set is created in its own thread.
 * Note that the random numbers are usually not created in the GRandomT
 * object, but by the GRandomFactory class in a different thread.
 * GRandomT just acts as a proxy.
 *
 * The results of the test are output in the ROOT format. See
 * http://root.cern.ch for further information.
 */

// Standard header files
#include <vector>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <random>

// Boost header files

// Geneva header files
#include "common/GLogger.hpp"
#include "common/GPlotDesigner.hpp"
#include "common/GParserBuilder.hpp"
#include "hap/GRandomT.hpp"
#include "hap/GRandomDistributionsT.hpp"

using namespace Gem::Hap;
using namespace Gem::Common;
using namespace boost;

/************************************************************************************************/
// Default settings
const std::size_t DEFAULTNENTRIES=10000;
const std::uint16_t DEFAULTNPRODUCERTHREADS=10;
const bool DEFAULTVERBOSE=true;
const std::uint16_t DEFAULTRNRPRODUCTIONMODE=0;
const std::string DEFAULTRESULTFILE="randomResult.C";

/************************************************************************************************/
/**
 * A function that parses the command line for all required parameters
 */

bool parseCommandLine(
	int argc, char **argv
	, std::size_t& nEntries
	, std::uint16_t& nProducerThreads
	, std::uint16_t& rnrProductionMode
	, std::string& resultFile
) {
	// Create the parser builder
	Gem::Common::GParserBuilder gpb;

	gpb.registerCLParameter<std::size_t>(
		"nEntries,n"
		, nEntries
		, DEFAULTNENTRIES
		, "Number of random numbers to generate for each distribution"
	);

	gpb.registerCLParameter<std::uint16_t>(
		"nProducerThreads,t"
		, nProducerThreads
		, DEFAULTNPRODUCERTHREADS
		, "The amount of random number producer threads"
	);

	gpb.registerCLParameter<std::uint16_t>(
		"rnrProductionMode,r"
		, rnrProductionMode
		, DEFAULTRNRPRODUCTIONMODE
		, "FACTORY(0), or LOCAL(1)"
	);

	gpb.registerCLParameter<std::string>(
		"resultFile,f"
		, resultFile
		, DEFAULTRESULTFILE
		, "The name of the file to which results should be written"
	);

	// Parse the command line and leave if the help flag was given. The parser
	// will emit an appropriate help message by itself
	if(Gem::Common::GCL_HELP_REQUESTED == gpb.parseCommandLine(argc, argv, true /*verbose*/)) {
		return false; // Do not continue
	}

	return true;

}

/************************************************************************************************/

enum class distType : Gem::Common::ENUMBASETYPE {
	 GAUSSIAN
	 , DOUBLEGAUSSIAN
	 , EVEN
	 , EVENWITHBOUNDARIES
	 , DISCRETE
	 , DISCRETEBOUND
	 , BITPROB
	 , BITSIMPLE
	 , EXPGAUSS01
	 , EXPGAUSS02
	 , EXPGAUSS04
	 , EXPGAUSS08
	 , EXPGAUSS16
};

/************************************************************************************************/
/**
 * Puts a distType item into a stream
 */
std::ostream &operator<<(std::ostream &o, const distType &x) {
	Gem::Common::ENUMBASETYPE tmp = static_cast<Gem::Common::ENUMBASETYPE>(x);
	o << tmp;
	return o;
}

/************************************************************************************************/
/**
 * Reads a distType item from a stream
 */
std::istream &operator>>(std::istream &i, distType &x) {
	Gem::Common::ENUMBASETYPE tmp;
	i >> tmp;

#ifdef DEBUG
	x = boost::numeric_cast<distType>(tmp);
#else
	x = static_cast<distType>(tmp);
#endif /* DEBUG */

	return i;
}

/************************************************************************************************/

template <class T>
void createRandomVector(
	std::vector<T>& vec_t
	, const distType& dType
	, const std::size_t& nEntries
	, std::shared_ptr<Gem::Hap::GRandomBase> gr_ptr
){
	std::size_t i;

	std::normal_distribution<double> normal_distribution(-3.,1.);
	Gem::Hap::bi_normal_distribution<double> bi_normal_distribution(-3,0.5,0.5, 3.);
	std::uniform_real_distribution<double> uniform_real_distribution_01;
	std::uniform_int_distribution<std::int32_t> uniform_int_distribution;

	switch(dType){
		case distType::GAUSSIAN: // standard distribution
			for(i=0; i<nEntries; i++) {
				vec_t.push_back(T(normal_distribution(*gr_ptr)));
			}
			break;

		case distType::DOUBLEGAUSSIAN:
			for(i=0; i<nEntries; i++) {
				vec_t.push_back(T(bi_normal_distribution(*gr_ptr)));
			} // (mean, sigma, distance)
			break;

		case distType::EVEN: // double in the range [0,1[
			for(i=0; i<nEntries; i++) {
				vec_t.push_back(T(uniform_real_distribution_01(*gr_ptr)));
			}
			break;

		case distType::EVENWITHBOUNDARIES: // double in the range [-3,2[
			for(i=0; i<nEntries; i++) {
				vec_t.push_back(T(uniform_real_distribution_01(*gr_ptr, std::uniform_real_distribution<double>::param_type(-3.,2.))));
			}
			break;

		case distType::DISCRETE:
			for(i=0; i<nEntries; i++) {
				vec_t.push_back(uniform_int_distribution(*gr_ptr, std::uniform_int_distribution<std::int32_t>::param_type(0,10)));
			}
			break;

		case distType::DISCRETEBOUND:
			for(i=0; i<nEntries; i++) {
				vec_t.push_back(uniform_int_distribution(*gr_ptr, std::uniform_int_distribution<std::int32_t>::param_type(-3,10)));
			}
			break;

		case distType::BITPROB: {
			// Note: A probaility of 0.7 results in roughly 70% "true" values
			std::bernoulli_distribution weighted_bool(0.7);
			for (i = 0; i < nEntries; i++) {
				if (weighted_bool(*gr_ptr))
					vec_t.push_back(1);
				else
					vec_t.push_back(0);
			}
		}
			break;

		case distType::BITSIMPLE: {
			std::bernoulli_distribution uniform_bool; // defaults to 0.5
			for (i = 0; i < nEntries; i++) {
				if (uniform_bool(*gr_ptr))
					vec_t.push_back(1);
				else
					vec_t.push_back(0);
			}
		}
			break;

		case distType::EXPGAUSS01:
			for(i=0; i<nEntries; i++) {
				vec_t.push_back(T(exp(normal_distribution(*gr_ptr, std::normal_distribution<double>::param_type(0.,0.1)))));
			}
			break;

		case distType::EXPGAUSS02:
			for(i=0; i<nEntries; i++) {
				vec_t.push_back(T(exp(normal_distribution(*gr_ptr, std::normal_distribution<double>::param_type(0.,0.2)))));
			}
			break;

		case distType::EXPGAUSS04:
			for(i=0; i<nEntries; i++) {
				vec_t.push_back(T(exp(normal_distribution(*gr_ptr, std::normal_distribution<double>::param_type(0.,0.4)))));
			}
			break;

		case distType::EXPGAUSS08:
			for(i=0; i<nEntries; i++) {
				vec_t.push_back(T(exp(normal_distribution(*gr_ptr, std::normal_distribution<double>::param_type(0.,0.8)))));
			}
			break;

		case distType::EXPGAUSS16:
			for(i=0; i<nEntries; i++) {
				vec_t.push_back(T(exp(normal_distribution(*gr_ptr, std::normal_distribution<double>::param_type(0.,1.6)))));
			}
			break;

		default:
		{
			throw gemfony_exception(
				g_error_streamer(DO_LOG,  time_and_place)
					<< "In createRandomVector(): Error!" << std::endl
					<< "Received invalid distType " << dType << std::endl
			);
		}
			break;
	}
}

/************************************************************************************************/

int main(int argc, char **argv) {
	std::shared_ptr<Gem::Hap::GRandomBase> gr_ptr;

	std::size_t nEntries;
	std::uint16_t nProducerThreads;
	std::uint16_t rnrProductionMode;
	std::string resultFile;

	if (!parseCommandLine(
		argc, argv
		, nEntries
		, nProducerThreads
		, rnrProductionMode
		, resultFile
	)) { exit(1); }

	std::size_t i;
	std::vector<double> gaussian, doublegaussian, even, evenwithboundaries;
	std::vector<double> expgauss01, expgauss02, expgauss04, expgauss08, expgauss16;
	std::vector<std::int32_t> discrete, discretebound, bitprob, bitsimple, charrnd;
	std::vector<double> initCorr, initLFCorr;

	GRANDOMFACTORY->setNProducerThreads(nProducerThreads);

	// Set the random number generation mode as requested
	switch (rnrProductionMode) {
		case 0:
			gr_ptr = std::shared_ptr<GRandomT<RANDFLAVOURS::RANDOMPROXY>>(new GRandomT<RANDFLAVOURS::RANDOMPROXY>());
			break;

		case 1:
			gr_ptr = std::shared_ptr<GRandomT<RANDFLAVOURS::RANDOMLOCAL>>(new GRandomT<RANDFLAVOURS::RANDOMLOCAL>());
			break;

		default:
		{
			throw gemfony_exception(
				g_error_streamer(DO_LOG,  time_and_place)
					<< "In main(): Error!" << std::endl
					<< "Received invalid rnrProductionMode " << rnrProductionMode << std::endl
			);
		}
			break;
	};


	// Create the GPlotDesigner object
	GPlotDesigner gpd("GRandomUsage", 4,4);
	gpd.setCanvasDimensions(1200,1200);


	std::ofstream ofs("randomResult.C");
	if (!ofs) {
		glogger
			<< "Error: Could not write file" << std::endl
			<< GWARNING;
		return 1;
	}

	// The header of the root file
	ofs << "{" << std::endl;
	ofs << "  TCanvas *cc = new TCanvas(\"cc\",\"cc\",0,0,1000,1200);" << std::endl
		 << "  cc->Divide(4,4);" << std::endl
		 << std::endl
		 << "  TH1F *gauss = new TH1F(\"gauss\",\"gauss\",200,-8.,2.);" << std::endl
		 << "  TH1F *dgauss = new TH1F(\"dgauss\",\"dgauss\",200,-8.,2.);" << std::endl
		 << "  TH1F *expGauss01 = new TH1F(\"expGauss01\",\"expGauss01\",110,-1.,10.);" << std::endl
		 << "  TH1F *expGauss02 = new TH1F(\"expGauss02\",\"expGauss02\",110,-1.,10.);" << std::endl
		 << "  TH1F *expGauss04 = new TH1F(\"expGauss04\",\"expGauss04\",110,-1.,10.);" << std::endl
		 << "  TH1F *expGauss08 = new TH1F(\"expGauss08\",\"expGauss08\",110,-1.,10.);" << std::endl
		 << "  TH1F *expGauss16 = new TH1F(\"expGauss16\",\"expGauss16\",110,-1.,10.);" << std::endl
		 << "  TH1F *even = new TH1F(\"even\",\"even\",200,-0.5,1.5);" << std::endl
		 << "  TH1F *evenwb = new TH1F(\"evenwb\",\"evenwb\",200,-3.5,2.5);" << std::endl
		 << "  TH1I *discrete = new TH1I(\"discrete\",\"discrete\",12,-1,10);" << std::endl
		 << "  TH1I *discretewb = new TH1I(\"discretewb\",\"discretewb\",16,-4,11);" << std::endl
		 << "  TH1I *bitprob = new TH1I(\"bitprob\",\"bitprob\",4,-1,2);" << std::endl
		 << "  TH1I *bitsimple = new TH1I(\"bitsimple\",\"bitsimple\",4,-1,2);" << std::endl
		 << "  TH1I *charrnd = new TH1I(\"charrnd\",\"charrnd\",131,-1,129);" << std::endl
		 << "  TH2F *evenSelfCorrelation = new TH2F(\"evenSelfCorrelation\",\"evenSelfCorrelation\",100, 0.,1.,100, 0.,1.);" << std::endl
		 << "  TH1F *initCorrelation = new TH1F(\"initCorrelation\",\"initCorrelation\",10,0.5,10.5);" << std::endl
		 << "  TH1F *initLFCorrelation = new TH1F(\"initLFCorrelation\",\"initLFCorrelation\",10,0.5,10.5);" << std::endl // Lagged Fibonacci
		 << "  TH2F *evenRNGCorrelation = new TH2F(\"evenRNGCorrelation\",\"evenRNGCorrelation\",100, 0.,1.,100, 0.,1.);" << std::endl
		 << "  TH1F *rngDiff = new TH1F(\"rngDiff\",\"rngDiff\"," << nEntries << ", " << 0.5 << "," << 100.5 << ");" << std::endl
		 << std::endl;

	// In this test correlations between sequential random numbers (with same proxy/seed) are sought for
	{
		std::uniform_real_distribution<double> uniform_real_distribution(0., 1.);
		for (i = 0; i < nEntries; i++) {
			ofs << "  evenSelfCorrelation->Fill(" << uniform_real_distribution(*gr_ptr) << ", " <<
				 uniform_real_distribution(*gr_ptr) << ");" << std::endl;
		}
		ofs << std::endl;
	}

	// In this test correlations between subsequent numbers of two generators (with different seeds) are sought for
	std::shared_ptr<Gem::Hap::GRandomBase> gr_ptr_one;
	std::shared_ptr<Gem::Hap::GRandomBase> gr_ptr_two;

	switch (rnrProductionMode) {
		case 0:
			gr_ptr_one = std::shared_ptr<GRandomT<RANDFLAVOURS::RANDOMPROXY>>(
				new GRandomT<RANDFLAVOURS::RANDOMPROXY>());
			gr_ptr_two = std::shared_ptr<GRandomT<RANDFLAVOURS::RANDOMPROXY>>(
				new GRandomT<RANDFLAVOURS::RANDOMPROXY>());
			break;

		case 1:
			gr_ptr_one = std::shared_ptr<GRandomT<RANDFLAVOURS::RANDOMLOCAL>>(
				new GRandomT<RANDFLAVOURS::RANDOMLOCAL>());
			gr_ptr_two = std::shared_ptr<GRandomT<RANDFLAVOURS::RANDOMLOCAL>>(
				new GRandomT<RANDFLAVOURS::RANDOMLOCAL>());
			break;
	};

	{
		std::uniform_real_distribution<double> uniform_real_distribution(0., 1.);
		for (i = 0; i < nEntries; i++) {
			ofs << "  evenRNGCorrelation->Fill(" << uniform_real_distribution(*gr_ptr_one) << ", " <<
				 uniform_real_distribution(*gr_ptr_two) << ");" << std::endl;
			ofs << "  rngDiff->Fill(double(" << i << "), " <<
				 uniform_real_distribution(*gr_ptr_one) - uniform_real_distribution(*gr_ptr_two) << ");" <<
				 std::endl;
		}
	}

	// In this test, a number of GRandomT objects are instantiated and their
	// initial values (after a number of calls) are asked for. There should be no
	// correlation.
	{
		std::uniform_real_distribution<double> uniform_real_distribution(0., 1.);
		for (i = 1; i <= 10; i++) {
			std::shared_ptr<Gem::Hap::GRandomBase> gr_ptr_seed;
			switch (rnrProductionMode) {
				case 0:
					gr_ptr_seed = std::shared_ptr<GRandomT<RANDFLAVOURS::RANDOMPROXY>>(
						new GRandomT<RANDFLAVOURS::RANDOMPROXY>());
					break;

				case 1:
					gr_ptr_seed = std::shared_ptr<GRandomT<RANDFLAVOURS::RANDOMLOCAL>>(
						new GRandomT<RANDFLAVOURS::RANDOMLOCAL>());
					break;
			};

			initCorr.push_back(uniform_real_distribution(*gr_ptr_seed));
		}
	}

	// In this test, a number of lagged fibonacci generators are instantiated with
	// different, sequential seeds, and their initial values (after a number of calls)
	// are asked for. There should be no correlation.
	{
		for (i = 1; i <= 10; i++) {
			std::subtract_with_carry_engine<std::uint_fast64_t, 48, 5, 12> lf(
				boost::numeric_cast<std::subtract_with_carry_engine<std::uint_fast64_t, 48, 5, 12>::result_type>(i)
			);
			initLFCorr.push_back(boost::numeric_cast<double>(lf()));
		}
	}

	createRandomVector<double>(gaussian, distType::GAUSSIAN, nEntries, gr_ptr);
	createRandomVector<double>(doublegaussian, distType::DOUBLEGAUSSIAN, nEntries, gr_ptr);
	createRandomVector<double>(even, distType::EVEN, nEntries, gr_ptr);
	createRandomVector<double>(evenwithboundaries, distType::EVENWITHBOUNDARIES, nEntries, gr_ptr);
	createRandomVector<std::int32_t>(discrete, distType::DISCRETE, nEntries,gr_ptr);
	createRandomVector<std::int32_t>(discretebound, distType::DISCRETEBOUND, nEntries, gr_ptr);
	createRandomVector<std::int32_t>(bitprob, distType::BITPROB, nEntries, gr_ptr);
	createRandomVector<std::int32_t>(bitsimple, distType::BITSIMPLE, nEntries, gr_ptr);
	createRandomVector<double>(expgauss01, distType::EXPGAUSS01, nEntries, gr_ptr);
	createRandomVector<double>(expgauss02, distType::EXPGAUSS02, nEntries, gr_ptr);
	createRandomVector<double>(expgauss04, distType::EXPGAUSS04, nEntries, gr_ptr);
	createRandomVector<double>(expgauss08, distType::EXPGAUSS08, nEntries, gr_ptr);
	createRandomVector<double>(expgauss16, distType::EXPGAUSS16, nEntries, gr_ptr);

	if(gaussian.size() != nEntries ||
		doublegaussian.size() != nEntries ||
		even.size() != nEntries ||
		evenwithboundaries.size() != nEntries ||
		discrete.size() != nEntries ||
		discretebound.size() != nEntries ||
		bitprob.size() != nEntries ||
		bitsimple.size() != nEntries ||
		expgauss01.size() != nEntries ||
		expgauss02.size() != nEntries ||
		expgauss04.size() != nEntries ||
		expgauss08.size() != nEntries ||
		expgauss16.size() != nEntries){
		std::cout << "Error: received invalid sizes for at least one vector" << std::endl;
		return 1;
	}

	for(i=0; i<nEntries; i++){
		ofs << "  gauss->Fill(" << gaussian.at(i) << ");" << std::endl;
	}
	ofs << std::endl;

	for(i=0; i<nEntries; i++){
		ofs << "  dgauss->Fill(" << doublegaussian.at(i) << ");" << std::endl;
	}
	ofs << std::endl;

	for(i=0; i<nEntries; i++){
		ofs << "  expGauss01->Fill(" << expgauss01.at(i) << ");" << std::endl;
	}
	ofs << std::endl;

	for(i=0; i<nEntries; i++){
		ofs << "  expGauss02->Fill(" << expgauss02.at(i) << ");" << std::endl;
	}
	ofs << std::endl;

	for(i=0; i<nEntries; i++){
		ofs << "  expGauss04->Fill(" << expgauss04.at(i) << ");" << std::endl;
	}
	ofs << std::endl;

	for(i=0; i<nEntries; i++){
		ofs << "  expGauss08->Fill(" << expgauss08.at(i) << ");" << std::endl;
	}
	ofs << std::endl;

	for(i=0; i<nEntries; i++){
		ofs << "  expGauss16->Fill(" << expgauss16.at(i) << ");" << std::endl;
	}
	ofs << std::endl;

	for(i=0; i<nEntries; i++){
		ofs << "  even->Fill(" << even.at(i) << ");" << std::endl;
	}
	ofs << std::endl;

	for(i=0; i<nEntries; i++){
		ofs << "  evenwb->Fill(" << evenwithboundaries.at(i) << ");" << std::endl;
	}
	ofs << std::endl;

	for(i=0; i<nEntries; i++){
		ofs << "  discrete->Fill(" << discrete.at(i) << ");" << std::endl;
	}
	ofs << std::endl;

	for(i=0; i<nEntries; i++){
		ofs << "  discretewb->Fill(" << discretebound.at(i) << ");" << std::endl;
	}
	ofs << std::endl;

	for(i=0; i<nEntries; i++){
		ofs << "  bitprob->Fill(" << bitprob.at(i) << ");" << std::endl;
	}
	ofs << std::endl;

	for(i=0; i<nEntries; i++){
		ofs << "  bitsimple->Fill(" << bitsimple.at(i) << ");" << std::endl;
	}
	ofs << std::endl;

	for(i=1; i<=10; i++){
		ofs << "  initCorrelation->Fill(" << i << ", " << initCorr.at(i-1) << ");" << std::endl;
	}
	ofs << std::endl;

	for(i=1; i<=10; i++){
		ofs << "  initLFCorrelation->Fill(" << i << ", " << initLFCorr.at(i-1) << ");" << std::endl;
	}
	ofs << std::endl;

	ofs << "  cc->cd(1);" << std::endl
		 << "  gauss->Draw();" << std::endl
		 << "  cc->cd(2);" << std::endl
		 << "  dgauss->Draw();" << std::endl
		 << "  cc->cd(3);" << std::endl
		 << "  expGauss01->Draw();" << std::endl
		 << "  expGauss02->Draw(\"same\");" << std::endl
		 << "  expGauss04->Draw(\"same\");" << std::endl
		 << "  expGauss08->Draw(\"same\");" << std::endl
		 << "  expGauss16->Draw(\"same\");" << std::endl
		 << "  cc->cd(4);" << std::endl
		 << "  even->Draw();" << std::endl
		 << "  cc->cd(5);" << std::endl
		 << "  evenwb->Draw();" << std::endl
		 << "  cc->cd(6);" << std::endl
		 << "  discrete->Draw();" << std::endl
		 << "  cc->cd(7);" << std::endl
		 << "  discretewb->Draw();" << std::endl
		 << "  cc->cd(8);" << std::endl
		 << "  bitprob->Draw();" << std::endl
		 << "  cc->cd(9);" << std::endl
		 << "  bitsimple->Draw();" << std::endl
		 << "  cc->cd(11);" << std::endl
		 << "  evenSelfCorrelation->Draw(\"contour\");" << std::endl
		 << "  cc->cd(12);" << std::endl
		 << "  initCorrelation->Draw();" << std::endl
		 << "  cc->cd(13);" << std::endl
		 << "  initLFCorrelation->Draw();" << std::endl
		 << "  cc->cd(14);" << std::endl
		 << "  evenRNGCorrelation->Draw(\"contour\");" << std::endl
		 << "  cc->cd(15);" << std::endl
		 << "  rngDiff->Draw();" << std::endl
		 << "  cc->cd();" << std::endl;
	ofs << "}" << std::endl;

	ofs.close();


	// Write the result to disk
	gpd.writeToFile(resultFile);

	return 0;
}

/************************************************************************************************/

