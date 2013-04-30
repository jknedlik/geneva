/**
 * @file GBasePS.cpp
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

#include "geneva/GBasePS.hpp"

BOOST_CLASS_EXPORT_IMPLEMENT(Gem::Geneva::GBasePS::GPSOptimizationMonitor)

namespace Gem {
namespace Geneva {

/******************************************************************************/
/**
 * A simple output operator for parSet object, mostly meant for debugging
 */
std::ostream& operator<<(std::ostream& os, const parSet& pS) {
   os
   << "###########################################################" << std::endl
   << "# New parSet object:" << std::endl;

   // Boolean data
   if(!pS.bParVec.empty()) {
      os << "# Boolean data" << std::endl;
      std::vector<singleBPar>::const_iterator cit;
      for(cit=pS.bParVec.begin(); cit!=pS.bParVec.end(); ++cit) {
         os << (boost::get<1>(*cit)?"true":"false") << ":" << boost::get<0>(*cit);
         if(cit+1 != pS.bParVec.end()) {
            os << ", ";
         }
      }
      os << std::endl;
   }

   // boost::int32_t data
   if(!pS.iParVec.empty()) {
      os << "# boost::int32_t data" << std::endl;
      std::vector<singleInt32Par>::const_iterator cit;
      for(cit=pS.iParVec.begin(); cit!=pS.iParVec.end(); ++cit) {
         os << boost::get<1>(*cit) << ":" << boost::get<0>(*cit);
         if(cit+1 != pS.iParVec.end()) {
            os << ", ";
         }
      }
      os << std::endl;
   }

   // float data
   if(!pS.fParVec.empty()) {
      os << "# float data" << std::endl;
      std::vector<singleFPar>::const_iterator cit;
      for(cit=pS.fParVec.begin(); cit!=pS.fParVec.end(); ++cit) {
         os << boost::get<1>(*cit) << ":" << boost::get<0>(*cit);
         if(cit+1 != pS.fParVec.end()) {
            os << ", ";
         }
      }
      os << std::endl;
   }

   // double data
   if(!pS.dParVec.empty()) {
      os << "# double data" << std::endl;
      std::vector<singleDPar>::const_iterator cit;
      for(cit=pS.dParVec.begin(); cit!=pS.dParVec.end(); ++cit) {
         os << boost::get<1>(*cit) << ":" << boost::get<0>(*cit);
         if(cit+1 != pS.dParVec.end()) {
            os << ", ";
         }
      }
      os << std::endl;
   }

   return os;
}

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/**
 * The default constructor
 */
GBasePS::GBasePS()
   : GOptimizationAlgorithmT<GParameterSet>()
   , cycleLogicHalt_(false)
   , scanRandomly_(true)
   , nMonitorInds_(DEFAULTNMONITORINDS)
{
   // Register the default optimization monitor
   this->registerOptimizationMonitor(
         boost::shared_ptr<GOptimizationAlgorithmT<GParameterSet>::GOptimizationMonitorT>(
               new GPSOptimizationMonitor()
         )
   );
}

/******************************************************************************/
/**
 * A standard copy constructor.
 *
 * @param cp A copy of another GradientDescent object
 */
GBasePS::GBasePS(const GBasePS& cp)
   : GOptimizationAlgorithmT<GParameterSet>(cp)
   , cycleLogicHalt_(cp.cycleLogicHalt_)
   , scanRandomly_(cp.scanRandomly_)
   , nMonitorInds_(cp.nMonitorInds_)
{
   // Copying / setting of the optimization algorithm id is done by the parent class. The same
   // applies to the copying of the optimization monitor.

   // Load the parameter objects
   std::vector<boost::shared_ptr<bScanPar> >::const_iterator b_it;
   for(b_it=cp.bVec_.begin(); b_it!=cp.bVec_.end(); ++b_it) {
      bVec_.push_back((*b_it)->clone());
   }

   std::vector<boost::shared_ptr<int32ScanPar> >::const_iterator i_it;
   for(i_it=cp.int32Vec_.begin(); i_it!=cp.int32Vec_.end(); ++i_it) {
      int32Vec_.push_back((*i_it)->clone());
   }

   std::vector<boost::shared_ptr<dScanPar> >::const_iterator d_it;
   for(d_it=cp.dVec_.begin(); d_it!=cp.dVec_.end(); ++d_it) {
      dVec_.push_back((*d_it)->clone());
   }

   std::vector<boost::shared_ptr<fScanPar> >::const_iterator f_it;
   for(f_it=cp.fVec_.begin(); f_it!=cp.fVec_.end(); ++f_it) {
      fVec_.push_back((*f_it)->clone());
   }
}

/******************************************************************************/
/**
 * The destructor
 */
GBasePS::~GBasePS()
{ /* nothing */ }

/******************************************************************************/
/**
 * Returns information about the type of optimization algorithm. This function needs
 * to be overloaded by the actual algorithms to return the correct type.
 *
 * @return The type of optimization algorithm
 */
personality_oa GBasePS::getOptimizationAlgorithm() const {
   return PERSONALITY_PS;
}

/******************************************************************************/
/**
 * Retrieve the number of processable items in the current iteration.
 *
 * @return The number of processable items in the current iteration
 */
std::size_t GBasePS::getNProcessableItems() const {
   return this->size(); // Evaluation always needs to be done for the entire population
}

/******************************************************************************/
/**
 * Returns the name of this optimization algorithm
 *
 * @return The name assigned to this optimization algorithm
 */
std::string GBasePS::getAlgorithmName() const {
   return std::string("Parameter Scan");
}

/******************************************************************************/
/**
 * A standard assignment operator
 *
 * @param cp A copy of another GRadientDescent object
 */
const GBasePS& GBasePS::operator=(const GBasePS& cp) {
   GBasePS::load_(&cp);
   return *this;
}

/******************************************************************************/
/**
 * Checks for equality with another GBasePS object
 *
 * @param cp A copy of another GRadientDescent object
 */
bool GBasePS::operator==(const GBasePS& cp) const {
   using namespace Gem::Common;
   // Means: The expectation of equality was fulfilled, if no error text was emitted (which converts to "true")
   return !checkRelationshipWith(cp, CE_EQUALITY, 0.,"GBasePS::operator==","cp", CE_SILENT);
}

/******************************************************************************/
/**
 * Checks for inequality with another GBasePS object
 *
 * @param cp A copy of another GRadientDescent object
 */
bool GBasePS::operator!=(const GBasePS& cp) const {
   using namespace Gem::Common;
   // Means: The expectation of inequality was fulfilled, if no error text was emitted (which converts to "true")
   return !checkRelationshipWith(cp, CE_INEQUALITY, 0.,"GBasePS::operator!=","cp", CE_SILENT);
}

/******************************************************************************/
/**
 * Checks whether this object fulfills a given expectation in relation to another object
 *
 * @param cp A constant reference to another object, camouflaged as a GObject
 * @param e The expected outcome of the comparison
 * @param limit The maximum deviation for floating point values (important for similarity checks)
 * @param caller An identifier for the calling entity
 * @param y_name An identifier for the object that should be compared to this one
 * @param withMessages Whether or not information should be emitted in case of deviations from the expected outcome
 * @return A boost::optional<std::string> object that holds a descriptive string if expectations were not met
 */
boost::optional<std::string> GBasePS::checkRelationshipWith(
      const GObject& cp
      , const Gem::Common::expectation& e
      , const double& limit
      , const std::string& caller
      , const std::string& y_name
      , const bool& withMessages
) const {
    using namespace Gem::Common;

   // Check that we are indeed dealing with a GParamterBase reference
   const GBasePS *p_load = GObject::gobject_conversion<GBasePS>(&cp);

   // Will hold possible deviations from the expectation, including explanations
    std::vector<boost::optional<std::string> > deviations;

   // Check our parent class'es data ...
    deviations.push_back(GOptimizationAlgorithmT<GParameterSet>::checkRelationshipWith(cp, e, limit, "GOptimizationAlgorithmT<GParameterSet>", y_name, withMessages));

   // ... and then our local data
   deviations.push_back(checkExpectation(withMessages, "GBasePS", cycleLogicHalt_, p_load->cycleLogicHalt_, "cycleLogicHalt_", "p_load->cycleLogicHalt_", e , limit));
   deviations.push_back(checkExpectation(withMessages, "GBasePS", scanRandomly_, p_load->scanRandomly_, "scanRandomly_", "p_load->scanRandomly_", e , limit));
   deviations.push_back(checkExpectation(withMessages, "GBasePS", nMonitorInds_, p_load->nMonitorInds_, "nMonitorInds_", "p_load->nMonitorInds_", e , limit));


   // We do not check our temporary parameter objects yet (bVec_ etc.)

   return evaluateDiscrepancies("GBasePS", caller, deviations, e);
}

/***********************************************************************************/
/**
 * Emits a name for this class / object
 */
std::string GBasePS::name() const {
   return std::string("GBasePS");
}

/******************************************************************************/
/**
 * Allows to set the number of "best" individuals to be monitored
 * over the course of the algorithm run
 */
void GBasePS::setNMonitorInds(std::size_t nMonitorInds) {
   nMonitorInds_ = nMonitorInds;
}

/******************************************************************************/
/**
 * Allows to retrieve  the number of "best" individuals to be monitored
 * over the course of the algorithm run
 */
std::size_t GBasePS::getNMonitorInds() const {
   return nMonitorInds_;
}

/******************************************************************************/
/**
 * Loads the data of another population
 *
 * @param cp A pointer to another GBasePS object, camouflaged as a GObject
 */
void GBasePS::load_(const GObject *cp) {
   const GBasePS *p_load = this->gobject_conversion<GBasePS>(cp);

   // First load the parent class'es data.
   // This will also take care of copying all individuals.
   GOptimizationAlgorithmT<GParameterSet>::load_(cp);

   // ... and then our own data
   cycleLogicHalt_ = p_load->cycleLogicHalt_;
   scanRandomly_   = p_load->scanRandomly_;
   nMonitorInds_   = p_load->nMonitorInds_;

   // Load the parameter objects
   bVec_.clear();
   std::vector<boost::shared_ptr<bScanPar> >::const_iterator b_it;
   for(b_it=(p_load->bVec_).begin(); b_it!=(p_load->bVec_).end(); ++b_it) {
      bVec_.push_back((*b_it)->clone());
   }

   int32Vec_.clear();
   std::vector<boost::shared_ptr<int32ScanPar> >::const_iterator i_it;
   for(i_it=(p_load->int32Vec_).begin(); i_it!=(p_load->int32Vec_).end(); ++i_it) {
      int32Vec_.push_back((*i_it)->clone());
   }

   dVec_.clear();
   std::vector<boost::shared_ptr<dScanPar> >::const_iterator d_it;
   for(d_it=(p_load->dVec_).begin(); d_it!=(p_load->dVec_).end(); ++d_it) {
      dVec_.push_back((*d_it)->clone());
   }

   fVec_.clear();
   std::vector<boost::shared_ptr<fScanPar> >::const_iterator f_it;
   for(f_it=(p_load->fVec_).begin(); f_it!=(p_load->fVec_).end(); ++f_it) {
      fVec_.push_back((*f_it)->clone());
   }
}

/******************************************************************************/
/**
 * The actual business logic to be performed during each iteration. Returns the best achieved fitness
 *
 * @return The value of the best individual found
 */
double GBasePS::cycleLogic() {
   // Apply all necessary modifications to individuals
   updateIndividuals();

   // Trigger value calculation for all individuals
   // This function is purely virtual and needs to be
   // re-implemented in derived classes
   double result=doFitnessCalculation(this->size());

   // This function will sort the population according to
   // its primary fitness value
   sortPopulation();

   // Updates the vector of best individuals found so far
   updateBests();

   // Let the audience know
   return result;
}

/******************************************************************************/
/**
 * Adds new values to the population's individuals. Note that this function
 * may resize the population and set the default population size, if there
 * is no sufficient number of data sets to be evaluated left.
 */
void GBasePS::updateIndividuals() {
   std::size_t indPos = 0;
   std::vector<bool> bData;
   std::vector<boost::int32_t> iData;
   std::vector<float> fData;
   std::vector<double> dData;

   while(true) {
      //------------------------------------------------------------------------
      // Retrieve a work item
      boost::shared_ptr<parSet> pS = getParameterSet();

      //------------------------------------------------------------------------
      // Fill the parameter set data into the current individual

      // Retrieve the parameter vectors
      this->at(indPos)->streamline<bool>(bData);
      this->at(indPos)->streamline<boost::int32_t>(iData);
      this->at(indPos)->streamline<float>(fData);
      this->at(indPos)->streamline<double>(dData);

      // Add the data items from the parSet object to the vectors

      // 1) For boolean data
      std::vector<singleBPar>::iterator b_it;
      for(b_it=pS->bParVec.begin(); b_it!=pS->bParVec.end(); ++b_it) {
         this->addDataPoint<bool>(*b_it, bData);
      }

      // 2) For boost::int32_t data
      std::vector<singleInt32Par>::iterator i_it;
      for(i_it=pS->iParVec.begin(); i_it!=pS->iParVec.end(); ++i_it) {
         this->addDataPoint<boost::int32_t>(*i_it, iData);
      }

      // 3) For float values
      std::vector<singleFPar>::iterator f_it;
      for(f_it=pS->fParVec.begin(); f_it!=pS->fParVec.end(); ++f_it) {
         this->addDataPoint<float>(*f_it, fData);
      }

      // 4) For double values
      std::vector<singleDPar>::iterator d_it;
      for(d_it=pS->dParVec.begin(); d_it!=pS->dParVec.end(); ++d_it) {
         this->addDataPoint<double>(*d_it, dData);
      }

      // Copy the data back into the individual
      this->at(indPos)->assignValueVector<bool>(bData);
      this->at(indPos)->assignValueVector<boost::int32_t>(iData);
      this->at(indPos)->assignValueVector<float>(fData);
      this->at(indPos)->assignValueVector<double>(dData);

      // Mark the individual as "dirty", so it gets re-evaluated the
      // next time the fitness() function is called
      this->at(indPos)->setDirtyFlag();

      // We were successful
      cycleLogicHalt_ = false;

      //------------------------------------------------------------------------
      // Make sure we continue with the next parameter set in the next iteration
      if(!this->switchToNextParameterSet()) {
         // Let the audience know that the optimization may be stopped
         this->cycleLogicHalt_ = true;

         // Reset all parameter objects for the next run (if desired)
         this->resetParameterObjects();

         // Resize the population, so we only have modified individuals
         this->resize(indPos+1);

         // Terminate the loop
         break;
      }

      //------------------------------------------------------------------------
      // We do not want to exceed the boundaries of the population
      if(++indPos >= this->getDefaultPopulationSize()) break;
   }
}

/******************************************************************************/
/**
 * Updates the best individuals found
 */
void GBasePS::updateBests() {
   // Some error checks
#ifdef DEBUG
   if(this->empty()) {
      glogger
      << "In GBasePS::updateBests(): Error!" << std::endl
      << "Algorithm does not contain any individuals." << std::endl
      << GEXCEPTION;
   }
#endif /* DEBUG */

   if(this->getIteration() > this->getStartIteration()) {
#ifdef DEBUG
      if(this->getIteration() > this->getStartIteration() && bestIndividuals_.empty()) {
         glogger
         << "In GBasePS::updateBests(): Error!" << std::endl
         << "Vector of best individuals is empty when it shouldn't be."
         << GEXCEPTION;
      }
#endif /* DEBUG */

      // Note: The best individuals are found in the beginning of this population
      for(std::size_t ind=0; ind<std::min(nMonitorInds_,this->size()); ind++) {
         // Compare the fitness of the best individual with the last individual
         // in the bestIndividuals_ vector. If it is better, replace it.
         if(isBetter(this->at(ind)->fitness(0),bestIndividuals_.back()->fitness(0))) {
            // Copy a clone over
            bestIndividuals_.back() = this->at(ind)->clone<GParameterSet>();

            // Sort the "bests" vector for the next iteration
            if(true == this->getMaxMode()) { // Maximization
               std::sort(bestIndividuals_.begin(), bestIndividuals_.end(), boost::bind(&GParameterSet::fitness, _1, 0) > boost::bind(&GParameterSet::fitness, _2, 0));
            } else { // Minimization
               std::sort(bestIndividuals_.begin(), bestIndividuals_.end(), boost::bind(&GParameterSet::fitness, _1, 0) < boost::bind(&GParameterSet::fitness, _2, 0));
            }
         }
      }
   } else {  // No update has taken place yet
      // We clone the nMonitorInds_ best individuals and store them in the bestIndividuals_ vector
      for(std::size_t ind=0; ind<std::min(nMonitorInds_,this->size()); ind++) {
         bestIndividuals_.push_back(this->at(ind)->clone<GParameterSet>());
      }
   }
}

/******************************************************************************/
/**
 * Resets all parameter objects
 */
void GBasePS::resetParameterObjects() {
   std::vector<boost::shared_ptr<bScanPar> >::iterator b_it;
   for(b_it=bVec_.begin(); b_it!=bVec_.end(); ++b_it) {
      (*b_it)->resetPosition();
   }

   std::vector<boost::shared_ptr<int32ScanPar> >::iterator i_it;
   for(i_it=int32Vec_.begin(); i_it!=int32Vec_.end(); ++i_it) {
      (*i_it)->resetPosition();
   }

   std::vector<boost::shared_ptr<fScanPar> >::iterator f_it;
   for(f_it=fVec_.begin(); f_it!=fVec_.end(); ++f_it) {
      (*f_it)->resetPosition();
   }

   std::vector<boost::shared_ptr<dScanPar> >::iterator d_it;
   for(d_it=dVec_.begin(); d_it!=dVec_.end(); ++d_it) {
      (*d_it)->resetPosition();
   }
}

/******************************************************************************/
/**
 * Retrieves a parameter set by filling the current parameter combinations
 * into a parSet object.
 */
boost::shared_ptr<parSet> GBasePS::getParameterSet() {
   // Create a new parSet object
   boost::shared_ptr<parSet> result(new parSet());

   // Extract the relevant data and store it in a parSet object
   // 1) For boolean objects
   std::vector<boost::shared_ptr<bScanPar> >::iterator b_it;
   for(b_it=bVec_.begin(); b_it!=bVec_.end(); ++b_it) {
      boost::tuple<bool, std::size_t> item((*b_it)->getCurrentItem(), (*b_it)->getPos());
      (result->bParVec).push_back(item);
   }
   // 2) For boost::int32_t objects
   std::vector<boost::shared_ptr<int32ScanPar> >::iterator i_it;
   for(i_it=int32Vec_.begin(); i_it!=int32Vec_.end(); ++i_it) {
      boost::tuple<boost::int32_t, std::size_t> item((*i_it)->getCurrentItem(), (*i_it)->getPos());
      (result->iParVec).push_back(item);
   }
   // 3) For float objects
   std::vector<boost::shared_ptr<fScanPar> >::iterator f_it;
   for(f_it=fVec_.begin(); f_it!=fVec_.end(); ++f_it) {
      boost::tuple<float, std::size_t> item((*f_it)->getCurrentItem(), (*f_it)->getPos());
      (result->fParVec).push_back(item);
   }
   // 4) For double objects
   std::vector<boost::shared_ptr<dScanPar> >::iterator d_it;
   for(d_it=dVec_.begin(); d_it!=dVec_.end(); ++d_it) {
      boost::tuple<double, std::size_t> item((*d_it)->getCurrentItem(), (*d_it)->getPos());
      (result->dParVec).push_back(item);
   }

   return result;
}

/******************************************************************************/
/**
 * Switches to the next parameter set
 *
 * @return A boolean indicating whether there indeed is a following
 * parameter set (true) or whether we have reached the end of the
 * collection (false)
 */
bool GBasePS::switchToNextParameterSet() {
   std::vector<boost::shared_ptr<scanParI> >::iterator it = allParVec_.begin();

   // Switch to the next parameter set
   while(true) {
      if((*it)->goToNextItem()) { // Will trigger if a warp has occurred
         if(it+1 == allParVec_.end()) return false; // All possible combinations were found
         else ++it; // Try the next parameter object
      } else {
         return true; // We have successfully switched to the next parameter set
      }
   }

   // Make the compiler happy
   return false;
}

/******************************************************************************/
/**
 * This function will sort the population according to its primary fitness value.
 */
void GBasePS::sortPopulation() {
   if(true == this->getMaxMode()) { // Maximization
      std::sort((this->data).begin(), (this->data).end(), boost::bind(&GParameterSet::fitness, _1, 0) > boost::bind(&GParameterSet::fitness, _2, 0));
   } else { // Minimization
      std::sort((this->data).begin(), (this->data).end(), boost::bind(&GParameterSet::fitness, _1, 0) < boost::bind(&GParameterSet::fitness, _2, 0));
   }
}

/******************************************************************************/
/** @brief Fills all parameter objects into the allParVec_ vector */
void GBasePS::fillAllParVec() {
   // 1) For boolean objects
   std::vector<boost::shared_ptr<bScanPar> >::iterator b_it;
   for(b_it=bVec_.begin(); b_it!=bVec_.end(); ++b_it) {
      allParVec_.push_back(*b_it);
   }
   // 2) For boost::int32_t objects
   std::vector<boost::shared_ptr<int32ScanPar> >::iterator i_it;
   for(i_it=int32Vec_.begin(); i_it!=int32Vec_.end(); ++i_it) {
      allParVec_.push_back(*i_it);
   }
   // 3) For float objects
   std::vector<boost::shared_ptr<fScanPar> >::iterator f_it;
   for(f_it=fVec_.begin(); f_it!=fVec_.end(); ++f_it) {
      allParVec_.push_back(*f_it);
   }
   // 4) For double objects
   std::vector<boost::shared_ptr<dScanPar> >::iterator d_it;
   for(d_it=dVec_.begin(); d_it!=dVec_.end(); ++d_it) {
      allParVec_.push_back(*d_it);
   }
}

/******************************************************************************/
/**
 * Clears the allParVec_ vector
 */
void GBasePS::clearAllParVec() {
   allParVec_.clear();
}

/******************************************************************************/
/**
 * Retrieves the best individual of the population and returns it in Gem::Geneva::GIndividual format.
 * Note that this protected function will return the item itself. Direct usage of this function should
 * be avoided even by derived classes. We suggest to use the function
 * GOptimizableI::getBestIndividual<individual_type>() instead, which internally uses
 * this function and returns copies of the best individual, converted to the desired target type.
 *
 * @return A shared_ptr to the best individual of the population
 */
boost::shared_ptr<GIndividual> GBasePS::getBestIndividual(){
#ifdef DEBUG
   if(bestIndividuals_.empty()) {
      glogger
      << "In GBasePS::getBestIndividual() :" << std::endl
      << "Tried to access individual at position 0 of bestIndividuals_" << std::endl
      << "even though population is empty." << std::endl
      << GEXCEPTION;
   }
#endif /* DEBUG */

   return bestIndividuals_.at(0);
}

/******************************************************************************/
/**
 * Retrieves a list of the best individuals found. For now we only return a single
 * (the best individual).
 *
 * @return A list of the best individuals found
 */
std::vector<boost::shared_ptr<GIndividual> > GBasePS::getBestIndividuals() {
   std::vector<boost::shared_ptr<GIndividual> > result;
   for(std::size_t ind=0; ind<bestIndividuals_.size(); ind++) {
      result.push_back(bestIndividuals_.at(ind));
   }
   return result;
}

/******************************************************************************/
/**
 * A custom halt criterion for the optimization, allowing to stop the loop
 * when no items are left to be scanned
 */
bool GBasePS::customHalt() const {
   if(this->cycleLogicHalt_) {
      std::cerr
      << "Terminating the loop as no items are left to be" << std::endl
      << "processed in parameter scan." << std::endl;
      return true;
   } else {
      return false;
   }
}

/******************************************************************************/
/**
 * Adds local configuration options to a GParserBuilder object
 *
 * @param gpb The GParserBuilder object to which configuration options should be added
 * @param showOrigin Makes the function indicate the origin of parameters in comments
 */
void GBasePS::addConfigurationOptions (
   Gem::Common::GParserBuilder& gpb
   , const bool& showOrigin
) {
   std::string comment;

   // Call our parent class'es function
   GOptimizationAlgorithmT<GParameterSet>::addConfigurationOptions(gpb, showOrigin);

   comment = ""; // Reset the first comment string
   comment += "The total size of the population;";
   if(showOrigin) comment += " [GBasePS]";
   gpb.registerFileParameter<std::size_t>(
      "size" // The name of the first variable
      , DEFAULTPOPULATIONSIZE
      , boost::bind(
         &GBasePS::setDefaultPopulationSize
         , this
         , _1
        )
      , Gem::Common::VAR_IS_ESSENTIAL // Alternative: VAR_IS_SECONDARY
      , comment
   );

   comment = ""; // Reset the comment string
   comment += "Indicates whether scans should be done randomly;";
   comment += "(1) or on a grid (0);";
   if(showOrigin) comment += "[GBasePS]";
   gpb.registerFileParameter<bool>(
      "scanRandomly_" // The name of the variable
      , scanRandomly_
      , true // The default value
      , Gem::Common::VAR_IS_ESSENTIAL // Alternative: VAR_IS_SECONDARY
      , comment
   );

   std::vector<std::string> defaultStrParVec; // The default values
   defaultStrParVec.push_back("d 0 0.3 0.7 10"); // double value in position 0, scan from 0.3-0.7 (inclusive) in 10 steps
   defaultStrParVec.push_back("f 1 0.3 0.7 10"); // float value in position 1, scan from 0.3-0.7 (inclusive) in 10 steps
   defaultStrParVec.push_back("i 2 5 8 10");     // 32 bit integer in position 2, scan from 5-8 (inclusive). 10 steps are specified. Higher number of steps than possible will be ignored, except for random mode.
   defaultStrParVec.push_back("b 3 0 1 10");     // Boolean in position 3; false and true will be tested. Note that the "boundaries" still need to be specified. 10 steps are specified. A number of steps other than 2 will be ignored, except for random mode.

   comment = ""; // Reset the comment string
   comment += "Specification of the parameters to be used;";
   comment += "in the parameter scan.;";
   gpb.registerFileParameter<std::string>(
      "parameterOptions"
      , defaultStrParVec
      , boost::bind(
            &GBasePS::parseParameterValues // The call-back function
            , this
            , _1
        )
      , Gem::Common::VAR_IS_ESSENTIAL // Could also be VAR_IS_SECONDARY
      , comment
   );
}

/******************************************************************************/
/**
 * Fills vectors with parameter values
 */
void GBasePS::parseParameterValues(std::vector<std::string> parStrVec) {
   std::vector<std::string>::iterator it;
   for(it=parStrVec.begin(); it!=parStrVec.end(); ++it) {
      // Remove white-space characters at the edges of the string
      boost::trim(*it);

      // Check that the parameter string isn't empty
      if(it->empty()) {
         glogger
         << "In GBasePS::parseParameterValues(): Error!" << std::endl
         << "Parameter string in position " << it-parStrVec.begin() << " seems to be empty" << std::endl
         << GEXCEPTION;
      }

      // Retrieve the first character
      char first = it->at(0);

      // Act on the character -- This will result in a collection of parameter objects,
      // which can be used to extract allowed parameter values
      if(first == 'd') {
         dVec_.push_back(boost::shared_ptr<dScanPar>(new dScanPar(*it, scanRandomly_)));
      } else if(first == 'f') {
         fVec_.push_back(boost::shared_ptr<fScanPar>(new fScanPar(*it, scanRandomly_)));
      } else if(first == 'i') {
         int32Vec_.push_back(boost::shared_ptr<int32ScanPar>(new int32ScanPar(*it, scanRandomly_)));
      } else if(first == 'b') {
         bVec_.push_back(boost::shared_ptr<bScanPar>(new bScanPar(*it, scanRandomly_)));
      } else { // Raise an exception
         glogger
         << "In GBasePS::parseParameterValues(): Error!" << std::endl
         << "Parameter string in position " << it-parStrVec.begin() << " has invalid type: \"" << first << "\"" << std::endl
         << GEXCEPTION;
      }
   }
}

/******************************************************************************/
/**
 * Does some preparatory work before the optimization starts
 */
void GBasePS::init() {
   // To be performed before any other action
   GOptimizationAlgorithmT<GParameterSet>::init();

   // Reset the custom halt criterion
   cycleLogicHalt_ = false;

   // Remove all previous "best individuals"
   bestIndividuals_.clear();

   // Make sure we start with a fresh central vector of parameter objects
   this->clearAllParVec();

   // Copy all parameter objects to the central vector for easier handling
   this->fillAllParVec();
}

/******************************************************************************/
/**
 * Does any necessary finalization work
 */
void GBasePS::finalize() {
   // Last action
   GOptimizationAlgorithmT<GParameterSet>::finalize();
}

/******************************************************************************/
/**
 * Retrieve a GPersonalityTraits object belonging to this algorithm
 */
boost::shared_ptr<GPersonalityTraits> GBasePS::getPersonalityTraits() const {
   return boost::shared_ptr<GPSPersonalityTraits>(new GPSPersonalityTraits());
}

/******************************************************************************/
/**
 * Resizes the population to the desired level and does some error checks.
 */
void GBasePS::adjustPopulation() {
   // Check how many individuals we already have
   std::size_t nStart = this->size();

   // Do some error checking ...

   // An empty population is an error
   if(nStart == 0) {
      glogger
      << "In GBasePS::adjustPopulation(): Error!" << std::endl
      << "You didn't add any individuals to the collection. We need at least one." << std::endl
      << GEXCEPTION;
   }

   // We want exactly one individual in the beginning. All other registered
   // individuals will be discarded.
   if(nStart > 1) {
      this->resize(1);
      nStart = 1;
   }

   // Check that we have a valid default population size
   if(0 == this->getDefaultPopulationSize()) {
      glogger
      << "In GBasePS::adjustPopulation(): Error!" << std::endl
      << "Default-size of the population is 0" << std::endl
      << GEXCEPTION;
   }

   // Create the desired number of (identical) individuals in the population.
   for(std::size_t ind=1; ind<this->getDefaultPopulationSize(); ind++) {
      this->push_back(this->at(0)->GObject::clone<GParameterSet>());
   }
}

/******************************************************************************/
/**
 * Loads a checkpoint from disk
 *
 * @param cpFile The name of the file the checkpoint should be loaded from
 */
void GBasePS::loadCheckpoint(const std::string& cpFile) {
   this->fromFile(cpFile, getCheckpointSerializationMode());
}

/******************************************************************************/
/**
 * Saves the state of the object to disc. We simply serialize the entire object.
 */
void GBasePS::saveCheckpoint() const {
   bool isDirty;
   double newValue = this->at(0)->GIndividual::getCachedFitness(isDirty);

#ifdef DEBUG
   if(isDirty) {
      glogger
      << "In GBasePS::saveCheckpoint():" << std::endl
      << "Best individual has dirty flag set!" << std::endl
      << GEXCEPTION;
   }
#endif

   // Determine a suitable name for the output file
   std::string outputFile = getCheckpointDirectory() + boost::lexical_cast<std::string>(this->getIteration()) + "_"
      + boost::lexical_cast<std::string>(newValue) + "_" + getCheckpointBaseName();

   this->toFile(outputFile, getCheckpointSerializationMode());
}

/******************************************************************************/
/**
 * Applies modifications to this object. This is needed for testing purposes
 */
bool GBasePS::modify_GUnitTests() {
#ifdef GEM_TESTING
   bool result = false;

   // Call the parent class'es function
   if(GOptimizationAlgorithmT<GParameterSet>::modify_GUnitTests()) result = true;

   return result;
#else /* GEM_TESTING */
   condnotset("GBasePS::modify_GUnitTests", "GEM_TESTING");
   return false;
#endif /* GEM_TESTING */
}

/******************************************************************************/
/**
 * Performs self tests that are expected to succeed. This is needed for testing purposes
 */
void GBasePS::specificTestsNoFailureExpected_GUnitTests() {
#ifdef GEM_TESTING
   // Call the parent class'es function
   GOptimizationAlgorithmT<GParameterSet>::specificTestsNoFailureExpected_GUnitTests();
#else /* GEM_TESTING */ // If this function is called when GEM_TESTING isn't set, throw
   condnotset("GBasePS::specificTestsNoFailureExpected_GUnitTests", "GEM_TESTING");
#endif /* GEM_TESTING */
}

/******************************************************************************/
/**
 * Performs self tests that are expected to fail. This is needed for testing purposes
 */
void GBasePS::specificTestsFailuresExpected_GUnitTests() {
#ifdef GEM_TESTING
   // Call the parent class'es function
   GOptimizationAlgorithmT<GParameterSet>::specificTestsFailuresExpected_GUnitTests();
#else /* GEM_TESTING */ // If this function is called when GEM_TESTING isn't set, throw
   condnotset("GBasePS::specificTestsNoFailureExpected_GUnitTests", "GEM_TESTING");
#endif /* GEM_TESTING */
}

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/**
 * The default constructor
 */
GBasePS::GPSOptimizationMonitor::GPSOptimizationMonitor()
   : csvResultFile_(DEFAULTCSVRESULTFILEOM)
{ /* nothing */ }

/******************************************************************************/
/**
 * The copy constructor
 *
 * @param cp A copy of another GPSOptimizationMonitor object
 */
GBasePS::GPSOptimizationMonitor::GPSOptimizationMonitor(const GBasePS::GPSOptimizationMonitor& cp)
   : GOptimizationAlgorithmT<GParameterSet>::GOptimizationMonitorT(cp)
   , csvResultFile_(cp.csvResultFile_)
{ /* nothing */ }

/******************************************************************************/
/**
 * The destructor
 */
GBasePS::GPSOptimizationMonitor::~GPSOptimizationMonitor()
{ /* nothing */ }

/******************************************************************************/
/**
 * A standard assignment operator.
 *
 * @param cp A copy of another GPSOptimizationMonitor object
 * @return A constant reference to this object
 */
const GBasePS::GPSOptimizationMonitor& GBasePS::GPSOptimizationMonitor::operator=(const GBasePS::GPSOptimizationMonitor& cp){
   GBasePS::GPSOptimizationMonitor::load_(&cp);
   return *this;
}

/******************************************************************************/
/**
 * Checks for equality with another GParameter Base object
 *
 * @param  cp A constant reference to another GPSOptimizationMonitor object
 * @return A boolean indicating whether both objects are equal
 */
bool GBasePS::GPSOptimizationMonitor::operator==(const GBasePS::GPSOptimizationMonitor& cp) const {
   using namespace Gem::Common;
   // Means: The expectation of equality was fulfilled, if no error text was emitted (which converts to "true")
   return !checkRelationshipWith(cp, CE_EQUALITY, 0.,"GBasePS::GPSOptimizationMonitor::operator==","cp", CE_SILENT);
}

/******************************************************************************/
/**
 * Checks for inequality with another GPSOptimizationMonitor object
 *
 * @param  cp A constant reference to another GPSOptimizationMonitor object
 * @return A boolean indicating whether both objects are inequal
 */
bool GBasePS::GPSOptimizationMonitor::operator!=(const GBasePS::GPSOptimizationMonitor& cp) const {
   using namespace Gem::Common;
   // Means: The expectation of inequality was fulfilled, if no error text was emitted (which converts to "true")
   return !checkRelationshipWith(cp, CE_INEQUALITY, 0.,"GBasePS::GPSOptimizationMonitor::operator!=","cp", CE_SILENT);
}

/******************************************************************************/
/**
 * Checks whether a given expectation for the relationship between this object and another object
 * is fulfilled.
 *
 * @param cp A constant reference to another object, camouflaged as a GObject
 * @param e The expected outcome of the comparison
 * @param limit The maximum deviation for floating point values (important for similarity checks)
 * @param caller An identifier for the calling entity
 * @param y_name An identifier for the object that should be compared to this one
 * @param withMessages Whether or not information should be emitted in case of deviations from the expected outcome
 * @return A boost::optional<std::string> object that holds a descriptive string if expectations were not met
 */
boost::optional<std::string> GBasePS::GPSOptimizationMonitor::checkRelationshipWith(
      const GObject& cp
      , const Gem::Common::expectation& e
      , const double& limit
      , const std::string& caller
      , const std::string& y_name
      , const bool& withMessages
) const {
   using namespace Gem::Common;

   // Check that we are indeed dealing with a GParamterBase reference
   const GBasePS::GPSOptimizationMonitor *p_load = GObject::gobject_conversion<GBasePS::GPSOptimizationMonitor >(&cp);

   // Will hold possible deviations from the expectation, including explanations
   std::vector<boost::optional<std::string> > deviations;

   // Check our parent class'es data ...
   deviations.push_back(GOptimizationAlgorithmT<GParameterSet>::GOptimizationMonitorT::checkRelationshipWith(cp, e, limit, "GBasePS::GPSOptimizationMonitor", y_name, withMessages));

   // ... and then our local data
   deviations.push_back(checkExpectation(withMessages, "GBasePS::GPSOptimizationMonitor", csvResultFile_, p_load->csvResultFile_, "csvResultFile_", "p_load->csvResultFile_", e , limit));

   return evaluateDiscrepancies("GBasePS::GPSOptimizationMonitor", caller, deviations, e);
}

/******************************************************************************/
/**
 * Allows to specify a different name for the result file
 *
 * @param resultFile The desired name of the result file
 */
void GBasePS::GPSOptimizationMonitor::setCSVResultFileName(
      const std::string& csvResultFile
) {
   csvResultFile_ = csvResultFile;
}

/******************************************************************************/
/**
 * Allows to retrieve the current value of the result file name
 *
 * @return The current name of the result file
 */
std::string GBasePS::GPSOptimizationMonitor::getCSVResultFileName() const {
  return csvResultFile_;
}

/******************************************************************************/
/**
 * A function that is called once before the optimization starts
 *
 * @param goa A pointer to the current optimization algorithm for which information should be emitted
 */
void GBasePS::GPSOptimizationMonitor::firstInformation(GOptimizationAlgorithmT<GParameterSet> * const goa) {
#ifdef DEBUG
   if(goa->getOptimizationAlgorithm() != PERSONALITY_PS) {
      glogger
      << "In GBasePS::GPSOptimizationMonitor::cycleInformation():" << std::endl
      << "Provided optimization algorithm has wrong type: " << goa->getOptimizationAlgorithm() << std::endl
      << GEXCEPTION;
   }
#endif /* DEBUG */

   std::ofstream result(csvResultFile_.c_str());
   result
   << "# Parameter scan results" << std::endl;
   result.close();
}

/******************************************************************************/
/**
 * A function that is called during each optimization cycle. It is possible to
 * extract quite comprehensive information in each iteration. For examples, see
 * the standard overloads provided for the various optimization algorithms.
 *
 * @param goa A pointer to the current optimization algorithm for which information should be emitted
 */
void GBasePS::GPSOptimizationMonitor::cycleInformation(GOptimizationAlgorithmT<GParameterSet> * const goa) {
   // Perform the conversion to the target algorithm
   GBasePS * const gd = static_cast<GBasePS * const>(goa);

   // Open the result file in append mode
   std::ofstream result(csvResultFile_.c_str(), std::ofstream::app);

   result
   << std::endl
   << "################################################################################" << std::endl
   << "# Iteration " << gd->getIteration() << " with " << gd->size() << " values" << std::endl
   << "#" << std::endl;

   GBasePS::iterator it;
   for(it=gd->begin(); it!=gd->end(); ++it) {
      result << (*it)->toCSV();
   }

   result.close();
}

/******************************************************************************/
/**
 * A function that is called once at the end of the optimization cycle
 *
 * @param goa A pointer to the current optimization algorithm for which information should be emitted
 */
void GBasePS::GPSOptimizationMonitor::lastInformation(GOptimizationAlgorithmT<GParameterSet> * const goa) {
   // nothing
}

/******************************************************************************/
/**
 * Loads the data of another object
 *
 * @param cp A pointer to another GPSOptimizationMonitor object, camouflaged as a GObject
 */
void GBasePS::GPSOptimizationMonitor::load_(const GObject* cp) {
   const GBasePS::GPSOptimizationMonitor *p_load = gobject_conversion<GBasePS::GPSOptimizationMonitor>(cp);

   // Load the parent classes' data ...
   GOptimizationAlgorithmT<GParameterSet>::GOptimizationMonitorT::load_(cp);

   // ... and then our local data
   csvResultFile_ = p_load->csvResultFile_;
}

/******************************************************************************/
/**
 * Creates a deep clone of this object
 *
 * @return A deep clone of this object
 */
GObject* GBasePS::GPSOptimizationMonitor::clone_() const {
   return new GBasePS::GPSOptimizationMonitor(*this);
}

/******************************************************************************/
/**
 * Applies modifications to this object. This is needed for testing purposes
 */
bool GBasePS::GPSOptimizationMonitor::modify_GUnitTests() {
#ifdef GEM_TESTING
   bool result = false;

   // Call the parent class'es function
   if(GOptimizationAlgorithmT<GParameterSet>::GOptimizationMonitorT::modify_GUnitTests()) result = true;

   return result;

#else /* GEM_TESTING */  // If this function is called when GEM_TESTING isn't set, throw
   condnotset("GBasePS::GPSOptimizationMonitor::modify_GUnitTests", "GEM_TESTING");
   return false;
#endif /* GEM_TESTING */
}

/******************************************************************************/
/**
 * Performs self tests that are expected to succeed. This is needed for testing purposes
 */
void GBasePS::GPSOptimizationMonitor::specificTestsNoFailureExpected_GUnitTests() {
#ifdef GEM_TESTING
   // Call the parent class'es function
   GOptimizationAlgorithmT<GParameterSet>::GOptimizationMonitorT::specificTestsNoFailureExpected_GUnitTests();
#else /* GEM_TESTING */ // If this function is called when GEM_TESTING isn't set, throw
   condnotset("GBasePS::GPSOptimizationMonitor::specificTestsNoFailureExpected_GUnitTests", "GEM_TESTING");
#endif /* GEM_TESTING */
}

/******************************************************************************/
/**
 * Performs self tests that are expected to fail. This is needed for testing purposes
 */
void GBasePS::GPSOptimizationMonitor::specificTestsFailuresExpected_GUnitTests() {
#ifdef GEM_TESTING
   // Call the parent class'es function
   GOptimizationAlgorithmT<GParameterSet>::GOptimizationMonitorT::specificTestsFailuresExpected_GUnitTests();
#else /* GEM_TESTING */ // If this function is called when GEM_TESTING isn't set, throw
   condnotset("GBasePS::GPSOptimizationMonitor::specificTestsFailuresExpected_GUnitTests", "GEM_TESTING");
#endif /* GEM_TESTING */
}

/******************************************************************************/

} /* namespace Geneva */
} /* namespace Gem */

