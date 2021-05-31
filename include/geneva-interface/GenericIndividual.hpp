#pragma once
#include "common/GGlobalDefines.hpp"

// Standard header files go here
#include <cmath>
#include <functional>
#include <iostream>
#include <sstream>
#include <tuple>
#include <vector>

// Boost header files go here

// Geneva header files go here
#include "common/GCommonMathHelperFunctionsT.hpp"
#include "common/GFactoryT.hpp"
#include "common/GParserBuilder.hpp"
#include "geneva/GConstrainedDoubleCollection.hpp"
#include "geneva/GConstrainedDoubleObject.hpp"
#include "geneva/GDoubleBiGaussAdaptor.hpp"
#include "geneva/GDoubleCollection.hpp"
#include "geneva/GDoubleGaussAdaptor.hpp"
#include "geneva/GParameterSet.hpp"

namespace Gem {
namespace Geneva {

/******************************************************************************/
/**
 * This enum denotes the possible demo function types
 */
/******************************************************************************/
// A number of default settings for the factory
const double GSI_DEF_ADPROB = 1.0;
const double GSI_DEF_SIGMA = 0.025;
const double GSI_DEF_SIGMASIGMA = 0.2;
const double GSI_DEF_MINSIGMA = 0.001;
const double GSI_DEF_MAXSIGMA = 1;

/******************************************************************************/
/**
 * This individual searches for a minimum of a number of predefined functions,
 * each capable of processing their input in multiple dimensions.
 */

template <typename Function>
class GenericIndividual : public GParameterSet {
  ///////////////////////////////////////////////////////////////////////
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive &ar, const unsigned int version)
  {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(GParameterSet);
  }
  static std::function<double(std::vector<double> &)> func;
  double Fitness(std::vector<double> &params) { return func(params); }

  public:
  static void setFunc(std::function<double(std::vector<double> &)> &&f)
  {
    func = f;
  }
  // static void setFunc(Function_t&& funcc){func=funcc;}
  /** @brief The default constructor */
  // GenericIndividual(Function_t&& func);
  GenericIndividual();
  /** @brief A constructor that receives all arguments */
  GenericIndividual(const std::size_t &, const std::vector<double>,
		    const std::vector<double>, const std::vector<double>,
		    const double &, const double &, const double &,
		    const double &, const double &);
  /** @brief A standard copy constructor */
  GenericIndividual(const GenericIndividual<Function> &);
  /** @brief The standard destructor */
  virtual ~GenericIndividual();

  /** @brief Searches for compliance with expectations with respect to another
   * object of the same type */
  virtual void compare(
      const GObject &  // the other object
      ,
      const Gem::Common::expectation &	// the expectation for this object,
					// e.g. equality
      ,
      const double &  // the limit for allowed deviations of floating point
		      // types
  ) const final;

  /** @brief Adds local configuration options to a GParserBuilder object */
  virtual void addConfigurationOptions(Gem::Common::GParserBuilder &) final;

  /** @brief Retrieves the average value of the sigma used in local Gauss
   * adaptors */
  double getAverageSigma() const;
  double getSigma(size_t indx) const;

  /** @brief Emit information about this individual */
  std::string print();

  /***************************************************************************/
  /**
   * This function is used to unify the setup from within the constructor
   * and factory.
   */
  static void addContent(GenericIndividual &p, const std::size_t &prod_id,
			 const std::vector<double> &startValues,
			 const std::vector<double> &lowerBoundaries,
			 const std::vector<double> &upperBoundaries,
			 const double &sigma, const double &sigmaSigma,
			 const double &minSigma, const double &maxSigma,
			 const double &adProb)
  {
    // Some error checking
#ifdef DEBUG
    // Check whether values have been provided
    if (startValues.empty()) {
      glogger << "In GenericIndividual::addContent(): Error!" << std::endl
	      << "No parameters given" << std::endl
	      << GTERMINATION;
    }

    // Check whether all sizes match
    if (startValues.size() != lowerBoundaries.size() ||
	startValues.size() != upperBoundaries.size()) {
      glogger << "In GenericIndividual::addContent(): Error!" << std::endl
	      << "Invalid sizes" << startValues.size() << " / "
	      << lowerBoundaries.size() << " / " << upperBoundaries.size()
	      << std::endl
	      << GTERMINATION;
    }

    // Check that start values and boundaries have valid values
    for (std::size_t i = 0; i < startValues.size(); i++) {
      Gem::Common::checkValueRange(  // We expect the start value to be in the
				     // range [lower, upper[
	  startValues.at(i), lowerBoundaries.at(i), upperBoundaries.at(i),
	  false	 // closed lower boundary
	  ,
	  true	// open upper boundary
      );
    }

#endif /* DEBUG */

    // Add the required number of GConstrainedDoubleObject objects to the
    // individual
    auto make_gcon = [&](int i) {
      if (Gem::Common::GFACTTORYFIRSTID == prod_id)
	// intialize first using startValues
	return std::make_shared<GConstrainedDoubleObject>(
	    startValues.at(i), lowerBoundaries.at(i), upperBoundaries.at(i));
      // intialize randomly
      return std::make_shared<GConstrainedDoubleObject>(lowerBoundaries.at(i),
							upperBoundaries.at(i));
    };
    for (std::size_t i = 0; i < startValues.size(); i++) {
      auto gcdo_ptr = make_gcon(i);

      auto gdga_ptr = std::make_shared<GDoubleGaussAdaptor>(sigma, sigmaSigma,
							    minSigma, maxSigma);

      gdga_ptr->setAdaptionProbability(adProb);
      gcdo_ptr->addAdaptor(gdga_ptr);

      p.push_back(gcdo_ptr);
    }
  }

  protected:
  /***************************************************************************/
  /** @brief Loads the data of another GenericIndividual */
  virtual void load_(const GObject *) final;

  /** @brief The actual value calculation takes place here */
  virtual double fitnessCalculation() final;

  /***************************************************************************/

  private:
  /***************************************************************************/
  /** @brief Creates a deep clone of this object */
  virtual GObject *clone_() const final;

  /***************************************************************************/
  /** @brief A simple n-dimensional parabola */
  double parabola(const std::vector<double> &parVec) const;
  /** @brief A "noisy" parabola */
  double noisyParabola(const std::vector<double> &parVec) const;

  /***************************************************************************/
};

/** @brief Allows to output a GenericIndividual or convert it to a string
 * using boost::lexical_cast */
template <typename Function>
std::ostream &operator<<(std::ostream &,
			 std::shared_ptr<GenericIndividual<Function>>);

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/**
 * A factory for GenericIndividual objects
 */
template <typename Function>
class GenericIndividualFactory : public Gem::Common::GFactoryT<GParameterSet> {
  public:
  using Function_t = Function;
  /** @brief The standard constructor */

  GenericIndividualFactory(const std::string &, Function &&func);
  GenericIndividualFactory(const std::string &, const std::vector<double> &,
			   const std::vector<double> &,
			   const std::vector<double> &, Function &&func);
  /** @brief The destructor */
  virtual ~GenericIndividualFactory();

  protected:
  /** @brief Creates individuals of this type */
  virtual std::shared_ptr<GParameterSet> getObject_(
      Gem::Common::GParserBuilder &, const std::size_t &);
  /** @brief Allows to describe local configuration options in derived classes
   */
  virtual void describeLocalOptions_(Gem::Common::GParserBuilder &);
  /** @brief Allows to act on the configuration options received from the
   * configuration file */
  virtual void postProcess_(std::shared_ptr<GParameterSet> &);

  private:
  // std::function<double(std::vector<double>&)> func;
  /** @brief The default constructor. Intentionally private and undefined */
  GenericIndividualFactory() = delete;

  double adProb_;      ///< Probability for a parameter to be mutated
  double sigma_;       ///< Step-width
  double sigmaSigma_;  ///< Speed of sigma_-adaption
  double minSigma_;    ///< Minimum allowed sigma value
  double maxSigma_;    ///< Maximum allowed sigma value

  std::vector<double> startValues_;  ///< Start values for all parameters
  std::vector<double>
      lowerBoundaries_;	 ///< Lower boundaries for all parameters
  std::vector<double>
      upperBoundaries_;	 ///< Upper boundaroes for all parameters
  public:
  Function_t func;
  double getMaxSigma() { return maxSigma_; }
  double getMinSigma() { return minSigma_; }
  double getsigmaSigma() { return sigmaSigma_; }
  double getAdProb() { return adProb_; }
};

/******************************************************************************/

} /* namespace Geneva */
} /* namespace Gem */
