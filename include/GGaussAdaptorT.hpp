/**
 * @file GGaussAdaptorT.hpp
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

// Standard headers go here
#include <cmath>
#include <string>
#include <sstream>
#include <utility> // For std::pair

// Boost headers go here

#include <boost/version.hpp>
#include "GGlobalDefines.hpp"
#if BOOST_VERSION < ALLOWED_BOOST_VERSION
#error "Error: Boost has incorrect version !"
#endif /* BOOST_VERSION */

#include <boost/cstdint.hpp>
#include <boost/limits.hpp>
#include <boost/cast.hpp>

#ifndef GGAUSSADAPTORT_HPP_
#define GGAUSSADAPTORT_HPP_

// GenEvA headers go here

#include "GAdaptorT.hpp"
#include "GObject.hpp"
#include "GRandom.hpp"
#include "GLogger.hpp"
#include "GenevaExceptions.hpp"
#include "GEnums.hpp"

namespace Gem {
namespace GenEvA {

const std::string GGAUSSADAPTORSTANDARDNAME = "GGaussAdaptorT"; ///< The designated name of this adaptor

/************************************************************************************************/
/**
 * GGaussAdaptorT represents an adaptor used for the mutation of numeric
 * types, by the addition of gaussian-distributed random numbers. Different numeric
 * types may be used, including Boost's integer representations.
 * The type used needs to be specified as a template parameter.
 */
template<typename num_type>
class GGaussAdaptorT:
	public GAdaptorT<num_type>
{
	///////////////////////////////////////////////////////////////////////
	friend class boost::serialization::access;

	template<typename Archive>
	void serialize(Archive & ar, const unsigned int version) {
		using boost::serialization::make_nvp;
		ar & make_nvp("GAdaptorT_num", boost::serialization::base_object<GAdaptorT<num_type> >(*this));
		ar & make_nvp("sigma_", sigma_);
		ar & make_nvp("sigmaSigma_", sigmaSigma_);
		ar & make_nvp("minSigma_", minSigma_);
		ar & make_nvp("maxSigma_", maxSigma_);
	}
	///////////////////////////////////////////////////////////////////////

public:
	/********************************************************************************************/
	/**
	 * The standard constructor. It passes the adaptor's standard name to the
	 * parent class and initializes the internal variables.
	 */
	GGaussAdaptorT()
		:GAdaptorT<num_type> (GGAUSSADAPTORSTANDARDNAME),
		 sigma_(DEFAULTSIGMA),
		 sigmaSigma_(DEFAULTSIGMASIGMA),
		 minSigma_(DEFAULTMINSIGMA),
		 maxSigma_(DEFAULTMAXSIGMA)
	{ /* nothing */ }

	/********************************************************************************************/
	/**
	 * In addition to passing the name of the adaptor to the parent class,
	 * it is also possible to specify a value for the sigma_ parameter in
	 * this constructor.
	 *
	 * @param sigma The initial value for the sigma_ parameter
	 */
	explicit GGaussAdaptorT(const double& sigma)
		:GAdaptorT<num_type> (GGAUSSADAPTORSTANDARDNAME),
		 sigmaSigma_(DEFAULTSIGMASIGMA),
		 minSigma_(DEFAULTMINSIGMA),
		 maxSigma_(DEFAULTMAXSIGMA)
	{
		// This functions does an error check on sigma, so we do not assign
		// the value directly to the private variable.
		setSigma(sigma);
	}

	/********************************************************************************************/
	/**
	 * This constructor lets a user set all parameters in one go.
	 *
	 * @param sigma The initial value for the sigma_ parameter
	 * @param sigmaSigma The initial value for the sigmaSigma_ parameter
	 * @param minSigma The minimal value allowed for sigma_
	 * @param name The name assigned to this adaptor
	 */
	GGaussAdaptorT(const double& sigma, const double& sigmaSigma,
				const double& minSigma, const double& maxSigma)
		:GAdaptorT<num_type> (GGAUSSADAPTORSTANDARDNAME)
	{
		// These functions do error checks on their values
		setSigmaAdaptionRate(sigmaSigma);
		setSigmaRange(minSigma, maxSigma);
		setSigma(sigma);
	}

	/********************************************************************************************/
	/**
	 * A standard copy constructor.
	 *
	 * @param cp Another GGaussAdaptorT object
	 */
	GGaussAdaptorT(const GGaussAdaptorT<num_type>& cp)
		:GAdaptorT<num_type> (cp)
	{
		setSigmaAdaptionRate(cp.sigmaSigma_);
		setSigmaRange(cp.minSigma_, cp.maxSigma_);
		setSigma(cp.sigma_);
	}

	/********************************************************************************************/
	/**
	 * The standard destructor. Empty, as we have no local, dynamically
	 * allocated data.
	 */
	~GGaussAdaptorT()
	{ /* nothing */ }

	/********************************************************************************************/
	/**
	 * A standard assignment operator for GGaussAdaptorT objects,
	 *
	 * @param cp A copy of another GGaussAdaptorT object
	 */
	const GGaussAdaptorT<num_type>& operator=(const GGaussAdaptorT<num_type>& cp)
	{
		GGaussAdaptorT<num_type>::load(&cp);
		return *this;
	}

	/********************************************************************************************/
	/**
	 * This function loads the data of another GGaussAdaptorT, camouflaged as a GObject.
	 *
	 * @param A copy of another GGaussAdaptorT, camouflaged as a GObject
	 */
	void load(const GObject *cp)
	{
		// Convert GObject pointer to local format
		const GGaussAdaptorT<num_type> *gdga = this->conversion_cast(cp, this);

		// Load the data of our parent class ...
		GAdaptorT<num_type>::load(cp);

		// ... and then our own data
		sigma_ = gdga->sigma_;
		sigmaSigma_ = gdga->sigmaSigma_;
		minSigma_ = gdga->minSigma_;
		maxSigma_ = gdga->maxSigma_;
	}


	/********************************************************************************************/
	/**
	 * This function creates a deep copy of this object
	 *
	 * @return A deep copy of this object
	 */
	GObject *clone() const
	{
		return new GGaussAdaptorT<num_type>(*this);
	}

	/********************************************************************************************/
	/**
	 * Checks for equality with another GGaussAdaptorT<num_type> object
	 *
	 * @param  cp A constant reference to another GGaussAdaptorT<num_type> object
	 * @return A boolean indicating whether both objects are equal
	 */
	bool operator==(const GGaussAdaptorT<num_type>& cp) const {
		return GGaussAdaptorT<num_type>::isEqualTo(cp);
	}

	/********************************************************************************************/
	/**
	 * Checks for inequality with another GGaussAdaptorT<num_type> object
	 *
	 * @param  cp A constant reference to another GGaussAdaptorT<num_type> object
	 * @return A boolean indicating whether both objects are inequal
	 */
	bool operator!=(const GGaussAdaptorT<num_type>& cp) const {
		return !GGaussAdaptorT<num_type>::isEqualTo(cp);
	}

	/********************************************************************************************/
	/**
	 * Checks for equality with another GGaussAdaptorT<num_type> object Equality means
	 * that all individual sub-values are equal and that the parent class is equal.
	 *
	 * @param  cp A constant reference to another GGaussAdaptorT<num_type> object
	 * @return A boolean indicating whether both objects are equal
	 */
	bool isEqualTo(const GGaussAdaptorT<num_type>& cp) const {
		if(!GAdaptorT<num_type>::isEqualTo(cp)) return false;

		if(sigma_ != cp.sigma_) return false;
		if(sigmaSigma_ != cp.sigmaSigma_) return false;
		if(minSigma_ != cp.minSigma_) return false;
		if(maxSigma_ != cp.maxSigma_) return false;

		return true;
	}

	/********************************************************************************************/
	/**
	 * Checks for similarity with another GGaussAdaptorT<num_type> object. Similarity means
	 * that all double values are similar to each other within a given limit and that all other
	 * values are equal. Also, parent classes must be similar to each other.
	 *
	 * @param  cp A constant reference to another GGaussAdaptorT<num_type> object
	 * @param limit A double value specifying the acceptable level of differences of floating point values
	 * @return A boolean indicating whether both objects are similar to each other
	 */
	bool isSimilarTo(const GGaussAdaptorT<num_type>& cp, const double& limit=0.) const {
		if(!GAdaptorT<num_type>::isSimilarTo(cp, limit)) return false;

		if(fabs(sigma_ - cp.sigma_) > fabs(limit)) return false;
		if(fabs(sigmaSigma_ - cp.sigmaSigma_) > fabs(limit)) return false;
		if(fabs(minSigma_ - cp.minSigma_) > fabs(limit)) return false;
		if(fabs(maxSigma_ - cp.maxSigma_) > fabs(limit)) return false;

		return true;
	}

	/********************************************************************************************/
	/**
	 * This function sets the value of the sigma_ parameter. Note that this function
	 * will silently set a 0 sigma to a very small value.
	 *
	 * @param sigma The new value of the sigma_ parameter
	 */
	void setSigma(const double& sigma)
	{
		if(sigma < 0.) {
			std::ostringstream error;
			error << "In GGaussAdaptorT::setSigma(const double&): Error!" << std::endl
			  	  << "sigma is negative: " << sigma << std::endl;

			throw geneva_error_condition(error.str());
		}

		double tmpSigma;
		if(sigma == 0.) tmpSigma = DEFAULTMINSIGMA;
		else tmpSigma = sigma;

		// Sigma must be in the allowed value range
		if(tmpSigma < minSigma_ || tmpSigma > maxSigma_)
		{
			std::ostringstream error;
			error << "In GGaussAdaptorT::setSigma(const double&): Error!" << std::endl
			  	  << "sigma is not in the allowed range: " << std::endl
			  	  << tmpSigma << " " << minSigma_ << " " << maxSigma_ << std::endl
			  	  << "If you want to use these values you need to" << std::endl
			  	  << "adapt the allowed range first." << std::endl;

			throw geneva_error_condition(error.str());
		}

		sigma_ = tmpSigma;
	}

	/********************************************************************************************/
	/**
	 * Retrieves the current value of sigma_.
	 *
	 * @return The current value of sigma_
	 */
	double getSigma() const throw() {
		return sigma_;
	}

	/********************************************************************************************/
	/**
	 * Sets the allowed value range of sigma_. A minimum sigma of 0 will silently be adapted
	 * to a very small value (DEFAULTMINSIGMA), as otherwise mutations would stop entirely,
	 * which does not make sense.  Using 0. as lower boundary is however allowed for practical
	 * reasons. Note that this function will also adapt sigma itself, if it falls outside of the allowed
	 * range.
	 *
	 * @param minSigma The minimum allowed value of sigma_
	 * @param minSigma The maximum allowed value of sigma_
	 */
	void setSigmaRange(const double& minSigma, const double& maxSigma){
		double tmpMinSigma = minSigma;
		// Silently adapt minSigma, if necessary. A value of 0. does not make sense.
		if(minSigma == 0.) tmpMinSigma = DEFAULTMINSIGMA;

		// Do some error checks
		if(tmpMinSigma<=0. || minSigma >= maxSigma){ // maxSigma will automatically be > 0. now
			std::ostringstream error;
			error << "In GGaussAdaptorT::setSigmaRange(const double&, const double&): Error!" << std::endl
				  << "Invalid values for minSigma and maxSigma given:" << tmpMinSigma << " " << maxSigma << std::endl;

			throw geneva_error_condition(error.str());
		}

		minSigma_ = tmpMinSigma;
		maxSigma_ = maxSigma;

		// Adapt sigma, if necessary
		if(sigma_ < minSigma_) sigma_ = minSigma_;
		else if(sigma_>maxSigma_) sigma_ = maxSigma_;
	}

	/********************************************************************************************/
	/**
	 * Retrieves the allowed value range for sigma. You can retrieve the values
	 * like this: getSigmaRange().first , getSigmaRange().second .
	 *
	 * @return The allowed value range for sigma
	 */
	std::pair<double,double> getSigmaRange() const throw() {
		return std::make_pair(minSigma_, maxSigma_);
	}

	/********************************************************************************************/
	/**
	 * This function sets the values of the sigmaSigma_ parameter and the
	 * minimal value allowed for sigma_. 0 is not allowed. If you do want to
	 * prevent adaption of sigma, you can use the GAdaptorT<T>::setAdaptionThreshold()
	 * function. It determines, after how many mutations the internal parameters
	 * of the mutation should be adapted. If set to 0, no adaption takes place.
	 *
	 * @param sigmaSigma The new value of the sigmaSigma_ parameter
	 */
	void setSigmaAdaptionRate(const double& sigmaSigma)
	{
		// A value of sigmaSigma <= 0. is not useful.
		if(sigmaSigma <= 0.)
		{
			std::ostringstream error;
			error << "In GGaussAdaptorT::setSigmaSigma(double, double): Error!" << std::endl
				  << "Bad value for sigmaSigma given: " << sigmaSigma << std::endl;

			throw geneva_error_condition(error.str());
		}

		sigmaSigma_ = sigmaSigma;
	}

	/********************************************************************************************/
	/**
	 * Retrieves the value of sigmaSigma_ .
	 *
	 * @return The value of the sigmaSigma_ parameter
	 */
	double getSigmaAdaptionRate() const throw() {
		return sigmaSigma_;
	}

	/********************************************************************************************/
	/**
	 * Convenience function that lets users set all relevant parameters of this class
	 * at once.
	 *
	 * @param sigma The initial value for the sigma_ parameter
	 * @param sigmaSigma The initial value for the sigmaSigma_ parameter
	 * @param minSigma The minimal value allowed for sigma_
	 * @param minSigma The maximum value allowed for sigma_
	 */
	void setAll(const double& sigma, const double& sigmaSigma, const double& minSigma, const double& maxSigma)
	{
		setSigmaAdaptionRate(sigmaSigma);
		setSigmaRange(minSigma, maxSigma);
		setSigma(sigma);
	}

	/********************************************************************************************/
	/**
	 * Returns the standard name of a GGaussAdaptorT
	 *
	 * @return The name assigned to adaptors of this type
	 */
	static std::string adaptorName() throw() {
		return GGAUSSADAPTORSTANDARDNAME;
	}

protected:
	/********************************************************************************************/
	/**
	 * This adaptor allows the evolutionary adaption of sigma_. This allows the
	 * algorithm to adapt to changing geometries of the quality surface. The
	 * function is declared inline, as it might be called very often.
	 */
	inline void adaptMutation()
	{
		sigma_ *= exp(this->gr.gaussRandom(0.,sigmaSigma_));

		// make sure sigma_ doesn't get out of range
		if(fabs(sigma_) < minSigma_) sigma_ = minSigma_;
		else if(fabs(sigma_) > maxSigma_) sigma_ = maxSigma_;
	}

	/********************************************************************************************/
	/**
	 * The actual mutation of the supplied value takes place here.
	 *
	 * @param value The value that is going to be mutated in situ
	 */
	inline void customMutations(num_type &value)
	{
		// adapt the value in situ. Note that this changes
		// the argument of this function
#if defined (CHECKOVERFLOWS)
		// Prevent over- and underflows.
#if defined (DEBUG)
		num_type addition = boost::numeric_cast<num_type>(this->gr.gaussRandom(0.,sigma_));
#else
		num_type addition = static_cast<num_type>(this->gr.gaussRandom(0.,sigma_));
#endif /* DEBUG */

		if(value >= 0){
			if(addition >= 0 && (std::numeric_limits<num_type>::max()-value < addition)) addition *= -1;
		}
		else { // < 0
			if(addition < 0 && (std::numeric_limits<num_type>::min()-value > addition)) addition *= -1;
		}

		value += addition;
#else
		// We do not check for over- or underflows for performance reasons.
#if defined (DEBUG)
		value += boost::numeric_cast<num_type>(this->gr.gaussRandom(0.,sigma_));
#else
		value += static_cast<num_type>(this->gr.gaussRandom(0.,sigma_));
#endif /* DEBUG */
#endif /* CHECKOVERFLOWS  */
	}

private:
	/********************************************************************************************/
	double sigma_; ///< The width of the gaussian used to adapt values
	double sigmaSigma_; ///< affects sigma_ adaption
	double minSigma_; ///< minimum allowed value for sigma_
	double maxSigma_; ///< maximum allowed value for sigma_
};

/************************************************************************************************/
// Declaration of some specializations
template<> void GGaussAdaptorT<double>::customMutations(double&);
template<> void GGaussAdaptorT<char>::customMutations(char&);
template<> void GGaussAdaptorT<short>::customMutations(short&);

} /* namespace GenEvA */
} /* namespace Gem */

#endif /* GGAUSSADAPTORT_HPP_ */
