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

#pragma once

// Global checks, defines and includes needed for all of Geneva
#include "common/GGlobalDefines.hpp"

// Standard headers go here

// Boost headers go here

// Geneva headers go here
#include "geneva/GAdaptorT.hpp"

namespace Gem {
namespace Geneva {

/******************************************************************************/
/**
 * GNumBiGaussAdaptorT is used for the adaption of numeric types, by the addition of random numbers
 * distributed as two adjacent gaussians. Different numeric types may be used, including Boost's
 * integer representations. The type used needs to be specified as a template parameter. In comparison
 * to GNumGaussAdaptorT, an additional parameter "delta" is added, which represents the distance between
 * both gaussians. Just like sigma, delta can be subject to mutations. It is also possible to use
 * two different sigma/sigmaSigma values and adaption rates for both gaussians. Note that this adaptor
 * is experimental. Your mileage may vary.
 */
template<typename num_type, typename fp_type>
class GNumBiGaussAdaptorT
    : public GAdaptorT<num_type, fp_type>
{
    ///////////////////////////////////////////////////////////////////////
    friend class boost::serialization::access;

    template<typename Archive>
    void serialize(Archive &ar, const unsigned int) {
        using boost::serialization::make_nvp;

        // Save all necessary data
        ar
        & make_nvp(
            "GAdaptorT_num"
            , boost::serialization::base_object<GAdaptorT<num_type>>(*this))
        & BOOST_SERIALIZATION_NVP(useSymmetricSigmas_)
        & BOOST_SERIALIZATION_NVP(sigma1_)
        & BOOST_SERIALIZATION_NVP(sigmaSigma1_)
        & BOOST_SERIALIZATION_NVP(minSigma1_)
        & BOOST_SERIALIZATION_NVP(maxSigma1_)
        & BOOST_SERIALIZATION_NVP(sigma2_)
        & BOOST_SERIALIZATION_NVP(sigmaSigma2_)
        & BOOST_SERIALIZATION_NVP(minSigma2_)
        & BOOST_SERIALIZATION_NVP(maxSigma2_)
        & BOOST_SERIALIZATION_NVP(delta_)
        & BOOST_SERIALIZATION_NVP(sigmaDelta_)
        & BOOST_SERIALIZATION_NVP(minDelta_)
        & BOOST_SERIALIZATION_NVP(maxDelta_);
    }
    ///////////////////////////////////////////////////////////////////////

public:
    /***************************************************************************/
    /**
     * Default constructor
     */
    GNumBiGaussAdaptorT() = default;

    /***************************************************************************/
    /**
     * Initialization of the parent class'es adaption probability.
     *
     * @param probability The likelihood for a adaption actually taking place
     */
    explicit GNumBiGaussAdaptorT(const fp_type &probability)
        :
        GAdaptorT<num_type>(probability)
    { /* nothing */ }

    /***************************************************************************/
    /**
     * A standard copy constructor. It assumes that the values of the other object are correct
     * and does no additional error checks.
     *
     * @param cp Another GNumBiGaussAdaptorT object
     */
    GNumBiGaussAdaptorT(const GNumBiGaussAdaptorT<num_type, fp_type> &cp) = default;

    /***************************************************************************/
    /**
     * The standard destructor. Empty, as we have no local, dynamically
     * allocated data.
     */
    ~GNumBiGaussAdaptorT() override = default;

    /***************************************************************************/
    /**
     * Determines whether the two sigmas of the double-gaussian should be identical
     *
     * @param useSymmetricSigmas A boolean which determines whether the two sigmas of the double-gaussian should be identical
     */
    void setUseSymmetricSigmas(const bool &useSymmetricSigmas) {
        useSymmetricSigmas_ = useSymmetricSigmas;
    }

    /***************************************************************************/
    /**
     * Retrieves the value of the "useSymmetricSigmas_" variable
     *
     * @return The value of the "useSymmetricSigmas_" variable
     */
    bool getUseSymmetricSigmas() const {
        return useSymmetricSigmas_;
    }

    /***************************************************************************/
    /**
     * This function sets the value of the sigma1_ parameter. It is recommended
     * that the value lies in the range [0.:1.]. A value below 0 is not allowed.
     * Sigma is interpreted as a percentage of the allowed or desired value range
     * of the target variable.
     *
     * @param sigma1 The new value of the sigma_ parameter
     */
    void setSigma1(const fp_type &sigma1) {
        // Sigma1 must be in the allowed value range
        if (sigma1 < minSigma1_ || sigma1 > maxSigma1_ || sigma1 < fp_type(0)) {
            throw gemfony_exception(
                g_error_streamer(
                    DO_LOG
                    , time_and_place
                )
                    << "In GNumBiGaussAdaptorT<num_type, fp_type>::setSigma1(const fp_type&):" << std::endl
                    << "sigma1 is not in the allowed range: " << std::endl
                    << minSigma1_ << " <= " << sigma1 << " < " << maxSigma1_ << std::endl
                    << "If you want to use these values you need to" << std::endl
                    << "adapt the allowed range first." << std::endl
            );
        }

        sigma1_ = sigma1;
    }

    /***************************************************************************/
    /**
     * Retrieves the current value of sigma1_.
     *
     * @return The current value of sigma1_
     */
    fp_type getSigma1() const {
        return sigma1_;
    }

    /***************************************************************************/
    /**
     * Sets the allowed value range of sigma1_. A minimum sigma1 of 0 will silently be adapted
     * to a very small value (DEFAULTMINSIGMA), as otherwise adaptions would stop entirely,
     * which does not make sense.  Using 0. as lower boundary is however allowed for practical
     * reasons. Note that this function will also adapt sigma1 itself, if it falls outside of the
     * allowed range. It is not recommended (but not enforced) to set a maxSigma1 > 1, as sigma
     * is interpreted as a percentage of the allowed or desired value range of the target variable.
     *
     * @param minSigma1 The minimum allowed value of sigma1_
     * @param maxSigma1 The maximum allowed value of sigma1_
     */
    void setSigma1Range(
        const fp_type &minSigma1
        , const fp_type &maxSigma1
    ) {
        using namespace Gem::Common;

        if (minSigma1 < fp_type(0.) || minSigma1 > maxSigma1 ||
            maxSigma1 < boost::numeric_cast<fp_type>(DEFAULTMINSIGMA)) {
            throw gemfony_exception(
                g_error_streamer(
                    DO_LOG
                    , time_and_place
                )
                    << "In GNumBiGaussAdaptorT::setSigma1Range(const fp_type&, const fp_type&):" << std::endl
                    << "Invalid values for minSigma1 and maxSigma1 given: " << minSigma1 << " / " << maxSigma1
                    << std::endl
            );
        }

        minSigma1_ = minSigma1;
        maxSigma1_ = maxSigma1;

        // Silently adapt m_minSigma1, if it is smaller than DEFAULTMINSIGMA. E.g., a value of 0 does not make sense
        if (minSigma1_ < fp_type(DEFAULTMINSIGMA)) {
            minSigma1_ = fp_type(DEFAULTMINSIGMA);
        }

        // Rectify m_sigma1, if necessary
        enforceRangeConstraint(
            sigma1_
            , minSigma1_
            , maxSigma1_
            , "GNumBiGaussAdaptorT<>::setSigma1Range()"
        );
    }

    /***************************************************************************/
    /**
     * Retrieves the allowed value range for sigma1. You can retrieve the values
     * like this: getSigma1Range().first , getSigmaRange().second .
     *
     * @return The allowed value range for sigma1
     */
    std::tuple<fp_type, fp_type> getSigma1Range() const {
        return std::make_tuple(
            minSigma1_
            , maxSigma1_
        );
    }

    /***************************************************************************/
    /**
     * This function sets the values of the sigmaSigma1_ parameter. Values <= 0 mean "do not adapt
     * sigma1_". If you do want to prevent adaption of sigma1, you can also use the
     * GAdaptorT<T>::setAdaptionThreshold() function. It determines, after how many calls the
     * internal parameters of the adaption should be adapted. If set to 0, no adaption takes place.
     *
     * TODO: Cross-check suitable values
     *
     * @param sigmaSigma1 The new value of the sigmaSigma1_ parameter
     */
    void setSigma1AdaptionRate(const fp_type &sigmaSigma1) {
        sigmaSigma1_ = sigmaSigma1;
    }

    /***************************************************************************/
    /**
     * Retrieves the value of sigmaSigma1_ .
     *
     * @return The value of the sigmaSigma1_ parameter
     */
    fp_type getSigma1AdaptionRate() const {
        return sigmaSigma1_;
    }

    /***************************************************************************/
    /**
     * Convenience function that lets users set all relevant parameters of the sigma1_ parameter
     * at once
     *
     * @param sigma1 The initial value for the sigma1_ parameter
     * @param sigmaSigma1 The initial value for the sigmaSigma1_ parameter
     * @param minSigma1 The minimal value allowed for sigma1_
     * @param minSigma1 The maximum value allowed for sigma1_
     */
    void setAllSigma1(
        const fp_type &sigma1
        , const fp_type &sigmaSigma1
        , const fp_type &minSigma1
        , const fp_type &maxSigma1
    ) {
        setSigma1AdaptionRate(sigmaSigma1);
        setSigma1Range(
            minSigma1
            , maxSigma1
        );
        setSigma1(sigma1);
    }

    /***************************************************************************/
    /**
     * This function sets the value of the sigma2_ parameter. It is recommended
     * that the value lies in the range [0.:1.]. A value below 0 is not allowed.
     * Sigma is interpreted as a percentage of the allowed or desired value range
     * of the target variable.
     *
     * @param sigma2 The new value of the sigma_ parameter
     */
    void setSigma2(const fp_type &sigma2) {
        // Sigma2 must be in the allowed value range
        if (sigma2 < minSigma2_ || sigma2 > maxSigma2_ || sigma2 < fp_type(0)) {
            throw gemfony_exception(
                g_error_streamer(
                    DO_LOG
                    , time_and_place
                )
                    << "In GNumBiGaussAdaptorT<num_type, fp_type>::setSigma2(const fp_type&):" << std::endl
                    << "sigma2 is not in the allowed range: " << std::endl
                    << minSigma2_ << " <= " << sigma2 << " < " << maxSigma2_ << std::endl
                    << "If you want to use this value for sigma you need to" << std::endl
                    << "adapt the allowed range first." << std::endl
            );
        }

        sigma2_ = sigma2;
    }

    /***************************************************************************/
    /**
     * Retrieves the current value of sigma2_.
     *
     * @return The current value of sigma2_
     */
    fp_type getSigma2() const {
        return sigma2_;
    }

    /***************************************************************************/
    /**
     * Sets the allowed value range of sigma2_. A minimum sigma2 of 0 will silently be adapted
     * to a very small value (DEFAULTMINSIGMA), as otherwise adaptions would stop entirely,
     * which does not make sense.  Using 0. as lower boundary is however allowed for practical
     * reasons. Note that this function will also adapt sigma2 itself, if it falls outside of the
     * allowed range. It is not recommended (but not enforced) to set a maxSigma2 > 1, as sigma
     * is interpreted as a percentage of the allowed or desired value range of the target variable.
     *
     * @param minSigma2 The minimum allowed value of sigma2_
     * @param maxSigma2 The maximum allowed value of sigma2_
     */
    void setSigma2Range(const fp_type &minSigma2, const fp_type &maxSigma2) {
        using namespace Gem::Common;

        if (minSigma2 < fp_type(0.) || minSigma2 > maxSigma2 ||
            maxSigma2 < boost::numeric_cast<fp_type>(DEFAULTMINSIGMA)) {
            throw gemfony_exception(
                g_error_streamer(
                    DO_LOG
                    , time_and_place
                )
                    << "In GNumBiGaussAdaptorT::setSigma2Range(const fp_type&, const fp_type&):" << std::endl
                    << "Invalid values for minSigma2 and maxSigma2 given: " << minSigma2 << " / " << maxSigma2
                    << std::endl
            );
        }


        minSigma2_ = minSigma2;
        maxSigma2_ = maxSigma2;

        // Silently adapt m_minSigma1, if it is smaller than DEFAULTMINSIGMA. E.g., a value of 0 does not make sense
        if (minSigma2_ < fp_type(DEFAULTMINSIGMA)) {
            minSigma2_ = fp_type(DEFAULTMINSIGMA);
        }

        // Rectify m_sigma1, if necessary
        enforceRangeConstraint(
            sigma2_
            , minSigma2_
            , maxSigma2_
            , "GNumBiGaussAdaptorT<>::setSigma2Range()"
        );
    }

    /***************************************************************************/
    /**
     * Retrieves the allowed value range for sigma2. You can retrieve the values
     * like this: getSigma2Range().first , getSigmaRange().second .
     *
     * @return The allowed value range for sigma2
     */
    std::tuple<fp_type, fp_type> getSigma2Range() const {
        return std::make_tuple(
            minSigma2_
            , maxSigma2_
        );
    }

    /***************************************************************************/
    /**
     * This function sets the values of the sigmaSigma2_ parameter. Values <= 0 mean "do not adapt
     * sigma2_". If you do want to prevent adaption of sigma1, you can also use the
     * GAdaptorT<T>::setAdaptionThreshold() function. It determines, after how many calls the
     * internal parameters of the adaption should be adapted. If set to 0, no adaption takes place.
     *
     * @param sigmaSigma2 The new value of the sigmaSigma2_ parameter
     */
    void setSigma2AdaptionRate(const fp_type &sigmaSigma2) {
        sigmaSigma2_ = sigmaSigma2;
    }

    /***************************************************************************/
    /**
     * Retrieves the value of sigmaSigma2_ .
     *
     * @return The value of the sigmaSigma2_ parameter
     */
    fp_type getSigma2AdaptionRate() const {
        return sigmaSigma2_;
    }

    /***************************************************************************/
    /**
     * Convenience function that lets users set all relevant parameters of the sigma2_ parameter
     * at once
     *
     * @param sigma2 The initial value for the sigma2_ parameter
     * @param sigmaSigma2 The initial value for the sigmaSigma2_ parameter
     * @param minSigma2 The minimal value allowed for sigma2_
     * @param minSigma2 The maximum value allowed for sigma2_
     */
    void setAllSigma2(
        const fp_type &sigma2
        , const fp_type &sigmaSigma2
        , const fp_type &minSigma2
        , const fp_type &maxSigma2
    ) {
        setSigma2AdaptionRate(sigmaSigma2);
        setSigma2Range(
            minSigma2
            , maxSigma2
        );
        setSigma2(sigma2);
    }

    /***************************************************************************/
    /**
     * This function sets the value of the delta_ parameter. It is recommended
     * that the value lies in the range [0.:0.5]. A value below 0 is not allowed.
     *
     * @param delta The new value of the sigma_ parameter
     */
    void setDelta(const fp_type &delta) {
        // Delta must be in the allowed value range
        if (delta < minDelta_ || delta > maxDelta_ || delta_ < fp_type(0)) {
            throw gemfony_exception(
                g_error_streamer(
                    DO_LOG
                    , time_and_place
                )
                    << "In GNumBiGaussAdaptorT::setDelta(const fp_type&):" << std::endl
                    << "delta is not in the allowed range: " << std::endl
                    << minDelta_ << " <= " << delta << " < " << maxDelta_ << std::endl
                    << "If you want to use these values you need to" << std::endl
                    << "adapt the allowed range first." << std::endl
            );
        }

        delta_ = delta;
    }

    /***************************************************************************/
    /**
     * Retrieves the current value of delta_.
     *
     * @return The current value of delta_
     */
    fp_type getDelta() const {
        return delta_;
    }

    /***************************************************************************/
    /**
     * Sets the allowed value range of delta_. A minimum delta of 0 will silently be adapted
     * to DEFAULTMINSIGMA, if that value is > 0. Note that this function will also adapt delta
     * itself, if it falls outside of the allowed range. A maximum of 0.5 for maxDelta_ is
     * recommended, but not enforced.delta is interpreted as a percentage of the allowed or
     * desired value range of the target variable.
     *
     * @param minDelta The minimum allowed value of delta_
     * @param maxDelta The maximum allowed value of delta_
     */
    void setDeltaRange(
        const fp_type &minDelta
        , const fp_type &maxDelta
    ) {
        if (minDelta < fp_type(0.) || minDelta > maxDelta || maxDelta < boost::numeric_cast<fp_type>(DEFAULTMINDELTA)) {
            throw gemfony_exception(
                g_error_streamer(
                    DO_LOG
                    , time_and_place
                )
                    << "In GNumBiGaussAdaptorT::setDeltaRange(const fp_type&, const fp_type&):" << std::endl
                    << "Invalid values for minDelta and maxDelta given: " << minDelta << " / " << maxDelta << std::endl
            );
        }

        minDelta_ = minDelta;
        maxDelta_ = maxDelta;

        // Note: In contrast to setSigmaXRange(...) we allow a delta < DEFAULTMINDELTA
        // (as long as it is >= 0), as a delta of 0 makes sense

        // Rectify m_delta, if necessary
        if (delta_ < minDelta_) {
            delta_ = minDelta_;
        } else if (delta_ > maxDelta_) {
            delta_ = maxDelta_;
        }
    }

    /***************************************************************************/
    /**
     * Retrieves the allowed value range for delta. You can retrieve the values
     * like this: getDeltaRange().first , getSigmaRange().second .
     *
     * @return The allowed value range for delta
     */
    std::tuple<fp_type, fp_type> getDeltaRange() const {
        return std::make_tuple(
            minDelta_
            , maxDelta_
        );
    }

    /***************************************************************************/
    /**
     * This function sets the values of the sigmaSigma2_ parameter. Values <= 0 mean "do not adapt
     * delta_". If you do want to prevent adaption of delta_, you can also use the
     * GAdaptorT<T>::setAdaptionThreshold() function. It determines, after how many calls the
     * internal parameters of the adaption should be adapted. If set to 0, no adaption takes place.
     *
     * @param sigmaDelta The new value of the sigmaDelta_ parameter
     */
    void setDeltaAdaptionRate(const fp_type &sigmaDelta) {
        sigmaDelta_ = sigmaDelta;
    }

    /***************************************************************************/
    /**
     * Retrieves the value of sigmaDelta_ .
     *
     * @return The value of the sigmaDelta_ parameter
     */
    fp_type getDeltaAdaptionRate() const {
        return sigmaDelta_;
    }

    /***************************************************************************/
    /**
     * Convenience function that lets users set all relevant parameters of the delta_ parameter
     * at once
     *
     * @param delta The initial value for the delta_ parameter
     * @param sigmaDelta The initial value for the sigmaDelta_ parameter
     * @param minDelta The minimal value allowed for delta_
     * @param minDelta The maximum value allowed for delta_
     */
    void setAllDelta(
        const fp_type &delta
        , const fp_type &sigmaDelta
        , const fp_type &minDelta
        , const fp_type &maxDelta
    ) {
        setDeltaAdaptionRate(sigmaDelta);
        setDeltaRange(
            minDelta
            , maxDelta
        );
        setDelta(delta);
    }

    /***************************************************************************/
    /**
     * Allows to randomly initialize parameter members
     */
    bool randomInit(
        Gem::Hap::GRandomBase &gr
    ) override {
        using namespace Gem::Common;
        using namespace Gem::Hap;

        sigma1_ = GAdaptorT<num_type, fp_type>::m_uniform_real_distribution(
            gr
            , typename std::uniform_real_distribution<fp_type>::param_type(
                minSigma1_
                , maxSigma1_
            ));
        sigma2_ = GAdaptorT<num_type, fp_type>::m_uniform_real_distribution(
            gr
            , typename std::uniform_real_distribution<fp_type>::param_type(
                minSigma2_
                , maxSigma2_
            ));
        delta_ = GAdaptorT<num_type, fp_type>::m_uniform_real_distribution(
            gr
            , typename std::uniform_real_distribution<fp_type>::param_type(
                minDelta_
                , maxDelta_
            ));

        return true;
    }

protected:
    /***************************************************************************/
    // For performance reasons, so we do not have to go through access functions
    bool useSymmetricSigmas_ = true; ///< Determines whether the sigmas of both gaussians should be the same

    fp_type sigma1_ = DEFAULTSIGMA; ///< The width of the first gaussian used to adapt values
    fp_type sigmaSigma1_ = DEFAULTSIGMASIGMA; ///< affects sigma1_ adaption
    fp_type minSigma1_ = DEFAULTMINSIGMA; ///< minimum allowed value for sigma1_
    fp_type maxSigma1_ = DEFAULTMAXSIGMA; ///< maximum allowed value for sigma1_
    fp_type sigma2_ = DEFAULTSIGMA; ///< The width of the second gaussian used to adapt values
    fp_type sigmaSigma2_ = DEFAULTSIGMASIGMA; ///< affects sigma2_ adaption
    fp_type minSigma2_ = DEFAULTMINSIGMA; ///< minimum allowed value for sigma2_
    fp_type maxSigma2_ = DEFAULTMAXSIGMA; ///< maximum allowed value for sigma2_
    fp_type delta_ = DEFAULTDELTA; ///< The distance between both gaussians
    fp_type sigmaDelta_ = DEFAULTSIGMADELTA; ///< affects the adaption of delta_
    fp_type minDelta_ = DEFAULTMINDELTA; ///< minimum allowed value for delta_
    fp_type maxDelta_ = DEFAULTMAXDELTA; ///< maximum allowed value for delta_

    Gem::Hap::bi_normal_distribution<fp_type>
        m_bi_normal_distribution; ///< Access to random numbers with a bi_normal distribution

    /***************************************************************************/
    /**
     * This function loads the data of another GNumBiGaussAdaptorT, camouflaged as a GObject.
     * We assume that the values given to us by the other object are correct and do no error checks.
     *
     * @param A copy of another GNumBiGaussAdaptorT, camouflaged as a GObject
     */
    void load_(const GObject *cp) override {
        // Check that we are dealing with a GNumBiGaussAdaptorT<num_type, fp_type> reference independent of this object and convert the pointer
        const GNumBiGaussAdaptorT<num_type, fp_type>
            *p_load = Gem::Common::g_convert_and_compare<GObject, GNumBiGaussAdaptorT<num_type, fp_type>>(
            cp
            , this
        );

        // Load the data of our parent class ...
        GAdaptorT<num_type>::load_(cp);

        // ... and then our own data
        useSymmetricSigmas_ = p_load->useSymmetricSigmas_;
        sigma1_ = p_load->sigma1_;
        sigmaSigma1_ = p_load->sigmaSigma1_;
        minSigma1_ = p_load->minSigma1_;
        maxSigma1_ = p_load->maxSigma1_;
        sigma2_ = p_load->sigma2_;
        sigmaSigma2_ = p_load->sigmaSigma2_;
        minSigma2_ = p_load->minSigma2_;
        maxSigma2_ = p_load->maxSigma2_;
        delta_ = p_load->delta_;
        sigmaDelta_ = p_load->sigmaDelta_;
        minDelta_ = p_load->minDelta_;
        maxDelta_ = p_load->maxDelta_;
    }

    /***************************************************************************/
    /** @brief Allow access to this classes compare_ function */
    friend void Gem::Common::compare_base_t<GNumBiGaussAdaptorT<num_type, fp_type>>(
        GNumBiGaussAdaptorT<num_type, fp_type> const &
        , GNumBiGaussAdaptorT<num_type, fp_type> const &
        , Gem::Common::GToken &
    );

    /***************************************************************************/
    /**
     * Searches for compliance with expectations with respect to another object
     * of the same type
     *
     * @param cp A constant reference to another GObject object
     * @param e The expected outcome of the comparison
     * @param limit The maximum deviation for floating point values (important for similarity checks)
     */
    void compare_(
        const GObject &cp
        , const Gem::Common::expectation &e
        , const fp_type &limit
    ) const override {
        using namespace Gem::Common;

        // Check that we are dealing with a GNumBiGaussAdaptorT<num_type, fp_type> reference independent of this object and convert the pointer
        const GNumBiGaussAdaptorT<num_type, fp_type>
            *p_load = Gem::Common::g_convert_and_compare<GObject, GNumBiGaussAdaptorT<num_type, fp_type>>(
            cp
            , this
        );

        GToken token(
            "GNumBiGaussAdaptorT<num_type, fp_type>"
            , e
        );

        // Compare our parent data ...
        Gem::Common::compare_base_t<GAdaptorT<num_type>>(
            *this
            , *p_load
            , token
        );

        // ... and then the local data
        compare_t(
            IDENTITY(useSymmetricSigmas_
                     , p_load->useSymmetricSigmas_)
            , token
        );
        compare_t(
            IDENTITY(sigma1_
                     , p_load->sigma1_)
            , token
        );
        compare_t(
            IDENTITY(sigmaSigma1_
                     , p_load->sigmaSigma1_)
            , token
        );
        compare_t(
            IDENTITY(minSigma1_
                     , p_load->minSigma1_)
            , token
        );
        compare_t(
            IDENTITY(maxSigma1_
                     , p_load->maxSigma1_)
            , token
        );
        compare_t(
            IDENTITY(sigma2_
                     , p_load->sigma2_)
            , token
        );
        compare_t(
            IDENTITY(sigmaSigma2_
                     , p_load->sigmaSigma2_)
            , token
        );
        compare_t(
            IDENTITY(minSigma2_
                     , p_load->minSigma2_)
            , token
        );
        compare_t(
            IDENTITY(maxSigma2_
                     , p_load->maxSigma2_)
            , token
        );
        compare_t(
            IDENTITY(delta_
                     , p_load->delta_)
            , token
        );
        compare_t(
            IDENTITY(sigmaDelta_
                     , p_load->sigmaDelta_)
            , token
        );
        compare_t(
            IDENTITY(minDelta_
                     , p_load->minDelta_)
            , token
        );
        compare_t(
            IDENTITY(maxDelta_
                     , p_load->maxDelta_)
            , token
        );

        // React on deviations from the expectation
        token.evaluate();
    }

    /***************************************************************************/
    /**
     * Adds a given property value to the vector or returns false, if the property
     * was not found.
     */
    bool customQueryProperty(
        const std::string &property
        , std::vector<boost::any> &data
    ) const override {
        if (property == "sigma1") {
            data.push_back(boost::any(sigma1_));
        } else if (property == "sigma2") {
            data.push_back(boost::any(sigma1_));
        } else if (property == "delta") {
            data.push_back(boost::any(delta_));
        } else {
            return false;
        }

        return true;
    }

    /***************************************************************************/
    /**
     * This adaptor allows the evolutionary adaption of sigma_. This allows the
     * algorithm to adapt to changing geometries of the quality surface.
     *
     * @param range A typical range for the parameter with type num_type (unused here)
     */
    void customAdaptAdaption(
        const num_type &
        , Gem::Hap::GRandomBase &gr
    ) override {
        using namespace Gem::Common;
        using namespace Gem::Hap;

        // The following random distribution slightly favours values < 1. Selection pressure
        // will keep the values higher if needed
        sigma1_ *= gexp(
            GAdaptorT<num_type>::m_normal_distribution(
                gr
                , typename std::normal_distribution<fp_type>::param_type(
                    0.
                    , gfabs(sigmaSigma1_))));
        sigma2_ *= gexp(
            GAdaptorT<num_type>::m_normal_distribution(
                gr
                , typename std::normal_distribution<fp_type>::param_type(
                    0.
                    , gfabs(sigmaSigma2_))));
        delta_ *= gexp(
            GAdaptorT<num_type>::m_normal_distribution(
                gr
                , typename std::normal_distribution<fp_type>::param_type(
                    0.
                    , gfabs(sigmaDelta_))));

        // Make sure valued don't get out of range
        enforceRangeConstraint(
            sigma1_
            , minSigma1_
            , maxSigma1_
            , "GNumBiGaussAdaptorT<>::customAdaptAdaption() / 1"
        );
        enforceRangeConstraint(
            sigma2_
            , minSigma2_
            , maxSigma2_
            , "GNumBiGaussAdaptorT<>::customAdaptAdaption() / 2"
        );
        enforceRangeConstraint(
            delta_
            , minDelta_
            , maxDelta_
            , "GNumBiGaussAdaptorT<>::customAdaptAdaption() / 3"
        );
    }

    /***************************************************************************/
    /**
     * The actual adaption of the supplied value takes place here. Purely virtual, as the actual
     * adaptions are defined in the derived classes.
     *
     * @param value The value that is going to be adapted in situ
     * @param range A typical range for the parameter with type num_type (unused here)
     */
    void customAdaptions(
        num_type &
        , const num_type &
        , Gem::Hap::GRandomBase &gr
    ) override = 0;

    /***************************************************************************/
    /**
     * Applies modifications to this object. This is needed for testing purposes
     *
     * @return A boolean which indicates whether modifications were made
     */
    bool modify_GUnitTests_() override {
#ifdef GEM_TESTING
        using boost::unit_test_framework::test_suite;
        using boost::unit_test_framework::test_case;

        bool result = false;

        // Call the parent classes' functions
        if (GAdaptorT<num_type>::modify_GUnitTests_()) { result = true; }

        // A relatively harmless change
        sigmaSigma1_ *= 1.1;
        result = true;

        return result;

#else /* GEM_TESTING */  // If this function is called when GEM_TESTING isn't set, throw
        Gem::Common::condnotset("GNumBiGaussAdaptorT<>::modify_GUnitTests", "GEM_TESTING");
       return false;
#endif /* GEM_TESTING */
    }

    /***************************************************************************/
    /**
     * Performs self tests that are expected to succeed. This is needed for testing purposes
     */
    void specificTestsNoFailureExpected_GUnitTests_() override {
#ifdef GEM_TESTING
        using boost::unit_test_framework::test_suite;
        using boost::unit_test_framework::test_case;

        // Call the parent classes' functions
        GAdaptorT<num_type>::specificTestsNoFailureExpected_GUnitTests_();

#else /* GEM_TESTING */  // If this function is called when GEM_TESTING isn't set, throw
        Gem::Common::condnotset("GNumBiGaussAdaptorT<>::specificTestsNoFailureExpected_GUnitTests", "GEM_TESTING");
#endif /* GEM_TESTING */
    }

    /***************************************************************************/
    /**
     * Performs self tests that are expected to fail. This is needed for testing purposes
     */
    void specificTestsFailuresExpected_GUnitTests_() override {
#ifdef GEM_TESTING
        using boost::unit_test_framework::test_suite;
        using boost::unit_test_framework::test_case;

        // Call the parent classes' functions
        GAdaptorT<num_type>::specificTestsFailuresExpected_GUnitTests_();

#else /* GEM_TESTING */  // If this function is called when GEM_TESTING isn't set, throw
        Gem::Common::condnotset("GNumBiGaussAdaptorT<>::specificTestsFailuresExpected_GUnitTests", "GEM_TESTING");
#endif /* GEM_TESTING */
    }

private:
    /***********************************************************************************/
    /** @brief Retrieves the id of the adaptor */
    Gem::Geneva::adaptorId getAdaptorId_() const override = 0;

    /***************************************************************************/
    /**
     * Emits a name for this class / object
     */
    std::string name_() const override {
        return std::string("GNumBiGaussAdaptorT");
    }

    /***************************************************************************/
    /** @brief This function creates a deep copy of this object */
    GObject *clone_() const override = 0;
};

/******************************************************************************/

} /* namespace Geneva */
} /* namespace Gem */

/******************************************************************************/
// The content of BOOST_SERIALIZATION_ASSUME_ABSTRACT(T)
namespace boost {
namespace serialization {
template<typename num_type, typename fp_type>
struct is_abstract<Gem::Geneva::GNumBiGaussAdaptorT<num_type, fp_type>> :
    public boost::true_type
{
};
template<typename num_type, typename fp_type>
struct is_abstract<const Gem::Geneva::GNumBiGaussAdaptorT<num_type, fp_type>> :
    public boost::true_type
{
};
}
}

/******************************************************************************/

