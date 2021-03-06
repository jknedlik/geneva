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
#include <fstream>
#include <memory>

// Boost headers go here

// Geneva headers go here
#include "hap/GRandomT.hpp"
#include "common/GExceptions.hpp"
#include "common/GCommonHelperFunctionsT.hpp"
#include "common/GPlotDesigner.hpp"
#include "common/GSerializeTupleT.hpp"
#include "common/GPODVectorT.hpp"
#include "geneva/GObject.hpp"
#include "geneva/GParameterPropertyParser.hpp"
#include "geneva/G_OptimizationAlgorithm_Base.hpp"
#include "geneva/GOptimizationEnums.hpp"
#include "geneva/GParameterSet.hpp"
#include "geneva/G_OptimizationAlgorithm_ParameterScan_PersonalityTraits.hpp"

namespace Gem {
namespace Geneva {

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/** Indicates that all possible parameter values have been explored */
class g_end_of_par :
    public std::exception
{
public:
    using std::exception::exception;
};

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/**
 * This function fills a given std::vector<T> with items. It needs to be re-implemented
 * in concrete specializations. This generic function is just a trap.
 */
template<typename T>
std::vector<T> fillWithData(
    std::size_t /*nSteps*/
    , T /* lower */
    , T /* upper */
) {
    throw gemfony_exception(
        g_error_streamer(
            DO_LOG
            , time_and_place
        )
            << "In generic function template <typename T> std::vector<T> fillWithData(): Error!" << std::endl
            << "This function should never be called directly. Use one of the specializations." << std::endl
    );

    // Make the compiler happy
    return std::vector<T>();
}

template<> G_API_GENEVA
std::vector<bool> fillWithData<bool>(
    std::size_t nSteps
    , bool lower
    , bool upper
);

template<> G_API_GENEVA
std::vector<std::int32_t> fillWithData<std::int32_t>(
    std::size_t nSteps // will only be used for random entries
    , std::int32_t lower
    , std::int32_t upper // inclusive
);

template<> G_API_GENEVA
std::vector<float> fillWithData<float>(
    std::size_t nSteps
    , float lower
    , float upper
);

template<> G_API_GENEVA
std::vector<double> fillWithData<double>(
    std::size_t nSteps
    , double lower
    , double upper
);

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/**
 * An interface class for parameter scan objects
 */
class scanParInterface
{
public:
    virtual G_API_GENEVA ~scanParInterface() BASE = default;
    virtual G_API_GENEVA NAMEANDIDTYPE getVarAddress() const BASE = 0;
    virtual G_API_GENEVA bool goToNextItem() BASE = 0;
    virtual G_API_GENEVA bool isAtTerminalPosition() const BASE = 0;
    virtual G_API_GENEVA bool isAtFirstPosition() const BASE = 0;
    virtual G_API_GENEVA void resetPosition() BASE = 0;
    virtual G_API_GENEVA std::string getTypeDescriptor() const BASE = 0;
};

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/**
 * Basic parameter functionality
 */
template<typename T>
class baseScanParT
    :
        public Gem::Common::GPODVectorT<T>
        , public scanParInterface
{
    ///////////////////////////////////////////////////////////////////////
    friend class boost::serialization::access;

    template<typename Archive>
    void serialize(Archive &ar, const unsigned int) {
        using boost::serialization::make_nvp;

        ar
        & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Gem::Common::GPODVectorT<T>)
        & BOOST_SERIALIZATION_NVP(m_var)
        & BOOST_SERIALIZATION_NVP(m_step)
        & BOOST_SERIALIZATION_NVP(m_nSteps)
        & BOOST_SERIALIZATION_NVP(m_lower)
        & BOOST_SERIALIZATION_NVP(m_upper)
        & BOOST_SERIALIZATION_NVP(m_randomScan)
        & BOOST_SERIALIZATION_NVP(m_typeDescription);
    }

    ///////////////////////////////////////////////////////////////////////

public:
    /***************************************************************************/
    /**
     * The standard constructor
     */
    baseScanParT(
        parPropSpec<T> pps
        , bool randomScan
        , const std::string &t // m_typeDescription
    )
        :
        Gem::Common::GPODVectorT<T>()
        , m_var(pps.var)
        , m_step(0)
        , m_nSteps(pps.nSteps)
        , m_lower(pps.lowerBoundary)
        , m_upper(pps.upperBoundary)
        , m_randomScan(randomScan)
        , m_typeDescription(t) {
        if (not m_randomScan) {
            // Fill the object with data
            this->m_data_cnt = fillWithData<T>(
                m_nSteps
                , m_lower
                , m_upper
            );
        }
    }

    /***************************************************************************/
    /**
     * Copy constructor. Not defaulted, so we can avoid copying of the
     * random number generator.
     */
    baseScanParT(const baseScanParT<T> &cp)
        : m_var(cp.m_var)
        , m_step(cp.m_step)
        , m_nSteps(cp.m_step)
        , m_lower(cp.m_lower)
        , m_upper(cp.m_upper)
        , m_randomScan(cp.m_randomScan)
        , m_typeDescription(cp.m_typeDescription)
    { /* nothing */ }

    /***************************************************************************/
    /**
     * The destructor
     */
    ~baseScanParT() override = default;

    /***************************************************************************/
    /**
     * Retrieve the address of this object
     */
    NAMEANDIDTYPE getVarAddress() const override {
        return m_var;
    }

    /***************************************************************************/
    /**
     * Retrieves the current item position
     */
    std::size_t getCurrentItemPos() const {
        return m_step;
    }

    /***************************************************************************/
    /**
     * Retrieve the current item
     */
    T getCurrentItem(
        Gem::Hap::GRandomBase &gr
    ) const {
        if (m_randomScan) {
            return getRandomItem(gr);
        } else {
            return this->at(m_step);
        }
    }

    /***************************************************************************/
    /**
     * Switch to the next position in the vector or rewind
     *
     * @return A boolean indicating whether a warp has taken place
     */
    bool goToNextItem() override {
        if (++m_step >= m_nSteps) {
            m_step = 0;
            return true;
        }
        return false;
    }

    /***************************************************************************/
    /**
     * Checks whether step_ points to the last item in the array
     */
    bool isAtTerminalPosition() const override {
        return m_step >= m_nSteps;
    }

    /***************************************************************************/
    /**
     * Checks whether step_ points to the first item in the array
     */
    bool isAtFirstPosition() const override {
        return 0 == m_step;
    }

    /***************************************************************************/
    /**
     * Resets the current position
     */
    void resetPosition() override {
        m_step = 0;
    }

    /***************************************************************************/
    /**
     * Retrieve the type descriptor
     */
    std::string getTypeDescriptor() const override {
        return m_typeDescription;
    }

protected:
    /***************************************************************************/
    // Data

    NAMEANDIDTYPE m_var; ///< Name and/or position of the variable
    std::size_t m_step; ///< The current position in the data vector
    std::size_t m_nSteps; ///< The number of steps to be taken in a scan
    T m_lower; ///< The lower boundary of an item
    T m_upper; ///< The upper boundary of an item
    bool m_randomScan; ///< Indicates whether we are dealing with a random scan or not
    std::string m_typeDescription; ///< Holds an identifier for the type described by this class

    mutable Gem::Hap::GRandom m_gr; ///< Simple access to a random number generator

    /***************************************************************************/
    /** @brief The default constructor -- only needed for de-serialization, hence protected */
    baseScanParT()
        :
        m_var(
            NAMEANDIDTYPE(
                0
                , "empty"
                , 0
            ))
        , m_step(0)
        , m_nSteps(2)
        , m_lower(T(0))
        , m_upper(T(1))
        , m_randomScan(true)
        , m_typeDescription("")
    { /* nothing */ }

    /***************************************************************************/
    /**
     * Retrieves a random item. To be re-implemented for each supported type
     */
    T getRandomItem(
        Gem::Hap::GRandomBase &gr
    ) const {
        // A trap. This function needs to be re-implemented for each supported type
        throw gemfony_exception(
            g_error_streamer(
                DO_LOG
                , time_and_place
            )
                << "In baseScanParT::getRandomItem(): Error!" << std::endl
                << "Function called for unsupported type" << std::endl
        );

        // Make the compiler happy
        return T(0);
    }

private:
    mutable std::bernoulli_distribution m_uniform_bool; ///< boolean random numbers with an even distribution
    mutable std::uniform_real_distribution<float> m_uniform_float_distribution;  ///< Uniformly distributed fp numbers
    mutable std::uniform_real_distribution<double> m_uniform_double_distribution; ///< Uniformly distributed fp numbers
    mutable std::uniform_int_distribution<std::int32_t>
        m_uniform_int_distribution; ///< Uniformly distributed integer numbers
};

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/**
 * Retrieval of a random value for type bool
 */
template<>
inline bool baseScanParT<bool>::getRandomItem(
    Gem::Hap::GRandomBase &gr
) const {
    return m_uniform_bool(gr);
}

/******************************************************************************/
/**
 * Retrieval of a random value for type float
 */
template<>
inline float baseScanParT<float>::getRandomItem(
    Gem::Hap::GRandomBase &gr
) const {
    return m_uniform_float_distribution(
        gr
        , std::uniform_real_distribution<float>::param_type(
            m_lower
            , m_upper
        ));
}

/******************************************************************************/
/**
 * Retrieval of a random value for type double
 */
template<>
inline double baseScanParT<double>::getRandomItem(
    Gem::Hap::GRandomBase &gr
) const {
    return m_uniform_double_distribution(
        gr
        , std::uniform_real_distribution<double>::param_type(
            m_lower
            , m_upper
        ));
}

/******************************************************************************/
/**
 * Retrieval of a random value for type std::int32_t
 */
template<>
inline std::int32_t baseScanParT<std::int32_t>::getRandomItem(
    Gem::Hap::GRandomBase &gr
) const {
    return m_uniform_int_distribution(
        gr
        , std::uniform_int_distribution<std::int32_t>::param_type(
            m_lower
            , m_upper + 1
        ));
}

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/**
 * This class holds boolean parameters
 */
class bScanPar
    :
        public baseScanParT<bool>
{
    ///////////////////////////////////////////////////////////////////////
    friend class boost::serialization::access;

    template<typename Archive>
    void serialize(Archive &ar, const unsigned int) {
        using boost::serialization::make_nvp;

        ar
        & BOOST_SERIALIZATION_BASE_OBJECT_NVP(baseScanParT<bool>);
    }

    ///////////////////////////////////////////////////////////////////////

public:
    /** @brief Construction from local variables */
    G_API_GENEVA bScanPar(
        parPropSpec<bool>
        , bool
    );
    /** @brief Copy constructor */
    G_API_GENEVA bScanPar(const bScanPar &) = default;
    /** @brief The destructor */
    G_API_GENEVA ~bScanPar() override = default;

    /** @brief Cloning of this object */
    G_API_GENEVA std::shared_ptr<bScanPar> clone() const;

private:
    /** @brief The default constructor -- only needed for de-serialization, hence private */
    G_API_GENEVA bScanPar();
};

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/**
 * A derivative of baseScanParT for std::int32_t values
 */
class int32ScanPar
    :
        public baseScanParT<std::int32_t>
{
    ///////////////////////////////////////////////////////////////////////
    friend class boost::serialization::access;

    template<typename Archive>
    void serialize(Archive &ar, const unsigned int) {
        using boost::serialization::make_nvp;

        ar
        & BOOST_SERIALIZATION_BASE_OBJECT_NVP(baseScanParT<std::int32_t>);
    }

    ///////////////////////////////////////////////////////////////////////

public:
    /** @brief The standard destructor */
    G_API_GENEVA int32ScanPar(
        parPropSpec<std::int32_t>
        , bool
    );
    /** @brief Copy constructor */
    G_API_GENEVA int32ScanPar(const int32ScanPar &) = default;
    /** @brief The destructor */
    G_API_GENEVA ~int32ScanPar() override = default;

    /** @brief Cloning of this object */
    G_API_GENEVA std::shared_ptr<int32ScanPar> clone() const;

private:
    /** @brief The default constructor -- only needed for de-serialization, hence private */
    G_API_GENEVA int32ScanPar();
};

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/**
 * A derivative of fpScanParT for double values
 */
class dScanPar
    :
        public baseScanParT<double>
{
    ///////////////////////////////////////////////////////////////////////
    friend class boost::serialization::access;

    template<typename Archive>
    void serialize(Archive &ar, const unsigned int) {
        using boost::serialization::make_nvp;

        ar
        & BOOST_SERIALIZATION_BASE_OBJECT_NVP(baseScanParT<double>);
    }

    ///////////////////////////////////////////////////////////////////////

public:
    /** @brief The standard destructor */
    G_API_GENEVA dScanPar(
        parPropSpec<double>
        , bool
    );
    /** @brief The copy constructor */
    G_API_GENEVA dScanPar(const dScanPar &) = default;
    /** @brief The destructor */
    G_API_GENEVA ~dScanPar() override = default;

    /** @brief Cloning of this object */
    G_API_GENEVA std::shared_ptr<dScanPar> clone() const;

private:
    /** @brief The default constructor -- only needed for de-serialization, hence private */
    G_API_GENEVA dScanPar();
};


/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/**
 * A derivative of fpScanParT for float values
 */
class fScanPar
    :
        public baseScanParT<float>
{
    ///////////////////////////////////////////////////////////////////////
    friend class boost::serialization::access;

    template<typename Archive>
    void serialize(Archive &ar, const unsigned int) {
        using boost::serialization::make_nvp;

        ar
        & BOOST_SERIALIZATION_BASE_OBJECT_NVP(baseScanParT<float>);
    }

    ///////////////////////////////////////////////////////////////////////

public:
    /** @brief The standard destructor */
    G_API_GENEVA fScanPar(
        parPropSpec<float>
        , bool
    );
    /** @brief The copy constructor */
    G_API_GENEVA fScanPar(const fScanPar &) = default;
    /** @brief The destructor */
    G_API_GENEVA ~fScanPar() override = default;

    /** @brief Cloning of this object */
    G_API_GENEVA std::shared_ptr<fScanPar> clone() const;

private:
    /** @brief The default constructor -- only needed for de-serialization, hence private */
    G_API_GENEVA fScanPar();
};

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/


/******************************************************************************/
// A number of typedefs that indicate the position and value of a parameter inside of an individual
using singleBPar     = std::tuple<bool, std::size_t, std::string, std::size_t>;
using singleInt32Par = std::tuple<std::int32_t, std::size_t, std::string, std::size_t>;
using singleFPar     = std::tuple<float, std::size_t, std::string, std::size_t>;
using singleDPar     = std::tuple<double, std::size_t, std::string, std::size_t>;

/******************************************************************************/
/**
 * This struct holds the entire data to be updated inside of an individual
 */
struct parSet
{
    std::vector<singleBPar> bParVec;
    std::vector<singleInt32Par> iParVec;
    std::vector<singleFPar> fParVec;
    std::vector<singleDPar> dParVec;
};

/******************************************************************************/
/** @brief A simple output operator for parSet object, mostly meant for debugging */
G_API_GENEVA std::ostream &operator<<(std::ostream &os, const parSet &pS);

/******************************************************************************/
/** @brief The default number of "best" individuals to be kept during the algorithm run */
const std::size_t DEFAULTNMONITORINDS = 10;

/******************************************************************************/
/**
 * This algorithm scans a given parameter range, either in a random order,
 * or on a grid. On a grid, for each integer- or floating point-coordinate to be scanned,
 * it is given the lower and upper boundaries (both inclusive) and the number
 * of steps (including the boundaries). For boolean parameters, both true and
 * false will be tested. The algorithm only takes into consideration the first
 * individual that was registered. It will be duplicated for all possible
 * combinations, and the parameters adapted as required. The algorithm will
 * decide itself about the number of iterations, based on the number of required
 * tests and the desired population size. Please note that the amount of tests
 * required grows quickly with the number of steps and parameters and can easily
 * extend beyond the range where computation still makes sense. E.g., if you
 * plan to test but 4 values for each of 100 parameters, you'd have to evaluate
 * 4^100 individuals which, at a millisecond evaluation time per individual, would
 * require approximately 7*10^49 years to compute ... (on a side note, this is
 * the very reason why optimization algorithms are needed to search for the
 * best solution). So realistically, this algorithm can only be used for small
 * numbers of parameters and steps. In random sampling mode, the algorithm will
 * try to evenly scatter random individuals throughout the parameter space (defined
 * by those parameters intended to be modified). The optimization monitor associated
 * with this class will simply store all parameters and results in an XML file.
 */
class GParameterScan
    :
        public G_OptimizationAlgorithm_Base
{
    ///////////////////////////////////////////////////////////////////////
    friend class boost::serialization::access;

    template<typename Archive>
    void serialize(Archive &ar, const unsigned int) {
        using boost::serialization::make_nvp;

        ar
        & make_nvp(
            "G_OptimizationAlgorithm_Base"
            , boost::serialization::base_object<G_OptimizationAlgorithm_Base>(*this))
        & BOOST_SERIALIZATION_NVP(m_scanRandomly)
        & BOOST_SERIALIZATION_NVP(m_nMonitorInds)
        & BOOST_SERIALIZATION_NVP(m_b_cnt)
        & BOOST_SERIALIZATION_NVP(m_int32_cnt)
        & BOOST_SERIALIZATION_NVP(m_d_cnt)
        & BOOST_SERIALIZATION_NVP(m_f_cnt)
        & BOOST_SERIALIZATION_NVP(m_simpleScanItems)
        & BOOST_SERIALIZATION_NVP(m_scansPerformed);
    }

    ///////////////////////////////////////////////////////////////////////

public:
    /** @brief The default constructor */
    G_API_GENEVA GParameterScan() = default;
    /** @brief A standard copy constructor */
    G_API_GENEVA GParameterScan(const GParameterScan &);
    /** @brief The destructor */
    G_API_GENEVA ~GParameterScan() override = default;

    /** @brief Allows to set the number of "best" individuals to be monitored over the course of the algorithm run */
    G_API_GENEVA void setNMonitorInds(std::size_t);
    /** @brief Allows to retrieve  the number of "best" individuals to be monitored over the course of the algorithm run */
    G_API_GENEVA std::size_t getNMonitorInds() const;

    /** @brief Fills vectors with parameter specifications */
    G_API_GENEVA void setParameterSpecs(std::string);

    /** @brief Puts the class in "simple scan" mode */
    G_API_GENEVA void setNSimpleScans(std::size_t);
    /** @brief Retrieves the number of simple scans (or 0, if disabled) */
    G_API_GENEVA std::size_t getNSimpleScans() const;
    /** @brief Retrieves the number of scans performed so far */
    G_API_GENEVA std::size_t getNScansPerformed() const;

    /** @brief Allows to specify whether the parameter space should be scanned randomly or on a grid */
    G_API_GENEVA void setScanRandomly(bool);
    /** @brief Allows to check whether the parameter space should be scanned randomly or on a grid */
    G_API_GENEVA bool getScanRandomly() const;

protected:
    /***************************************************************************/
    // Virtual or overridden protected functions

    /** @brief Adds local configuration options to a GParserBuilder object */
    G_API_GENEVA void addConfigurationOptions_(
        Gem::Common::GParserBuilder &gpb
    ) override;
    /** @brief Loads the data of another population */
    G_API_GENEVA void load_(const GObject *) override;

    /** @brief Allow access to this classes compare_ function */
    friend void Gem::Common::compare_base_t<GParameterScan>(
        GParameterScan const &
        , GParameterScan const &
        , Gem::Common::GToken &
    );

    /** @brief Searches for compliance with expectations with respect to another object of the same type */
    G_API_GENEVA void compare_(
        const GObject & // the other object
        , const Gem::Common::expectation & // the expectation for this object, e.g. equality
        , const double & // the limit for allowed deviations of floating point types
    ) const override;

    /** @brief Resets the settings of this population to what was configured when the optimize()-call was issued */
    G_API_GENEVA void resetToOptimizationStart_() override;

    /** @brief Does some preparatory work before the optimization starts */
    G_API_GENEVA void init() override;
    /** @brief Does any necessary finalization work */
    G_API_GENEVA void finalize() override;

    /** @brief Applies modifications to this object. This is needed for testing purposes */
    G_API_GENEVA bool modify_GUnitTests_() override;
    /** @brief Performs self tests that are expected to succeed. This is needed for testing purposes */
    G_API_GENEVA void specificTestsNoFailureExpected_GUnitTests_() override;
    /** @brief Performs self tests that are expected to fail. This is needed for testing purposes */
    G_API_GENEVA void specificTestsFailuresExpected_GUnitTests_() override;

    /***************************************************************************/

private:
    /***************************************************************************/
    // Virtual or overridden private functions

    /** @brief Emits a name for this class / object */
    G_API_GENEVA std::string name_() const override;
    /** @brief Creates a deep clone of this object */
    G_API_GENEVA GObject *clone_() const override;

    /** @brief The actual business logic to be performed during each iteration. Returns the best achieved fitness */
    G_API_GENEVA std::tuple<double, double> cycleLogic_() override;
    /** @brief Triggers fitness calculation of a number of individuals */
    G_API_GENEVA void runFitnessCalculation_() override;

    /** @brief Returns information about the type of optimization algorithm */
    G_API_GENEVA std::string getAlgorithmPersonalityType_() const override;
    /** @brief Returns the name of this optimization algorithm */
    G_API_GENEVA std::string getAlgorithmName_() const override;

    /** @brief Retrieves the number of processable items for the current iteration */
    G_API_GENEVA std::size_t getNProcessableItems_() const override;

    /** @brief A custom halt criterion for the optimization, allowing to stop the loop when no items are left to be scanned */
    G_API_GENEVA bool customHalt_() const override;

    /** @brief Resizes the population to the desired level and does some error checks */
    G_API_GENEVA void adjustPopulation_() override;
    /** @brief Retrieve a GPersonalityTraits object belonging to this algorithm */
    G_API_GENEVA std::shared_ptr<GPersonalityTraits> getPersonalityTraits_() const override;
    /** @brief Gives individuals an opportunity to update their internal structures */
    G_API_GENEVA void actOnStalls_() override;

    /***************************************************************************/
    /**
     * Adds a given data point to a data vector
     */
    template<typename data_type>
    void addDataPoint(
        const std::tuple<data_type, std::size_t, std::string, std::size_t> &dataPoint
        , std::vector<data_type> &dataVec
    ) {
#ifdef DEBUG
        if (0 != std::get<1>(dataPoint)) {
            throw gemfony_exception(
                g_error_streamer(
                    DO_LOG
                    , time_and_place
                )
                    << "In GParameterScan::addDataPoint(mode 0): Error!" << std::endl
                    << "Function was called for invalid mode " << std::get<1>(dataPoint) << std::endl
            );
        }
#endif

        data_type lData = std::get<0>(dataPoint);
        std::size_t lPos = std::get<3>(dataPoint);

        // Check that we haven't exceeded the size of the boolean data vector
        if (lPos >= dataVec.size()) {
            throw gemfony_exception(
                g_error_streamer(
                    DO_LOG
                    , time_and_place
                )
                    << "In GParameterScan::addDataPoint(): Error!" << std::endl
                    << "Got position beyond end of data vector: " << lPos << " / " << dataVec.size() << std::endl
            );
        }

        dataVec.at(lPos) = lData;
    }

    /***************************************************************************/
    /**
     * Adds a given data point to a data map
     */
    template<typename data_type>
    void addDataPoint(
        const std::tuple<data_type, std::size_t, std::string, std::size_t> &dataPoint
        , std::map<std::string, std::vector<data_type>> &dataMap
    ) {
        data_type lData = std::get<0>(dataPoint);
        std::string lName = std::get<2>(dataPoint);
        std::size_t lPos = std::get<3>(dataPoint);

        (
            Gem::Common::getMapItem(
                dataMap
                , lName
            )).at(lPos) = lData;
    }

    /***************************************************************************/
    /** @brief Resets all parameter objects */
    void resetParameterObjects();

    /** @brief Adds new parameter sets to the population */
    void updateSelectedParameters();

    /** @brief Randomly shuffle the work items a number of times */
    void randomShuffle();

    /** @brief Retrieves the next available parameter set */
    std::shared_ptr<parSet> getParameterSet(std::size_t &);

    /** @brief Switches to the next parameter set */
    bool switchToNextParameterSet();

    /** @brief Fills all parameter objects into the m_all_par_vec vector */
    void fillAllParVec();

    /** @brief Clears the m_all_par_vec vector */
    void clearAllParVec();

    bool m_cycleLogicHalt = false; ///< Temporary flag used to specify that the optimization should be halted
    bool m_scanRandomly
        = true;   ///< Determines whether the algorithm should scan the parameter space randomly or on a grid
    std::size_t m_nMonitorInds = DEFAULTNMONITORINDS; ///< The number of best individuals of the entire run to be kept

    std::vector<std::shared_ptr<bScanPar>> m_b_cnt;     ///< Holds boolean parameters to be scanned
    std::vector<std::shared_ptr<int32ScanPar>> m_int32_cnt; ///< Holds 32 bit integer parameters to be scanned
    std::vector<std::shared_ptr<dScanPar>> m_d_cnt;     ///< Holds double values to be scanned
    std::vector<std::shared_ptr<fScanPar>> m_f_cnt;     ///< Holds float values to be scanned

    std::vector<std::shared_ptr<scanParInterface>> m_all_par_cnt; /// Holds pointers to all parameter objects

    std::size_t m_simpleScanItems
        = 0; ///< When set to a value > 0, a random scan of the entire parameter space will be made instead of individual parameters -- set through the configuration file
    std::size_t m_scansPerformed = 0; ///< Holds the number of processed items so far while a simple scan is performed

    /***************************************************************************/
};

} /* namespace Geneva */
} /* namespace Gem */

BOOST_CLASS_EXPORT_KEY(Gem::Geneva::bScanPar)
BOOST_CLASS_EXPORT_KEY(Gem::Geneva::int32ScanPar)
BOOST_CLASS_EXPORT_KEY(Gem::Geneva::dScanPar)
BOOST_CLASS_EXPORT_KEY(Gem::Geneva::fScanPar)

BOOST_CLASS_EXPORT_KEY(Gem::Geneva::GParameterScan)

