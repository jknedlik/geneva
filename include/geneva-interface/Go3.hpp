#include <memory>
#include <stdexcept>
#include <variant>
#include <vector>

#include "geneva-interface/GenericIndividual.tcc"
#include "geneva/Go2.hpp"
namespace GO3 {

enum Algorithms { EA, GD };
namespace cfg {
template <size_t I>
using ic = std::integral_constant<std::size_t, I>;
static inline constexpr ic<0> Size {};
static inline constexpr ic<0> Iterations {};
static inline constexpr ic<1> test {};
static inline constexpr ic<2> x {};
}  // namespace cfg
using namespace Gem::Geneva;
template <typename... Ts>
class integralconfig
{
protected:
public:
    std::tuple<Ts...> cfg;
    template <typename Integral>
    constexpr auto & operator[]( Integral )
    {
        return std::get<Integral::value>( cfg );
    }
    integralconfig( std::tuple<Ts...> ts ) : cfg( ts ) {};
};
using popconf = integralconfig<int, int, std::string, double>;
template <typename Function>
class Population : public popconf
{
public:
    using Individual = GenericIndividual<Function>;
    using Factory    = GenericIndividualFactory<Function>;
    std::shared_ptr<Factory> factory_ptr;
    Population( std::vector<double> start,
                std::vector<double> left,
                std::vector<double> right,
                Function &          func,
                int                 Size       = 100,
                int                 NumParents = 5 )
        : Population( start, left, right, std::forward<Function>( func ), Size, NumParents )
    {
    }
    Population( std::vector<double> start,
                std::vector<double> left,
                std::vector<double> right,
                Function &&         func,
                int                 Size       = 100,
                int                 NumParents = 5 )
        : factory_ptr(
            new Factory( "config/GenericIndividual.json", start, left, right, std::forward<decltype( func )>( func ) ) )
        , popconf( { Size, NumParents, "testfile", 0.0 } )
    {
        Individual::setFunc( std::forward<Function>( func ) );
    }
    auto
    get()
    {
        return factory_ptr;
    };
};
using algoconf = integralconfig<int, int>;

template <typename FactoryT, typename AlgoT>
class Algorithm : public algoconf
{
public:
    Algorithm( int Iterations = 5 ) : algoconf( { Iterations, 2 } )
    {
    }
    std::shared_ptr<AlgoT>
    get()
    {
        std::shared_ptr<AlgoT> algo = FactoryT().template get<AlgoT>();
        algo->setMaxIteration( ( *this )[cfg::Iterations] );
        return algo;
    }
};
using Algorithm_EA = Algorithm<GEvolutionaryAlgorithmFactory, GEvolutionaryAlgorithm>;
using Algorithm_GD = Algorithm<GGradientDescentFactory, GGradientDescent>;

using algorithmsT = std::vector<std::variant<Algorithm_EA, Algorithm_GD>>;
class GenevaOptimizer3
{

public:
    Go2 go;
    GenevaOptimizer3( int argc, char ** argv ) : go( argc, argv, "config/Go2.json" ) {};

    template <typename Population>
    auto
    optimize( Population & pop, algorithmsT algos = { Algorithm_EA() } )
    {
        if ( go.clientMode() ) go.clientRun();

        int currMaxIteration = 0;
        for ( auto algo: algos )
            std::visit(
                [&]( auto & al ) {
                    using T  = std::decay_t<decltype( al )>;
                    auto alg = al.get();
                    if constexpr ( std::is_same_v<T, Algorithm_EA> ) alg->setPopulationSizes( pop[cfg::Size], 5 );
                    /* User expects each algo to add its
	     * iterations to the process*/
                    alg->setMaxIteration( currMaxIteration += al[cfg::Iterations] );
                    alg->setMaxStallIteration( currMaxIteration );
                    go & alg;
                },
                algo );

        go.registerContentCreator( pop.get() );
        using Individual = typename Population::Individual;
        return go.optimize()->getBestGlobalIndividual<Individual>();
    }
};

}  // namespace GO3
