#ifndef GOPTIONS_GENEVAOPT
#define GOPTIONS_GENEVAOPT
#include "boost/bind.hpp"
#include "boost/program_options.hpp"
#include "boost/program_options/options_description.hpp"
#include "boost/program_options/variables_map.hpp"
#include "boost/system/error_code.hpp"
#include "common/GGlobalDefines.hpp"
#include <iostream>
#define GOSTRING(s) GSTRING(s)
#define GSTRING(s) #s

namespace GenevaOpt {
bool GIsClient = false;
bool revalBest = false;
double pertubationScalar = 0;
std::string xpname = "";
std::string cpname = "";

using variable_t = boost::program_options::variable_value;
class OptionRegistry {
  OptionRegistry() : handled(false){};
  boost::program_options::variables_map parsemap;
  bool handled;

  public:
  std::vector<boost::program_options::options_description> options;
  std::vector<std::function<void(boost::program_options::variables_map &)>>
      handlers;
  static std::shared_ptr<OptionRegistry> registry;
  void handle(int argc, char **argv)
  {
    boost::program_options::options_description desc;
    for (auto a : options) desc.add(a);

    try {
      boost::program_options::store(
          boost::program_options::parse_command_line(argc, argv, desc),
          parsemap);
      boost::program_options::notify(parsemap);
    }

    catch (boost::program_options::unknown_option e) {
      std::cout << "unknown Option, " << e.what() << desc << std::endl;
      exit(0);
    }
    // real handle
    for (auto handler : handlers) handler(parsemap);
    handled = true;
  }
  std::vector<std::string> flatMap()
  {
    std::vector<std::string> flat;
    if (handled) {
      for (auto it : parsemap) {
        flat.push_back(std::string("--").append(it.first));
        try {
          if (!it.second.empty()) flat.push_back(it.second.as<std::string>());
        }
        catch (...) {
        }
        try {
          if (!it.second.empty()) {
            for (auto a : it.second.as<std::vector<std::string>>())
              flat.push_back(a);
          }
        }
        catch (...) {
        }
      }

      for (auto it : flat) std::cout << it << " ";
      std::cout << std::endl;
    }
    return flat;
  }
};
std::shared_ptr<OptionRegistry> OptionRegistry::registry = []() {
  std::shared_ptr<OptionRegistry> reg(new OptionRegistry());
  (*reg).options.push_back([]() {
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()("help,h", "show geneva help")(
        "optimizationAlgorithms,a",
        boost::program_options::value<std::vector<std::string>>(),
        "--help to get Geneva parameter info")(
        "executionMode,e", boost::program_options::value<std::string>(),
        "--help to get Geneva parameter info")(
        "client", "--help to get Geneva parameter info")(
        "beast", "use geneva websocket implementatins(boost::beast)")(
        "revalBest", "--help to get Geneva parameter info")(
        "consumer,c", boost::program_options::value<std::string>(),
        "--help to get Geneva parameter info")(
        "pertubation", boost::program_options::value<double>(),
        "creates a pertubation run, set to the double value of the scalar for "
        "sigma*scalar  and bound*scalar")(
        "stcpc_ip", boost::program_options::value<std::string>(),
        "--help to get Geneva parameter info")(
        "ip,i", boost::program_options::value<std::string>(),
        "--help to get Geneva parameter info")(
        "stcpc_port", boost::program_options::value<std::string>(),
        "--help to get Geneva parameter info")(
        "port,p", boost::program_options::value<std::string>(),
        "--help to get Geneva parameter info")(
        "serializationMode,s", boost::program_options::value<std::string>(),
        "--help to get Geneva parameter info")(
        "maxStalls", boost::program_options::value<std::string>(),
        "--help to get Geneva parameter info")(
        "maxConnectionAttempts", boost::program_options::value<std::string>(),
        "--help to get Geneva parameter info")(
        "returnRegardless", boost::program_options::value<std::string>(),
        "--help to get Geneva parameter info")(
        "nListenerThreads", boost::program_options::value<std::string>(),
        "--help to get Geneva parameter info")(
        "maxClientDuration", boost::program_options::value<std::string>(),
        "--help to get Geneva parameter info")(
        "cp_file", boost::program_options::value<std::string>(),
        "--help to get Geneva parameter info")(
        "xp_file", boost::program_options::value<std::string>(),
        "--help to get Geneva parameter info")(
        "parameterSpec",
        boost::program_options::value<std::vector<std::string>>(),
        "--help to get Geneva parameter info");
    return desc;
  }());
  (*reg).handlers.push_back(
      [](boost::program_options::variables_map &parsemap) {
        std::string ip{"-i"}, port{"-p"}, consumer{"tcpc"},
            lthreads("--nListenerThreads");
        //	std::cout << "Geneva version string: " <<
        // GOSTRING(GENEVA_VERSION)
        //		  << " extracted version:" <<
        // atoi(GOSTRING(GENEVA_VERSION))
        //		  << std::endl;
        if (parsemap.count("beast")) {
          ip = "beast_ip";
          port = "beast_port";
          consumer = "beast";
          lthreads = "beast_nListenerThreads";
          parsemap.erase("beast");
        }
        else {
          ip = "asio_ip";
          port = "asio_port";
          consumer = "asio";
          lthreads = "stcpc_nListenerThreads";
        }
        if (parsemap.count("xp_file")) {
          xpname = parsemap["xp_file"].as<std::string>();
          parsemap.erase("xp_file");
        }
        if (parsemap.count("cp_file")) {
          cpname = parsemap["cp_file"].as<std::string>();
          parsemap.erase("cp_file");
        }

        if (parsemap.count("pertubation")) {
          std::cout << "pertubation inside" << std::endl;
          pertubationScalar = parsemap["pertubation"].as<double>();
          parsemap.erase("pertubation");
        }
        if (parsemap.count("executionMode")) {
          if (parsemap["executionMode"].as<std::string>().compare("1") == 0) {
            consumer = "sc";
            parsemap.emplace("consumer", variable_t(consumer, false));
          }
          parsemap.erase("executionMode");
        }
        if (parsemap.count("consumer")) {
          // if ((*svec.rbegin()).compare("tcpc") == 0) *svec.rbegin() =
          if (parsemap["consumer"].as<std::string>().compare("tcpc") == 0) {
            parsemap.erase("consumer");
            parsemap.emplace("consumer", variable_t(consumer, false));
          }
        }
        else {
          consumer = "sc";
          parsemap.emplace("consumer", variable_t(consumer, false));
        }
        if (parsemap.count("stcpc_ip")) {
          parsemap.emplace(ip, variable_t(parsemap["stcpc_ip"].value(), false));
          parsemap.erase("stcpc_ip");
        }
        if (parsemap.count("ip")) {
          parsemap.emplace(ip, variable_t(parsemap["ip"].value(), false));
          parsemap.erase("ip");
        }
        if (parsemap.count("client")) {
          GIsClient = true;
        }
        if (parsemap.count("revalBest")) {
          revalBest = true;
          parsemap.erase("revalBest");
        }
        if (parsemap.count("port")) {
          parsemap.emplace(port, variable_t(parsemap["port"].value(), false));
          parsemap.erase("port");
        }
        if (parsemap.count("stcpc_port")) {
          parsemap.emplace(port,
                           variable_t(parsemap["stcpc_port"].value(), false));
          parsemap.erase("stcpc_port");
        }
        if (parsemap.count("nListenerThreads")) {
          parsemap.emplace(
              lthreads,
              variable_t(parsemap["nListenerThreads"].value(), false));
          parsemap.erase("nListenerThreads");
        }
        if (parsemap.count("stcpc_nListenerThreads")) {
          parsemap.emplace(
              lthreads,
              variable_t(parsemap["stcpc_nListenerThreads"].value(), false));
          parsemap.erase("stcpc_nListenerThreads");
        }
      });

  return reg;
}();

class GPairOptions {
  bool was_handled;
  std::vector<std::pair<std::string, int *>> parlist;

  public:
  GPairOptions(std::vector<std::pair<std::string, int *>> par) : parlist(par)
  {
    boost::program_options::options_description desc;
    for (auto it : parlist) {
      desc.add_options()(it.first.c_str(), boost::program_options::value<int>(),
                         it.first.c_str());
    }
    OptionRegistry::registry->options.push_back(desc);
    OptionRegistry::registry->handlers.push_back(
        [this](boost::program_options::variables_map &map) {
          for (auto it : parlist) {
            if (map.count(it.first.c_str())) {
              *(it.second) = map[it.first.c_str()].as<int>();
              map.erase(it.first.c_str());
            }
          }
        });
  }
};
} // namespace GenevaOpt
#endif
