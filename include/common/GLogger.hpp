/**
 * @file GLogger.hpp
 */

/*
 * Copyright (C) Gemfony scientific UG (haftungsbeschraenkt)
 *
 * This file is part of the Geneva library collection.
 *
 * The following license applies to the code in this file:
 *
 * ***************************************************************************
 *
 * Boost Software License - Version 1.0 - August 17th, 2003
 *
 * Permission is hereby granted, free of charge, to any person or organization
 * obtaining a copy of the software and accompanying documentation covered by
 * this license (the "Software") to use, reproduce, display, distribute,
 * execute, and transmit the Software, and to prepare derivative works of the
 * Software, and to permit third-parties to whom the Software is furnished to
 * do so, all subject to the following:
 *
 * The copyright notices in the Software and this entire statement, including
 * the above license grant, this restriction and the following disclaimer,
 * must be included in all copies of the Software, in whole or in part, and
 * all derivative works of the Software, unless such copies or derivative
 * works are solely in the form of machine-executable object code generated by
 * a source language processor.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
 * FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * ***************************************************************************
 *
 * NOTE THAT THE BOOST-LICENSE DOES NOT APPLY TO ANY OTHER FILES OF THE
 * GENEVA LIBRARY, UNLESS THIS IS EXPLICITLY STATED IN THE CORRESPONDING FILE!
 *
 * See the AUTHORS file in the top-level directory for a list of authors.
 *
 * Contact: contact [at] gemfony (dot) eu
 *
 * Geneva was developed with kind support from Karlsruhe Institute of
 * Technology (KIT) and Steinbuch Centre for Computing (SCC). Further
 * information about KIT and SCC can be found at http://www.kit.edu/english
 * and http://scc.kit.edu .
 *
 * Geneva is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * Boost Software License for more details.
 *
 * For further information on Gemfony scientific and Geneva, visit
 * http://www.gemfony.eu .
 */

#ifndef GEXCEPTIONSTREAMER_HPP_
#define GEXCEPTIONSTREAMER_HPP_

// Global checks, defines and includes needed for all of Geneva
#include "common/GGlobalDefines.hpp"

// Standard header files go here

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <exception>
#include <vector>
#include <memory>
#include <tuple>
#include <mutex>
#include <cstdlib>
#include <cmath>
#include <chrono>
#include <iomanip>

// Boost header files go here
#include <boost/noncopyable.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/lexical_cast.hpp>

// Geneva header files go here
#include "common/GSingletonT.hpp"
#include "common/GCommonEnums.hpp"
#include "common/GTupleIO.hpp"
#include "common/GExceptions.hpp"

namespace Gem {
namespace Common {

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/**
 * This class defines the interface of log targets, i.e. targets for the logging of
 * messages through the GLogStreamer class. Essentially all that is needed is
 * the log function. Pointers to this class are stored in the GLogStreamer. They
 * point to objects of the GConsoleLogger or GFileLogger classes, or other log targets
 * defined by the user.
 */
class GBaseLogTarget {
public:
	 /** @brief The default constructor */
	 G_API_COMMON GBaseLogTarget();

	 /** @brief The standard destructor */
	 virtual G_API_COMMON ~GBaseLogTarget() BASE;

	 /** @brief The logging interface */
	 virtual G_API_COMMON void log(const std::string &) const BASE = 0;

	 /** @brief Adds an extension to the output */
	 virtual G_API_COMMON void logWithSource(const std::string &, const std::string &) const BASE = 0;
};

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/**
 * The console logger writes log messages to the console.
 */
class GConsoleLogger : public GBaseLogTarget {
public:
	 /** @brief A standard constructor */
	 G_API_COMMON GConsoleLogger();

	 /** @brief The standard destructor */
	 G_API_COMMON ~GConsoleLogger() override;

	 /** @brief Implements the logging to the console */
	 G_API_COMMON void log(const std::string &) const override;

	 /** @brief Adds a specifier to the output */
	 G_API_COMMON void logWithSource(
		 const std::string &, const std::string &
	 ) const override;
};

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/**
 * The file logger writes log messages to a file.
 */
class GFileLogger : public GBaseLogTarget {
public:
/** @brief A standard constructor */
	 G_API_COMMON GFileLogger();

	 /** @brief This constructor accepts a boost path to a file name as argument */
	 explicit G_API_COMMON GFileLogger(const boost::filesystem::path &);

	 /** @brief The standard destructor */
	 G_API_COMMON ~GFileLogger() override;

	 /** @brief Implements logging to a file on disk */
	 G_API_COMMON void log(const std::string &msg) const override;

	 /** @brief Adds an extension to the output file */
	 G_API_COMMON void logWithSource(
		 const std::string &, const std::string &
	 ) const override;

private:
	 std::string m_fname; ///< The name of the log file
	 mutable bool m_first; ///< Indicates whether any logging has already been done
};

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/**
 * This class serves as the front end of the logging infrastructure. An object of
 * this type is accessible through a singleton to all entities in the program.
 * Upon invocation of the streaming operator it produces an object which is supposed
 * to handle the rest of the work, either using the log targets stored in the
 * GLogger object or letting manipulators output the work.
 */
template<class S> // "S" means "streamer"
class GLogger
	: boost::noncopyable {
public:
	 /***************************************************************************/
	 /** @brief The default constructor - needed for the singleton */
	 GLogger()
		 : m_default_logger(new GConsoleLogger())
	 { /* nothing */ }

	 /***************************************************************************/
	 /**
		 * This function will forward all arguments to a newly created object
		 * of type S. Note that the function returns the S object by value. It
		 * will not survive beyond the end of the stream-chain.
		 */
	 template<typename T>
	 S operator<<(const T &t) {
		 S s;
		 s << t;
		 return s;
	 }

	 /******************************************************************************/
	 /**
	  * Needed for ostringstream
	  */
	 S operator<<(std::ostream &( *val )(std::ostream &)) {
		 S s;
		 s << val;
		 return s;
	 }

	 /******************************************************************************/
	 /**
	  * Needed for ostringstream
	  */
	 S operator<<(std::ios &( *val )(std::ios &)) {
		 S s;
		 s << val;
		 return s;
	 }

	 /******************************************************************************/
	 /**
	  *  Needed for ostringstream
	  */
	 S operator<<(std::ios_base &( *val )(std::ios_base &)) {
		 S s;
		 s << val;
		 return s;
	 }

	 /***************************************************************************/
	 /**
		 * This function instructs the logger architecture to emit additional
		 * specifications for the data being logged. When writing to the console,
		 * a corresponding text will be emitted. When writing to a file, the
		 * modifier will be appended with an underscore to the filename.
		 */
	 S operator()(const std::string &extension) {
		 S s(extension);
		 return s;
	 }

	 /***************************************************************************/
	 /**
		 * This function instructs the logger architecture to emit data to the file
		 * specified by the boost::path object
		 */
	 S operator()(boost::filesystem::path p) {
		 S s(p);
		 return s;
	 }

	 /***************************************************************************/
	 /**
		 * Allows to set the default log target
		 */
	 void setDefaultLogTarget(std::shared_ptr <GBaseLogTarget> gblt) {
		 if (gblt) {
			 m_default_logger = gblt;
		 } else {
			 raiseException(
				 "In GLogger::setDefaultLogTarget(): Error!" << std::endl
																			<< "Tried to register empty default logger" << std::endl
			 );
		 }
	 }

	 /***************************************************************************/
	 /**
		 * Adds a log target, such as console or file
		 */
	 void addLogTarget(std::shared_ptr <GBaseLogTarget> gblt) {
		 if (gblt) {
			 m_log_vector.push_back(gblt);
		 } else {
			 raiseException(
				 "In GLogger::addLogTarget(): Error!" << std::endl
																  << "Tried to register empty logger" << std::endl
			 );
		 }
	 }

	 /***************************************************************************/
	 /**
		 * Checks whether any log targets are present
		 */
	 bool hasLogTargets() const {
		 return !m_log_vector.empty();
	 }

	 /***************************************************************************/
	 /**
		 * Clears local log-targets
		 */
	 void resetLogTargets() {
		 m_log_vector.clear();
	 }

	 /***************************************************************************/
	 /**
		 * Allows S-objects to submit strings to the log targets. Note that this
		 * function is thread-safe and thus may be called from different threads.
		 * Note that this function throws if no logging targets have been registered.
		 */
	 void log(const std::string &message) const {
		 // Make sure only one entity outputs data
		 std::unique_lock<std::mutex> lk(m_logger_mutex);

		 if (!m_log_vector.empty()) {
			 // Do the actual logging
			 for(auto cit: m_log_vector) { // std::shared_ptr may be copied
				 cit->log(message);
			 }
		 } else {
			 if (m_default_logger) {
				 m_default_logger->log(message);
			 } else {
				 raiseException(
					 "In GLogger::log(): Error!" << std::endl
														  << "No loggers found" << std::endl
				 );
			 }
		 }
	 }

	 /***************************************************************************/
	 /**
		 * Allows S-objects to submit strings to the log targets. Note that this
		 * function is thread-safe and thus may be called from different threads.
		 * Note that this function throws if no logging targets have been registered.
		 */
	 void logWithSource(const std::string &message, const std::string &extension) const {
		 // Make sure only one entity outputs data
		 std::unique_lock<std::mutex> lk(m_logger_mutex);

		 if (!m_log_vector.empty()) {
			 // Do the actual logging
			 for(auto cit: m_log_vector) { // std::shared_ptr max be copied
				 cit->logWithSource(message, extension);
			 }
		 } else {
			 if (m_default_logger) {
				 m_default_logger->logWithSource(message, extension);
			 } else {
				 raiseException(
					 "In GLogger::logWithSource(): Error!" << std::endl
																		<< "No loggers found" << std::endl
				 );
			 }
		 }
	 }

	 /***************************************************************************/
	 /**
		 * Throws an exception from a global position. This prevents exceptions thrown
		 * from within threads from getting lost.
		 */
	 void throwException(const std::string &error) {
		 // Make sure only one entity outputs data
		 std::unique_lock<std::mutex> lk(m_logger_mutex);

		 throw(gemfony_exception(error));
	 }

	 /***************************************************************************/
	 /**
		 * Initiates the termination sequence
		 */
	 void terminateApplication(const std::string &error) {
		 // Make sure only one entity outputs data
		 std::unique_lock<std::mutex> lk(m_logger_mutex);

		 std::cerr << error;
		 std::terminate();
	 }

	 /***************************************************************************/
	 /**
		 * Output to stdout
		 */
	 void toStdOut(const std::string &message) {
		 // Make sure only one entity outputs data
		 std::unique_lock<std::mutex> lk(m_logger_mutex);

		 std::cout << message;
	 }

	 /***************************************************************************/
	 /**
		 * Output to stderr
		 */
	 void toStdErr(const std::string &message) {
		 // Make sure only one entity outputs data
		 std::unique_lock<std::mutex> lk(m_logger_mutex);

		 std::cerr << message;
	 }

private:
	 /***************************************************************************/

	 std::vector<std::shared_ptr<GBaseLogTarget>> m_log_vector; ///< Contains the log targets
	 mutable std::mutex m_logger_mutex; ///< Needed for concurrent access to the log targets

	 std::shared_ptr <GBaseLogTarget> m_default_logger; ///< The default log target
};

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/**
 * Objects of this class need to be added as the last element of a logging or
 * exception chain, possibly wrapped into a macro giving it information about
 * the file and lines from which it has been called.
 */
class GManipulator {
public:
	 /** @brief A constructor that stores both accompanying information and the logging type */
	 G_API_COMMON GManipulator(
		 const std::string &accompInfo, const logType &lt
	 );
	 /** @brief The copy constructor */
	 G_API_COMMON GManipulator(const GManipulator &);

	 /** @brief A constructor that stores the logging type only */
	 explicit G_API_COMMON GManipulator(const logType &lt);

	 /** @brief Retrieves the stored logging type */
	 G_API_COMMON logType getLogType() const;
	 /** @brief Retrieves stored accompanying information (if any) */
	 G_API_COMMON std::string getAccompInfo() const;
	 /** @brief Checks whether any accompanying information is available */
	 G_API_COMMON bool hasAccompInfo() const;

	 GManipulator() = delete; ///< Intentionally private and undefined

private:
	 std::string m_accomp_info; ///< Holds accompanying information
	 logType m_log_type; ///< Holds the type of logging event used for instantiating the manipulator
};

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/**
 * Every entity in Geneva should be able to throw exceptions, regardless of whether
 * this happens from within a thread a in the context of serial execution. The output
 * should go to different log targets defined by the user, such as stdout or a file
 * (or possibly both). Emitting as much information as possible should be encouraged.
 * Hence adding information to the exception handler should be as easy as adding
 * data to a stream.
 */
class GLogStreamer {
public:
	 /** @brief The default constructor */
	 G_API_COMMON GLogStreamer();
	 /** @brief The copy constructor */
	 G_API_COMMON GLogStreamer(const GLogStreamer &);

	 /** @brief A constructor that adds an extension string to the output */
	 explicit G_API_COMMON GLogStreamer(const std::string &);

	 /** @brief A constructor that logs data to a file specified by a boost::filesystem::path object */
	 explicit G_API_COMMON GLogStreamer(boost::filesystem::path);

	 /** @brief A standard destructor */
	 virtual G_API_COMMON ~GLogStreamer() BASE;

	 /** @brief Needed for std::ostringstream */
	 G_API_COMMON GLogStreamer &operator<<(std::ostream &(*val)(std::ostream &));
	 /** @brief Needed for std::ostringstream */
	 G_API_COMMON GLogStreamer &operator<<(std::ios &(*val)(std::ios &));
	 /** @brief Needed for std::ostringstream */
	 G_API_COMMON GLogStreamer &operator<<(std::ios_base &(*val)(std::ios_base &));

	 /** @brief A GManipulator object triggers the actual logging procedure */
	 G_API_COMMON void operator<<(const GManipulator &gm);

	 /** @brief Returns the content of the stream */
	 G_API_COMMON std::string content() const;
	 /** @brief Resets the stream content */
	 G_API_COMMON void reset();

	 /** @brief Checks whether an extension string has been registered */
	 G_API_COMMON bool hasExtension() const;
	 /** @brief The content of the extension_ string */
	 G_API_COMMON std::string getExtension() const;
	 /** @brief Checks whether a log file name has been registered */
	 G_API_COMMON bool hasOneTimeLogFile() const;
	 /** @brief The name of the manually specified file */
	 G_API_COMMON boost::filesystem::path getOneTimeLogFile() const;

	 /****************************************************************************/
	 /**
	  * Output of all standard values and types with a predefined operator<<
	  */
	 template<typename T>
	 GLogStreamer &operator<<(const T &val) {
		 m_oss << val;
		 return *this;
	 }

	 /****************************************************************************/

private:
	 /**
	  * Retrieve a string representing the current time and date. Note that
	  * this function is duplicated from a function in GCommonHelperFunctions.hpp
	  * in order to break circular header inclusion.
	  *
	  * @return A string representing the current time and date
	  */
	 static std::string currentTimeAsString() {
#if BOOST_COMP_GNUC && (BOOST_COMP_GNUC < BOOST_VERSION_NUMBER(5,0,0))
		 return std::string("Dummy (g++ < 5.0 does not support put_time)");
#else
		 std::ostringstream oss;
		 std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
		 oss << std::put_time(std::localtime(&now), "%c");
		 return oss.str();
#endif
	 }

	 std::ostringstream m_oss; ///< Holds the actual streamed data
	 std::string m_extension; ///< Additional information about the logging source
	 boost::filesystem::path m_log_file; ///< The name of a manually specified log file
};

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/

} /* namespace Common */
} /* namespace Gem */

/******************************************************************************/
/**
* We currently require the global GLogStreamer object to be a singleton
*/
using log_singleton = Gem::Common::GSingletonT<Gem::Common::GLogger<Gem::Common::GLogStreamer>>;
#define glogger_ptr log_singleton::Instance(0)
#define glogger (*(log_singleton::Instance(0)))

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
// Some related defines

#define LOCATIONSTRING std::string("in file ") + std::string(__FILE__) + std::string(" near line ") + std::to_string(__LINE__)

#define GEXCEPTION   Gem::Common::GManipulator( LOCATIONSTRING, Gem::Common::logType::EXCEPTION)
#define GTERMINATION Gem::Common::GManipulator( LOCATIONSTRING, Gem::Common::logType::TERMINATION)
#define GWARNING     Gem::Common::GManipulator( LOCATIONSTRING, Gem::Common::logType::WARNING)
#define GLOGGING     Gem::Common::GManipulator( Gem::Common::logType::LOGGING)
#define GFILE        Gem::Common::GManipulator( Gem::Common::logType::FILE)
#define GSTDOUT      Gem::Common::GManipulator( Gem::Common::logType::STDOUT)
#define GSTDERR      Gem::Common::GManipulator( LOCATIONSTRING, Gem::Common::logType::STDERR)

/******************************************************************************/

#endif /* GEXCEPTIONSTREAMER_HPP_ */
