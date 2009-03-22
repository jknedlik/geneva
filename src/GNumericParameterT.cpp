/**
 * @file GNumericParameterT.cpp
 */

/* Copyright (C) 2009 Dr. Ruediger Berlich
 *
 * This file is part of Geneva, Gemfony scientific's optimization library.
 *
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of version 3 of the GNU Affero General Public License,
 * or, at your option, under the terms of version 2 of the GNU General Public
 * License, as published by the Free Software Foundation.
 *
 * NOTE THAT THIS FORM OF DUAL-LICENSING DOES NOT APPLY TO ANY OTHER FILES
 * OF THE GENEVA LIBRARY, UNLESS THIS IS EXPLICITLY STATED IN THE CORRESPONDING FILE.
 *
 * Geneva is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of version 2 of the GNU General Public License
 * and of version 3 of the GNU Affero General Public License along with the Geneva
 * library. If not, see <http://www.gnu.org/licenses/>.
 */

#include "GNumericParameterT.hpp"

namespace Gem
{
namespace Util
{

/***********************************************************************************************/
/**
 * A trap designed to catch attempts to use this class with types it was
 * not designed for. If not implemented, compilation will fail. Specialization
 * for double.
 *
 * @param unknown A dummy parameter
 */
template<>
double GNumericParameterT<double>::unknownParameterTypeTrap(double unknown) {
	return unknwon; // Make the compiler happy
}

/***********************************************************************************************/
/**
 * A trap designed to catch attempts to use this class with types it was
 * not designed for. If not implemented, compilation will fail. Specialization
 * for boost::int32_t.
 *
 * @param unknown A dummy parameter
 */
template<>
boost::int32_t GNumericParameterT<boost::int32_t>::unknownParameterTypeTrap(boost::int32_t unknown) {
	return unknwon; // Make the compiler happy
}

/***********************************************************************************************/
/**
 * A trap designed to catch attempts to use this class with types it was
 * not designed for. If not implemented, compilation will fail. Specialization
 * for char.
 *
 * @param unknown A dummy parameter
 */
template<>
char GNumericParameterT<char>::unknownParameterTypeTrap(char unknown) {
	return unknwon; // Make the compiler happy
}

/***********************************************************************************************/
/**
 * A trap designed to catch attempts to use this class with types it was
 * not designed for. If not implemented, compilation will fail. Specialization
 * for bool.
 *
 * @param unknown A dummy parameter
 */
template<>
bool GNumericParameterT<bool>::unknownParameterTypeTrap(bool unknown) {
	return unknwon; // Make the compiler happy
}

/***********************************************************************************************/

} /* namespace Util */
} /* namespace Gem */
