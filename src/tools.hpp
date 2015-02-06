/*
 * tools
 *
 * Copyright (c) 2014-2015 FOXEL SA - http://foxel.ch
 * Please read <http://foxel.ch/license> for more information.
 *
 *
 * Author(s):
 *
 *      Stéphane Flotron <s.flotron@foxel.ch>
 *
 *
 * This file is part of the FOXEL project <http://foxel.ch>.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * Additional Terms:
 *
 *      You are required to preserve legal notices and author attributions in
 *      that material or in the Appropriate Legal Notices displayed by works
 *      containing it.
 *
 *      You are required to attribute the work as explained in the "Usage and
 *      Attribution" section of <http://foxel.ch/license>.
 */

#ifndef TOOLS_HPP_
#define TOOLS_HPP_


#include <fastcal-all.h>
#include <vector>
#include <string.h>
#include <types.hpp>

using namespace std;

/*******************************************************************************
* Split an input string with a delimiter and fill a string vector
*
********************************************************************************
*/

bool split ( const std::string src, const std::string& delim, std::vector<std::string>& vec_value );


/*******************************************************************************
*  Given 4 angles, compute Elphel rotation
*
********************************************************************************
*/
 void computeRotationEl ( li_Real_t* R , li_Real_t az , li_Real_t head, li_Real_t ele , li_Real_t roll);

/********************************************************************************
*  Given three angles, entrance pupil forward, radius and height, compute optical center position.
*
*********************************************************************************
*/

 void getOpticalCenter ( li_Real_t* C ,
                const li_Real_t& radius,
                const li_Real_t& height,
                const li_Real_t& azimuth,
                const li_Real_t* R,
                const li_Real_t& entrancePupilForward );

#endif /* TOOLS_HPP_ */
