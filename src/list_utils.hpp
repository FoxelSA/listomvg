/*
 * list_utils
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

#ifndef LIST_UTILS_HPP_
#define LIST_UTILS_HPP_


#include <fastcal-all.h>
#include <types.hpp>
#include <tools.hpp>

using namespace std;

/*******************************************************************************
* verify that software argument are valid
*
********************************************************************************
*/

bool isInputValid(  const char* softName,
                    const std::string& sImageDir,
                    const std::string& sOutputDir,
                    const std::string& smacAddress,
                    const std::string& sMountPoint,
                    const std::string& sChannelFile,
                    const bool & bRigidRig,
                    const bool & bUseCalibPrincipalPoint,
                    const double & focalPixPermm );

/*********************************************************************
 *  load calibration data related to elphel cameras
 *
 *********************************************************************/

bool  loadCalibrationData( std::vector< sensorData >  & vec_sensorData,
                    const std::string & sMountPoint,
                    const std::string & smacAddress) ;



#endif /* LIST_UTILS_HPP_ */
