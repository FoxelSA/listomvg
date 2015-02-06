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

#include <fastcal-all.h>
#include <types.hpp>
#include <tools.hpp>
#include <cstring>
#include <iostream>
#include <list_utils.hpp>
#include "../lib/stlplus3/filesystemSimplified/file_system.hpp"

using namespace std;

/*******************************************************************************
* verify that software argument are valid
*
********************************************************************************
*/

bool isInputValid( const char*  softName,
                   const std::string& sImageDir,
                   const std::string& sOutputDir,
                   const std::string& smacAddress,
                   const std::string& sMountPoint,
                   const std::string& sChannelFile,
                   const bool & bRigidRig,
                   const bool & bUseCalibPrincipalPoint,
                   const double & focalPixPermm )
{

    std::cout << " You called : " <<std::endl
              << softName << std::endl
              << "--imageDirectory " << sImageDir << std::endl
              << "--macAddress " << smacAddress << std::endl
              << "--outputDirectory " << sOutputDir << std::endl
              << "--mountPoint " << sMountPoint << std::endl
              << "--channelFile " << sChannelFile << std::endl
              << "--rigidRig " << bRigidRig << std::endl
              << "--useCalibPrincipalPoint " << bUseCalibPrincipalPoint << std::endl
              << "--focal " << focalPixPermm << std::endl;

    // check if image dir exists
    if ( !stlplus::folder_exists( sImageDir ) )
    {
      std::cerr << "\nThe input directory doesn't exist" << std::endl;
      return false;
    }
    else
    {
        // check if output dir is given
        if (sOutputDir.empty())
        {
          std::cerr << "\nInvalid output directory" << std::endl;
          return false;
        }
        else
        {
            // if output dir is empty, create it
            if ( !stlplus::folder_exists( sOutputDir ) )
            {
                std::cerr << "\nCannot create output directory" << std::endl;
                return false;
            }
            else
            {
                // check if mac address is given
                if( smacAddress.empty() )
                {
                  std::cerr << "\n No mac address given " << std::endl;
                  return false;
                }
                else
                {
                    // check if mount point is given
                    if( sMountPoint.empty() )
                    {
                      std::cerr << "\n No mount point given " << std::endl;
                      return false;
                    }
                    else
                      return true;
                }

            }

        }

    }
}

/*********************************************************************
*  load calibration data related to elphel cameras
*
*********************************************************************/

bool  loadCalibrationData(  std::vector< sensorData >  & vec_sensorData,
                            const std::string & sMountPoint,
                            const std::string & smacAddress)
{
      /* Key/value-file descriptor */
      lf_Descriptor_t lfDesc;
      lf_Size_t       lfChannels=0;

      /* Creation and verification of the descriptor */
      char *c_data = new char[sMountPoint.length() + 1];
      std::strcpy(c_data, sMountPoint.c_str());

      char *c_mac = new char[smacAddress.length() + 1];
      std::strcpy(c_mac, smacAddress.c_str());

      if ( lf_parse( (unsigned char*)c_mac, (unsigned char*)c_data, & lfDesc ) == LF_TRUE ) {

          /* Query number of camera channels */
          lfChannels = lf_query_channels( & lfDesc );

          /* iter on sucamera */
          for( li_Size_t sensor_index = 0 ; sensor_index < lfChannels ; ++sensor_index )
          {
            sensorData  sD;

            /* Query number width and height of sensor image */
            sD.lfWidth  = lf_query_pixelCorrectionWidth ( sensor_index, & lfDesc );
            sD.lfHeight = lf_query_pixelCorrectionHeight( sensor_index, & lfDesc );

            /* Query focal length of camera sensor index */
            sD.lfFocalLength = lf_query_focalLength( sensor_index , & lfDesc );
            sD.lfPixelSize   = lf_query_pixelSize  ( sensor_index , & lfDesc );

            /* Query angles used for gnomonic rotation */
            sD.lfAzimuth    = lf_query_azimuth    ( sensor_index , & lfDesc );
            sD.lfHeading    = lf_query_heading    ( sensor_index , & lfDesc );
            sD.lfElevation  = lf_query_elevation  ( sensor_index , & lfDesc );
            sD.lfRoll       = lf_query_roll       ( sensor_index , & lfDesc );

            // compute rotation and store it.
            computeRotationEl ( &sD.R[0] , sD.lfAzimuth , sD.lfHeading, sD.lfElevation, sD.lfRoll );

            /* Query principal point */
            sD.lfpx0 = lf_query_px0 ( sensor_index , & lfDesc );
            sD.lfpy0 = lf_query_py0 ( sensor_index , & lfDesc );

            /* Query information related to entrance pupil center */
            sD.lfRadius   = lf_query_radius               ( sensor_index , & lfDesc );
            sD.lfCheight  = lf_query_height               ( sensor_index , & lfDesc );
            sD.lfEntrance = lf_query_entrancePupilForward ( sensor_index , & lfDesc );

            // compute optical center in camera coordinate and store it
            getOpticalCenter ( &sD.C[0] , sD.lfRadius, sD.lfCheight, sD.lfAzimuth, sD.R, sD.lfEntrance );

            vec_sensorData.push_back(sD);
          }

          /* Release descriptor */
          lf_release( & lfDesc );

          /* calibration information are loaded*/
          return true;
      }
      else
      {
          std::cerr << " Could not read calibration data. Exit" << std::endl;
          return false;
      }

}
