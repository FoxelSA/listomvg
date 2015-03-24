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
#include <list_utils.hpp>
#include <cstring>
#include <iostream>
#include <fstream>
#include <sstream>
#include <set>
#include <map>
#include <stdlib.h>
#include <ctype.h>
#include <stdlib.h>
#include "progress.hpp"
#include "../lib/stlplus3/filesystemSimplified/file_system.hpp"

using namespace std;

/*******************************************************************************
* verify that given timestamp range is valid
*
********************************************************************************
*/

/*! \brief Check input data validity
*
* This function checks that given timestamp range is valid
*
* \param sTimestampLow  Lower bound for timestamp value
* \param sTimestampUp   Upper bound for timestampe value
*
* \return bool value indicating if the timestamp range is valid or not
*/

bool isRangeValid(  const std::string& sTimestampLow,
                    const std::string& sTimestampUp)
{

  // split timestamp in order to check format
  std::vector <std::string>  splitLow;
  std::vector <std::string>  splitUp;

  split( sTimestampLow, "_", splitLow );
  split( sTimestampUp, "_", splitUp );

  // check that the two timestamps are in the format 0123456789_012345
  if(  splitLow.size() == 2
       && splitUp.size() == 2
       && splitLow[0].size() == 10
       && splitLow[1].size() == 6
       && splitUp[0].size()  == 10
       && splitUp[1].size()  == 6
    )
  {
      // check that lower bound is less than upper
      if( sTimestampLow < sTimestampUp )
      {
          return true;
      }
      else
      {
          std::cerr << "\nTimestamp Upper bound is less than lower bound " << std::endl;
          return false;
      }

  }
  else
  {
      std::cerr << "\nThe provided timestamp are not in the right format " << std::endl;
      return false;
  }

}

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
                   const double & focalPixPermm,
                   const std::string& sTimestampLow,
                   const std::string& sTimestampUp)
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
              << "--focal " << focalPixPermm << std::endl
              << "--lowerBound " << sTimestampLow << std::endl
              << "--upperBound " << sTimestampUp << std::endl;

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
                if( !stlplus::folder_create ( sOutputDir ) )
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

/*********************************************************************
*  read channel file (if exists )
*
*********************************************************************/

void loadChannelFile( std::vector< li_Size_t >  & keptChan,
                      const std::string & sChannelFile )
{
    // if channel file is given, read it
    if( !sChannelFile.empty() )
    {
        std::ifstream  inFile(sChannelFile.c_str());
        li_Size_t chan;

        if( inFile.is_open() )
        {
            while( inFile >> chan ) // one channel per line
            {
                keptChan.push_back(chan);
            }

            if( keptChan.size() == 0 )
            {
                std::cerr << "Warning:: No channels are read in file, use all image instead" << std::endl;
            }
        }
        else
        {
            std::cerr << "Warning :: Could not open channel file. Use all image instead" << std::endl;
        }

        inFile.close();
    }
    else
    {
        std::cout << "\n Keep all channels " << std::endl;
    }

}

/*********************************************************************
*  compute image intrinsic parameter
*
*********************************************************************/

void computeImageIntrinsic(
                camInformation & camInfo,
                const std::vector < sensorData > & vec_sensorData,
                const std::string & timestamp,
                const size_t   & sensor_index,
                const double   & focalPixPermm,
                const bool     & bUseCalibPrincipalPoint,
                const bool     & bRigidRig )
{
    // affect width and height
    camInfo.width  = vec_sensorData[sensor_index].lfWidth;
    camInfo.height = vec_sensorData[sensor_index].lfHeight;

    // Create list if focal is given
    if( focalPixPermm > 0.0 )
    {

        camInfo.focal = focalPixPermm ;

        if(!bUseCalibPrincipalPoint)
        {
            camInfo.px0   = camInfo.width  / 2.0 ;
            camInfo.py0   = camInfo.height / 2.0 ;
        }
        else
        {
            camInfo.px0   = vec_sensorData[sensor_index].lfpx0;
            camInfo.py0   = vec_sensorData[sensor_index].lfpy0;
        }

    }
    // Create list if full calibration data are used
    else
    {
        li_Real_t focalPix = vec_sensorData[sensor_index].lfFocalLength / vec_sensorData[sensor_index].lfPixelSize ;

        camInfo.focal = focalPix;
        camInfo.px0   = vec_sensorData[sensor_index].lfpx0;
        camInfo.py0   = vec_sensorData[sensor_index].lfpy0;
    }

    // if rigid rig, add some informations
    if(bRigidRig)
    {
        // export rig index
        camInfo.sRigName = timestamp;

        // export channel
        camInfo.subChan = sensor_index;

        // export rotation
        camInfo.R[0] = vec_sensorData[sensor_index].R[0] ;
        camInfo.R[1] = vec_sensorData[sensor_index].R[1] ;
        camInfo.R[2] = vec_sensorData[sensor_index].R[2] ;
        camInfo.R[3] = vec_sensorData[sensor_index].R[3] ;
        camInfo.R[4] = vec_sensorData[sensor_index].R[4] ;
        camInfo.R[5] = vec_sensorData[sensor_index].R[5] ;
        camInfo.R[6] = vec_sensorData[sensor_index].R[6] ;
        camInfo.R[7] = vec_sensorData[sensor_index].R[7] ;
        camInfo.R[8] = vec_sensorData[sensor_index].R[8] ;

        // export translation
        camInfo.C[0] = vec_sensorData[sensor_index].C[0] ;
        camInfo.C[1] = vec_sensorData[sensor_index].C[1] ;
        camInfo.C[2] = vec_sensorData[sensor_index].C[2] ;

    }

}

/*********************************************************************
*  keep only most representative rigs
*
*********************************************************************/

void keepRepresentativeRigs(
          std::set <string> & imageToRemove,
          const std::map<std::string, std::vector<string> > & mapSubcamPerTimestamp,
          const size_t imageNumber,
          const std::string& sTimestampLower,
          const std::string& sTimestampUpper )
{
    // check the number of subcamera per rig. Remove less representated rigs.
    li_Size_t   timeStamp = 0;
    std::map < std::string, std::vector<string> >::const_iterator iter_timestamp = mapSubcamPerTimestamp.begin();
    std::pair< std::map < li_Size_t , li_Size_t >::iterator, bool > insert_check;
    std::map < li_Size_t , li_Size_t >  occurencePerSubcamNumber;

    // compute occurrence of each subcamera number
    for( timeStamp = 0 ; timeStamp < mapSubcamPerTimestamp.size(); ++timeStamp )
    {
        //advance iterator
        iter_timestamp = mapSubcamPerTimestamp.begin();
        std::advance(iter_timestamp, timeStamp);

        //update map
        insert_check = occurencePerSubcamNumber.insert (
        std::pair<li_Size_t, li_Size_t>( iter_timestamp->second.size(), occurencePerSubcamNumber.size()) );

        if(insert_check.second == true )
        {
            occurencePerSubcamNumber[iter_timestamp->second.size() ] = 1 ;
        }
        else
        {
            occurencePerSubcamNumber[iter_timestamp->second.size() ] += 1 ;
        }
    }

    // compute most representative number of subcamera
    li_Size_t               k = 0;
    li_Size_t               i = 0;
    li_Size_t   max_Occurence = 0;
    li_Size_t   subCamNumber  = 0;

    std::map < li_Size_t , li_Size_t >::const_iterator iter_occurrence = occurencePerSubcamNumber.begin();

    for( k = 0 ; k < occurencePerSubcamNumber.size(); ++k )
    {
        //advance iterator
        iter_occurrence = occurencePerSubcamNumber.begin();
        std::advance(iter_occurrence, k);

        //update map
        if( iter_occurrence->second > max_Occurence )
        {
            max_Occurence = iter_occurrence->second;
            subCamNumber  = iter_occurrence->first;
        }

    }

    // check timestamp range validity
    bool validRange = isRangeValid(sTimestampLower, sTimestampUpper);
    if( !validRange )
    {
        std::cerr << "\n[keepRepresentativeRigs::WARNING] openMVG will use all images, timestamp range is not valid " << std::endl;
    }

    // remove rig not having the max occurrence subcam number
    for( k = 0 ; k < mapSubcamPerTimestamp.size(); ++k )
    {
        //advance iterator
        iter_timestamp = mapSubcamPerTimestamp.begin();
        std::advance(iter_timestamp, k);

        //update map
        if( iter_timestamp->second.size() != subCamNumber
            || ( iter_timestamp->first < sTimestampLower && validRange )
            || ( iter_timestamp->first > sTimestampUpper && validRange )
          )
        {
            for( i  = 0 ; i < iter_timestamp->second.size() ; ++i )
            {
                imageToRemove.insert( iter_timestamp->second[i] );
            }
        }
    }

    std::cout << "\n\n Max rig occurence is " << max_Occurence
              << ", rig number of sub cameras is " << subCamNumber
              << std::endl;
    std::cout << " OpenMVG will use " << imageNumber - imageToRemove.size()
              << " of " << imageNumber << " input images "
              << std::endl << std::endl;

}
/*********************************************************************
*  write image list and intrinsic to file
*
*********************************************************************/

void exportToFile(
          const std::set <string> & imageToRemove,
          const std::set<imageNameAndIntrinsic> & camAndIntrinsics,
          std::ofstream & listTXT,
          const bool & bRigidRig )
{

    C_Progress_display my_progress_bar_export( camAndIntrinsics.size(),
    std::cout, "\n Write list in file lists.txt :\n");

    // export list to file
    li_Size_t   img       = 0;
    std::set<imageNameAndIntrinsic>::const_iterator   iter = camAndIntrinsics.begin();
    std::set<std::string>::iterator  it = imageToRemove.end();

    for ( img = 0; img < (li_Size_t) camAndIntrinsics.size(); ++img)
    {
        //advance iterator
        iter = camAndIntrinsics.begin();
        std::advance(iter, img);

        // create stream
        std::ostringstream os;
        os.precision(6);

        // check if we have to keep this timestamp
        it=imageToRemove.find(iter->first);
        if ( it == imageToRemove.end() )
        {
            os << iter->first ;

            // retreive intrinsic info
            camInformation intrinsic = iter->second;

            // export instrinsics
            os << ";"  << intrinsic.width;
            os << ";"  << intrinsic.height;

            // export camera matrix
            os << ";"  << intrinsic.focal;
            os << ";"  << 0;
            os << ";"  << intrinsic.px0;
            os << ";"  << 0;
            os << ";"  << intrinsic.focal;
            os << ";"  << intrinsic.py0;
            os << ";"  << 0;
            os << ";"  << 0;
            os << ";"  << 1;

            if(bRigidRig)
            {
              // export rig Id and subchannel
              os << ";" << intrinsic.sRigName;
              os << ";" << intrinsic.subChan;

              // export rotation matrix
              os << ";"  << intrinsic.R[0];
              os << ";"  << intrinsic.R[1];
              os << ";"  << intrinsic.R[2];
              os << ";"  << intrinsic.R[3];
              os << ";"  << intrinsic.R[4];
              os << ";"  << intrinsic.R[5];
              os << ";"  << intrinsic.R[6];
              os << ";"  << intrinsic.R[7];
              os << ";"  << intrinsic.R[8];

              // export camera center
              os << ";"  << intrinsic.C[0];
              os << ";"  << intrinsic.C[1];
              os << ";"  << intrinsic.C[2];
          }

          os << endl;

          listTXT << os.str();
        }

        ++my_progress_bar_export;
    }

}


/*********************************************************************
*  compute camera and rig intrinsic parameters
*
*********************************************************************/

bool computeInstrinsicPerImages(
              std::vector<std::string> & vec_image,
              const std::vector< sensorData > & vec_sensorData,
              const std::vector< li_Size_t >  & keptChan,
              const std::string & sImageDir,
              const std::string & sOutputDir,
              const double & focalPixPermm,
              const bool & bUseCalibPrincipalPoint,
              const bool & bRigidRig,
              std::string& sTimestampLower,
              std::string& sTimestampUpper)
{
    //initialize rig map
    std::map<std::string, li_Size_t>  mapRigPerImage;

    // Write the new file
    std::ofstream listTXT( stlplus::create_filespec( sOutputDir,"lists.txt" ).c_str() );

    if ( listTXT )
    {
        std::sort(vec_image.begin(), vec_image.end());

        // create output
        std::set<imageNameAndIntrinsic> camAndIntrinsics;

        C_Progress_display my_progress_bar_image( vec_image.size(),
        std::cout, "\n List computation progession:\n");

        // declare variable before loop
        std::vector<std::string>::const_iterator iter_image = vec_image.begin();
        std::pair< std::map<std::string, li_Size_t>::iterator, bool > ret;
        std::map<std::string, std::vector<string> >  mapSubcamPerTimestamp;
        std::vector<string>     splitted_name;
        std::string             timestamp;
        std::string             sImageFilename;
        std::string             minTimestamp="";
        std::string             maxTimestamp="";

        li_Size_t sensor_index  = 0;
        li_Size_t i             = 0;
        li_Size_t idx           = 0;
        bool      bKeepChannel  = false;

        // do a parallel loop to improve CPU TIME
        #pragma omp parallel firstprivate(iter_image, splitted_name, timestamp, sensor_index, i, bKeepChannel, sImageFilename )
        #pragma omp for schedule(dynamic)
        for ( idx = 0; idx < vec_image.size(); ++idx)
        {
            //advance iterator
            iter_image = vec_image.begin();
            std::advance(iter_image, idx);

            // Read meta data to fill width height and focalPixPermm
            sImageFilename = stlplus::create_filespec( sImageDir, *iter_image );

            // Test if the image format is supported:
            if (GetFormat(sImageFilename.c_str()) == Unknown)
            {
                std::cerr << " Warning : image " << sImageFilename << "\'s format is not supported." << std::endl;
            }
            else
            {
                // extract channel information from image name
                splitted_name.clear();

                split( *iter_image, "-", splitted_name );
                sensor_index=atoi(splitted_name[1].c_str());

                // now load image information and keep channel index and timestamp
                timestamp=splitted_name[0];

                #pragma omp critical
                {
                    // update min timestamp
                    if( minTimestamp.empty() )
                    {
                        minTimestamp = timestamp;
                    }
                    else
                    {
                        minTimestamp = std::min( minTimestamp, timestamp);
                    }

                    // update max timestamp
                    if( maxTimestamp.empty() )
                    {
                        maxTimestamp = timestamp;
                    }
                    else
                    {
                        maxTimestamp = std::max( maxTimestamp, timestamp);
                    }
                }

                // if no channel file is given, keep all images
                bKeepChannel = false;

                if( keptChan.empty() )
                {
                    bKeepChannel = true ;
                }
                else
                {
                    for( i = 0 ; i < keptChan.size() ; ++i )
                    {
                        if( sensor_index == keptChan[i] )
                            bKeepChannel = true;
                    }
                }

                // export only kept channel
                if( bKeepChannel )
                {
                    #pragma omp critical
                    {
                        // insert timestamp in the map
                        ret = mapRigPerImage.insert ( std::pair<std::string, li_Size_t>(timestamp, mapRigPerImage.size()) );
                        if(ret.second == true )
                        {
                            mapRigPerImage[timestamp] = mapRigPerImage.size()-1;
                        }

                        //insert image in map timestamp -> subcam
                        mapSubcamPerTimestamp[timestamp].push_back( *iter_image );
                    }

                    // export camera infor
                    camInformation   camInfo;

                    computeImageIntrinsic(
                        camInfo,
                        vec_sensorData,
                        timestamp,
                        sensor_index,
                        focalPixPermm,
                        bUseCalibPrincipalPoint,
                        bRigidRig);

                    //export info
                    #pragma omp critical
                    {
                        camAndIntrinsics.insert(std::make_pair(*iter_image, camInfo));
                    }

                };

                //update progress bar
                #pragma omp critical
                {
                    ++my_progress_bar_image;
                }

            } //endif format known

        }; // looop on vector image

        if( sTimestampLower.empty() )
            sTimestampLower = minTimestamp;

        if( sTimestampUpper.empty() )
            sTimestampUpper = maxTimestamp;

        // keep only most represented rig
        std::set < std::string >  imageToRemove;
        keepRepresentativeRigs( imageToRemove,
                                mapSubcamPerTimestamp,
                                camAndIntrinsics.size(),
                                sTimestampLower,
                                sTimestampUpper);

        // export list to file
        exportToFile(   imageToRemove,
                        camAndIntrinsics,
                        listTXT,
                        bRigidRig );

    }
    else
    {
        return false;
    }

    listTXT.close();
    return true;

}
