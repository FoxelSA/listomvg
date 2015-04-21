/*
 * json
 *
 * Copyright (c) 2015 FOXEL SA - http://foxel.ch
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

/*! \file json.hpp
* \author Stephane Flotron <s.flotron@foxel.ch>
*/
/*! \mainpage listomvg
* \section listomvg
*
* For Elphel's camera, generate file lists.txt needed by openMVG, for rigid rig and standard openMVG.
*
* \section Documentation
*
* Documentation can be consulted on the [wiki](https://github.com/FoxelSA/listomvg/wiki).
*
* \section Copyright
*
* Copyright (c) 2014-2015 FOXEL SA - [http://foxel.ch](http://foxel.ch)<br />
* This program is part of the FOXEL project <[http://foxel.ch](http://foxel.ch)>.
*
* Please read the [COPYRIGHT.md](COPYRIGHT.md) file for more information.
*
* \section License
*
* This program is licensed under the terms of the
* [GNU Affero General Public License v3](http://www.gnu.org/licenses/agpl.html)
* (GNU AGPL), with two additional terms. The content is licensed under the terms
* of the [Creative Commons Attribution-ShareAlike 4.0 International](http://creativecommons.org/licenses/by-sa/4.0/)
* (CC BY-SA) license.
*
* You must read <[http://foxel.ch/license](http://foxel.ch/license)> for more
*information about our Licensing terms and our Usage and Attribution guidelines.
*
*/

#ifndef JSON_HPP_
#define JSON_HPP_

// Serialization
#include <cereal/cereal.hpp>
#include <cereal/archives/json.hpp>
#include <openMVG/types.hpp>
#include <openMVG/numeric/numeric.h>

#include <cereal/types/map.hpp>
#include <cereal/types/unordered_map.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/array.hpp>
#include <cereal/types/string.hpp>

// c++ and foxel dependencies
#include <fastcal-all.h>
#include <types.hpp>
#include <tools.hpp>
#include <map>
#include <set>
#include <fstream>
#include <sstream>
#include <iomanip>

using namespace std;
using namespace openMVG;

// define pose structure
struct  pose  {
    std::vector<double>  position;   // gps informations
    std::vector<double>  orientation; // imu informations

    bool    still  = false;   // is it a still pose
    string  raw    = "valid"; // is it a useable panorama
    size_t  sec    = 0 ;      // first part of timestamp
    size_t  usec   = 0 ;      // second part of timestamp
    std::string  timestamp  = "";

    // Serialization
    template <class Archive>
    void load( Archive & ar )
    {
      std::vector<double>  ori(10); // imu informations
      std::vector<double>  pos(4);   // gps informations
      try{
          ar( cereal::make_nvp("orientation", ori));
      }
      // exception gestion
      catch (const cereal::Exception & e)
      {
         ori.clear();
         std::cerr << " failed to load orientation" << std::endl;
      }

      orientation = ori;
      try{
          ar( cereal::make_nvp("position", pos));
      }
      // exception gestion
      catch (const cereal::Exception & e)
      {
        pos.clear();
        std::cerr << " failed to load position" << std::endl;
      }
      position = pos;
      ar(
         cereal::make_nvp("raw", raw),
         cereal::make_nvp("sec", sec),
         cereal::make_nvp("still", still),
         cereal::make_nvp("usec", usec));
    }

    // Serialization
    template <class Archive>
    void save( Archive & ar ) const
    {
      ar( cereal::make_nvp("orientation", orientation));
      ar( cereal::make_nvp("position", position));
      ar(
         cereal::make_nvp("raw", raw),
         cereal::make_nvp("sec", sec),
         cereal::make_nvp("still", still),
         cereal::make_nvp("usec", usec));
    }
};

// define map structure
typedef   std::map < size_t, std::string >  gpsDataMap ;  // map index to gps info

// define final gps_structure
struct   SfM_Gps_Data
{
    std::vector<pose>  gpsData;
};

// parse gps / imu json file
bool Load_gpsimu_cereal( SfM_Gps_Data & data,
                         const std::string & filename )
{
  //Create the stream and check it is ok
  std::ifstream stream(filename.c_str());
  if (!stream.is_open())
    return false;

  // create JSON input stream
  cereal::JSONInputArchive   archive(stream);

  // parse json file
  try{
    archive( cereal::make_nvp("pose", data.gpsData) );
  }

  // exception gestion
  catch (const cereal::Exception & e)
  {
    std::cerr << e.what() << std::endl;
    return false;
  }

#if 0
  // export data for debug
  std::string  outfile = "./export.json";

  //Create the stream and check it is ok
  std::ofstream ostream(outfile.c_str());
  if (!ostream.is_open())
    return false;

  cereal::JSONOutputArchive oarchive(ostream);

  oarchive(cereal::make_nvp("pose", data.gpsData) );
#endif

  return true;
};

// create map timstamp -> rotation / gps informations
bool create_gps_imu_map( SfM_Gps_Data & data,
                         std::map < std::string, Mat3 >  map_rotationPerTimestamp,
                         std::map < std::string, Vec3 >  map_translationPerTimestamp )
{
  //parse gps data set
  for( size_t i = 0 ; i < data.gpsData.size() ; ++i )
  {
      // extract pose information
      pose  rigI  = data.gpsData[i];

      // create timestamp
      std::string   timestamp = std::to_string(rigI.sec) + "_" + std::to_string(rigI.usec);

      //extract rotation information
      std::vector <double>  rigR = rigI.orientation;
      if( rigR.size() == 10) // if we have imu informations
      {
          Mat3  R = Mat3::Identity();

          // initialize R
          for( size_t k = 0; k < 3; ++k )
          {
            for( size_t l= 0; l < 3 ; ++l )
                R(k,l) = rigR[3*k+l];
          }

          //update map
          map_rotationPerTimestamp.insert(std::make_pair( timestamp, R) );
      }

  }

  return true;
};

#endif
