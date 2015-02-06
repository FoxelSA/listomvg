/*
 * listOMV
 *
 * Copyright (c) 2014-2015 FOXEL SA - http://foxel.ch
 * Please read <http://foxel.ch/license> for more information.
 *
 *
 * Author(s):
 *
 *      Stéphane Flotron <s.flotron@foxel.ch>
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


#include <tools.hpp>
#include <types.hpp>
#include <list_utils.hpp>
#include <stdlib.h>
#include <ctype.h>
#include <stdlib.h>
#include <fstream>
#include <cstring>
#include "../lib/stlplus3/filesystemSimplified/file_system.hpp"
#include "../lib/cmdLine/cmdLine.h"

using namespace std;

int main(int argc, char **argv)
{
  CmdLine cmd;

  std::string sImageDir,
    sChannelFile = "",
    smacAddress = "",
    sOutputDir = "",
    sMountPoint = "";

  li_Real_t     focalPixPermm = -1.0;
  bool          bRigidRig     = true;
  bool          bUseCalibPrincipalPoint = false;

  cmd.add( make_option('i', sImageDir, "imageDirectory") );
  cmd.add( make_option('m', smacAddress, "macAddress") );
  cmd.add( make_option('o', sOutputDir, "outputDirectory") );
  cmd.add( make_option('d', sMountPoint, "mountPoint") );
  cmd.add( make_option('c', sChannelFile, "channelFile") );
  cmd.add( make_option('r', bRigidRig, "rigidRig") );
  cmd.add( make_option('p', bUseCalibPrincipalPoint, "useCalibPrincipalPoint") );
  cmd.add( make_option('f', focalPixPermm, "focal") );

  try {
      if (argc == 1) throw std::string("Invalid command line parameter.");
      cmd.process(argc, argv);
  } catch(const std::string& s) {
      std::cerr << "Usage: " << argv[0] << '\n'
      << "[-i|--imageDirectory]\n"
      << "[-m]--macAddress\n"
      << "[-o|--outputDirectory]\n"
      << "[-d]--mountPoint]\n"
      << "[-c|--channelFile]\n"
      << "[-r]--rigidRig \n"
      << "   -r 0 : no rigid rig \n"
      << "   -r 1 : with rigid rig structure\n"
      << "[-p]--useCalibPrincipalPoint\n"
      << "   -p 0 : do not use calibration principal point \n"
      << "   -p 1 : use calibration principal point \n"
      << "[-f|--focal] (pixels)\n"
      << std::endl;

      std::cerr << s << std::endl;
      return EXIT_FAILURE;
  }

  // verify that input argumet are valid
  const bool bValidInput = isInputValid
                              (
                                    argv[0],
                                    sImageDir,
                                    sOutputDir,
                                    smacAddress,
                                    sMountPoint,
                                    sChannelFile,
                                    bRigidRig,
                                    bUseCalibPrincipalPoint,
                                    focalPixPermm
                              );
   if( !bValidInput )
   {
       std::cerr << " Input data are not valid. Please check your input\n" << std::endl;
       return EXIT_FAILURE;
   }
   else  // if input data is valid, go ahead
   {

       // now extract calibration information related to each module
       std::vector < sensorData > vec_sensorData;
       const bool bLoadCalibration = loadCalibrationData( vec_sensorData, sMountPoint, smacAddress);

       if( !bLoadCalibration )
       {
            return EXIT_FAILURE;
       }
       else  // if calibration information are loaded, go ahead
       {

           // load image filename
           std::vector<std::string> vec_image = stlplus::folder_files( sImageDir );

           // load kept channel
           std::vector< li_Size_t > keptChan;
           loadChannelFile( keptChan, sChannelFile );

           //create and export list to folder
           bool isExported =  computeInstrinsicPerImages(
                                   vec_image,
                                   vec_sensorData,
                                   keptChan,
                                   sImageDir,
                                   sOutputDir,
                                   focalPixPermm,
                                   bUseCalibPrincipalPoint,
                                   bRigidRig );

          // do final check to ensure all went well
          if( isExported )
          {
              std::cout << "Sucessfully exported list to folder. Quit" << std::endl;
              return EXIT_SUCCESS;
          }
          else
          {
              std::cerr << "Could not export list to folder. Exit " << std::endl;
              return EXIT_FAILURE;
          }
     }

   }
}
