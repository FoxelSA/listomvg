/*
 * listOMV
 *
 * Copyright (c) 2013-2014 FOXEL SA - http://foxel.ch
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


#include "list.hpp"
#include "progress.hpp"

using namespace std;

int main(int argc, char **argv)
{
  CmdLine cmd;

  std::string sImageDir,
    sChannelFile = "",
    smacAddress = "",
    sOutputDir = "",
    sMountPoint = "";

  double focalPixPermm = -1.0;
  bool   bRigidRig     = true;
  bool   bUseCalibPrincipalPoint = false;

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

  std::cout << " You called : " <<std::endl
            << argv[0] << std::endl
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
    return EXIT_FAILURE;
  }

  // check if output dir is given
  if (sOutputDir.empty())
  {
    std::cerr << "\nInvalid output directory" << std::endl;
    return EXIT_FAILURE;
  }

  // if output dir is empty, create it
  if ( !stlplus::folder_exists( sOutputDir ) )
  {
    if ( !stlplus::folder_create( sOutputDir ))
    {
      std::cerr << "\nCannot create output directory" << std::endl;
      return EXIT_FAILURE;
    }
  }

  // check if mac address is given
  if( smacAddress.empty() )
  {
     std::cerr << "\n No mac address given " << std::endl;
     return EXIT_FAILURE;
  }

  // check if mount point is given
  if( sMountPoint.empty() )
  {
     std::cerr << "\n No mount point given " << std::endl;
     return EXIT_FAILURE;
  }

  // extract used calibration informations
  /* Key/value-file descriptor */
  lf_Descriptor_t lfDesc;

  lf_Size_t lfWidth = 0;
  lf_Size_t lfHeight = 0;
  lf_Size_t lfChannels = 0;

  lf_Real_t lfFocalLength = 0.0;
  lf_Real_t lfPixelSize = 0.0;
  lf_Real_t lfAzimuth = 0.0;
  lf_Real_t lfHeading = 0.0;
  lf_Real_t lfElevation = 0.0;
  lf_Real_t lfRoll = 0.0;
  lf_Real_t lfpx0 = 0.0;
  lf_Real_t lfpy0 = 0.0;
  lf_Real_t lfRadius = 0.0;
  lf_Real_t lfCheight = 0.0;
  lf_Real_t lfEntrance = 0.0;

  /* Creation and verification of the descriptor */
  char *c_data = new char[sMountPoint.length() + 1];
  std::strcpy(c_data, sMountPoint.c_str());

  char *c_mac = new char[smacAddress.length() + 1];
  std::strcpy(c_mac, smacAddress.c_str());

  if ( lf_parse( (unsigned char*)c_mac, (unsigned char*)c_data, & lfDesc ) == LF_TRUE ) {

    /* Query number of camera channels */
    lfChannels = lf_query_channels( & lfDesc );
  }
  else
  {
    std::cerr << " Could not read calibration data. " << std::endl;
    return EXIT_FAILURE;
  }

  // now extract calibration information related to each module
  std::vector <lf_Size_t > vec_width;
  std::vector <lf_Size_t > vec_height;
  std::vector <lf_Real_t > vec_focal;
  std::vector <lf_Real_t > vec_px0;
  std::vector <lf_Real_t > vec_py0;
  std::vector <vector <double> > vec_rotations;
  std::vector <vector <double> > vec_offsets;

  for( lf_Size_t sensor_index = 0 ; sensor_index < lfChannels ; ++sensor_index )
  {
    /* Query number width and height of sensor image */
    lfWidth = lf_query_pixelCorrectionWidth ( sensor_index, & lfDesc );
    lfHeight = lf_query_pixelCorrectionHeight( sensor_index, & lfDesc );

    vec_width.push_back ( lfWidth );
    vec_height.push_back( lfHeight );

    /* Query focal length of camera sensor index */
    lfFocalLength = lf_query_focalLength( sensor_index , & lfDesc );
    lfPixelSize = lf_query_pixelSize ( sensor_index , & lfDesc );

    vec_focal.push_back( lfFocalLength / lfPixelSize );

    /* Query angles used for gnomonic rotation */
    lfAzimuth = lf_query_azimuth ( sensor_index , & lfDesc );
    lfHeading = lf_query_heading ( sensor_index , & lfDesc );
    lfElevation = lf_query_elevation ( sensor_index , & lfDesc );
    lfRoll = lf_query_roll ( sensor_index , & lfDesc );

    // compute rotation and store it.
    double R[9] = {0};
    computeRotationEl ( &R[0] , lfAzimuth + lfHeading, lfElevation, lfRoll);
    vector <double > temp;

    for(int i(0); i <9 ; ++i)
      temp.push_back(R[i]);

    vec_rotations.push_back(temp);

    /* Query principal point */
    lfpx0 = lf_query_px0 ( sensor_index , & lfDesc );
    lfpy0 = lf_query_py0 ( sensor_index , & lfDesc );

    vec_px0.push_back(lfpx0);
    vec_py0.push_back(lfpy0);

    /* Query information related to entrance pupil center */
    lfRadius = lf_query_radius ( sensor_index , & lfDesc );
    lfCheight = lf_query_height ( sensor_index , & lfDesc );
    lfEntrance = lf_query_entrancePupilForward ( sensor_index , & lfDesc );

    // compute optical center in camera coordinate and store it
    double C[3] = {0,0,0};
    getOpticalCenter ( &C[0] , lfRadius, lfCheight, lfAzimuth, R, lfEntrance );

    vector<double> Cp;
    Cp.push_back(C[0]);  Cp.push_back(C[1]); Cp.push_back(C[2]);
    vec_offsets.push_back(Cp);
  }

  /* Release descriptor */
  lf_release( & lfDesc );

  // load image filename
  std::vector<std::string> vec_image = stlplus::folder_files( sImageDir );

  // load kept channel
  std::vector< unsigned int > keptChan;

  if( !sChannelFile.empty() ){
    std::ifstream  inFile(sChannelFile.c_str());
    unsigned int chan;

    if( inFile.is_open() )
    {
         while( inFile >> chan )
         {
           keptChan.push_back(chan);
         }
    }

    inFile.close();
  }

  if( keptChan.empty() )
  {
    std::cerr << "\n Keep all channels " << std::endl;
  }

  //initialize rig map
  std::map<std::string, unsigned int>  mapRigPerImage;

  // Write the new file
  std::ofstream listTXT( stlplus::create_filespec( sOutputDir,
                                                   "lists.txt" ).c_str() );
  if ( listTXT )
  {
    std::sort(vec_image.begin(), vec_image.end());

    // create output
    std::set<imageNameAndIntrinsic> camAndIntrinsics;

    C_Progress_display my_progress_bar_image( vec_image.size(),
    std::cout, "\n List computation progession:\n");

    // do a parallel loop to improve CPU TIME
    #pragma omp parallel for
    for ( int idx = 0; idx < (int) vec_image.size(); ++idx)
    {
      //advance iterator
      std::vector<std::string>::const_iterator iter_image = vec_image.begin();
      std::advance(iter_image, idx);

      // Read meta data to fill width height and focalPixPermm
      std::string sImageFilename = stlplus::create_filespec( sImageDir, *iter_image );

      // Test if the image format is supported:
      if (GetFormat(sImageFilename.c_str()) == Unknown)
        continue; // image cannot be opened

       // extract channel information from image name
      std::vector<string> splitted_name;

      split( *iter_image, "-", splitted_name );
      unsigned int sensor_index=atoi(splitted_name[1].c_str());

      // extract image width and height
      const unsigned int width  = vec_width[sensor_index];
      const unsigned int height = vec_height[sensor_index];

      // now load image information and keep channel index and timestamp
      std::string timestamp=splitted_name[0];

      // check if channel is kept
      bool  bKeepChannel(false);

      // if no channel file is given, keep all images
      if( keptChan.empty() )
          bKeepChannel = true ;
      else
      {
        for( unsigned int i(0) ; i < keptChan.size() ; ++i )
        {
          if( sensor_index == keptChan[i] )
              bKeepChannel = true;
        }
      }

      // create output
      std::vector <double> intrinsic;

      // export only kept channel
      if( bKeepChannel )
      {

          // identify to which rig belongs the camera
          unsigned int  rig_index;

          // insert timestamp in the map
          std::pair<std::map< std::string, unsigned int>::iterator,bool> ret;
          #pragma omp critical
          {
            ret = mapRigPerImage.insert ( std::pair<std::string, unsigned int>(timestamp, mapRigPerImage.size()) );
            if(ret.second == true )
            {
                mapRigPerImage[timestamp] = mapRigPerImage.size()-1;
            }
          }

          //extract rig_index
          rig_index = mapRigPerImage[timestamp];

          // Create list if focal is given
          if( focalPixPermm > 0.0 )
          {
               intrinsic.push_back(width);
               intrinsic.push_back(height);

              if(!bUseCalibPrincipalPoint)
              {
                  intrinsic.push_back(focalPixPermm); intrinsic.push_back(0.0);           intrinsic.push_back(width /2.0);
                  intrinsic.push_back(0.0);           intrinsic.push_back(focalPixPermm); intrinsic.push_back(height/2.0);
                  intrinsic.push_back(0.0);           intrinsic.push_back(0.0);           intrinsic.push_back(1.0);
              }
              else
              {
                  intrinsic.push_back(focalPixPermm); intrinsic.push_back(0.0);           intrinsic.push_back(vec_px0[sensor_index]);
                  intrinsic.push_back(0.0);           intrinsic.push_back(focalPixPermm); intrinsic.push_back(vec_py0[sensor_index]);
                  intrinsic.push_back(0.0);           intrinsic.push_back(0.0);           intrinsic.push_back(1.0);
              }
          }
          // Create list if full calibration data are used
          else
          {
              const double  focalPixPermm = vec_focal[sensor_index];

              intrinsic.push_back(width);
              intrinsic.push_back(height);

              intrinsic.push_back(focalPixPermm); intrinsic.push_back(0.0);           intrinsic.push_back(vec_px0[sensor_index]);
              intrinsic.push_back(0.0);           intrinsic.push_back(focalPixPermm); intrinsic.push_back(vec_py0[sensor_index]);
              intrinsic.push_back(0.0);           intrinsic.push_back(0.0);           intrinsic.push_back(1.0);
          }

          // if rigid rig, add some informations
          if(bRigidRig)
          {
              // export rig index
              intrinsic.push_back(rig_index);

              // export channel
              intrinsic.push_back(sensor_index);

              // export rotation
              vector<double> R = vec_rotations[sensor_index];
              intrinsic.push_back(R[0]); intrinsic.push_back(R[1]); intrinsic.push_back(R[2]);
              intrinsic.push_back(R[3]); intrinsic.push_back(R[4]); intrinsic.push_back(R[5]);
              intrinsic.push_back(R[6]); intrinsic.push_back(R[7]); intrinsic.push_back(R[8]);

              // export translation
              vector<double> C = vec_offsets[sensor_index];
              intrinsic.push_back( C[0] );
              intrinsic.push_back( C[1] );
              intrinsic.push_back( C[2] );
          };

          //export info
          #pragma omp critical
          camAndIntrinsics.insert(std::make_pair(*iter_image, intrinsic));
        };

      //update progress bar
      #pragma omp critical
      {
        ++my_progress_bar_image;
      }
    }

    C_Progress_display my_progress_bar_export( camAndIntrinsics.size(),
    std::cout, "\n Write list in file lists.txt :\n");

    // export list to file
    for ( int img = 0; img < (int) camAndIntrinsics.size(); ++img)
    {
        //advance iterator
        std::set<imageNameAndIntrinsic>::const_iterator iter = camAndIntrinsics.begin();
        std::advance(iter, img);

        // create stream
        std::ostringstream os;
        os.precision(6);

        os << iter->first ;

        // retreive intrinsic info
        std::vector<double> intrinsic = iter->second;

        // export instrinsics
        for(int j=0; j < (int) intrinsic.size() ; ++j )
          os << ";"  << intrinsic[j];

        os << endl;

        listTXT << os.str();

        ++my_progress_bar_export;
    }

  }
  else
  {
     return EXIT_FAILURE;
  }

  listTXT.close();
  return EXIT_SUCCESS;

}

/**
*  Given 4 angles, compute Elphel rotation
*/
 void computeRotationEl ( double * R , double az , double ele , double roll)
 {
    //z-axis rotation
    double Rz[3][3] = {
       { cos(roll),-sin(roll), 0.0},
       {-sin(roll),-cos(roll), 0.0},
       {       0.0,      0.0, 1.0} };

    // x-axis rotation
    double Rx[3][3] = {
       {1.0,      0.0,     0.0},
       {0.0, cos(ele),sin(ele)},
       {0.0,-sin(ele),cos(ele)} };

    // y axis rotation
    double Ry[3][3] = {
       { cos( az ),  0.0, sin( az )},
       {       0.0, -1.0,       0.0},
       {-sin( az ),  0.0, cos( az )} };

    // 3) R = R2*R1*R0 transform sensor coordinate to panorama coordinate
    double RxRz[3][3] = {0.0};
    double   RT[3][3] = {0.0};

    // compute product of rotations
    int i=0, j=0;

    for(i=0 ; i < 3 ; ++i)
      for(j=0; j < 3 ; ++j)
         RxRz[i][j] = Rx[i][0] * Rz[0][j] + Rx[i][1] * Rz[1][j] + Rx[i][2] * Rz[2][j];

    for(i=0 ; i < 3 ; ++i)
      for(j=0; j < 3 ; ++j)
         RT[i][j] = Ry[i][0] * RxRz[0][j] + Ry[i][1] * RxRz[1][j] + Ry[i][2] * RxRz[2][j];

    // transpose because we need the transformation panorama to sensor coordinate !
    R[0] = RT[0][0];
    R[1] = RT[1][0];
    R[2] = RT[2][0];
    R[3] = RT[0][1];
    R[4] = RT[1][1];
    R[5] = RT[2][1];
    R[6] = RT[0][2];
    R[7] = RT[1][2];
    R[8] = RT[2][2];
}

/**
*  Given three angles, entrance pupil forward, radius and height, compute optical center position.
*/

 void getOpticalCenter ( double * C ,
                const double & radius,
                const double & height,
                const double & azimuth,
                const double * R,
                const double & entrancePupilForward )
{
  // compute lense Center from data
  double lensCenter[3] = {0.0, 0.0, 0.0};

  lensCenter[0] = radius * sin(azimuth);
  lensCenter[1] = height ;
  lensCenter[2] = radius * cos(azimuth);

  // C = lensCenter + R.entrancePupilForward, where R is roation sensor to world.
  C[0] =  lensCenter[0] + R[6] * entrancePupilForward;
  C[1] = -lensCenter[1] + R[7] * entrancePupilForward;
  C[2] =  lensCenter[2] + R[8] * entrancePupilForward;

}
