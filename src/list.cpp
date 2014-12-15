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
using namespace cv;
using namespace elphelphg;

int main(int argc, char **argv)
{
  CmdLine cmd;

  std::string sImageDir,
    sChannelFile = "",
    simagejXmlFile = "",
    sOutputDir = "";

  double focalPixPermm = -1.0;
  bool   bRigidRig     = false;
  bool   bUseCalibPrincipalPoint = false;

  cmd.add( make_option('i', sImageDir, "imageDirectory") );
  cmd.add( make_option('c', sChannelFile, "channelFile") );
  cmd.add( make_option('x', simagejXmlFile, "imagejXmlFile") );
  cmd.add( make_option('o', sOutputDir, "outputDirectory") );
  cmd.add( make_option('r', bRigidRig, "rigidRig") );
  cmd.add( make_option('p', bUseCalibPrincipalPoint, "useCalibPrincipalPoint") );
  cmd.add( make_option('f', focalPixPermm, "focal") );

  try {
      if (argc == 1) throw std::string("Invalid command line parameter.");
      cmd.process(argc, argv);
  } catch(const std::string& s) {
      std::cerr << "Usage: " << argv[0] << '\n'
      << "[-i|--imageDirectory]\n"
      << "[-c|--channelFile]\n"
      << "[-x]--imagejXmlFile\n"
      << "[-o|--outputDirectory]\n"
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
            << "--channelFile " << sChannelFile << std::endl
            << "--imagejXmlFile " << simagejXmlFile << std::endl
            << "--outputDirectory " << sOutputDir << std::endl
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

  // check if imagej xml file is given
  if( simagejXmlFile.empty() )
  {
     std::cerr << "\n No ImageJ Xml file given " << std::endl;
     return EXIT_FAILURE;
  }

  // Load imagej xml file
   CameraArray e4pi(CameraArray::EYESIS4PI_CAMERA,simagejXmlFile.c_str());

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

      // retrieve width and height of image using opencv
      IplImage* img = cvLoadImage(sImageFilename.c_str(), CV_LOAD_IMAGE_COLOR );

      // extract image width and height
      const unsigned int width  = img->width;
      const unsigned int height = img->height;

      // now load image information and keep channel index and timestamp
      struct utils::imagefile_info *info=utils::imagefile_parsename(sImageFilename.c_str());
      unsigned int channel_index=atoi(info->channel);
      std::string timestamp=info->timestamp;

      Channel *channel=e4pi.channel(channel_index);
      SensorData *sensor=channel->sensor;

      // check if channel is kept
      bool  bKeepChannel(false);

      // if no channel file is given, keep all images
      if( keptChan.empty() )
          bKeepChannel = true ;
      else
      {
        for(unsigned int i(0) ; i < keptChan.size() ; ++i )
        {
          if( channel_index == keptChan[i] )
              bKeepChannel = true;
        }
      }

      // compute optical center and rotation
      channel->getLensCenterVector();
      channel->getRotation();

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
                  intrinsic.push_back(focalPixPermm); intrinsic.push_back(0.0);           intrinsic.push_back(sensor->px0);
                  intrinsic.push_back(0.0);           intrinsic.push_back(focalPixPermm); intrinsic.push_back(sensor->py0);
                  intrinsic.push_back(0.0);           intrinsic.push_back(0.0);           intrinsic.push_back(1.0);
              }
          }
          // Create list if imagej-elphel file is given
          else
          {
              const double  focalPixPermm = sensor->focalLength / (0.001 * sensor->pixelSize);

              intrinsic.push_back(sensor->pixelCorrectionWidth);
              intrinsic.push_back(sensor->pixelCorrectionHeight);

              intrinsic.push_back(focalPixPermm); intrinsic.push_back(0.0);           intrinsic.push_back(sensor->px0);
              intrinsic.push_back(0.0);           intrinsic.push_back(focalPixPermm); intrinsic.push_back(sensor->py0);
              intrinsic.push_back(0.0);           intrinsic.push_back(0.0);           intrinsic.push_back(1.0);
          }

          // if rigid rig, add some informations
          if(bRigidRig)
          {
              // export rig index
              intrinsic.push_back(rig_index);

              // export channel
              intrinsic.push_back(channel_index);

              // export rotation
              intrinsic.push_back(channel->R[0]); intrinsic.push_back(channel->R[1]); intrinsic.push_back(channel->R[2]);
              intrinsic.push_back(channel->R[3]); intrinsic.push_back(channel->R[4]); intrinsic.push_back(channel->R[5]);
              intrinsic.push_back(channel->R[6]); intrinsic.push_back(channel->R[7]); intrinsic.push_back(channel->R[8]);

              // export translation
              intrinsic.push_back(channel->lensCenterVector[0] / 1000.0);
              intrinsic.push_back(channel->lensCenterVector[1] / 1000.0);
              intrinsic.push_back(channel->lensCenterVector[2] / 1000.0);
          };

          //export info
          #pragma omp critical
          camAndIntrinsics.insert(std::make_pair(*iter_image, intrinsic));
        };

      //free memory
      cvReleaseImage(&img);

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
