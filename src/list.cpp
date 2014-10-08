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

  cmd.add( make_option('i', sImageDir, "imageDirectory") );
  cmd.add( make_option('c', sChannelFile, "channelFile") );
  cmd.add( make_option('x', simagejXmlFile, "imagejXmlFile") );
  cmd.add( make_option('o', sOutputDir, "outputDirectory") );
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
            << "--focal " << focalPixPermm << std::endl;

  if ( !stlplus::folder_exists( sImageDir ) )
  {
    std::cerr << "\nThe input directory doesn't exist" << std::endl;
    return EXIT_FAILURE;
  }

  if (sOutputDir.empty())
  {
    std::cerr << "\nInvalid output directory" << std::endl;
    return EXIT_FAILURE;
  }

  if ( !stlplus::folder_exists( sOutputDir ) )
  {
    if ( !stlplus::folder_create( sOutputDir ))
    {
      std::cerr << "\nCannot create output directory" << std::endl;
      return EXIT_FAILURE;
    }
  }

  if ( !simagejXmlFile.empty() && focalPixPermm!= -1.0)
  {
     std::cerr << "\n Cannot combine -x and -f options " << std::endl;
     return EXIT_FAILURE;
  }

  if( simagejXmlFile.empty() && focalPixPermm== -1.0 )
  {
     std::cerr << "\n No ImageJ Xml file or focal given " << std::endl;
     return EXIT_FAILURE;
  }

  std::vector<std::string> vec_image = stlplus::folder_files( sImageDir );
  // Write the new file
  std::ofstream listTXT( stlplus::create_filespec( sOutputDir,
                                                   "lists.txt" ).c_str() );
  if ( listTXT )
  {
    std::sort(vec_image.begin(), vec_image.end());
    for ( std::vector<std::string>::const_iterator iter_image = vec_image.begin();
      iter_image != vec_image.end();
      iter_image++ )
    {
      // Read meta data to fill width height and focalPixPermm
      std::string sImageFilename = stlplus::create_filespec( sImageDir, *iter_image );

      // retrieve width and height of image using opencv
      IplImage* img = cvLoadImage(sImageFilename.c_str(), CV_LOAD_IMAGE_COLOR );

      // extract image width and height
      const unsigned int width  = img->width;
      const unsigned int height = img->height;

      std::ostringstream os;

      // Create list if focal is given
      if( focalPixPermm > 0.0 )
      {
           os << *iter_image << ";" << width << ";" << height;
           os << ";"
              << focalPixPermm << ";" << 0 << ";" << width/2.0 << ";"
              << 0 << ";" << focalPixPermm << ";" << height/2.0 << ";"
              << 0 << ";" << 0 << ";" << 1 << std::endl;
      }
      // Create list if imagej-elphel file is given
      if( !simagejXmlFile.empty() )
      {
          std::cout << " Not yet implemented " << endl;
      }

      // export list to file
      std::cout << os.str();
      listTXT << os.str();
    }
  }
  else
  {
     return EXIT_FAILURE;
  }

  listTXT.close();
  return EXIT_SUCCESS;

}
