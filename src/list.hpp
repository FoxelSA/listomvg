/*
 * listOMVG
 *
 * Copyright (c) 2013-2014 FOXEL SA - http://foxel.ch
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

#ifndef LIST_HPP_
#define LIST_HPP_

#include <iostream>
#include <iomanip>
#include <fstream>
#include <fastcal-all.h>
#include <ctype.h>
#include <unistd.h>
#include <vector>
#include <stdlib.h>
#include <string.h>
#include <cstring>
#include <set>
#include <map>

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

using namespace std;

#define DEBUG 0

enum Format {
  Pnm, Png, Jpg, Tiff, Unknown
};

Format GetFormat(const char *c);

/// The structure used to store intrinsic per image
typedef std::pair<std::string, std::vector<double> > imageNameAndIntrinsic;

static bool CmpFormatExt(const char *a, const char *b) {
  size_t len_a = strlen(a);
  size_t len_b = strlen(b);
  if (len_a != len_b) return false;
  for (size_t i = 0; i < len_a; ++i)
    if (tolower(a[i]) != tolower(b[i]))
      return false;
  return true;
}

Format GetFormat(const char *c) {
  const char *p = strrchr (c, '.');

  if (p == NULL)
    return Unknown;

  if (CmpFormatExt(p, ".png")) return Png;
  if (CmpFormatExt(p, ".ppm")) return Pnm;
  if (CmpFormatExt(p, ".pgm")) return Pnm;
  if (CmpFormatExt(p, ".pbm")) return Pnm;
  if (CmpFormatExt(p, ".pnm")) return Pnm;
  if (CmpFormatExt(p, ".jpg")) return Jpg;
  if (CmpFormatExt(p, ".jpeg")) return Jpg;
  if (CmpFormatExt(p, ".tif")) return Tiff;
  if (CmpFormatExt(p, ".tiff")) return Tiff;

  cerr << "Error: Couldn't open " << c << " Unknown file format" << std::endl;
  return Unknown;
}

bool operator==(const imageNameAndIntrinsic& i1, const imageNameAndIntrinsic& i2) {
  return (i1.first == i2.first);
}

bool operator!=(const imageNameAndIntrinsic& i1, const imageNameAndIntrinsic& i2) {
  return !(i1.first == i2.first);
}

// Lexicographical ordering of matches. Used to remove duplicates.
bool operator<(const imageNameAndIntrinsic& i1, const imageNameAndIntrinsic& i2) {
    return i1.first < i2.first;
}

/**
* Split an input string with a delimiter and fill a string vector
*/
static bool split ( const std::string src, const std::string& delim, std::vector<std::string>& vec_value )
{
  bool bDelimiterExist = false;
  if ( !delim.empty() )
  {
    vec_value.clear();
    std::string::size_type start = 0;
    std::string::size_type end = std::string::npos -1;

    while ( end != std::string::npos )
    {
      end = src.find ( delim, start );
      vec_value.push_back ( src.substr ( start, end - start ) );
      start = end + delim.size();
    }

    if ( vec_value.size() >= 2 )
      bDelimiterExist = true;
  }
  return bDelimiterExist;
}

/**
*  Given three angles, compute Elphel rotation
*/
 void computeRotationEl ( double * R , double az , double ele , double roll);

/**
*  Given three angles, entrance pupil forward, radius and height, compute optical center position.
*/

 void getOpticalCenter ( double * C ,
                const double & radius,
                const double & height,
                const double & azimuth,
                const double * R,
                const double & entrancePupilForward );

#endif /* LIST_HPP_ */
