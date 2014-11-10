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
#include <fstream>
#include <sstream>
#include <memory>
#include <string>
#include <vector>
#include <iomanip>
#include <ctype.h>
#include <unistd.h>
#include <string.h>

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <elphelphg/cameraArray.hpp>
#include <elphelphg/camera.hpp>
#include <elphelphg/channel.hpp>
#include <elphelphg/file.hpp>
#include <elphelphg/xml.hpp>
#include <elphelphg/eqrData.hpp>
#include <elphelphg/sensorData.hpp>
#include <elphelphg/utils.hpp>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>

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

#endif /* LIST_HPP_ */
