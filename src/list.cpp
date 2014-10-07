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
using namespace elphelphg;

int main(int argc, char** argv) {

    /* Usage branch */
    if ( argc<3 || argc>4 || !strcmp( argv[1], "help" ) || !strcmp(argv[1],"-h") || !strcmp(argv[1],"--help")  ) {
        /* Display help */
        printf( "Usage : %s <imagej_prefs_xml> <list_of_image.txt> \n\n",argv[0]);
        return 1;
    }

    // load inputs
    char *imagej_prefs_xml=argv[1];  //imagej xml configuration file

    // now load image, and do gnomonic projection
    try {

      cout << " Hello world " << endl;
      printf( " %s \n", imagej_prefs_xml);

    } catch(std::exception &e) {
      std::cerr << e.what() << std::endl;
      return 1;
    } catch(std::string &msg) {
      std::cerr << msg << std::endl;
      return 1;
    } catch(...) {
      std::cerr << "unhandled exception\n";
      return 1;
    }
}
