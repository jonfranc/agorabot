// $Id: bt2vrml.cpp 1162 2011-03-25 23:22:39Z hornunga@informatik.uni-freiburg.de $

/**
* OctoMap:
* A probabilistic, flexible, and compact 3D mapping library for robotic systems.
* @author K. M. Wurm, A. Hornung, University of Freiburg, Copyright (C) 2009.
* @see http://octomap.sourceforge.net/
* License: New BSD License
*/

/*
 * Copyright (c) 2009, K. M. Wurm, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <octomap/octomap.h>
#include <fstream>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <list>

using namespace std;
using namespace octomap;

void printUsage(char* self){
  std::cerr << "\nUSAGE: " << self << " input.bt\n\n";

  std::cerr << "This tool will convert the occupied voxels of a binary OctoMap \n"
      "file input.bt to a VRML2.0 file input.bt.wrl.\n\n";

  std::cerr << "WARNING: The output files will be quite large!\n\n";

  exit(0);
}

int main(int argc, char** argv) {
  // default values:
  string vrmlFilename = "";
  string btFilename = "";

  if (argc != 2 || (argc > 1 && strcmp(argv[1], "-h") == 0)){
    printUsage(argv[0]);
  }

  btFilename = std::string(argv[1]);
  vrmlFilename = btFilename + ".wrl";


  cout << "\nReading OcTree file\n===========================\n";
  // TODO: check if file exists and if OcTree read correctly?
  OcTree* tree = new OcTree(btFilename);

  std::list<OcTreeVolume> occupiedVoxels;
  tree->getOccupied(occupiedVoxels);
  delete tree;

  cout << "\nWriting " << occupiedVoxels.size()<<" occupied volumes to VRML\n===========================\n";

  std::ofstream outfile (vrmlFilename.c_str());

  outfile << "#VRML V2.0 utf8\n#\n";
  outfile << "# created from OctoMap file "<<btFilename<< " with bt2vrml\n";

  std::list<OcTreeVolume>::iterator it;

  for (it = occupiedVoxels.begin(); it != occupiedVoxels.end(); ++it){
    outfile << "Transform { translation "
          << it->first.x() << " " << it->first.y() << " " << it->first.z()
          << " \n  children ["
          << " Shape { geometry Box { size "
          << it->second << " " << it->second << " " << it->second << "} } ]\n"
          << "}\n";
  }

  outfile.close();

  std::cout << "Finished writing to " << vrmlFilename << std::endl;

}
