#ifndef OCTOMAP_OCTREE_LABELED_H
#define OCTOMAP_OCTREE_LABELED_H

// $Id: OcTreeLabeled.h 1162 2011-03-25 23:22:39Z hornunga@informatik.uni-freiburg.de $

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



#include "OccupancyOcTreeBase.h"
#include "OcTreeNodeLabeled.h"

namespace octomap {

  class OcTreeLabeled : public OccupancyOcTreeBase <OcTreeNodeLabeled> {

  public:
    static const int TREETYPE=4;

  public:

    OcTreeLabeled(double _resolution);
    virtual ~OcTreeLabeled();

    void toMaxLikelihood();

    void getChangedFreespace(std::list<OcTreeVolume>& nodes) const;

    void writeBinary(std::string filename); // TODO


  protected:

    void getChangedFreespaceRecurs(OcTreeNodeLabeled* node, unsigned int depth, unsigned int max_depth, 
                                   const point3d& parent_center, std::list<OcTreeVolume>& nodes) const;


    void toMaxLikelihoodRecurs(OcTreeNodeLabeled* node, unsigned int depth, unsigned int max_depth);

  };


} // end namespace

#endif
