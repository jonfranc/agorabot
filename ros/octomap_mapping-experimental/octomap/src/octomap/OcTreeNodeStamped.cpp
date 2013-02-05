// $Id: OcTreeNodeStamped.cpp 1162 2011-03-25 23:22:39Z hornunga@informatik.uni-freiburg.de $

#include "octomap/OcTreeNodeStamped.h"

namespace octomap {


  OcTreeNodeStamped::OcTreeNodeStamped()
    : OcTreeNode(), timestamp(0) {
  }


  bool OcTreeNodeStamped::createChild(unsigned int i) {
    if (itsChildren == NULL) {
      allocChildren();
    }
    itsChildren[i] = new OcTreeNodeStamped();
    return true;
  }


} // end namespace


