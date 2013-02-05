/**
* octomap_2D server: serves a projected 3D octomap as 2D map for compatibility
* with 2D navigation
*
* @author A. Hornung, University of Freiburg, Copyright (C) 2011.
* @see http://octomap.sourceforge.net/
* License: BSD
*/

/*
 * Copyright (c) 2011, A. Hornung, University of Freiburg
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

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <octomap_ros/GetOctomap.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>


using namespace std;
 
/**
 * @brief Map generation node.
 */
class MapProjector{
public:
	MapProjector()
	 : m_minZRange(-std::numeric_limits<double>::max()),
	   m_maxZRange(std::numeric_limits<double>::max()),
	   m_minSizeX(0.0),
	   m_minSizeY(0.0)
	{
		ros::NodeHandle privateNh("~");

		privateNh.param("min_z_range", m_minZRange,m_minZRange);
		privateNh.param("max_z_range", m_maxZRange,m_maxZRange);
		privateNh.param("min_x_size", m_minSizeX,m_minSizeX);
		privateNh.param("min_y_size", m_minSizeY,m_minSizeY);

		m_mapPub = m_nh.advertise<nav_msgs::OccupancyGrid>("map", 5, true);
		m_octomapSub = m_nh.subscribe("octomap_binary",5, &MapProjector::octomapBinaryCB, this);
	}

	void octomapBinaryCB(const octomap_ros::OctomapBinaryConstPtr& octomapMsg){
		nav_msgs::OccupancyGrid map;
		octomap::OcTree octree(0.1);
		octomap::octomapMsgDataToMap(octomapMsg->data, octree);
		map.info.resolution = octree.getResolution();
		map.header = octomapMsg->header;
		double minX, minY, minZ;
		double maxX, maxY, maxZ;
		octree.getMetricMin(minX, minY, minZ);
		octree.getMetricMax(maxX, maxY, maxZ);

		// override huge size of empty tree:
		if (octree.size() <= 1){
			minX = minY = minZ = 0.0;
			maxX = maxY = maxZ = 0.0;
		}

		minZ = std::max(m_minZRange, minZ);
		maxZ = std::min(m_maxZRange, maxZ);

		octomap::point3d minPt(minX, minY, minZ);
		octomap::point3d maxPt(maxX, maxY, maxZ);
		octomap::OcTreeKey minKey, maxKey, curKey;
		if (!octree.genKey(minPt, minKey)){
			ROS_ERROR("Could not create min OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
			return;
		}
		if (!octree.genKey(maxPt, maxKey)){
			ROS_ERROR("Could not create max OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
			return;
		}

		ROS_DEBUG("MinKey: %d %d %d / MaxKey: %d %d %d", minKey[0], minKey[1], minKey[2], maxKey[0], maxKey[1], maxKey[2]);

		// add padding if requested (= new min/maxPts in x&y):
		double halfPaddedX = 0.5*m_minSizeX;
		double halfPaddedY = 0.5*m_minSizeY;
		minX = std::min(minX, -halfPaddedX);
		maxX = std::max(maxX, halfPaddedX);
		minY = std::min(minY, -halfPaddedY);
		maxY = std::max(maxY, halfPaddedY);
		minPt = octomap::point3d(minX, minY, minZ);
		maxPt = octomap::point3d(maxX, maxY, maxZ);

		octomap::OcTreeKey paddedMinKey, paddedMaxKey;
		if (!octree.genKey(minPt, paddedMinKey)){
			ROS_ERROR("Could not create padded min OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
			return;
		}
		if (!octree.genKey(maxPt, paddedMaxKey)){
			ROS_ERROR("Could not create padded max OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
			return;
		}

		ROS_DEBUG("Padded MinKey: %d %d %d / padded MaxKey: %d %d %d", paddedMinKey[0], paddedMinKey[1], paddedMinKey[2], paddedMaxKey[0], paddedMaxKey[1], paddedMaxKey[2]);
		assert(paddedMaxKey[0] >= maxKey[0] && paddedMaxKey[1] >= maxKey[1]);

		map.info.width = paddedMaxKey[0] - paddedMinKey[0] +1;
		map.info.height = paddedMaxKey[1] - paddedMinKey[1] +1;
		int mapOriginX = minKey[0] - paddedMinKey[0];
		int mapOriginY = minKey[1] - paddedMinKey[1];
		assert(mapOriginX >= 0 && mapOriginY >= 0);

		// might not exactly be min / max of octree:
		octomap::point3d origin;
		octree.genCoords(paddedMinKey, octree.getTreeDepth()-1, origin);
		map.info.origin.position.x = origin.x() - octree.getResolution()*0.5;
		map.info.origin.position.y = origin.y() - octree.getResolution()*0.5;

		// Allocate space to hold the data (init to unknown)
		map.data.resize(map.info.width * map.info.height, -1);
		// iterate over all keys:
		unsigned i, j;
		for (curKey[1]=minKey[1], j=mapOriginY; curKey[1] <= maxKey[1]; ++curKey[1], ++j){
			for (curKey[0]=minKey[0], i=mapOriginX; curKey[0] <= maxKey[0]; ++curKey[0], ++i){
				bool allUnknown = true;
				for (curKey[2]=minKey[2]; curKey[2] <= maxKey[2]; ++curKey[2]){ //iterate over height
					octomap::OcTreeNode* node = octree.search(curKey);
					if (node){
						allUnknown = false;
						if(octree.isNodeOccupied(node)){
							map.data[map.info.width*j + i] = 100;
							break;
						}
					}
				}

				if (!allUnknown && map.data[map.info.width*j + i] == -1)
					map.data[map.info.width*j + i] = 0;
			}
		}


		ROS_DEBUG("Finished generating %d x %d 2D map", map.info.width, map.info.height);
		m_mapPub.publish(map);

	}


  protected:
    ros::NodeHandle m_nh;
    ros::Subscriber m_octomapSub;
    ros::Publisher m_mapPub;
    double m_minZRange;
    double m_maxZRange;
    double m_minSizeX;
    double m_minSizeY;
};


int main(int argc, char** argv){
  ros::init(argc, argv, "octomap_2D_server");

  try{
	  MapProjector ms;
	  ros::spin();
  }catch(std::runtime_error& e){
	  ROS_ERROR("octomap_2D_server exception: %s", e.what());
	  return -1;
  }

  return 0;
}


