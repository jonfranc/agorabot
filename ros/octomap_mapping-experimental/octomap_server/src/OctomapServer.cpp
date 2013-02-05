/**
* octomap_server: A Tool to serve 3D OctoMaps in ROS (binary and as visualization)
* (inspired by the ROS map_saver)
* @author A. Hornung, University of Freiburg, Copyright (C) 2009.
* @see http://octomap.sourceforge.net/
* License: BSD
*/

/*
 * Copyright (c) 2010, A. Hornung, University of Freiburg
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

#include <octomap_server/OctomapServer.h>

namespace octomap{

	OctomapServer::OctomapServer(const std::string& filename)
	  : m_nh(),
	    m_octoMap(0.05),
	    m_maxRange(-1.0),
	    m_frameId("/map"),
	    m_useHeightMap(true),
		m_colorFactor(0.8),
		m_visMinZ(-1.0*std::numeric_limits<double>::max()), m_visMaxZ(std::numeric_limits<double>::max())
	{
		ros::NodeHandle private_nh("~");
		private_nh.param("frame_id", m_frameId, m_frameId);
		private_nh.param("height_map", m_useHeightMap, m_useHeightMap);
		private_nh.param("color_factor", m_colorFactor, m_colorFactor);
		private_nh.param("vis_min_z", m_visMinZ, m_visMinZ);
		private_nh.param("vis_max_z", m_visMaxZ, m_visMaxZ);

		double res = 0.05;
		private_nh.param("resolution", res, res);
		m_octoMap.octree.setResolution(res);

		private_nh.param("max_sensor_range", m_maxRange, m_maxRange);

		double probHit = 0.7;
		double probMiss = 0.4;
		double thresMin = 0.12;
		double thresMax = 0.97;
		private_nh.param("sensor_model_hit", probHit, probHit);
		private_nh.param("sensor_model_miss", probMiss, probMiss);
		private_nh.param("sensor_model_min", thresMin, thresMin);
		private_nh.param("sensor_model_max", thresMax, thresMax);
		m_octoMap.octree.setProbHit(probHit);
		m_octoMap.octree.setProbMiss(probMiss);
		m_octoMap.octree.setClampingThresMin(thresMin);
		m_octoMap.octree.setClampingThresMax(thresMax);

		double r, g, b, a;
		private_nh.param("color/r", r, 0.0);
		private_nh.param("color/g", g, 0.0);
		private_nh.param("color/b", b, 1.0);
		private_nh.param("color/a", a, 1.0);
		m_color.r = r;
		m_color.g = g;
		m_color.b = b;
		m_color.a = a;


		m_markerPub = m_nh.advertise<visualization_msgs::MarkerArray>("occupied_cells_vis_array", 1,
				boost::bind(&OctomapServer::connectCallback, this, _1), ros::SubscriberStatusCallback(), ros::VoidConstPtr(), true);
		m_binaryMapPub = m_nh.advertise<octomap_ros::OctomapBinary>("octomap_binary", 1,
				boost::bind(&OctomapServer::connectCallback, this, _1), ros::SubscriberStatusCallback(), ros::VoidConstPtr(), true);
		m_pointCloudPub = m_nh.advertise<sensor_msgs::PointCloud2>("octomap_point_cloud_centers", 1,
				boost::bind(&OctomapServer::connectCallback, this, _1), ros::SubscriberStatusCallback(), ros::VoidConstPtr(), true);
		m_collisionObjectPub = m_nh.advertise<mapping_msgs::CollisionObject>("octomap_collision_object", 1,
				boost::bind(&OctomapServer::connectCallback, this, _1), ros::SubscriberStatusCallback(), ros::VoidConstPtr(), true);

		m_service = m_nh.advertiseService("octomap_binary", &OctomapServer::serviceCallback, this);

		if (filename != ""){
			m_octoMap.octree.readBinary(filename);
			ROS_INFO("Octomap file %s loaded (%zu nodes).", filename.c_str(),m_octoMap.octree.size());

			if (m_binaryMapPub.getNumSubscribers() > 0)
				publishMap();
			if (m_markerPub.getNumSubscribers() > 0)
				publishMarkers();
			if (m_pointCloudPub.getNumSubscribers() > 0)
				publishPointCloud();
			if (m_collisionObjectPub.getNumSubscribers() > 0)
				publishCollisionObject();
		}

		m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (m_nh, "cloud_in", 5);
		m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_pointCloudSub, m_tfListener, m_frameId, 5);
		m_tfPointCloudSub->registerCallback(boost::bind(&OctomapServer::insertCloudCallback, this, _1));


	}

	OctomapServer::~OctomapServer(){
		delete m_tfPointCloudSub;
		delete m_pointCloudSub;

	}

	void OctomapServer::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud){
		ros::WallTime startTime = ros::WallTime::now();
		// transform pointcloud from sensor frame to fixed_frame
		tf::StampedTransform trans;
		try {
			m_tfListener.waitForTransform(m_frameId, cloud->header.frame_id, cloud->header.stamp, ros::Duration(0.5));
			m_tfListener.lookupTransform (m_frameId, cloud->header.frame_id, cloud->header.stamp, trans);
		} catch(tf::TransformException& ex){
				ROS_WARN_STREAM( "Transform error from " << cloud->header.frame_id
					<< " to " << m_frameId << ", quitting callback");
				return;
		}

		Eigen::Matrix4f eigen_transform;
		sensor_msgs::PointCloud2 transformed_cloud;
		pcl_ros::transformAsMatrix (trans, eigen_transform);
		pcl_ros::transformPointCloud(eigen_transform, *cloud, transformed_cloud);

		geometry_msgs::Point origin;
		tf::pointTFToMsg(trans.getOrigin(), origin);
		m_octoMap.insertScan(transformed_cloud, origin, m_maxRange, true, true);
		m_octoMap.octree.updateInnerOccupancy();

		if (m_binaryMapPub.getNumSubscribers() > 0)
			publishMap(cloud->header.stamp);
		if (m_markerPub.getNumSubscribers() > 0)
			publishMarkers(cloud->header.stamp);
		if (m_pointCloudPub.getNumSubscribers() > 0)
			publishPointCloud(cloud->header.stamp);
		if (m_collisionObjectPub.getNumSubscribers() > 0)
			publishCollisionObject(cloud->header.stamp);

		double total_elapsed = (ros::WallTime::now() - startTime).toSec();
		ROS_DEBUG("Pointcloud insertion in OctomapServer done (%d pts, %f sec)", int(cloud->height*cloud->width), total_elapsed);


	}

	void OctomapServer::connectCallback(const ros::SingleSubscriberPublisher& pub){
		ROS_INFO("New subscriber on topic %s to OctomapServer connected", pub.getTopic().c_str());

		//(re)publish map data so that no outdated latched data is stored
		publishMap();
		publishMarkers();
		publishPointCloud();
		publishCollisionObject();
	}

	void OctomapServer::publishCollisionObject(const ros::Time& rostime){
		mapping_msgs::CollisionObject collisionObject;

		collisionObject.header.frame_id = m_frameId;
		collisionObject.header.stamp = rostime;
		collisionObject.id = "map";

		geometric_shapes_msgs::Shape shape;
		shape.type = geometric_shapes_msgs::Shape::BOX;
		shape.dimensions.resize(3);

		geometry_msgs::Pose pose;
		pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

		for (OcTreeROS::OcTreeType::iterator it = m_octoMap.octree.begin(),
				end = m_octoMap.octree.end(); it != end; ++it)
		{
			if (m_octoMap.octree.isNodeOccupied(*it)){
				double z = it.getZ();
				if (z > m_visMinZ && z < m_visMaxZ)
				{
					double size = it.getSize();
					shape.dimensions[0] = shape.dimensions[1] = shape.dimensions[2] = size;
					collisionObject.shapes.push_back(shape);
					pose.position.x = it.getX();
					pose.position.y = it.getY();
					pose.position.z = z;
					collisionObject.poses.push_back(pose);
				}
			}
		}

		m_collisionObjectPub.publish(collisionObject);


	}


	void OctomapServer::publishMarkers(const ros::Time& rostime){
		if (m_octoMap.octree.size() <= 1)
			return;

		double x, y, minZ, maxZ;
		m_octoMap.octree.getMetricMin(x, y, minZ);
		m_octoMap.octree.getMetricMax(y, y, maxZ);

		visualization_msgs::MarkerArray occupiedNodesVis;
		// each array stores all cubes of a different size, one for each depth level:
		occupiedNodesVis.markers.resize(m_octoMap.octree.getTreeDepth());
		double lowestRes = m_octoMap.octree.getResolution();


		unsigned numVoxels = 0;
		for (OcTreeROS::OcTreeType::iterator it = m_octoMap.octree.begin(),
				end = m_octoMap.octree.end(); it != end; ++it)
		{
			if (m_octoMap.octree.isNodeOccupied(*it)){
				double z = it.getZ();
				if (z > m_visMinZ && z < m_visMaxZ)
				{
					// which array to store cubes in?
					int idx = int(log2(it.getSize() / lowestRes) +0.5);
					assert (idx >= 0 && unsigned(idx) < occupiedNodesVis.markers.size());
					geometry_msgs::Point cubeCenter;
					cubeCenter.x = it.getX();
					cubeCenter.y = it.getY();
					cubeCenter.z = z;

						occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
						if (m_useHeightMap){
							double h = (1.0 - std::min(std::max((cubeCenter.z-minZ)/ (maxZ - minZ), 0.0), 1.0)) *m_colorFactor;
							occupiedNodesVis.markers[idx].colors.push_back(heightMapColor(h));
						}
						numVoxels++;
				}

			}

		}

		for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i){
			double size = lowestRes * pow(2,i);

			occupiedNodesVis.markers[i].header.frame_id = m_frameId;
			occupiedNodesVis.markers[i].header.stamp = rostime;
			occupiedNodesVis.markers[i].ns = "map";
			occupiedNodesVis.markers[i].id = i;
			occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
			occupiedNodesVis.markers[i].scale.x = size;
			occupiedNodesVis.markers[i].scale.y = size;
			occupiedNodesVis.markers[i].scale.z = size;
			occupiedNodesVis.markers[i].color = m_color;


			if (occupiedNodesVis.markers[i].points.size() > 0)
				occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
			else
				occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
		}

		m_markerPub.publish(occupiedNodesVis);
		ROS_DEBUG("%d occupied voxels visualized.", numVoxels);
	}

	void OctomapServer::publishPointCloud(const ros::Time& rostime){
		if (m_octoMap.octree.size() <= 1)
			return;

		pcl::PointCloud<pcl::PointXYZ> pclCloud;
		for (OcTreeROS::OcTreeType::iterator it = m_octoMap.octree.begin(),
				end = m_octoMap.octree.end(); it != end; ++it)
		{
			if (m_octoMap.octree.isNodeOccupied(*it)){
				double z = it.getZ();
				if (z > m_visMinZ && z < m_visMaxZ)
					pclCloud.push_back(pcl::PointXYZ(it.getX(), it.getY(), z));
			}
		}

		sensor_msgs::PointCloud2 cloud;
		pcl::toROSMsg (pclCloud, cloud);
		cloud.header.frame_id = m_frameId;
		cloud.header.stamp = rostime;
		m_pointCloudPub.publish(cloud);
	}

	bool OctomapServer::serviceCallback(octomap_ros::GetOctomap::Request  &req,
			octomap_ros::GetOctomap::Response &res)
	{
		ROS_INFO("Sending map data on service request");
		res.map.header.frame_id = m_frameId;
		res.map.header.stamp = ros::Time::now();
		octomap::octomapMapToMsgData(m_octoMap.octree, res.map.data);

		return true;
	}

	void OctomapServer::publishMap(const ros::Time& rostime){

		octomap_ros::OctomapBinary map;
		map.header.frame_id = m_frameId;
		map.header.stamp = rostime;

		octomap::octomapMapToMsgData(m_octoMap.octree, map.data);

		m_binaryMapPub.publish(map);
	}

	std_msgs::ColorRGBA OctomapServer::heightMapColor(double h) const {

		std_msgs::ColorRGBA color;
		color.a = 1.0;
		// blend over HSV-values (more colors)

		double s = 1.0;
		double v = 1.0;

		h -= floor(h);
		h *= 6;
		int i;
		double m, n, f;

		i = floor(h);
		f = h - i;
		if (!(i & 1))
			f = 1 - f; // if i is even
		m = v * (1 - s);
		n = v * (1 - s * f);

		switch (i) {
		case 6:
		case 0:
			color.r = v; color.g = n; color.b = m;
			break;
		case 1:
			color.r = n; color.g = v; color.b = m;
			break;
		case 2:
			color.r = m; color.g = v; color.b = n;
			break;
		case 3:
			color.r = m; color.g = n; color.b = v;
			break;
		case 4:
			color.r = n; color.g = m; color.b = v;
			break;
		case 5:
			color.r = v; color.g = m; color.b = n;
			break;
		default:
			color.r = 1; color.g = 0.5; color.b = 0.5;
			break;
		}

		return color;
	}
}



