/*  Copyright (C) 2010 UT-Austin &  Austin Robot Technology,
 *  David Claridge, Michael Quinlan
 *  Copyright (C) 2012 Jack O'Quin
 * 
 *  License: Modified BSD Software License 
 */
/** @file
		@brief ROS class for detecting obstacles in a point cloud.
	 This class produces a point cloud containing all points that lie on
	 an obstacle and are taller than the @c height_threshold parameter.
Subscribes:
- @b velodyne_points [sensor_msgs::PointCloud2] data from one
	revolution of the Velodyne LIDAR
Publishes:
- @b veloydne_obstacles [sensor_msgs::PointCloud2] grid cells that
	contain an obstacle
- @b veloydne_clear [sensor_msgs::PointCloud2] grid cells with no
	obstacles

@author David Claridge, Michael Quinlan 

*/
#include <ros/ros.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <omp.h>
//#include <armadillo>

class HeightMapStereo
{
	public:

  	typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

		HeightMapStereo(ros::NodeHandle node, ros::NodeHandle priv_nh)
		{
			// get parameters using private node handle
			priv_nh.param("cell_size", m_per_cell_, 0.10);
			priv_nh.param("full_clouds", full_clouds_, true);
			priv_nh.param("grid_dimensions", grid_dim_, 1000);
			priv_nh.param("height_threshold", height_diff_threshold_, .10);

			ROS_INFO_STREAM("height map parameters: "
									<< grid_dim_ << "x" << grid_dim_ << ", "
									<< m_per_cell_ << "m cells, "
									<< height_diff_threshold_ << "m threshold, "
									<< (full_clouds_? "": "not ") << "publishing full clouds");
			obstacle_publisher_dot_prod = node.advertise<PointCloud>("/carina/sensor/stereo/dot_prod",1);
			obstacle_publisher_ = node.advertise<PointCloud>("/carina/sensor/stereo/map_obstacles_full",1);

			velodyne_scan_ = node.subscribe("/carina/perception/stereo/point_cloud", 1, &HeightMapStereo::processData, this);
		}
		
		~HeightMapStereo() 
		{

		}

		void processData(const PointCloud::ConstPtr&  scan)
		{
			if (obstacle_publisher_.getNumSubscribers() == 0)
				return;
			obstacle_cloud_.header.stamp = scan->header.stamp;
			obstacle_vir_scan.header.stamp = scan->header.stamp;
			obstacle_cloud_.header.frame_id = scan->header.frame_id;
			obstacle_vir_scan.header.frame_id = scan->header.frame_id;
			size_t npoints = scan->points.size();
			obstacle_cloud_.points.resize(npoints);
			size_t obs_count=0;
			constructFullClouds(scan,npoints,obs_count);
			obstacle_cloud_.points.resize(obs_count);
			obstacle_publisher_dot_prod.publish(obstacle_vir_scan);
			obstacle_publisher_.publish(obstacle_cloud_);
			obstacle_vir_scan.points.clear(); 
		}

	private:
		
		void constructFullClouds(const PointCloud::ConstPtr& scan, unsigned npoints, size_t &obs_count)
		{
			cv::Mat minx = cv::Mat::zeros(grid_dim_,grid_dim_, CV_32FC1);
			cv::Mat maxx = cv::Mat::zeros(grid_dim_,grid_dim_, CV_32FC1);
			bool init[grid_dim_][grid_dim_];
			memset(&init, 0, grid_dim_*grid_dim_);
			for(size_t i = 0;  i<scan->width; i++)
			{
				for(size_t j = 1;  j<scan->height-1;  j++)
				{
					double abx = scan->at(i,j-1).x - scan->at(i,j).x;
					double aby = scan->at(i,j-1).y - scan->at(i,j).y;
					double abz = scan->at(i,j-1).z - scan->at(i,j).z;
					double bcx = scan->at(i,j-1).x - scan->at(i,j).x;
					double bcy = scan->at(i,j-1).y;
					double bcz = scan->at(i,j-1).z - scan->at(i,j).z;
					double dot_product = abx*bcx + aby*bcy + abz*bcz;

					if ( (dot_product < 0.001 and dot_product > -0.001))//or ( (abs(abx)<0.01) and (abs(abz)<0.01)))
					{
						obstacle_vir_scan.push_back (scan->at(i,j));
					}
				}
			}

			size_t npoints_v = obstacle_vir_scan.points.size();
			for (unsigned i = 0; i < npoints_v; ++i) 
			{
				int x = ((grid_dim_/2)+obstacle_vir_scan.points[i].x/m_per_cell_);
				int z = ((grid_dim_/2)+obstacle_vir_scan.points[i].z/m_per_cell_);//stereo
				if (x >= 0 && x < grid_dim_ && z >= 0 && z < grid_dim_) 
				{
					if (!init[x][z])
					{
						minx.at<float>(x, z) = obstacle_vir_scan.points[i].y;
						maxx.at<float>(x, z) = obstacle_vir_scan.points[i].y;
						init[x][z] = true;
					} else {
						minx.at<float>(x, z) = MIN(minx.at<float>(x, z), obstacle_vir_scan.points[i].y);
						maxx.at<float>(x, z) = MAX(maxx.at<float>(x, z), obstacle_vir_scan.points[i].y);
					}
				}
			}
			// display points where map has height-difference > threshold
			// #pragma omp for
			for (unsigned i = 0; i < npoints_v; ++i) 
			{
				int x = ((grid_dim_/2)+obstacle_vir_scan.points[i].x/m_per_cell_);
				int z = ((grid_dim_/2)+obstacle_vir_scan.points[i].z/m_per_cell_);
				if (x >= 0 && x < grid_dim_ && z >= 0 && z < grid_dim_ && init[x][z]) 
				{
					if ((maxx.at<float>(x, z) - minx.at<float>(x, z) > height_diff_threshold_) ) 
					{   
						obstacle_cloud_.points[obs_count] = obstacle_vir_scan.points[i];
						obs_count++;
					} 
				}
			}
		}
		// Parameters that define the grids and the height threshold
		// Can be set via the parameter server
		int grid_dim_;
		double m_per_cell_;
		double height_diff_threshold_;
		bool full_clouds_;

		// Point clouds generated in processData
		PointCloud obstacle_cloud_;      
		PointCloud obstacle_vir_scan;      

		// ROS topics
		ros::Subscriber velodyne_scan_;
		ros::Publisher obstacle_publisher_;
		ros::Publisher obstacle_publisher_dot_prod;

};




/** Main entry point. */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "heightmapstereo_node_full");
	ros::NodeHandle node;
	ros::NodeHandle priv_nh("~");

	// create height map class, which subscribes to velodyne_points
	HeightMapStereo hm(node, priv_nh);

	// handle callbacks until shut down
	ros::spin();

	return 0;
}
