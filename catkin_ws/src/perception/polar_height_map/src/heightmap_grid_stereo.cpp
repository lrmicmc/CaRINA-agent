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
#include <std_msgs/Bool.h>
#include <iostream>
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
			priv_nh.param("grid_dimensions", grid_dim_, 200);
			priv_nh.param("height_threshold", height_diff_threshold_, 0.3);

			// Topics
    		// std::string stereo_pc = node.resolveName("stereo_pc");
			

    		std::string subscribing_topic = "/carina/perception/stereo/point_cloud";
    		std::string publish_topic = "/carina/perception/lidar/velodyne_obstacles_stereo";//stereo_pc + "/stereo_obstacles";


  // Set up publishers  
  // obstacle_publisher_ = node.advertise<VPointCloud>("/carina/perception/lidar/velodyne_obstacles",1);
  // clear_publisher_ = node.advertise<VPointCloud>("velodyne_clear",1);  

  // // subscribe to Velodyne data points
  // velodyne_scan_ = node.subscribe("/carina/sensor/lidar/front/point_cloud", 1,
  // //velodyne_scan_ = node.subscribe("velodyne_points", 10,
  //                                 &HeightMap::processData, this,
  //                                 ros::TransportHints().tcpNoDelay(true));

  // shutdown_sub = node.subscribe("/carina/vehicle/shutdown", 1,
  // //velodyne_scan_ = node.subscribe("velodyne_points", 10,
  //                                 &HeightMap::shutdown_cb, this);




			ROS_INFO_STREAM("height map parameters: "
									<< grid_dim_ << "x" << grid_dim_ << ", "
									<< m_per_cell_ << "m cells, "
									<< height_diff_threshold_ << "m threshold, "
									<< (full_clouds_? "": "not ") << "publishing full clouds");
			// obstacle_publisher_dot_prod = node.advertise<PointCloud>("/carina/sensor/stereo/dot_prod",1);
			obstacle_publisher_ = node.advertise<PointCloud>(publish_topic,1);

			velodyne_scan_ = node.subscribe(subscribing_topic, 1, &HeightMapStereo::processData, this);
			ROS_INFO("Height map Subscribing to:%s",subscribing_topic.c_str());
			ROS_INFO("Height map Publishing to:%s",publish_topic.c_str());
            shutdown_sub = node.subscribe("/carina/vehicle/shutdown", 1, &HeightMapStereo::shutdown_cb, this);
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
			constructGridClouds(scan,npoints,obs_count);

			obstacle_cloud_.points.resize(obs_count);
 
			// obstacle_publisher_dot_prod.publish(obstacle_vir_scan);
			obstacle_publisher_.publish(obstacle_cloud_);

			obstacle_vir_scan.points.clear(); 

           
		}
        void shutdown_cb(const std_msgs::Bool::ConstPtr& msg){
            if (msg->data){
                    //delete obstacle_publisher_;
                    //delete velodyne_scan_;
                    //delete shutdown_sub;
                    std::cout<<"Bye!"<<std::endl;
                    ros::shutdown();
            }

        }

	private:
		
		void constructGridClouds(const PointCloud::ConstPtr& scan, unsigned npoints, size_t &obs_count)
		{
			cv::Mat minx = cv::Mat::zeros(grid_dim_,grid_dim_, CV_32FC1);
			cv::Mat maxx = cv::Mat::zeros(grid_dim_,grid_dim_, CV_32FC1);
			cv::Mat num_obs = cv::Mat::zeros(grid_dim_,grid_dim_, CV_32FC1);
			bool init[grid_dim_][grid_dim_];
			memset(&init, 0, grid_dim_*grid_dim_);

			for(size_t i = 0;  i<scan->width; i++)//dot product
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
						num_obs.at<float>(x, z) = 0;
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
						num_obs.at<float>(x, z)++;
					} 
				}
			}

			  // create clouds from grid
			double grid_offset=grid_dim_/2.0*m_per_cell_;
			for (int x = 0; x < grid_dim_; x++) 
			{
				for (int z = 0; z < grid_dim_; z++) 
				{
					if (num_obs.at<float>(x, z)>0) 
					{
						obstacle_cloud_.points[obs_count].x = -grid_offset + (x*m_per_cell_+m_per_cell_/2.0);
						obstacle_cloud_.points[obs_count].z = -grid_offset + (z*m_per_cell_+m_per_cell_/2.0);
						obstacle_cloud_.points[obs_count].y = height_diff_threshold_;
						//obstacle_cloud_.channels[0].values[obs_count] = (float) 255.0;
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
        ros::Subscriber shutdown_sub;

};




/** Main entry point. */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "heightmap_node_grid_stereo");
	ros::NodeHandle node;
	ros::NodeHandle priv_nh("~");

	// create height map class, which subscribes to velodyne_points
	HeightMapStereo hm(node, priv_nh);

	// handle callbacks until shut down
	ros::spin();

	return 0;
}
