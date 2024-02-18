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
#include <std_msgs/Bool.h>
#include <omp.h>
//#include <armadillo>

class PolarHeightMapStereo
{
	public:

  	typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

		PolarHeightMapStereo(ros::NodeHandle node, ros::NodeHandle priv_nh)
		{
			// get parameters using private node handle
			  // get parameters using private node handle
			  // priv_nh.param("cell_size", m_per_cell_, 0.15);
			  priv_nh.param("full_clouds", full_clouds_, false);
			  // priv_nh.param("grid_dimensions", grid_dim_, 2000);
			  priv_nh.param("height_diff_threshold", height_diff_threshold_, 0.6);
			  // priv_nh.param("height_diff_threshold", height_diff_threshold_, 0.25);

			  // priv_nh.param("g_per_cell_phi", g_per_cell_phi, 0.3515625*3);
			  priv_nh.param("g_per_cell_phi", g_per_cell_phi, 0.3515625);

			  priv_nh.param("m_per_cell_polar", m_per_cell_polar, 0.1);
			  priv_nh.param("grid_dim_z_polar", grid_dim_z_polar, 700);
			  priv_nh.param("grid_dim_phi", grid_dim_phi, 1024);//int(1024/3));

			ROS_INFO_STREAM("height map parameters: "
									// << grid_dim_ << "x" << grid_dim_ << ", "
									// << m_per_cell_ << "m cells, "
									<< height_diff_threshold_ << "m threshold, "
									<< (full_clouds_? "": "not ") << "publishing full clouds");
			// obstacle_publisher_dot_prod = node.advertise<PointCloud>("/carina/sensor/stereo/dot_prod",1);
			obstacle_publisher_ = node.advertise<PointCloud>("/carina/sensor/stereo/map_obstacles_full",1);


			velodyne_scan_ = node.subscribe("/carina/perception/stereo/point_cloud", 1, &PolarHeightMapStereo::processData, this);
			shutdown_sub = node.subscribe("/carina/vehicle/shutdown", 1,
			  //polar_scan_ = node.subscribe("polar_points", 10,
			                                  &PolarHeightMapStereo::shutdown_cb, this);		}
		








		~PolarHeightMapStereo() 
		{

		}


		void shutdown_cb(const std_msgs::Bool::ConstPtr& msg){
		    if (msg->data){
		        //delete obstacle_publisher_;
		        //delete polar_scan_;
		        //delete shutdown_sub;
		        std::cout<<"Bye!"<<std::endl;
		        ros::shutdown();
		    }
		}


		void processData(const PointCloud::ConstPtr&  scan)
		{
			if (obstacle_publisher_.getNumSubscribers() == 0)
				return;
			obstacle_cloud_.header.stamp = scan->header.stamp;
			// clear_cloud_.header.stamp = scan->header.stamp;
			obstacle_cloud_.header.frame_id = scan->header.frame_id;
			// clear_cloud_.header.frame_id = scan->header.frame_id;
			size_t npoints = scan->points.size();
			obstacle_cloud_.points.resize(npoints);
			size_t obs_count=0;
			size_t empty_count=0;

			constructFullClouds(scan,npoints,obs_count);
			obstacle_cloud_.points.resize(obs_count);
			// obstacle_publisher_dot_prod.publish(clear_cloud_);
			obstacle_publisher_.publish(obstacle_cloud_);
			// clear_cloud_.points.clear(); 
		}

	private:
		
		void constructFullClouds(const PointCloud::ConstPtr& scan, unsigned npoints, size_t &obs_count)
		{
			// cv::Mat minx = cv::Mat::zeros(grid_dim_,grid_dim_, CV_32FC1);
			// cv::Mat maxx = cv::Mat::zeros(grid_dim_,grid_dim_, CV_32FC1);
			cv::Mat min_z_polar = cv::Mat::zeros(grid_dim_z_polar, grid_dim_phi, CV_32FC1);
			cv::Mat max_z_polar = cv::Mat::zeros(grid_dim_z_polar, grid_dim_phi, CV_32FC1);
			// bool init[grid_dim_][grid_dim_];
			// memset(&init, 0, grid_dim_*grid_dim_);
			bool init[grid_dim_z_polar][grid_dim_phi];
			  // ROS_INFO_STREAM("info 2.1");

			  //memset(&init, 0, grid_dim_*grid_dim_);
			  
			for (int x = 0; x < grid_dim_z_polar; x++) {
			    for (int y = 0; y < grid_dim_phi; y++) {
			      init[x][y]=false;
			      // num_obs[x][y]=0;
			      // num_clear[x][y]=0;
			    }
			}


			// build height map
			for (unsigned i = 0; i < npoints; ++i) {
			    // if ((scan->points[i].z < -0.8) and (scan->points[i].z > -2.30)) {//cosider only  points lower than velodyne height and highest than road
			      if (scan->points[i].y > -2 && scan->points[i].y < 5){
			      float r =    sqrt( pow(scan->points[i].x,2)+  pow(scan->points[i].y,2) + pow(scan->points[i].z,2));         // #int((m_per_cell_r/2)+p[0]/g_per_cell_theta)  #X
			      float phi = atan2(-scan->points[i].x,scan->points[i].z)* 180 / M_PI;//#int((g_per_cell_phi/2)+p[1]/g_per_cell_theta) //#y

			        int y = (grid_dim_phi/2)+ phi /g_per_cell_phi;//((grid_dim_/2)+scan->points[i].x/m_per_cell_);
			        int x = r/m_per_cell_polar;//((grid_dim_/2)+scan->points[i].y/m_per_cell_);- 75//#-150 #considering 15 g uper fov --> two cells per grad 360 cells 0.5g uper 15g(30 cells) offset 75g(150 cells)
			        if (x >= 0 && x < grid_dim_z_polar && y >= 0 && y < grid_dim_phi) {
			          if (!init[x][y]) {
			            min_z_polar.at<float>(x, y) = scan->points[i].y;
			            max_z_polar.at<float>(x, y) = scan->points[i].y;
			            init[x][y] = true;
			          } else {
			            min_z_polar.at<float>(x, y) = MIN(min_z_polar.at<float>(x, y), scan->points[i].y);
			            max_z_polar.at<float>(x, y) = MAX(max_z_polar.at<float>(x, y), scan->points[i].y);
			        	}
			      	}
			    }

			}

			  // display points where map has height-difference > threshold
			for (unsigned i = 0; i < npoints; ++i) {
			    // if ((scan->points[i].z < -0.8) and (scan->points[i].z > -2.30)) {//cosider only  points lower than velodyne height and highest than road
			      if (scan->points[i].y > -2 && scan->points[i].y < 5){

			      float r =    sqrt( pow(scan->points[i].x,2)+  pow(scan->points[i].y,2) + pow(scan->points[i].z,2));        // #int((m_per_cell_r/2)+p[0]/g_per_cell_theta)  #X
			      float phi = atan2(-scan->points[i].x,scan->points[i].z)* 180 / M_PI;//#int((g_per_cell_phi/2)+p[1]/g_per_cell_theta) //#y
			      int y = (grid_dim_phi/2)+ phi /g_per_cell_phi;//((grid_dim_/2)+scan->points[i].x/m_per_cell_);
			      int x = r/m_per_cell_polar;//((grid_dim_/2)+scan->points[i].y/m_per_cell_);- 75//#-150 #considering 15 g uper fov --> two cells per grad 360 cells 0.5g uper 15g(30 cells) offset 75g(150 cells)
			      if (x >= 0 && x < grid_dim_z_polar && y >= 0 && y < grid_dim_phi) {
			        if ((max_z_polar.at<float>(x, y) - min_z_polar.at<float>(x, y) > height_diff_threshold_) ) {   
			          obstacle_cloud_.points[obs_count] = scan->points[i];
			          // obstacle_cloud_.points[obs_count].x = scan->points[i].x;
			          // obstacle_cloud_.points[obs_count].y = scan->points[i].y;
			          // obstacle_cloud_.points[obs_count].z = scan->points[i].z;
			          //obstacle_cloud_.channels[0].values[obs_count] = (float) scan->points[i].intensity;
			          obs_count++;
			        } 
			        // else {
			        //   clear_cloud_.points[empty_count].x = scan->points[i].x;
			        //   clear_cloud_.points[empty_count].y = scan->points[i].y;
			        //   clear_cloud_.points[empty_count].z = scan->points[i].z;
			        //   //clear_cloud_.channels[0].values[empty_count] = (float) scan->points[i].intensity;
			        //   empty_count++;
			        // }
			      }
			    }
			    // std::cout << "obstacles: " <<  obs_count << " \n";
			}
		}
			// Parameters that define the grids and the height threshold
			// Can be set via the parameter server


		bool full_clouds_;
		double height_diff_threshold_;
		double g_per_cell_phi;
		double m_per_cell_polar;
		int grid_dim_z_polar;
		int  grid_dim_phi;

		// Point clouds generated in processData
		PointCloud obstacle_cloud_;      
		// PointCloud clear_cloud_;      

		// ROS topics
		ros::Subscriber velodyne_scan_;
		ros::Publisher obstacle_publisher_;
		// ros::Publisher obstacle_publisher_dot_prod;
		ros::Subscriber shutdown_sub;

};




/** Main entry point. */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "polarheightmapstereo_node_full");
	ros::NodeHandle node;
	ros::NodeHandle priv_nh("~");

	// create height map class, which subscribes to velodyne_points
	PolarHeightMapStereo hm(node, priv_nh);

	// handle callbacks until shut down
	ros::spin();

	return 0;
}
