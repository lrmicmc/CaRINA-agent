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

- @b polar_points [sensor_msgs::PointCloud2] data from one
  revolution of the Velodyne LIDAR

Publishes:

- @b veloydne_obstacles [sensor_msgs::PointCloud2] grid cells that
  contain an obstacle

- @b veloydne_clear [sensor_msgs::PointCloud2] grid cells with no
  obstacles


@author David Claridge, Michael Quinlan 

*/

#include <polar_height_map/heightmap.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cmath>

namespace polar_height_map {

// #define MIN(x,y) ((x) < (y) ? (x) : (y))
// #define MAX(x,y) ((x) > (y) ? (x) : (y))

HeightMap::HeightMap(ros::NodeHandle node, ros::NodeHandle priv_nh)
{
  // get parameters using private node handle
  // priv_nh.param("cell_size", m_per_cell_, 0.15);
  priv_nh.param("full_clouds", full_clouds_, false);
  // priv_nh.param("grid_dimensions", grid_dim_, 2000);
  priv_nh.param("height_diff_threshold", height_diff_threshold_, 0.65);
  // priv_nh.param("height_diff_threshold", height_diff_threshold_, 0.25);

  priv_nh.param("g_per_cell_phi", g_per_cell_phi, 0.3515625*3);
  priv_nh.param("m_per_cell_polar", m_per_cell_polar, 0.4);
  priv_nh.param("grid_dim_z_polar", grid_dim_z_polar, 700);
  priv_nh.param("grid_dim_phi", grid_dim_phi, 1024);//int(1024/3));



  // ROS_INFO_STREAM("height map parameters: "
  //                 << grid_dim_ << "x" << grid_dim_ << ", "
  //                 << m_per_cell_ << "m cells, "
  //                 << height_diff_threshold_ << "m threshold, "
  //                 << (full_clouds_? "": "not ") << "publishing full clouds");

  // Set up publishers  
  obstacle_publisher_ = node.advertise<VPointCloud>("/carina/perception/lidar/velodyne_obstacles",1);
  // obstacle_publisher_ = node.advertise<VPointCloud>("/carina/perception/lidar/polar_obstacles",1);

  // clear_publisher_ = node.advertise<VPointCloud>("polar_clear",1);  

  // subscribe to polar data points
  polar_scan_ = node.subscribe("/carina/sensor/lidar/front/point_cloud", 1,
  //polar_scan_ = node.subscribe("polar_points", 10,
                                  &HeightMap::processData, this,
                                  ros::TransportHints().tcpNoDelay(true));

  shutdown_sub = node.subscribe("/carina/vehicle/shutdown", 1,
  //polar_scan_ = node.subscribe("polar_points", 10,
                                  &HeightMap::shutdown_cb, this);
}

HeightMap::~HeightMap() {}

void HeightMap::shutdown_cb(const std_msgs::Bool::ConstPtr &msg){
    if (msg->data){
        //delete obstacle_publisher_;
        //delete polar_scan_;
        //delete shutdown_sub;
        std::cout<<"Bye!"<<std::endl;
        ros::shutdown();
    }
}

void HeightMap::constructFullClouds(const VPointCloud::ConstPtr &scan,
                                    unsigned npoints, size_t &obs_count,
                                    size_t &empty_count)
{



  cv::Mat min_z_polar = cv::Mat::zeros(grid_dim_z_polar, grid_dim_phi, CV_32FC1);
  cv::Mat max_z_polar = cv::Mat::zeros(grid_dim_z_polar, grid_dim_phi, CV_32FC1);
              // obstacles_mat = np.zeros((grid_dim_z_polar, grid_dim_phi), dtype=float)#dtype=np.uint8)#dtype=float)
              // clear_area_mat = np.zeros((grid_dim_z_polar, grid_dim_phi), dtype=float)#dtype=np.uint8)#dtype=float)


              // init = np.zeros((grid_dim_z_polar, grid_dim_phi), dtype=bool)

  // float min[grid_dim_][grid_dim_];
  // float max[grid_dim_][grid_dim_];
  // bool init[grid_dim_][grid_dim_];
  // ROS_INFO_STREAM("info 2");
  // float min[grid_dim_][grid_dim_];
  // cv::Mat minx = cv::Mat::zeros(grid_dim_,grid_dim_, CV_32FC1);
  // float max[grid_dim_][grid_dim_];
  // cv::Mat maxx = cv::Mat::zeros(grid_dim_,grid_dim_, CV_32FC1);
  // float num_obs[grid_dim_][grid_dim_];
  // cv::Mat num_obs = cv::Mat::zeros(grid_dim_,grid_dim_, CV_32FC1);

  // float num_clear[grid_dim_][grid_dim_];
  // bool init[grid_dim_][grid_dim_];
  // bool init[grid_dim_z_polar][grid_dim_phi];



  // memset(&init, 0, grid_dim_z_polar*grid_dim_phi);

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


                // for p in points:
                // r =    np.sqrt( np.power(p[0],2)+  np.power(p[1],2) + np.power(p[2],2))         // #int((m_per_cell_r/2)+p[0]/g_per_cell_theta)  #X
                // phi = np.arctan2(p[1],p[0])* 180 / np.pi//#int((g_per_cell_phi/2)+p[1]/g_per_cell_theta) //#y
                // // # theta= (np.arccos(p[2]/r)* 180 / np.pi)
                // // # z=p[2]

                // y=  int((grid_dim_phi/2)+ phi /g_per_cell_phi) 
                // x=  int(r/m_per_cell_polar) //#int((grid_dim_theta/2)+ theta /g_per_cell_theta) - 75//#-150 #considering 15 g uper fov --> two cells per grad 360 cells 0.5g uper 15g(30 cells) offset 75g(150 cells)

                // // # if (x >= 0 and x < grid_dim_theta and y >= 0 and y < grid_dim_phi):
                // if (x >= 0 and x < grid_dim_z_polar and y >= 0 and y < grid_dim_phi):
                //   if (init[x][y] == False):
                //     // # xyz_cart[x][y]=[p[0],p[1],p[2]]
                //     // # seg_gt[x][y] = label
                //     min_z_polar[x][y] = p[2]
                //     max_z_polar[x][y] = p[2]
                //     init[x][y] = True
                //   else:
                //     min_z_polar[x][y] = np.min([min_z_polar[x][y], p[2]])
                //     max_z_polar[x][y] = np.max([max_z_polar[x][y], p[2]])
  
  // build height map
  for (unsigned i = 0; i < npoints; ++i) {
    if ((scan->points[i].z < -0.8) and (scan->points[i].z > -2.30)) {//cosider only  points lower than velodyne height and highest than road
      float r =    sqrt( pow(scan->points[i].x,2)+  pow(scan->points[i].y,2) + pow(scan->points[i].z,2));         // #int((m_per_cell_r/2)+p[0]/g_per_cell_theta)  #X
      float phi = atan2(scan->points[i].y,scan->points[i].x)* 180 / M_PI;//#int((g_per_cell_phi/2)+p[1]/g_per_cell_theta) //#y

        int y = (grid_dim_phi/2)+ phi /g_per_cell_phi;//((grid_dim_/2)+scan->points[i].x/m_per_cell_);
        int x = r/m_per_cell_polar;//((grid_dim_/2)+scan->points[i].y/m_per_cell_);- 75//#-150 #considering 15 g uper fov --> two cells per grad 360 cells 0.5g uper 15g(30 cells) offset 75g(150 cells)
        if (x >= 0 && x < grid_dim_z_polar && y >= 0 && y < grid_dim_phi) {
          if (!init[x][y]) {
            min_z_polar.at<float>(x, y) = scan->points[i].z;
            max_z_polar.at<float>(x, y) = scan->points[i].z;
            init[x][y] = true;
          } else {
            min_z_polar.at<float>(x, y) = MIN(min_z_polar.at<float>(x, y), scan->points[i].z);
            max_z_polar.at<float>(x, y) = MAX(max_z_polar.at<float>(x, y), scan->points[i].z);
        }
      }
    }

  }


              // for p in points:
              //   r =    np.sqrt( np.power(p[0],2)+  np.power(p[1],2) + np.power(p[2],2))          #int((m_per_cell_r/2)+p[0]/g_per_cell_theta)  #X
              //   phi = np.arctan2(p[1],p[0])* 180 / np.pi#int((g_per_cell_phi/2)+p[1]/g_per_cell_theta) #y


              //   y=  int((grid_dim_phi/2)+ phi /g_per_cell_phi) 
              //   x=  int(r/m_per_cell_polar) //#int((grid_dim_theta/2)+ theta /g_per_cell_theta) - 75#-150 #considering 15 g uper fov --> two cells per grad 360 cells 0.5g uper 15g(30 cells) offset 75g(150 cells)

              //   if (x >= 0 and x < grid_dim_z_polar and y >= 0 and y < grid_dim_phi):

              //     if ((max_z_polar[x][y] - min_z_polar[x][y] > height_diff_threshold) ):



  // display points where map has height-difference > threshold
  for (unsigned i = 0; i < npoints; ++i) {
    if ((scan->points[i].z < -0.8) and (scan->points[i].z > -2.30)) {//cosider only  points lower than velodyne height and highest than road
      float r =    sqrt( pow(scan->points[i].x,2)+  pow(scan->points[i].y,2) + pow(scan->points[i].z,2));        // #int((m_per_cell_r/2)+p[0]/g_per_cell_theta)  #X
      float phi = atan2(scan->points[i].y,scan->points[i].x)* 180 / M_PI;//#int((g_per_cell_phi/2)+p[1]/g_per_cell_theta) //#y
      int y = (grid_dim_phi/2)+ phi /g_per_cell_phi;//((grid_dim_/2)+scan->points[i].x/m_per_cell_);
      int x = r/m_per_cell_polar;//((grid_dim_/2)+scan->points[i].y/m_per_cell_);- 75//#-150 #considering 15 g uper fov --> two cells per grad 360 cells 0.5g uper 15g(30 cells) offset 75g(150 cells)
      if (x >= 0 && x < grid_dim_z_polar && y >= 0 && y < grid_dim_phi) {
        if ((max_z_polar.at<float>(x, y) - min_z_polar.at<float>(x, y) > height_diff_threshold_) ) {   
          obstacle_cloud_.points[obs_count].x = scan->points[i].x;
          obstacle_cloud_.points[obs_count].y = scan->points[i].y;
          obstacle_cloud_.points[obs_count].z = scan->points[i].z;
          //obstacle_cloud_.channels[0].values[obs_count] = (float) scan->points[i].intensity;
          obs_count++;
        } else {
          clear_cloud_.points[empty_count].x = scan->points[i].x;
          clear_cloud_.points[empty_count].y = scan->points[i].y;
          clear_cloud_.points[empty_count].z = scan->points[i].z;
          //clear_cloud_.channels[0].values[empty_count] = (float) scan->points[i].intensity;
          empty_count++;
        }
      }
    }
    //std::cout << "if obstacles: " <<  obs_count << " \n";
    }

}

void HeightMap::constructGridClouds(const VPointCloud::ConstPtr &scan,
                                    unsigned npoints, size_t &obs_count,
                                    size_t &empty_count)
{
  // ROS_INFO_STREAM("info 2");
  // float min[grid_dim_][grid_dim_];
  // cv::Mat minx = cv::Mat::zeros(grid_dim_,grid_dim_, CV_32FC1);
  cv::Mat minx = cv::Mat::zeros(grid_dim_z_polar, grid_dim_phi, CV_32FC1);
  // float max[grid_dim_][grid_dim_];
  // cv::Mat maxx = cv::Mat::zeros(grid_dim_,grid_dim_, CV_32FC1);
  cv::Mat maxx = cv::Mat::zeros(grid_dim_z_polar, grid_dim_phi, CV_32FC1);
  // float num_obs[grid_dim_][grid_dim_];
  // cv::Mat num_obs = cv::Mat::zeros(grid_dim_,grid_dim_, CV_32FC1);
  cv::Mat num_obs = cv::Mat::zeros(grid_dim_z_polar, grid_dim_phi, CV_32FC1);

  // float num_clear[grid_dim_][grid_dim_];
  // bool init[grid_dim_][grid_dim_];
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



  // for (int i=0; i < grayMat.rows; ++i){
  //     for (int j=0; j < grayMat.cols; ++j){
  //         Totalintensity += (int)grayMat.at<float>(i, j);
  //     }
  // }


  // build height map
  for (unsigned i = 0; i < npoints; ++i) {
    // int x = ((grid_dim_/2)+scan->points[i].x/m_per_cell_);
    // int y = ((grid_dim_/2)+scan->points[i].y/m_per_cell_);
    if ((scan->points[i].z < -0.8) and (scan->points[i].z > -2.30)) //cosider only  points lower than velodyne height and highest than road
    {
      float r =    sqrt( pow(scan->points[i].x,2)+  pow(scan->points[i].y,2) + pow(scan->points[i].z,2));         // #int((m_per_cell_r/2)+p[0]/g_per_cell_theta)  #X
      float phi = atan2(scan->points[i].y,scan->points[i].x)* 180 / M_PI;//#int((g_per_cell_phi/2)+p[1]/g_per_cell_theta) //#y
      int y = (grid_dim_phi/2)+ phi /g_per_cell_phi;//((grid_dim_/2)+scan->points[i].x/m_per_cell_);
      int x = r/m_per_cell_polar;
      // if (phi < 75 && phi > -75 )
      {
        if (x >= 0 && x < grid_dim_z_polar && y >= 0 && y < grid_dim_phi) {
        // if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_) {
          if (!init[x][y]) {
            minx.at<float>(x, y) = scan->points[i].z;
            maxx.at<float>(x, y) = scan->points[i].z;
            // min[x][y] = scan->points[i].z;
            // max[x][y] = scan->points[i].z;
            num_obs.at<float>(x, y) = 0;
            // num_obs[x][y] = 0;
            // num_clear[x][y] = 0;
            init[x][y] = true;
          } else {
            minx.at<float>(x, y)  = MIN(minx.at<float>(x, y), scan->points[i].z);
            maxx.at<float>(x, y)  = MAX(maxx.at<float>(x, y), scan->points[i].z);
            // min[x][y] = MIN(min[x][y], scan->points[i].z);
            // max[x][y] = MAX(max[x][y], scan->points[i].z);
          }
        }
      }
    }
  }

  // calculate number of obstacles in each cell
  for (unsigned i = 0; i < npoints; ++i) {
    if ((scan->points[i].z < -0.8) and (scan->points[i].z > -2.30)) //cosider only  points lower than velodyne height and highest than road
    {
      // int x = ((grid_dim_/2)+scan->points[i].x/m_per_cell_);
      // int y = ((grid_dim_/2)+scan->points[i].y/m_per_cell_);
      float r =    sqrt( pow(scan->points[i].x,2)+  pow(scan->points[i].y,2) + pow(scan->points[i].z,2));         // #int((m_per_cell_r/2)+p[0]/g_per_cell_theta)  #X
      float phi = atan2(scan->points[i].y,scan->points[i].x)* 180 / M_PI;//#int((g_per_cell_phi/2)+p[1]/g_per_cell_theta) //#y
      int y = (grid_dim_phi/2)+ phi /g_per_cell_phi;//((grid_dim_/2)+scan->points[i].x/m_per_cell_);
      int x = r/m_per_cell_polar;
      if (x >= 0 && x < grid_dim_z_polar && y >= 0 && y < grid_dim_phi) {
      // if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_ && init[x][y]) {
        // if ((max[x][y] - min[x][y] > height_diff_threshold_) ) {  
        //   num_obs[x][y]++;
        // } 
        if ((maxx.at<float>(x, y) - minx.at<float>(x, y) > height_diff_threshold_)) {  
          num_obs.at<float>(x, y)++;
        } 
        // else 
        // {
        //   num_clear[x][y]++;
        // }
      }
    }
  }

  // create clouds from grid
  double grid_offset_z   = grid_dim_z_polar/2.0*m_per_cell_polar;
  double grid_offset_phi = grid_dim_phi/2.0*g_per_cell_phi;





  for (int x = 0; x < grid_dim_z_polar; x++) {
    for (int y = 0; y < grid_dim_phi; y++) {
      // std::cout << "phi: " <<  y << " \n";
      // std::cout << "r: " <<  x << " \n";

      // if (num_obs[x][y]>0) {
      if (  num_obs.at<float>(x, y) >0) {

        obstacle_cloud_.points[obs_count].x = -x*m_per_cell_polar* cos( y*g_per_cell_phi * (M_PI/180));//-grid_offset_z + (x*m_per_cell_polar+m_per_cell_polar/2.0);
        obstacle_cloud_.points[obs_count].y = -x*m_per_cell_polar* sin( y*g_per_cell_phi * (M_PI/180));//y;//-grid_offset_phi + (y*g_per_cell_phi+g_per_cell_phi/2.0);
        obstacle_cloud_.points[obs_count].z = -2.0;//height_diff_threshold_;
        //obstacle_cloud_.channels[0].values[obs_count] = (float) 255.0;
        obs_count++;
        //std::cout << "if obstacles: " <<  obs_count << " \n";
      }
      // if (num_clear[x][y]>0) {
      //   clear_cloud_.points[empty_count].x = -grid_offset + (x*m_per_cell_+m_per_cell_/2.0);
      //   clear_cloud_.points[empty_count].y = -grid_offset + (y*m_per_cell_+m_per_cell_/2.0);
      //   clear_cloud_.points[empty_count].z = height_diff_threshold_;
      //   //clear_cloud_.channels[0].values[empty_count] = (float) 255.0;
      //   empty_count++;
      // }
    }
  }
  //std::cout << "if obstacles: " <<  obs_count << " \n";
}

/** point cloud input callback */
void HeightMap::processData(const VPointCloud::ConstPtr &scan)
{
  if ((obstacle_publisher_.getNumSubscribers() == 0)  && (clear_publisher_.getNumSubscribers() == 0))
  {
    ROS_INFO_STREAM("0 NumSubscribers");
    return;
  }
  
  // pass along original time stamp and frame ID
  obstacle_cloud_.header.stamp = scan->header.stamp;
  obstacle_cloud_.header.frame_id = scan->header.frame_id;

  // pass along original time stamp and frame ID
  // clear_cloud_.header.stamp = scan->header.stamp;
  // clear_cloud_.header.frame_id = scan->header.frame_id;

  // set the exact point cloud size -- the vectors should already have
  // enough space
  // ROS_INFO_STREAM("info 1");

  size_t npoints = scan->points.size();

  obstacle_cloud_.points.resize(npoints);

  // obstacle_cloud_.points.resize(grid_dim_*grid_dim_);

  // obstacle_cloud_.channels[0].values.resize(npoints);

  // clear_cloud_.points.resize(npoints/2);
  //clear_cloud_.channels[0].values.resize(npoints);
  // ROS_INFO_STREAM("info 2");
  size_t obs_count=0;
  size_t empty_count=0;
  // either return full point cloud or a discretized version
  //ROS_INFO_STREAM("Build clouds");
  // if (full_clouds_)
    // constructFullClouds(scan,npoints,obs_count, empty_count);
  // else
    constructGridClouds(scan,npoints,obs_count, empty_count);
  // ROS_INFO_STREAM("info 3");

  obstacle_cloud_.points.resize(obs_count);
  //obstacle_cloud_.channels[0].values.resize(obs_count);

  // clear_cloud_.points.resize(empty_count);
  //clear_cloud_.channels[0].values.resize(empty_count);
  // ROS_INFO_STREAM("info 4");

  if (obstacle_publisher_.getNumSubscribers() > 0)
    obstacle_publisher_.publish(obstacle_cloud_);

  // if (clear_publisher_.getNumSubscribers() > 0)
  //   clear_publisher_.publish(clear_cloud_);
}

} // namespace polar_height_map
