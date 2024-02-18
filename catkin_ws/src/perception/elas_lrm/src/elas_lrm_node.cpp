/*
 Copywrite 2012. All rights reserved.
 Cyphy Lab, https://wiki.qut.edu.au/display/cyphy/Robotics,+Vision+and+Sensor+Networking+at+QUT
 Queensland University of Technology
 Brisbane, Australia

 Author: Patrick Ross
 Contact: patrick.ross@connect.qut.edu.au

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <ros/ros.h>
#include <stdlib.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/subscriber_filter.h>

#include <image_geometry/stereo_camera_model.h>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <std_msgs/Bool.h>

// #include <elas_ros/ElasFrameData.h>

#include <elas.h>


#include <fstream>
#include <iostream>

#include <vector>

using namespace std;
//#define DOWN_SAMPLE

class Elas_Proc
{
  struct support_pt {
    int32_t u;
    int32_t v;
    int32_t d;
    support_pt(int32_t u,int32_t v,int32_t d):u(u),v(v),d(d){}
  };
public:
  Elas_Proc(const std::string& transport)
  {

    ros::NodeHandle _nh;

    ros::NodeHandle local_nh("~");
    local_nh.param("queue_size", queue_size_, 5);

    // Topics
    std::string stereo_ns = nh.resolveName("stereo");
    std::string left_topic = ros::names::clean(stereo_ns + "/left/" + nh.resolveName("image"));
    std::string right_topic = ros::names::clean(stereo_ns + "/right/" + nh.resolveName("image"));
    std::string left_info_topic = stereo_ns + "/left/camera_info";
    std::string right_info_topic = stereo_ns + "/right/camera_info";
    // std::string lidar_topic = "/kitti_player/hdl64e_from_depth_left";
    // std::string lidar_topic = "/kitti_player/hdl64e";

  

    image_transport::ImageTransport it(nh);
    left_sub_.subscribe(it, left_topic, 1, transport);
    right_sub_.subscribe(it, right_topic, 1, transport);
    left_info_sub_.subscribe(nh, left_info_topic, 1);
    right_info_sub_.subscribe(nh, right_info_topic, 1);
    // cloud_sub_.subscribe( nh, lidar_topic, 1 );


    ROS_INFO("Subscribing to:\n%s\n%s\n%s\n%s",left_topic.c_str(),right_topic.c_str(),left_info_topic.c_str(),right_info_topic.c_str());

    image_transport::ImageTransport local_it(local_nh);
    // disp_pub_.reset(new Publisher(local_it.advertise("image_disparity", 1)));

    bev_pub_.reset(new Publisher(local_it.advertise("/carina/sensor/stereo/bev_rgb_point_cloud", 1)));
    rgb_pub_.reset(new Publisher(local_it.advertise("/carina/sensor/stereo/left_image_rgb_elas", 1)));

    // depth_pub_.reset(new CameraPublisher(local_it.advertiseCamera("depth", 1)));
    depth_pub_.reset(new Publisher(local_it.advertise("/carina/sensor/stereo/depth", 1)));
    //pc_pub_.reset(new ros::Publisher(local_nh.advertise<PointCloud>("point_cloud", 1)));
    pc_pub_.reset(new ros::Publisher(_nh.advertise<PointCloud>("/carina/perception/stereo/point_cloud", 1)));

    shutdown_sub = nh.subscribe("/carina/vehicle/shutdown", 1, &Elas_Proc::shutdown_cb, this);
    

    //pc_pub_ = node.advertise<PointCloud>("/carina/perception/stereo/point_cloud", 1);
    // elas_fd_pub_.reset(new ros::Publisher(local_nh.advertise<elas_ros::ElasFrameData>("frame_data", 1)));

    // pub_disparity_ = local_nh.advertise<stereo_msgs::DisparityImage>("disparity", 1);

    // Synchronize input topics. Optionally do approximate synchronization.
    bool approx;
    //local_nh.param("approximate_sync", approx, false);
    local_nh.param("approximate_sync", approx, true);

    if (approx)
    {
      ao = new ApproximateSync(ApproximatePolicy(queue_size_),
                                                  left_sub_, right_sub_, left_info_sub_, right_info_sub_) ;
      approximate_sync_.reset(ao);
      approximate_sync_->registerCallback(boost::bind(&Elas_Proc::process, this, _1, _2, _3, _4));
    }
    else
    {
      es = new ExactSync(ExactPolicy(queue_size_),
                                      left_sub_, right_sub_, left_info_sub_, right_info_sub_);
      exact_sync_.reset(es);
      exact_sync_->registerCallback(boost::bind(&Elas_Proc::process, this, _1, _2, _3, _4));
    }

    param.reset(new Elas::parameters);

    /* Parameters tunned*/
    param->disp_min              = 0;
    param->disp_max              = 255;
    param->support_threshold     = 0.95;
    param->support_texture       = 10;
    param->candidate_stepsize    = 5;
    param->incon_window_size     = 5;
    param->incon_threshold       = 5;
    param->incon_min_support     = 5;
    param->add_corners           = 0;
    param->grid_size             = 20;
    param->beta                  = 0.02;
    param->gamma                 = 3;
    param->sigma                 = 1;
    param->sradius               = 2;
    param->match_texture         = 1;
    param->lr_threshold          = 2;
    param->speckle_sim_threshold = 1;
    param->speckle_size          = 200;
    param->ipol_gap_width        = 300;
    param->filter_median         = 0;
    param->filter_adaptive_mean  = 1;
    param->postprocess_only_left = 1;
    param->subsampling           = 0;

    //param->match_texture = 1;
    //param->postprocess_only_left = 1;
    //param->ipol_gap_width = 2;
// #ifdef DOWN_SAMPLE
    param->subsampling = false;
// #endif
    elas_.reset(new Elas(*param));
  }

  typedef image_transport::SubscriberFilter Subscriber;
  typedef message_filters::Subscriber<sensor_msgs::CameraInfo> InfoSubscriber;
  typedef image_transport::Publisher Publisher;
  typedef image_transport::CameraPublisher CameraPublisher;

  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo > ExactPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximatePolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
  // typedef message_filters::Subscriber<sensor_msgs::PointCloud2> CloudSubscriber;


  void publish_point_cloud(const sensor_msgs::ImageConstPtr& l_image_msg, 
                           float* l_disp_data, const std::vector<int32_t>& inliers,
                           int32_t l_width, int32_t l_height,
                           const sensor_msgs::CameraInfoConstPtr& l_info_msg, 
                           const sensor_msgs::CameraInfoConstPtr& r_info_msg)
  {
    try
    {
      cv_bridge::CvImageConstPtr cv_ptr;
      cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::RGB8);
      image_geometry::StereoCameraModel model;
      model.fromCameraInfo(*l_info_msg, *r_info_msg);
      pcl::PCLHeader l_info_header = pcl_conversions::toPCL(l_info_msg->header);

      PointCloud::Ptr point_cloud(new PointCloud());
      point_cloud->header.frame_id = l_info_header.frame_id;
      point_cloud->header.stamp = l_info_header.stamp;
      point_cloud->width =  l_width;
      point_cloud->height = l_height;
      point_cloud->points.resize(point_cloud->width * point_cloud->height); 
      // ROS_INFO_STREAM( point_cloud->width <<" "<<  point_cloud->height);
      cv::Point3d point;

      int size_image=700;
      int map_scale=8;
      cv::Mat bev_lidar_f(size_image,  size_image, CV_8UC3, cv::Scalar(0));

      for (size_t i=0; i<inliers.size(); i++)
      {
        cv::Point2d left_uv;
        int32_t index = inliers[i];

        // #ifdef DOWN_SAMPLE
        if(param->subsampling == true)
        {
          left_uv.x = (index % l_width) * 2;
          left_uv.y = (index / l_width) * 2;
        }
        
        // #else
        else{
          left_uv.x = index % l_width;
          left_uv.y = index / l_width;
        }

        // #endif
        // if( left_uv.y > 400  )
         // and left_uv.x < 200 )
        // {
        model.projectDisparityTo3d(left_uv, l_disp_data[index], point);
        // if( point.z < 70 )
        // {
        cv::Vec3b col = cv_ptr->image.at<cv::Vec3b>(left_uv.y,left_uv.x);
        point_cloud->at(left_uv.x,left_uv.y).x = point.x;
        point_cloud->at(left_uv.x,left_uv.y).y = point.y;
        point_cloud->at(left_uv.x,left_uv.y).z = point.z;
        point_cloud->at(left_uv.x,left_uv.y).r = col[0];
        point_cloud->at(left_uv.x,left_uv.y).g = col[1];
        point_cloud->at(left_uv.x,left_uv.y).b = col[2]; 

        if(( point.z < 80 ) and  ( point.z > -20 )  and ( left_uv.y > 600  ))
        {
          int px = (int)(size_image/2)+(int)(-point.x*map_scale);
          int py = (int)(size_image/3)+(int)(point.z*map_scale);
          circle( bev_lidar_f, cv::Point(px, py),  0, cv::Scalar( col[2], col[1], col[0] ), -1, 8 );


        }
                   
        // }


      }
      // cv::namedWindow( "Display window ", cv::WINDOW_AUTOSIZE );// Create a window for display.
      // cv::imshow( "Display window", bev_lidar_f );                   // Show our image inside it.
      // // cv::waitKey(100);
      // cv::waitKey(1);




      sensor_msgs::ImagePtr bev_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bev_lidar_f).toImageMsg();



      pc_pub_->publish(point_cloud);
      bev_pub_->publish(bev_img_msg);
      rgb_pub_->publish(l_image_msg);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }

  void shutdown_cb(const std_msgs::Bool::ConstPtr &msg){
    if (msg->data){
      
      left_sub_.unsubscribe() ;
      right_sub_.unsubscribe();
      left_info_sub_.unsubscribe();
      right_info_sub_.unsubscribe();

      shutdown_sub.shutdown();
      pc_pub_->shutdown();
      bev_pub_->shutdown();
      rgb_pub_->shutdown();
      elas_.reset();
      exact_sync_.reset();
      approximate_sync_.reset();

      std::cout<<"Bye!"<<std::endl;
      ros::shutdown();
    }
  }

  void process(const sensor_msgs::ImageConstPtr& l_image_msg, 
               const sensor_msgs::ImageConstPtr& r_image_msg,
               const sensor_msgs::CameraInfoConstPtr& l_info_msg, 
               const sensor_msgs::CameraInfoConstPtr& r_info_msg)
  {
    ROS_DEBUG("Received images and camera info.");
    // Update the camera model
    model_.fromCameraInfo(l_info_msg, r_info_msg);

    // Allocate new disparity image message
    // stereo_msgs::DisparityImagePtr disp_msg =
    //   boost::make_shared<stereo_msgs::DisparityImage>();
    // disp_msg->header         = l_info_msg->header;
    // disp_msg->image.header   = l_info_msg->header;
    // disp_msg->image.height   = l_image_msg->height;
    // disp_msg->image.width    = l_image_msg->width;
    // disp_msg->image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    // disp_msg->image.step     = disp_msg->image.width * sizeof(float);
    // disp_msg->image.data.resize(disp_msg->image.height * disp_msg->image.step);
    // disp_msg->min_disparity = param->disp_min;
    // disp_msg->max_disparity = param->disp_max;

    // Stereo parameters
    float f = model_.right().fx();
    float T = model_.baseline();
    float depth_fact = T*f*1000.0f;
    uint16_t bad_point = std::numeric_limits<uint16_t>::max();

    // Have a synchronised pair of images, now to process using elas
    // convert images if necessary
    uint8_t *l_image_data, *r_image_data;
    int32_t l_step, r_step;
    cv_bridge::CvImageConstPtr l_cv_ptr, r_cv_ptr;
    // if (l_image_msg->encoding == sensor_msgs::image_encodings::MONO8)
    // {
    //   l_image_data = const_cast<uint8_t*>(&(l_image_msg->data[0]));
    //   l_step = l_image_msg->step;
    //   ROS_INFO("mono8 l");
    // }
    // else
    // {
      // ROS_INFO("mono8 l else");
      l_cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8);
      l_image_data = l_cv_ptr->image.data;
      l_step = l_cv_ptr->image.step[0];

    // }
    // if (r_image_msg->encoding == sensor_msgs::image_encodings::MONO8)
    // {
    //   ROS_INFO("mono8 r");
    //   r_image_data = const_cast<uint8_t*>(&(r_image_msg->data[0]));
    //   r_step = r_image_msg->step;
    // }
    // else
    // {
      r_cv_ptr = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8);
      r_image_data = r_cv_ptr->image.data;
      r_step = r_cv_ptr->image.step[0];
    // }

    ROS_ASSERT(l_step == r_step);
    ROS_ASSERT(l_image_msg->width == r_image_msg->width);
    ROS_ASSERT(l_image_msg->height == r_image_msg->height);

// #ifdef DOWN_SAMPLE
  int32_t width = 0;
  int32_t height = 0;
  if(param->subsampling == true)
  {
    width = l_image_msg->width/2;
    height = l_image_msg->height/2;
  }
// #else
  else{
    width = l_image_msg->width;
    height = l_image_msg->height;    
  }

// #endif

    // Allocate
    const int32_t dims[3] = {l_image_msg->width,l_image_msg->height,l_step};
    //float* l_disp_data = new float[width*height*sizeof(float)];
    // float* l_disp_data = reinterpret_cast<float*>(&disp_msg->image.data[0]);

    float* l_disp_data = new float[width*height*sizeof(float)];
    float* r_disp_data = new float[width*height*sizeof(float)];




      // // ROS_INFO("recieved points");
      // pcl::PCLPointCloud2 pcl_pc;
      // pcl_conversions::toPCL(*msg, pcl_pc);
      // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      // pcl::fromPCLPointCloud2(pcl_pc,*cloud);

    // Process
    elas_->process(l_image_data, r_image_data, l_disp_data, r_disp_data, dims);

    // Find the max for scaling the image colour
    std::vector<int32_t> inliers;

    float disp_max = 0;

    // for (int32_t i=0; i<width*height; i++)
    // {
    //   // float disp = l_disp_data[i];
    //   // out_depth_msg_image_data[i] = disp <= 0.0f ? bad_point : (uint16_t)(depth_fact/disp);

    //   if (l_disp_data[i]>disp_max) 
    //     disp_max = l_disp_data[i];
    //   // if (r_disp_data[i]>disp_max) disp_max = r_disp_data[i];
    //   // if (l_disp_data[i] > 0) 
    //   //   inliers.push_back(i);

    // }

    // sensor_msgs::CameraInfo depthInfoMsg;

    cv_bridge::CvImage out_depth_msg;
    out_depth_msg.header = l_image_msg->header;
    out_depth_msg.encoding = sensor_msgs::image_encodings::MONO16;
    out_depth_msg.image = cv::Mat(height, width, CV_16UC1);
    uint16_t * out_depth_msg_image_data = reinterpret_cast<uint16_t*>(&out_depth_msg.image.data[0]);

    // cv_bridge::CvImage out_msg;
    // out_msg.header = l_image_msg->header;
    // out_msg.encoding = sensor_msgs::image_encodings::MONO8;
    // out_msg.image = cv::Mat(height, width, CV_8UC1);
    // std::vector<int32_t> inliers;
    for (int32_t i=0; i<width*height; i++)
    {
      if (l_disp_data[i]>disp_max) 
        disp_max = l_disp_data[i];
      // out_msg.image.data[i] = (uint8_t)std::max(255.0*l_disp_data[i]/disp_max,0.0);
      //disp_msg->image.data[i] = l_disp_data[i];
      //disp_msg->image.data[i] = out_msg.image.data[i]

      float disp =  l_disp_data[i];
      // In milimeters
      //out_depth_msg_image_data[i] = disp;
      out_depth_msg_image_data[i] = disp <= 0.0f ? bad_point : (uint16_t)(depth_fact/disp);

      if (l_disp_data[i] > 0) 
        inliers.push_back(i);
    }
    // std_msgs::Header h = l_image_msg->header;//only for print an sabe file;

    // std::string out_string;
    // std::stringstream ss;
    // ss << h.stamp;
    // out_string = ss.str();

    // cv::imwrite( "/home/luis/disp_elas_img/scene2/only_elas/"+ out_string +".png", out_msg.image );
    // Publish


    // depthInfoMsg = l_info_msg;
    // depthInfoMsg.height =out_depth_msg.image.rows;// -1;TODO: CHECK, qui potrebbe essere -1
    // depthInfoMsg.width  =out_depth_msg.image.cols;// -1;

    // depthInfoMsg.header.stamp = ros::Time::now();
    // depthInfoMsg.header.frame_id = "stero";
    // // ros_cameraInfoMsg_camera00.header.frame_id = ros::this_node::getName();

    // depthInfoMsg.height = 0;
    // depthInfoMsg.width  = 0;
   
    // depthInfoMsg.height =cv_image02.rows;// -1;TODO: CHECK, qui potrebbe essere -1
    // depthInfoMsg.width  =cv_image02.cols;// -1;

    // depthInfoMsg.K.data(), 
    // depthInfoMsg.D, 
    // depthInfoMsg.R.data(), 
    // depthInfoMsg.P.data()
    // pub02.publish(ros_msg02, ros_cameraInfoMsg_camera02);

    // disp_pub_->publish(out_msg.toImageMsg());

    // disp_pub_->publish(out_msg.toImageMsg());
    // depth_pub_->publish(out_depth_msg.toImageMsg(), l_info_msg); //using camerainfo from left image for depth image
    depth_pub_->publish(out_depth_msg.toImageMsg());

    publish_point_cloud(l_image_msg, l_disp_data, inliers, width, height, l_info_msg, r_info_msg);

    // pub_disparity_.publish(disp_msg);

    // Cleanup data
    delete l_disp_data;
    delete r_disp_data;
  }

  


private:

  ros::NodeHandle nh;
  Subscriber left_sub_, right_sub_;//, pcl_sub_;
  InfoSubscriber left_info_sub_, right_info_sub_;
  ros::Subscriber shutdown_sub;

  // CloudSubscriber cloud_sub_;

  // boost::shared_ptr<Publisher> disp_pub_;
  boost::shared_ptr<Publisher> depth_pub_;
  boost::shared_ptr<Publisher> bev_pub_;
  boost::shared_ptr<Publisher> rgb_pub_;

  boost::shared_ptr<ros::Publisher> pc_pub_;
  // boost::shared_ptr<ros::Publisher> elas_fd_pub_;
  ExactSync *es;
  boost::shared_ptr<ExactSync> exact_sync_;
  ApproximateSync *ao;
  boost::shared_ptr<ApproximateSync> approximate_sync_;
  boost::shared_ptr<Elas> elas_;
  int queue_size_;

  image_geometry::StereoCameraModel model_;
  // ros::Publisher pub_disparity_;
  boost::scoped_ptr<Elas::parameters> param;



};




int main(int argc, char** argv)
{
  ros::init(argc, argv, "elas_ros");
  if (ros::names::remap("stereo") == "stereo") {
    ROS_WARN("'stereo' has not been remapped! Example command-line usage:\n"
             "\t$ rosrun viso2_ros stereo_odometer stereo:=narrow_stereo image:=image_rect");
  }
  if (ros::names::remap("image").find("rect_color") == std::string::npos) {
    ROS_WARN("stereo_odometer needs rectified input images. The used image "
             "topic is '%s'. Are you sure the images are rectified?",
             ros::names::remap("image").c_str());
  }

  std::string transport = argc > 1 ? argv[1] : "raw";
  
  Elas_Proc processor(transport);

  ros::spin();
  return 0;
}
