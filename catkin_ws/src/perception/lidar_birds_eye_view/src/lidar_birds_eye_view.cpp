#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/types.hpp>
// #include <msgs_perception/ObstacleArray.h>
// #include <msgs_perception/Obstacle.h>
#include <tf/transform_datatypes.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <std_msgs/Bool.h>

//static const std::string OPENCV_WINDOW = "Image window";
class LidarBEV
  {
    ros::NodeHandle nh_;

    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    // msgs_perception::ObstacleArray::ConstPtr radarDataArray_;
    double map_scale;
    image_transport::ImageTransport it_;

    ros::Subscriber pcl_sub_;
    image_transport::Publisher image_pub_;
    ros::Subscriber shutdown_sub;

public:
  LidarBEV()
    : it_(nh_)
  {

    map_scale = 8.0;

    pcl_sub_ = nh_.subscribe<PointCloud>("/carina/sensor/lidar/front/point_cloud", 1, &LidarBEV::lidarCallback, this);
    image_pub_ = it_.advertise("/carina/sensor/lidar/bev_point_cloud", 1);
    shutdown_sub = nh_.subscribe("/carina/vehicle/shutdown", 1, &LidarBEV::shutdown_cb, this);
  }
 
  ~LidarBEV()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
  }

  void lidarCallback(const PointCloud::ConstPtr& pcl_in)
  {
     std::ostringstream oss;
     oss << pcl_in->header.stamp;
     std::string numString = oss.str();
     boost::erase_all(numString, ".");

     int size_image=700;
     int actual_xy;
   
    cv::Mat bev_lidar_f(size_image,  size_image, CV_8UC1, cv::Scalar(0));
    BOOST_FOREACH (const pcl::PointXYZ& pt, pcl_in->points)
    {

      double x_point = pt.y;
      double y_point = pt.x;
      float z_height = pt.z;

      double v =  (255.0f*(z_height-(-3.0f) )/((1.0f)-(-3.0f)));
      
      int px = (int)(size_image/2)+(int)(x_point*map_scale);
      int py = (int)(size_image/3)+(int)(y_point*map_scale);

      if (px < size_image and px >= 0 and py < size_image and py >= 0)
      {
        actual_xy = (int)bev_lidar_f.at<uchar>(py, px);
        if ((int)v > actual_xy)
        {
          circle( bev_lidar_f, cv::Point(px, py),  1, cv::Scalar( v ), -1, 8 );
          // std::cout << actual_xy <<'\n';
          // std::cout << v <<'\n';        
        }  

      }



    } 
    
    sensor_msgs::ImagePtr bev_img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", bev_lidar_f).toImageMsg();

    image_pub_.publish(bev_img_msg);

  }

  void shutdown_cb(const std_msgs::Bool::ConstPtr &msg){
    if (msg->data){
     
      pcl_sub_.shutdown() ;
      image_pub_.shutdown() ;
      shutdown_sub.shutdown() ;

      shutdown_sub.shutdown();

      std::cout<<"Bye!"<<std::endl;
      ros::shutdown();
    }
  }

};
int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_birds_eye_view_node");
  LidarBEV lbev;

  ros::spin();
  return 0;

}