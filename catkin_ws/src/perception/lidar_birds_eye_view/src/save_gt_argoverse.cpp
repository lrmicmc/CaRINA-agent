#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/types.hpp>
#include <msgs_perception/ObstacleArray.h>
#include <msgs_perception/Obstacle.h>
#include <tf/transform_datatypes.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  msgs_perception::ObstacleArray::ConstPtr radarDataArray_;
  double map_resolution;

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Subscriber sub_radar;


  // image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_map;
  image_transport::Publisher image_pub_map;
  image_transport::Publisher image_pub_pc_gray;
  image_transport::Publisher image_pub_gt;




public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    // image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);

    // image_pub_ = it_.advertise("/image_converter/output_video", 1);

    sub_radar = nh_.subscribe("/carina/perception/lidar/obstacles_array", 1,  &ImageConverter::callbackRadar, this);
  
    // image_pub_= it_.advertise("/image_converter/output_video", 1);

    // cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    // cv::destroyWindow(OPENCV_WINDOW);
  }







  void callbackRadar(const msgs_perception::ObstacleArray::ConstPtr& obstaclesGT) 
  {
        double map_resolution = 5.0;

  radarDataArray_ = obstaclesGT;

  cv::Mat bev_lidar_b_gt(568*2, 568*2, CV_8UC3, cv::Scalar(0,0,0));
  cv::Mat bev_lidar_f_gt(568*2, 568*2, CV_8UC3, cv::Scalar(0,0,0)); 

  std::ostringstream oss;
  oss << radarDataArray_->obstacle[0].ns;
  std::string numString = oss.str();
  // boost::erase_all(numString, ".");
  int classes_id = 0;
    for (int j = 0; j < radarDataArray_->obstacle.size(); j++) 
    { 

      double x_point = radarDataArray_->obstacle[j].pose.position.y;
      double y_point = radarDataArray_->obstacle[j].pose.position.x;
      // double z = radarDataArray_->obstacle[j].pose.position.z;
      double w = radarDataArray_->obstacle[j].scale.y * map_resolution;
      double h = radarDataArray_->obstacle[j].scale.x * map_resolution;
      double obj_yaw = tf::getYaw(radarDataArray_->obstacle[j].pose.orientation);

      double yaw_degrees = -(obj_yaw * 180.0 / M_PI); // conversion to degrees

      std::string classe_name = radarDataArray_->obstacle[j].classes[0];


      if (classe_name == "VEHICLE"){
        classes_id=1;
      }

      if (classe_name == "PEDESTRIAN"){
        classes_id=2;
      }

      if (classe_name == "ON_ROAD_OBSTACLE"){
        classes_id=3;
      }

      if (classe_name == "LARGE_VEHICLE"){
        classes_id=4;
      }

      if (classe_name == "BICYCLE"){
        classes_id=5;
      }

      if (classe_name == "BICYCLIST"){
        classes_id=6;
      }

      if (classe_name == "BUS"){
        classes_id=7;
      }

      if (classe_name == "OTHER_MOVER"){
        classes_id=8;
      }

      if (classe_name == "TRAILER"){
        classes_id=9;
      }



      if (classe_name == "MOTORCYCLIST"){
        classes_id=10;
      }

      if (classe_name == "MOPED"){
        classes_id=11;
      }

      if (classe_name == "MOTORCYCLE"){
        classes_id=12;
      }

      if (classe_name == "STROLLER"){
        classes_id=13;
      }

      if (classe_name == "EMERGENCY_VEHICLE"){
        classes_id=14;
      }

      if (classe_name == "ANIMAL"){
        classes_id=15;
      }

      if (classe_name == "WHEELCHAIR"){
        classes_id=16;
      }

      if (classe_name == "SCHOOL_BUS"){
        classes_id=17;
      }





        int px = 568+(int)(x_point*map_resolution);
        int py = 568+(int)(y_point*map_resolution);
        cv::RotatedRect rRect = cv::RotatedRect(cv::Point2f(px,py), 
        cv::Size2f(w,h), yaw_degrees);
        cv::Point2f vertices2f[4];
        rRect.points(vertices2f);
        cv::Point vertices[4];
        for (int i = 0; i < 4; i++)
            vertices[i] = vertices2f[i];

        cv::fillConvexPoly(bev_lidar_f_gt, vertices, 4, cv::Scalar(0,0,classes_id));
    }
  cv::flip(bev_lidar_f_gt, bev_lidar_f_gt, -1);
  imwrite( "/media/luis/Data/Argo_dataset/bird_eye_view_lidar/gt/"+ numString +".png", bev_lidar_f_gt);
    // sensor_msgs::ImagePtr msg_out = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bev_lidar_f_gt).toImageMsg();
    // image_pub_gt.publish(msg_out);
    // cv::imshow("lidar", bev_lidar_f_gt);
   //     // cv::imshow("imageMap", cv_bridge::toCvShare(msg, "bgr8")->image);
   //    cv::waitKey(100);    
  
  // numString.erase (numString.begin()+16, numString.end());
}





};






int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ImageConverter ic;
  ros::spin();
  return 0;
}


