/*
 *  Copyright (C) 2010 UT-Austin & Austin Robot Technology,
 *  David Claridge, Michael Quinlan, Jack O'Quin
 *  Copyright (C) 2012 Jack O'Quin
 * 
 *  License: Modified BSD Software License 
 */

/** @file

    @brief ROS nodelet for detecting obstacles in a point cloud.

*/

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <polar_height_map/heightmap.h>

namespace polar_height_map {

  class HeightMapNodelet: public nodelet::Nodelet
  {
  public:

    HeightMapNodelet() {}
    ~HeightMapNodelet() {}

    void onInit(void)
    {
      heightmap_.reset(new HeightMap(getNodeHandle(), getPrivateNodeHandle()));
    }

  private:

    boost::shared_ptr<HeightMap> heightmap_;
  };

}; // namespace polar_height_map

// Register this plugin with pluginlib.  Names must match nodelets.xml.
//
// parameters: class type, base class type
PLUGINLIB_EXPORT_CLASS(polar_height_map::HeightMapNodelet, nodelet::Nodelet)
