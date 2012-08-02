
#include <ros/ros.h>
#include <ros/package.h> // for getting file path for loading images
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "ompl_rviz_viewer/ppm.h"


using namespace std;

// *********************************************************************************************************************
// Struct
// *********************************************************************************************************************
struct Pixel
{
  double x_, y_, z_;
  int color_;

  Pixel( double x, double y, double z, int color ) :
    x_(x), y_(y), z_(z), color_(color)
  {}
};

// *********************************************************************************************************************
// Main
// *********************************************************************************************************************
int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate rate(40);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

  visualization_msgs::MarkerArray marker_array;


  /*
    std::string image_path = ros::package::getPath("ompl_rviz_viewer");
    if( image_path.empty() )
    {
    std::cout << "Unable to get OMPL RViz Viewer package path " << std::endl;
    exit(0);
    }
    image_path.append( "/resources/height_map1.ppm" );
    std::cout << image_path << std::endl;
  */

  // Image file path
  std::string image_path = "../resources/mountains.ppm";
  //  std::string image_path = "../resources/smile.ppm";
  //  std::string image_path = "../resources/height_map2.ppm";

  // Load cost map from image file
  PPMImage *image = readPPM( image_path.c_str() );

  // Error check
  if( !image )
  {
    ROS_ERROR( "No image data loaded " );
    return false;
  }

  std::cout << "Image height: " << image->y << " width: " << image->x << std::endl;
  //ros::Duration(5).sleep();

  int max_rgb = 0;

  for( int marker_id = 0; marker_id < image->x * image->y; ++marker_id )
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = marker_id;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;
    /*      shape = visualization_msgs::Marker::SPHERE;
            shape = visualization_msgs::Marker::ARROW;
            shape = visualization_msgs::Marker::CYLINDER;
            shape = visualization_msgs::Marker::CUBE;*/

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = (200 - image->data[ marker_id ].red) / 255.0;
    marker.color.g = (200 - image->data[ marker_id ].green) / 255.0;
    marker.color.b = (100 - image->data[ marker_id ].blue) / 255.0;
    marker.color.a = 1.0;

    int cost = (image->data[ marker_id ].red ) / 5;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = marker_id % image->x;    // Map index back to coordinates
    marker.pose.position.y = marker_id / image->x;    // Map index back to coordinates
    marker.pose.position.z = cost / 2;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = cost;

    if( marker.color.r > max_rgb )
      max_rgb = marker.color.r;


      cout << "#" << marker.id << " COST: " << cost
        //      << " COORD: "
        //      << marker.pose.position.x << " "
        //      << marker.pose.position.y << " "
        //      << marker.pose.position.z << " "
      << " COLOR: "
      << marker.color.r << " "
      << marker.color.g << " "
      << marker.color.b << " " << std::endl;


    //marker.lifetime = ros::Duration(10.0);
    //marker.lifetime = ros::Duration();

    //marker_pub.publish(marker);
    marker_array.markers.push_back( marker );

    //   if( marker_id > 100 )
    //      break;
  }

  // Publish the marker array
  marker_pub.publish( marker_array );

  ros::Duration(2).sleep();

  std::cout << "Done publishing. Max color value was " << max_rgb << std::endl;
}
