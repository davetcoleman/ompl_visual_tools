
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

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

  vector<Pixel> map;
  map.push_back( Pixel( 1, 1, 1, 1 ) );
  map.push_back( Pixel( 2, 1, 1, 1 ) );
  map.push_back( Pixel( 3, 1, 1, 1 ) );
  map.push_back( Pixel( 1, 2, 1, 2 ) );
  map.push_back( Pixel( 2, 2, 1, 3 ) );
  map.push_back( Pixel( 3, 2, 1, 3 ) );
  map.push_back( Pixel( 1, 3, 1, 2 ) );
  map.push_back( Pixel( 2, 3, 1, 3 ) );
  map.push_back( Pixel( 3, 3, 1, 1 ) );

  int marker_id = 0;
  for( vector<Pixel>::const_iterator pixel_it = map.begin(); pixel_it != map.end(); ++pixel_it )
  {
    rate.sleep();

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = ++marker_id;

    cout << "Adding shape #" << marker.id << " -- " << pixel_it->x_ << " " << pixel_it->y_ << " " << pixel_it->z_ << endl;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;
    /*      shape = visualization_msgs::Marker::SPHERE;
            shape = visualization_msgs::Marker::ARROW;
            shape = visualization_msgs::Marker::CYLINDER;
            shape = visualization_msgs::Marker::CUBE;*/

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = pixel_it->x_;
    marker.pose.position.y = pixel_it->y_;
    marker.pose.position.z = pixel_it->z_;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    switch( pixel_it->color_ )
    {
    case 1:
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;
      break;
    case 2:
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;
      break;
    default:
      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;
      marker.color.a = 1.0;
    }

    //marker.lifetime = ros::Duration(10.0);
    //marker.lifetime = ros::Duration();

    //marker_pub.publish(marker);
    marker_array.markers.push_back( marker );

  }

  // Publish the marker array
  marker_pub.publish( marker_array );

  ros::Duration(2).sleep();

  cout << "Done publishing" << endl;
}
