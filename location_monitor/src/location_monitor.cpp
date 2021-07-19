#include <vector>
#include <string>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
// #include "Landmark.h"
/* NOTE(elsuizo:2021-07-19): en lugar de hacer una variable global lo que hace
 * es crear una clase que encapsule el estado y ademas tenga el metodo que llamamos
 * como funcion de callback
 * Esto es muuucho mejor que andar haciendo variables globales...*/

/* NOTE(elsuizo:2021-07-19): yo lo hago a mi manera bien simple sin clases ni cosas
 * raras, solo una struct que tenga todo publico */

struct Landmark(string name, double x, double y): name(name), x(x), y(y) {
   string name;
   double x;
   double y;
};

struct LandmarkMonitor {
   vector<Landmark> landmarks;

   LandmarkMonitor(): landmarks() {
      init_landmarks();
   }

   void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
      double x = msg->pose.pose.position.x;
      double y = msg->pose.pose.position.y;
      ROS_INFO("x: %f, y: %f", x, y);
   }

   // unit_box = {1, 1}
   // unit_cylinder = {1, -1}
   // unit_sphere = {-1, 0}
   void init_landmarks() {
      landmarks.push_back(Landmark("unit_box", 1.0, 1.0));
      landmarks.push_back(Landmark("unit_cylinder", 1.0, -1.0));
      landmarks.push_back(Landmark("unit_sphere", -1.0, 0.0));
   }

};

int main(int argc, char** argv) {
   ros::init(argc, argv, "location_monitor");
   ros::NodeHandle node_handle;
   LandmarkMonitor monitor;
   /* NOTE(elsuizo:2021-07-19): no solo necesita la clase y el metodo sino que tambien la instancia en la que esta definida */
   ros::Subscriber subscriber = node_handle.subscribe("odom", 10, &LandmarkMonitor::OdomCallback, &monitor);
   ros::spin();
   return 0;
}
