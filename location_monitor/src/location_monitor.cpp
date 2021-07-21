#include <vector>
#include <string>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "location_monitor/LandmarkDistance.h"
// #include "Landmark.h"
/* NOTE(elsuizo:2021-07-19): en lugar de hacer una variable global lo que hace
 * es crear una clase que encapsule el estado y ademas tenga el metodo que llamamos
 * como funcion de callback
 * Esto es muuucho mejor que andar haciendo variables globales...*/

/* NOTE(elsuizo:2021-07-19): yo lo hago a mi manera bien simple sin clases ni cosas
 * raras, solo una struct que tenga todo publico */

/* NOTE(elsuizo:2021-07-19): podemos usar el namespace para asi no tenemos que tipear siempre esto */
using location_monitor::LandmarkDistance;

struct Landmark {
   std::string name;
   double x;
   double y;
};

struct LandmarkMonitor {
   std::vector<Landmark> landmarks;

   ros::Publisher landmark_publisher;

   LandmarkMonitor(const ros::Publisher& landmark_pub): landmarks(), landmark_publisher(landmark_pub) {
      init_landmarks();
   }

   double get_distance(Landmark landmark, double x, double y) {
      double diff_x = landmark.x - x;
      double diff_y = landmark.y - y;
      return std::sqrt(diff_x * diff_x + diff_y * diff_y);
   }

   LandmarkDistance find_closest(double x, double y) {
      LandmarkDistance result;
      result.distance = -1;
      for (size_t i = 0; i < landmarks.size(); ++i) {
         const Landmark& landmark = landmarks[i];
         double distance = get_distance(landmark, x, y);
         if(result.distance < 0 || distance < result.distance) {
            result.name = landmark.name;
            result.distance = distance;
         }
      }
      return result;
   }

   void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
      double x = msg->pose.pose.position.x;
      double y = msg->pose.pose.position.y;
      LandmarkDistance landmarkd_distance = find_closest(x, y);
      /* NOTE(elsuizo:2021-07-19): cuando tenemos que imprimir un string tenemos que convertirlo con `c_str()` */
      // ROS_INFO("landmark closest: %s, distance: %f", landmarkd_distance.name.c_str(), landmarkd_distance.distance);
      landmark_publisher.publish(landmarkd_distance);
   }

   /* NOTE(elsuizo:2021-07-21): here we harcode the position of the landmarks in the `empty.world` */
   void init_landmarks() {
      landmarks.push_back(Landmark{"unit_box", 1.0, 1.0});
      landmarks.push_back(Landmark{"unit_cylinder", 1.0, -1.0});
      landmarks.push_back(Landmark{"unit_sphere", -1.0, 0.0});
   }

};

int main(int argc, char** argv) {
   ros::init(argc, argv, "location_monitor");
   ros::NodeHandle node_handle;
   ros::Publisher landmark_publisher = node_handle.advertise<LandmarkDistance>("closest_landmark", 10);
   LandmarkMonitor monitor(landmark_publisher);
   /* NOTE(elsuizo:2021-07-19): no solo necesita la clase y el metodo sino que tambien la instancia en la que esta definida */
   ros::Subscriber subscriber = node_handle.subscribe("odom", 10, &LandmarkMonitor::OdomCallback, &monitor);
   ros::spin();
   return 0;
}
