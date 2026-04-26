#include "uav_city_navigation/geometric_controller.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "geometric_controller");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  geometricCtrl* geometricController = new geometricCtrl(nh, nh_private);

  dynamic_reconfigure::Server<uav_city_navigation::GeometricControllerConfig> srv;
  dynamic_reconfigure::Server<uav_city_navigation::GeometricControllerConfig>::CallbackType f;
  f = boost::bind(&geometricCtrl::dynamicReconfigureCallback, geometricController, _1, _2);
  srv.setCallback(f);

  ros::spin();
  return 0;
}
