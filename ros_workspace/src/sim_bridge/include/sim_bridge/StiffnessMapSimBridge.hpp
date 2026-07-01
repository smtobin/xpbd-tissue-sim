#ifndef __STIFFNESS_MAP_SIM_BRIDGE_HPP
#define __STIFFNESS_MAP_SIM_BRIDGE_HPP

#include "simulation/StiffnessMapSimulation.hpp"

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include "sim_bridge/msg/local_compliance_matrix.hpp"
#include "sim_bridge/msg/point_on_face.hpp"

class StiffnessMapSimBridge : public rclcpp::Node
{

public:
    StiffnessMapSimBridge(Sim::StiffnessMapSimulation* sim);

protected:

    Sim::StiffnessMapSimulation* _sim;

    rclcpp::Publisher<sim_bridge::msg::LocalComplianceMatrix>::SharedPtr _compliance_mat_publisher;
    rclcpp::Subscription<sim_bridge::msg::PointOnFace>::SharedPtr _query_subscriber;

    // last index in the vector of results published
    // use this to determine when new results are in that need to be published
    int _last_ind_published;    

};

#endif