#include "sim_bridge/StiffnessMapSimBridge.hpp"

StiffnessMapSimBridge::StiffnessMapSimBridge(Sim::StiffnessMapSimulation* sim)
    : rclcpp::Node("stiffness_map_sim_bridge"),
     _sim(sim),
     _last_ind_published(-1)
{
    // set up publisher for local stiffness matrices
    _compliance_mat_publisher = this->create_publisher<sim_bridge::msg::LocalComplianceMatrix>("sim/output/local_stiffness_mats", 10);

    auto publisher_callback = [this]() -> void
    {
        const auto& results = this->_sim->queryResults();
        if (this->_last_ind_published+1 < (int)results.size())
        {
            _last_ind_published++;

            // new results - publish the next one
            sim_bridge::msg::LocalComplianceMatrix msg;
            msg.header.stamp = this->now();
            msg.header.frame_id = "sim/world";

            msg.point_on_face.face_ind = results[_last_ind_published].query_point.face_ind;
            Vec3r barys = results[_last_ind_published].query_point.face_barys;
            msg.point_on_face.face_barys.x = barys[0];
            msg.point_on_face.face_barys.y = barys[1];
            msg.point_on_face.face_barys.z = barys[2];
            msg.compliance_mat.resize(9);
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    msg.compliance_mat[3*i + j] = results[_last_ind_published].compliance_mat(i,j);
            
            this->_compliance_mat_publisher->publish(msg);
        }
    };
    _sim->addCallback(1.0/30.0, publisher_callback);

    // set up subscriber for incoming query points
    auto query_point_callback = [&](sim_bridge::msg::PointOnFace::UniquePtr msg)
    {
        std::cout << "Received query point!" << std::endl;
        std::cout << "  Face index: " << msg->face_ind << std::endl;
        std::cout << "  Barys: " << msg->face_barys.x << ", " << msg->face_barys.y << ", " << msg->face_barys.z << std::endl;
        Vec3r barys(msg->face_barys.x, msg->face_barys.y, msg->face_barys.z);
        int face_ind = msg->face_ind;
        auto callback = [this, face_ind, barys]() -> void
        {
            this->_sim->addQueryPoint(face_ind, barys);
        };
        this->_sim->addOneTimeCallback(callback);
    };
    _query_subscriber = this->create_subscription<sim_bridge::msg::PointOnFace>("sim/input/stiffness_query_points", 10, query_point_callback);
}