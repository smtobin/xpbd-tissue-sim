#ifndef __STIFFNESS_MAP_SIMULATION_HPP
#define __STIFFNESS_MAP_SIMULATION_HPP

#include "simulation/Simulation.hpp"
#include "config/simulation/StiffnessMapSimulationConfig.hpp"

#include "simobject/XPBDMeshObjectBaseWrapper.hpp"

namespace Sim
{

class StiffnessMapSimulation : public Simulation
{
public:
    /** For querying points on the mesh */
    struct QueryPoint
    {
        int face_ind;
        Vec3r face_barys;

        QueryPoint(int face_ind_, const Vec3r& face_barys_)
            : face_ind(face_ind_), face_barys(face_barys_)
        {}

        QueryPoint()
        {}
    };

    struct QueryResult
    {
        QueryPoint query_point;
        Mat3r stiffness_mat;

        QueryResult(QueryPoint qp, Mat3r mat)
            : query_point(qp), stiffness_mat(mat)
        {}
    };

    StiffnessMapSimulation(const Config::StiffnessMapSimulationConfig* config);

    virtual std::string type() const override { return "SitffnessMapSimulation"; }

    virtual void setup() override;

    const Geometry::TetMesh* tissueMesh() const { assert(_tissue_obj); return _tissue_obj.tetMesh(); }

    void addQueryPoint(int face_ind, const Vec3r& face_barys) { 
        std::cout << "Adding query point! Face ind: " << face_ind << "  Barys: " << face_barys.transpose() << std::endl;
        _query_points.emplace(face_ind, face_barys); 
    }
    const std::vector<QueryResult>& queryResults() const { return _results; }

protected:
    
    void _timeStep() override;

protected:

    XPBDMeshObject_BasePtrWrapper _tissue_obj;    // the tissue XPBD object that is being manipulated

    Real _displacement_magnitude; 
    Real _time_to_steady_state;

    
    std::queue<QueryPoint> _query_points;
    
    QueryPoint _cur_query_point;
    int _dir_index;
    Vec3r _initial_attach_position;
    Vec3r _attach_position;
    Vec3r _prescribed_displacement;
    Mat3r _cur_stiffness_matrix;

    Solver::ConstraintProjectorReferenceWrapper<Solver::FaceOffsetAttachmentConstraint> _attachment_constraint_proj;

    Real _displacement_application_time;

    std::vector<QueryResult> _results;

    bool _applying_force;
};

} // namespace Sim

#endif // __STIFFNESS_MAP_SIMULATION_HPP