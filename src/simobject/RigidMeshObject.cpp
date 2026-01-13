#include "simobject/RigidMeshObject.hpp"
#include "utils/MeshUtils.hpp"
#include "utils/GeometryUtils.hpp"

namespace Sim
{

RigidMeshObject::RigidMeshObject(const Simulation* sim, const ConfigType* config)
    : RigidObject(sim, config), MeshObject(config, config)
{
    _density = config->density();
}

std::string RigidMeshObject::toString(const int indent) const
{
    std::string indent_str(indent, '\t');
    std::stringstream ss;
    ss << indent_str << "=====" << type() << "=====" << std::endl;
    ss << indent_str << "Mesh vertices: " << _mesh->numVertices() << std::endl;
    ss << indent_str << "Mesh faces: " << _mesh->numFaces() << std::endl;
    ss << RigidObject::toString(indent + 1);

    return ss.str(); 
}

Geometry::AABB RigidMeshObject::boundingBox() const
{
    return _mesh->boundingBox();
}

void RigidMeshObject::setup()
{
    loadAndConfigureMesh();

    // compute mass and inertia properties of mesh - in its REST STATE
    // meaning that we have to calculate the mass properties in the mesh's unrotated state
    Geometry::Mesh mesh_copy = *(_mesh);
    // untranslate the copy of the mesh
    mesh_copy.moveTogether(-_p);
    // unrotate the copy of the mesh
    const Mat3r rot_mat = GeometryUtils::quatToMat(GeometryUtils::inverseQuat(_q));
    mesh_copy.rotateAbout(Vec3r::Zero(), rot_mat);
    _initial_mesh = std::make_unique<Geometry::Mesh>(mesh_copy);
    std::tie(_m, std::ignore, _I) = mesh_copy.massProperties(_density);
    _I_inv = _I.inverse();
}

void RigidMeshObject::update()
{
    if(_fixed)
        return;

    RigidObject::update();

    // TODO: make this work without the need for _initial_mesh
    // move the mesh accordingly
    // THIS IS WRONG! (for some reason)
    const Vec4r dq = GeometryUtils::quatMult(GeometryUtils::inverseQuat(_q_prev), _q);
    const Mat3r rot_mat = GeometryUtils::quatToMat(dq);
    const Vec3r dx = _p - _p_prev;

    _mesh->moveTogether(dx);
    _mesh->rotateAbout(_p, rot_mat);

    // THIS SUCKS! have to copy the initial mesh every time - but it's right
    // *_mesh = *_initial_mesh;
    // const Mat3r rot_mat = GeometryUtils::quatToMat(_q);
    // _mesh->rotateAbout(Vec3r::Zero(), rot_mat);
    // _mesh->moveTogether(_p);

}

void RigidMeshObject::setPosition(const Vec3r& position)
{
    // 12/5/25 TODO: do we need this check?
    // I commented it out so that we can change the position of the trachea to be in the VB frame

    // if (_fixed)
    //     return;

    

    // TODO: make this work without the need for _initial_mesh
    // move the mesh accordingly
    // THIS IS WRONG! (for some reason)
    const Vec3r dx = position - _p;
    _mesh->moveTogether(dx);


    _p = position;
}

void RigidMeshObject::setOrientation(const Vec4r& orientation)
{
    // 12/5/25 TODO: do we need this check?
    // if (_fixed)
    //     return;
        
    // TODO: make this work without the need for _initial_mesh
    // move the mesh accordingly
    // THIS IS WRONG! (for some reason)
    const Vec4r dq = GeometryUtils::quatMult(GeometryUtils::inverseQuat(_q), orientation).normalized();
    const Mat3r rot_mat = GeometryUtils::quatToMat(dq);

    _mesh->rotateAbout(_p, rot_mat);

    _q = orientation;
}

void RigidMeshObject::rotateAboutOrigin(const Mat3r& rot_mat)
{
    // 12/5/25 TODO: do we need this check?
    // if (_fixed)
    //     return;

    _mesh->rotateAbout(Vec3r::Zero(), rot_mat);
    _q = GeometryUtils::quatMult(_q, GeometryUtils::matToQuat(rot_mat));
}

} // namespace Simulation