#ifndef __MESH_OBJECT_HPP
#define __MESH_OBJECT_HPP

#include "geometry/Mesh.hpp"
#include "geometry/TetMesh.hpp"
#include "geometry/RefinedTetMesh.hpp"
#include "utils/MeshUtils.hpp"
#include "config/simobject/ObjectConfig.hpp"
#include "config/simobject/MeshObjectConfig.hpp"

namespace Sim
{

class MeshObject
{
    // public typedefs
    public:
    using ConfigType = Config::MeshObjectConfig;

    public:
    MeshObject(const ConfigType* mesh_config, const Config::ObjectConfig* obj_config)
    {
        _filename = mesh_config->filename();

        _initial_position = obj_config->initialPosition();
        _initial_rotation = obj_config->initialRotation();

        _initial_size = mesh_config->size();
        _max_size = mesh_config->maxSize();
        _initial_scaling = mesh_config->scaling();
    }

    virtual void serialize(std::vector<std::byte>& buf) const
    {
        pack(buf, _mesh);
        pack(buf, _filename);
        pack(buf, _initial_position);
        pack(buf, _initial_rotation);
        pack(buf, _initial_size);
        pack(buf, _max_size);
        pack(buf, _initial_scaling);
    }
    virtual void deserialize(const std::byte*& buf)
    {
        unpack(buf, _mesh);
        unpack(buf, _filename);
        unpack(buf, _initial_position);
        unpack(buf, _initial_rotation);
        unpack(buf, _initial_size);
        unpack(buf, _max_size);
        unpack(buf, _initial_scaling);
    }

    const Geometry::Mesh* mesh() const { return _mesh.get(); }

    Geometry::Mesh* mesh() { return _mesh.get(); }

    void loadAndConfigureMesh()
    {
        _loadMeshFromFile(_filename);

        // order matters here...
        // first apply scaling before rotating - either through the max-size criteria or a user-specified size
        if (_max_size.has_value())
        {
            _mesh->resize(_max_size.value());
        }

        else if (_initial_size.has_value())
        {
            _mesh->resize(_initial_size.value());
        }

        else if (_initial_scaling.has_value())
        {
            Geometry::AABB bbox = _mesh->boundingBox();
            Vec3r new_size = bbox.size().array() * _initial_scaling.value().array();
            _mesh->resize(new_size);
        }

        const Vec3r center_of_mass = _mesh->massCenter();

        // move center of mass of the mesh to the specified initial position
        _mesh->moveTogether(-center_of_mass + _initial_position);
        // _mesh->moveTogether(-_mesh->meshOrigin() + _initial_position);

        // then do rigid transformation - rotation and translation
        _mesh->rotateAbout(_initial_position, _initial_rotation);

        // important: the current state of the mesh is the "initial" state that we would like to treat as the undeformed state
        // as such, tell the mesh to recompute quantities so that it treats this state as the undeformed state
        _mesh->setCurrentStateAsUndeformedState();
    }

    protected:

    virtual void _loadMeshFromFile(const std::string& fname)
    {
        _mesh = std::make_unique<Geometry::Mesh>(MeshUtils::loadSurfaceMeshFromFile(fname));
    }

    void _scaleMesh()
    {
        
    }

    protected:
    std::unique_ptr<Geometry::Mesh> _mesh;

    private:
    std::string _filename;
    Vec3r _initial_position;
    Vec3r _initial_rotation;
    std::optional<Vec3r> _initial_size;
    std::optional<Real> _max_size;
    std::optional<Vec3r> _initial_scaling;
    

};

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

class TetMeshObject : public MeshObject
{
    public:
    TetMeshObject(const ConfigType* mesh_config, const Config::ObjectConfig* obj_config)
        : MeshObject(mesh_config, obj_config)
    {

    }

    const Geometry::TetMesh* tetMesh() const { return dynamic_cast<Geometry::TetMesh*>(_mesh.get()); }
    Geometry::TetMesh* tetMesh() { return dynamic_cast<Geometry::TetMesh*>(_mesh.get()); }

    protected:
    virtual void _loadMeshFromFile(const std::string& fname)
    {
        _mesh = std::make_unique<Geometry::TetMesh>(MeshUtils::loadTetMeshFromGmshFile(fname));
    }
};



////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

class RefinedTetMeshObject : public TetMeshObject
{
    public:
    RefinedTetMeshObject(const ConfigType* mesh_config, const Config::ObjectConfig* obj_config)
        : TetMeshObject(mesh_config, obj_config)
    {

    }

    const Geometry::RefinedTetMesh* refinedTetMesh() const { return dynamic_cast<Geometry::RefinedTetMesh*>(_mesh.get()); }
    Geometry::RefinedTetMesh* refinedTetMesh() { return dynamic_cast<Geometry::RefinedTetMesh*>(_mesh.get()); }

    protected:
    virtual void _loadMeshFromFile(const std::string& fname)
    {
        _mesh = std::make_unique<Geometry::RefinedTetMesh>(MeshUtils::loadTetMeshFromGmshFile(fname));
    }
};

} //namespace Sim

#endif // __MESH_OBJECT_HPP