#ifndef __MESH_PROPERTY_HPP
#define __MESH_PROPERTY_HPP

#include "common/types.hpp"
#include "common/TypeList.hpp"
#include "common/VariadicVectorContainer.hpp"

namespace Geometry
{

template<typename T>
class MeshProperty
{

    public:
    MeshProperty() = default; // required for deserialization

    MeshProperty(const std::string& name, int size, bool is_field)
        : _name(name), _properties(size), _is_field(is_field)
    {
    }

    MeshProperty(const std::string& name, int size, const T& default_value, bool is_field)
        : _name(name), _properties(size, default_value), _is_field(is_field)
    {
    }

    void serialize(std::vector<std::byte>& buf) const
    {
        pack(buf, _name);
        pack(buf, _properties);
        pack(buf, _default_value);
        pack(buf, _is_field);
    }

    void deserialize(const std::byte*& cursor)
    {
        unpack(cursor, _name);
        unpack(cursor, _properties);
        unpack(cursor, _default_value);
        unpack(cursor, _is_field);
    }

    const std::string& name() const { return _name; }

    bool isField() const { return _is_field; }

    // Specialize the getter for bool to return by value
    std::conditional_t<std::is_same_v<T, bool>, T, const T&> 
    get(int index) const { 
        return _properties.at(index); 
    }

    void set(int index, const T& new_val) { _properties[index] = new_val; }

    void resize(size_t new_size) 
    {
        if (_default_value.has_value()) 
            _properties.resize(new_size, _default_value.value());
        else
            _properties.resize(new_size); 
    }

    void resize(size_t new_size, const T& val)
    {
        _properties.resize(new_size, val);
    }

    const std::vector<T>& properties() const { return _properties; }
    std::vector<T>& properties() { return _properties; }

    protected:
    std::string _name;      // the name of the mesh property
    std::vector<T> _properties;     // the actual per-feature property
    std::optional<T> _default_value;    // the default value when resizing 
    bool _is_field;     // whether or not this property is a "field", i.e. like voltage or temperature, smoothly varying over the mesh

};

typedef TypeList<bool, int, Real> MeshPropertyTypeList;

template <typename PropertyTypeList> class PropertyContainer;

template <typename ...PropertyTypes>
class PropertyContainer<TypeList<PropertyTypes...>> : public VariadicVectorContainer<MeshProperty<PropertyTypes>...>
{
};



} // namespace Geometry


#endif // __MESH_PROPERTY_HPP