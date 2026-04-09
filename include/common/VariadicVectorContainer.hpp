#ifndef __VARIADIC_VECTOR_CONTAINER_HPP
#define __VARIADIC_VECTOR_CONTAINER_HPP

#include "common/TypeList.hpp"
#include "common/TombstoneVector.hpp"

#include <vector>
#include <iostream>
#include <memory>

// helper to determine if the container type has the resize() method available
// (TombstoneVector doesn't have the resize method because "size" is ambiguous)
template<class T, typename = void>
struct has_resize : std::false_type {};

template<class T>
struct has_resize<T, std::void_t<decltype(std::declval<T>().resize(0))>> 
    : std::true_type {};



// adapted from this StackOverflow answer: https://stackoverflow.com/a/53112843

/** A class that stores a heterogeneous collection of vector types (i.e. vectors that each store different types).
 * The types stored are determined by the template parameters, meaning that it is determined at compile time.
 * 
 * Uses CRTP inheritance to recursively add a private member vector variable for each type.
 */
template<template<typename...> class Container, class L, class... R> class VariadicVectorContainer_;

template<template<typename...> class Container, class L>
class VariadicVectorContainer_<Container, L>
{
    protected:
    const Container<L>& _get() const
    {
        return _vec;
    }

    Container<L>& _get()
    {
        return _vec;
    }

    size_t _size() const
    {
        return _vec.size();
    }

    // SFINAE to only enable resize when it's actually available
    template<typename C = Container<L>>
    std::enable_if_t<has_resize<C>::value, void>
    _resize(int size)
    {
        _vec.resize(size);
    }

    void _reserve(int size)
    {
        _vec.reserve(size);
    }

    void _push_back(const L& elem)
    {
        _vec.push_back(elem);
    }

    void _push_back(L&& elem)
    {
        _vec.push_back(std::move(elem));
    }

    template<class ...Args>
    L& _emplace_back(Args&&... args)
    {
        return _vec.emplace_back(std::forward<Args>(args)...);
    } 

    L& _set(int index, const L& elem)
    {
        _vec[index] = elem;
        return _vec[index];
    }

    L& _set(int index, L&& elem)
    {
        _vec[index] = std::move(elem);
        return _vec[index];
    }

    void _clear()
    {
        _vec.clear();
    }

    // load and save
    void _serialize(std::vector<std::byte>& buf) const
    {
        pack(buf, _vec);
    }

    void _deserialize(const std::byte*& cursor)
    {
        unpack(cursor, _vec);
    }

    // internal method for visiting elements
    template<typename Visitor>
    void _for_each_element(Visitor&& visitor) const
    {
        for (const auto& elem : _vec)
        {
            visitor(elem);
        }
    }

    template<typename Visitor>
    void _for_each_element(Visitor&& visitor)
    {
        for (auto& elem : _vec)
        {
            visitor(elem);
        }
    }

    private:
    Container<L> _vec;
};

template<template<typename...> class Container, class L, class... R>
class VariadicVectorContainer_ : public VariadicVectorContainer_<Container, L>, public VariadicVectorContainer_<Container, R...>
{
    public:
    void serialize(std::vector<std::byte>& buf) const
    {
        _serialize_helper<L, R...>(buf);
    }
    void deserialize(const std::byte*& cursor)
    {
        _deserialize_helper<L, R...>(cursor);
    }

    size_t size() const
    {
        return _size_helper<L, R...>();
    }

    template<class T>
    const Container<T>& get() const
    {
        return this->VariadicVectorContainer_<Container, T>::_get();
    }

    template<class T>
    Container<T>& get()
    {
        return this->VariadicVectorContainer_<Container, T>::_get();
    }

    template<class T>
    void push_back(const T& elem)
    {
        return this->VariadicVectorContainer_<Container, T>::_push_back(elem);
    }

    template<class T>
    void push_back(T&& elem)
    {
        return this->VariadicVectorContainer_<Container, T>::_push_back(std::move(elem));
    }

    template<class T, class ...Args>
    T& emplace_back(Args&&... args)
    {
        return this->VariadicVectorContainer_<Container, T>::_emplace_back(std::forward<Args>(args)...);
    }

    template<class T>
    std::enable_if_t<has_resize<Container<T>>::value, void>
    resize(int size)
    {
        return this->VariadicVectorContainer_<Container, T>::_resize(size);
    }

    template<class T>
    void reserve(int size)
    {
        return this->VariadicVectorContainer_<Container, T>::_reserve(size);
    }

    template<class T>
    T& set(int index, const T& elem)
    {
        return this->VariadicVectorContainer_<Container, T>::_set(index, elem);
    }

    template<class T>
    T& set(int index, T&& elem)
    {
        return this->VariadicVectorContainer_<Container, T>::_set(index, std::move(elem));
    }

    template<class T>
    size_t size() const
    {
        return this->VariadicVectorContainer_<Container, T>::_size();
    }

    template<class T>
    void clear()
    {
        return this->VariadicVectorContainer_<Container, T>::_clear();
    }

    // visit all elements in a subset of types - only enable this overload if sizeof(Ts) > 0
    template<typename... Ts, typename Visitor>
    std::enable_if_t<(sizeof...(Ts) > 0), void>
    for_each_element(Visitor&& visitor) const
    {
        _visit_elements<Ts...>(std::forward<Visitor>(visitor));
    }

    template<typename... Ts, typename Visitor>
    std::enable_if_t<(sizeof...(Ts) > 0), void>
    for_each_element(Visitor&& visitor)
    {
        _visit_elements<Ts...>(std::forward<Visitor>(visitor));
    }

    // visit all elements across all vectors
    template<typename Visitor>
    void for_each_element(Visitor&& visitor) const
    {
        _visit_elements<L, R...>(std::forward<Visitor>(visitor));
    }

    template<typename Visitor>
    void for_each_element(Visitor&& visitor)
    {
        _visit_elements<L, R...>(std::forward<Visitor>(visitor));
    }

    private:
    // recursive implementation to visit elements of all types
    template<typename T, typename... Ts, typename Visitor>
    void _visit_elements(Visitor&& visitor) const
    {
        this->VariadicVectorContainer_<Container, T>::_for_each_element(visitor);

        if constexpr (sizeof...(Ts) > 0)
        {
            _visit_elements<Ts...>(std::forward<Visitor>(visitor));
        }
    }

    template<typename T, typename... Ts, typename Visitor>
    void _visit_elements(Visitor&& visitor)
    {
        this->VariadicVectorContainer_<Container, T>::_for_each_element(visitor);

        if constexpr (sizeof...(Ts) > 0)
        {
            _visit_elements<Ts...>(std::forward<Visitor>(visitor));
        }
    }

    template<typename T, typename... Ts>
    size_t _size_helper() const
    {
        size_t sizeT = this->VariadicVectorContainer_<Container, T>::_size();
        size_t sizeTs = 0;
        if constexpr (sizeof...(Ts) > 0)
        {
            sizeTs = _size_helper<Ts...>();
        }
        
        return sizeT + sizeTs;
    }

    template<typename T, typename... Ts>
    void _serialize_helper(std::vector<std::byte>& buf) const
    {
        this->VariadicVectorContainer_<Container, T>::_serialize(buf);
        if constexpr (sizeof...(Ts) > 0)
        {
            _serialize_helper<Ts...>(buf);
        }
    }

    template<typename T, typename... Ts>
    void _deserialize_helper(const std::byte*& cursor)
    {
        this->VariadicVectorContainer_<Container, T>::_deserialize(cursor);
        if constexpr (sizeof...(Ts) > 0)
        {
            _deserialize_helper<Ts...>(cursor);
        }
    }
};

template<typename... Types>
using VariadicVectorContainer = VariadicVectorContainer_<std::vector, Types...>;

template<typename... Types>
using VariadicTombstoneVectorContainer = VariadicVectorContainer_<TombstoneVector, Types...>;

//////////////////////////////////////////////////////////////////////////
// Construct VariadicVectorContainer_ from TypeList
//////////////////////////////////////////////////////////////////////////

template<typename List>
struct VariadicVectorContainerFromTypeList;

template<typename... Types>
struct VariadicVectorContainerFromTypeList<TypeList<Types...>>
{
    using type = VariadicVectorContainer_<std::vector, Types...>;
    using unique_ptr_type = VariadicVectorContainer_<std::vector, std::unique_ptr<Types>...>;
    using ptr_type = VariadicVectorContainer_<std::vector, Types*...>;
    using const_ptr_type = VariadicVectorContainer_<std::vector, const Types*...>;
};


#endif // __VARIADIC_VECTOR_CONTAINER_HPP