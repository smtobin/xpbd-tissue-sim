#ifndef __PACK_HPP
#define __PACK_HPP

#include <optional>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <memory>
#include <variant>
#include <cstddef>
#include <string>
#include <cassert>
#include <iostream>

#include <Eigen/Dense>

// detection helper
template<typename T, typename = void>
struct has_serialize : std::false_type {};

template<typename T>
struct has_serialize<T, std::void_t<decltype(std::declval<T>().serialize(std::declval<std::vector<std::byte>&>()))>> : std::true_type {};

template<typename T, typename = void>
struct has_deserialize : std::false_type {};

template<typename T>
struct has_deserialize<T, std::void_t<decltype(std::declval<T>().deserialize(std::declval<const std::byte*&>()))>> : std::true_type {};

// see if type T has serialize function through ADL lookup
template<typename T>
class has_adl_serialize
{
private:
    template<typename U>
    static auto test(int) -> decltype(
        serialize(std::declval<std::vector<std::byte>&>(),
                  std::declval<const U&>()),
        std::true_type{}
    );

    template<typename>
    static std::false_type test(...);

public:
    static constexpr bool value =
        decltype(test<T>(0))::value;
};

// see if type T has deserialize function through ADL lookup
template<typename T>
class has_adl_deserialize
{
private:
    template<typename U>
    static auto test(int) -> decltype(
        deserialize(std::declval<const std::byte*&>(),
                  std::declval<U&>()),
        std::true_type{}
    );

    template<typename>
    static std::false_type test(...);

public:
    static constexpr bool value =
        decltype(test<T>(0))::value;
};


/** Forward declarations */
template<typename T>
std::enable_if_t<has_serialize<T>::value>
pack(std::vector<std::byte>& buf, const T& val);
template<typename T>
std::enable_if_t<has_deserialize<T>::value>
unpack(const std::byte*& cursor, T& var);


template<typename T>
std::enable_if_t<!has_serialize<T>::value && std::is_trivially_copyable_v<T>>
pack(std::vector<std::byte>& buf, const T& val);
template<typename T>
std::enable_if_t<!has_deserialize<T>::value && std::is_trivially_copyable_v<T>>
unpack(const std::byte*& cursor, T& var);

void pack(std::vector<std::byte>& buf, const std::string& str);
void unpack(const std::byte*& cursor, std::string& str);

template<typename T>
void pack(std::vector<std::byte>& buf, const std::optional<T>& opt);
template<typename T>
void unpack(const std::byte*& cursor, std::optional<T>& opt);

template<typename... Ts>
void pack(std::vector<std::byte>& buf, const std::variant<Ts...>& var);
template<typename... Ts>
void unpack(const std::byte*& cursor, std::variant<Ts...>& var);

template<typename T>
void pack(std::vector<std::byte>& buf, const std::unique_ptr<T>& ptr);
template<typename T>
void unpack(const std::byte*& cursor, std::unique_ptr<T>& ptr);

template<typename T>
void pack(std::vector<std::byte>& buf, const std::vector<T>& vec);
template<typename T>
void unpack(const std::byte*& cursor, std::vector<T>& vec);

template<typename T, size_t N>
void pack(std::vector<std::byte>& buf, const std::array<T, N>& arr);
template<typename T, size_t N>
void unpack(const std::byte*& cursor, std::array<T, N>& arr);

template<typename K, typename V, typename Hash, typename KeyEqual, typename Alloc>
void pack(std::vector<std::byte>& buf, const std::unordered_map<K, V, Hash, KeyEqual, Alloc>& map);
template<typename K, typename V, typename Hash, typename KeyEqual, typename Alloc>
void unpack(const std::byte*& cursor, std::unordered_map<K, V, Hash, KeyEqual>& map);

template<typename K, typename V, typename Hash, typename KeyEqual, typename Alloc>
void pack(std::vector<std::byte>& buf, const std::unordered_multimap<K, V, Hash, KeyEqual, Alloc>& map);
template<typename K, typename V, typename Hash, typename KeyEqual, typename Alloc>
void unpack(const std::byte*& cursor, std::unordered_multimap<K, V, Hash, KeyEqual, Alloc>& map);

template<typename T, typename Hash, typename KeyEqual, typename Alloc>
void pack(std::vector<std::byte>& buf, const std::unordered_set<T, Hash, KeyEqual, Alloc>& set);
template<typename T, typename Hash, typename KeyEqual, typename Alloc>
void unpack(const std::byte*& cursor, std::unordered_set<T, Hash, KeyEqual, Alloc>& set);

template<typename T>
void pack(std::vector<std::byte>& buf, const std::queue<T>& queue);
template<typename T>
void unpack(const std::byte*& cursor, std::queue<T>& queue);

template<typename Derived>
std::enable_if_t<std::is_base_of_v<Eigen::MatrixBase<Derived>, Derived>>
pack(std::vector<std::byte>& buf, const Derived& mat);
template<typename Derived>
std::enable_if_t<std::is_base_of_v<Eigen::MatrixBase<Derived>, Derived>>
unpack(const std::byte*& cursor, Derived& mat);


/**
 * User-defined types with serialize and deserialize implemented
 */

template<typename T>
inline std::enable_if_t<has_serialize<T>::value>
pack(std::vector<std::byte>& buf, const T& val)
{
    val.serialize(buf);
}

template<typename T>
inline std::enable_if_t<has_deserialize<T>::value>
unpack(const std::byte*& cursor, T& var)
{
    var.deserialize(cursor);
}


/**
 * Trivially copyable types
 */
template<typename T>
inline std::enable_if_t<!has_serialize<T>::value && std::is_trivially_copyable_v<T>>
pack(std::vector<std::byte>& buf, const T& val)
{
    const auto* bytes = reinterpret_cast<const std::byte*>(&val);
    buf.insert(buf.end(), bytes, bytes + sizeof(T));
}

template<typename T>
inline std::enable_if_t<!has_deserialize<T>::value && std::is_trivially_copyable_v<T>>
unpack(const std::byte*& cursor, T& var)
{
    static_assert(std::is_trivially_copyable_v<T>);
    std::memcpy(&var, cursor, sizeof(T));
    cursor += sizeof(T);
}


/** std::string */

inline void pack(std::vector<std::byte>& buf, const std::string& str) {
    pack(buf, (int64_t)str.size());
    const auto* bytes = reinterpret_cast<const std::byte*>(str.data());
    buf.insert(buf.end(), bytes, bytes + str.size());
}

inline void unpack(const std::byte*& cursor, std::string& str) {
    int64_t size;
    unpack(cursor, size);
    str.assign(reinterpret_cast<const char*>(cursor), size);
    cursor += size;
}


/** std::optional */

template<typename T>
inline void pack(std::vector<std::byte>& buf, const std::optional<T>& opt)
{
    // pack true/false for whether it has a value
    pack(buf, opt.has_value());
    // if it has a value, pack the value
    if (opt.has_value())
        pack(buf, *opt);
}

template<typename T>
inline void unpack(const std::byte*& cursor, std::optional<T>& opt)
{
    // unpack whether or not it has a value
    bool has_value;
    unpack(cursor, has_value);
    if (has_value)
    {
        if (!opt.has_value())
            opt.emplace();
        unpack(cursor, *opt);
    }
    else
        opt = std::nullopt;
}

/** std::variant */

template<typename... Ts>
inline void pack(std::vector<std::byte>& buf, const std::variant<Ts...>& var)
{
    // pack the index of the active type
    pack(buf, var.index());
    
    // pack the active value
    std::visit([&buf](const auto& val) {
        pack(buf, val);
    }, var);
}

template<size_t I, typename... Ts>
inline bool unpack_variant_at(const std::byte*& cursor, std::variant<Ts...>& var, size_t index)
{
    if (I != index) return false;
    using T = std::variant_alternative_t<I, std::variant<Ts...>>;
    T val;
    unpack(cursor, val);
    var = std::move(val);
    return true;
}

template<typename... Ts, size_t... Is>
inline bool unpack_variant_impl(const std::byte*& cursor, std::variant<Ts...>& var, size_t index, std::index_sequence<Is...>)
{
    // fold expression over || short-circuits on first match
    return (unpack_variant_at<Is>(cursor, var, index) || ...);
}

template<typename... Ts>
inline void unpack(const std::byte*& cursor, std::variant<Ts...>& var)
{
    size_t index;
    unpack(cursor, index);

    bool found = unpack_variant_impl(cursor, var, index, std::index_sequence_for<Ts...>{});

    if (!found)
        throw std::runtime_error("variant index out of range");
}


/** std::unique_ptr 
 * 
 * NOTE: does not handle polymorphic pointer types!
*/
template<typename T>
inline void pack(std::vector<std::byte>& buf, const std::unique_ptr<T>& ptr)
{
    // pack whether the pointer is null
    pack(buf, (bool)ptr);
    if (ptr)
        ptr->serialize(buf);
}

template<typename T>
inline void unpack(const std::byte*& cursor, std::unique_ptr<T>& ptr)
{
    bool has_value;
    unpack(cursor, has_value);
    if (has_value)
    {
        if (ptr)
        {
            // already has an allocation, unpack directly into it
            ptr->deserialize(cursor);
        }
        else
        {
            // no allocation, make one then unpack into it
            std::cerr << "unpack(): Unique_ptr was not pre-allocated!" << std::endl;
            assert(0);
        }
    }
    else
    {
        ptr = nullptr;
    }
}


/** std::vector */

template<typename T>
inline void pack(std::vector<std::byte>& buf, const std::vector<T>& vec)
{
    // pack size, then elements
    pack(buf, vec.size());
    for (const auto& v : vec)
        pack(buf, v);
}

template<typename T>
inline void unpack(const std::byte*& cursor, std::vector<T>& vec)
{
    // unpack size, then elements
    size_t size;
    unpack(cursor, size);

    // std::cout << " Unpacked vector size: " << size << std::endl;

    vec.resize(size);
    for (auto& v : vec)
        unpack(cursor, v);
}

template<>
inline void unpack(const std::byte*& cursor, std::vector<bool>& vec)
{
    size_t size;
    unpack(cursor, size);
    vec.resize(size);
    for (size_t i = 0; i < size; i++)
    {
        bool val;
        unpack(cursor, val);
        vec[i] = val;
    }
}

/** std::array */

template<typename T, size_t N>
inline void pack(std::vector<std::byte>& buf, const std::array<T, N>& arr)
{
    for (const auto& v : arr)
        pack(buf, v);
}

template<typename T, size_t N>
inline void unpack(const std::byte*& cursor, std::array<T, N>& arr)
{
    for (auto& v : arr)
    {
        unpack(cursor, v);
    }
}



/** std::unordered_map */

template<typename K, typename V, typename Hash, typename KeyEqual, typename Alloc>
inline void pack(std::vector<std::byte>& buf, const std::unordered_map<K, V, Hash, KeyEqual, Alloc>& map)
{
    // pack size (number of key-value pairs)
    pack(buf, map.size());

    // pack key, then value
    for (const auto [key, value] : map)
    {
        pack(buf, key);
        pack(buf, value);
    }
}

template<typename K, typename V, typename Hash, typename KeyEqual, typename Alloc>
inline void unpack(const std::byte*& cursor, std::unordered_map<K, V, Hash, KeyEqual, Alloc>& map)
{
    // clear map
    map.clear();

    // get size of map
    size_t map_size;
    unpack(cursor, map_size);

    // load key value pairs
    for (size_t i = 0; i < map_size; i++)
    {
        K key;
        V val;
        unpack(cursor, key);
        unpack(cursor, val);
        map.insert({key, val});
    }
}


/** std::unordered_multimap */

template<typename K, typename V, typename Hash, typename KeyEqual, typename Alloc>
inline void pack(std::vector<std::byte>& buf, const std::unordered_multimap<K, V, Hash, KeyEqual, Alloc>& map)
{
    // pack size (number of key-value pairs)
    pack(buf, map.size());

    // pack key, then value
    for (const auto [key, value] : map)
    {
        pack(buf, key);
        pack(buf, value);
    }
}

template<typename K, typename V, typename Hash, typename KeyEqual, typename Alloc>
inline void unpack(const std::byte*& cursor, std::unordered_multimap<K, V, Hash, KeyEqual, Alloc>& map)
{
    // clear map
    map.clear();

    // get size of map
    size_t map_size;
    unpack(cursor, map_size);

    // load key value pairs
    for (size_t i = 0; i < map_size; i++)
    {
        K key;
        V val;
        unpack(cursor, key);
        unpack(cursor, val);
        map.insert({key, val});
    }
}


/** std::unordered_set */

template<typename T, typename Hash, typename KeyEqual, typename Alloc>
inline void pack(std::vector<std::byte>& buf, const std::unordered_set<T, Hash, KeyEqual, Alloc>& set)
{
    // pack size
    pack(buf, set.size());

    // pack elements
    for (const auto& v : set)
        pack(buf, v);
}

template<typename T, typename Hash, typename KeyEqual, typename Alloc>
inline void unpack(const std::byte*& cursor, std::unordered_set<T, Hash, KeyEqual, Alloc>& set)
{
    // clear set
    set.clear();

    // get size of set
    size_t set_size;
    unpack(cursor, set_size);

    // load elements
    for (size_t i = 0; i < set_size; i++)
    {
        T elem;
        unpack(cursor, elem);
        set.insert(elem);
    }
}


/** std::queue */
template<typename T>
inline void pack(std::vector<std::byte>& buf, const std::queue<T>& queue)
{
    // make copy of queue
    auto copy = queue;

    // pack size of queue
    pack(buf, copy.size());

    // drain copy of queue
    while (!copy.empty())
    {
        pack(buf, copy.front());
        copy.pop();
    }
}

template<typename T>
inline void unpack(const std::byte*& cursor, std::queue<T>& queue)
{
    // unpack size of queue
    size_t queue_size;
    unpack(cursor, queue_size);

    // drain the queue
    while (!queue.empty())
    {
        queue.pop();
    }

    // push elements onto queue
    for (size_t i = 0; i < queue_size; i++)
    {
        T val;
        unpack(cursor, val);
        queue.push(std::move(val));
    }
}



/** dynamic Eigen types */

template<typename Derived>
inline std::enable_if_t<std::is_base_of_v<Eigen::MatrixBase<Derived>, Derived>>
pack(std::vector<std::byte>& buf, const Derived& mat)
{
    if constexpr (Derived::RowsAtCompileTime == Eigen::Dynamic ||
                  Derived::ColsAtCompileTime == Eigen::Dynamic)
    {
        pack(buf, (size_t)mat.rows());
        pack(buf, (size_t)mat.cols());
    }
    const auto* bytes = reinterpret_cast<const std::byte*>(mat.derived().data());
    buf.insert(buf.end(), bytes, bytes + mat.size() * sizeof(typename Derived::Scalar));
}

template<typename Derived>
inline std::enable_if_t<std::is_base_of_v<Eigen::MatrixBase<Derived>, Derived>>
unpack(const std::byte*& cursor, Derived& mat)
{
    if constexpr (Derived::RowsAtCompileTime == Eigen::Dynamic ||
                  Derived::ColsAtCompileTime == Eigen::Dynamic)
    {
        size_t rows, cols;
        unpack(cursor, rows);
        unpack(cursor, cols);
        mat.derived().resize(rows, cols);
    }
    std::memcpy(mat.derived().data(), cursor, mat.size() * sizeof(typename Derived::Scalar));
    cursor += mat.size() * sizeof(typename Derived::Scalar);
}

#endif // __PACK_HPP