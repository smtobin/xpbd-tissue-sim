#ifndef __PACK_HPP
#define __PACK_HPP

#include <optional>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <memory>
#include <cstddef>
#include <string>

#include <Eigen/Dense>

// detection helper
template<typename T, typename = void>
struct has_save : std::false_type {};

template<typename T>
struct has_save<T, std::void_t<decltype(std::declval<T>().save(std::declval<std::vector<std::byte>&>()))>> : std::true_type {};

template<typename T>
void pack(std::vector<std::byte>& buf, const T& val);
template<typename T>
void unpack(const std::byte*& cursor, T& var);

/** std::string */

void pack(std::vector<std::byte>& buf, const std::string& str) {
    pack(buf, (int64_t)str.size());
    const auto* bytes = reinterpret_cast<const std::byte*>(str.data());
    buf.insert(buf.end(), bytes, bytes + str.size());
}

void unpack(const std::byte*& cursor, std::string& str) {
    int64_t size;
    unpack(cursor, size);
    str.assign(reinterpret_cast<const char*>(cursor), size);
    cursor += size;
}


/** std::optional */

template<typename T>
void pack(std::vector<std::byte>& buf, const std::optional<T>& opt)
{
    // pack true/false for whether it has a value
    pack(buf, opt.has_value());
    // if it has a value, pack the value
    if (opt.has_value())
        pack(buf, *opt);
}

template<typename T>
void unpack(const std::byte*& cursor, std::optional<T>& opt)
{
    // unpack whether or not it has a value
    bool has_value;
    unpack(cursor, has_value);
    if (has_value)
        unpack(cursor, *opt);
    else
        opt = std::nullopt;
}


/** std::vector */

template<typename T>
void pack(std::vector<std::byte>& buf, const std::vector<T>& vec)
{
    // pack size, then elements
    pack(buf, vec.size());
    for (auto& v : vec)
        pack(buf, v);
}

template<typename T>
void unpack(const std::byte*& cursor, std::vector<T>& vec)
{
    // unpack size, then elements
    size_t size;
    unpack(cursor, size);
    vec.resize(size);
    for (auto& v : vec)
        unpack(cursor, v);
}



/** std::unordered_map */

template<typename K, typename V>
void pack(std::vector<std::byte>& buf, const std::unordered_map<K, V>& map)
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

template<typename K, typename V>
void unpack(const std::byte*& cursor, std::unordered_map<K, V>& map)
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

template<typename K, typename V>
void pack(std::vector<std::byte>& buf, const std::unordered_multimap<K, V>& map)
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

template<typename K, typename V>
void pack(const std::byte*& cursor, std::unordered_multimap<K, V>& map)
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

template<typename T>
void pack(std::vector<std::byte>& buf, const std::unordered_set<T>& set)
{
    // pack size
    pack(buf, set.size());

    // pack elements
    for (const auto& v : set)
        pack(buf, v);
}

template<typename T>
void unpack(const std::byte*& cursor, std::unordered_set<T>& set)
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
void pack(std::vector<std::byte>& buf, const std::queue<T>& queue)
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
void unpack(const std::byte*& cursor, std::queue<T>& queue)
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
        queue.push(val);
    }
}



/** dynamic Eigen types */

template<typename Derived>
void pack(std::vector<std::byte>& buf, const Eigen::DenseBase<Derived>& mat)
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
void unpack(const std::byte*& cursor, Eigen::DenseBase<Derived>& mat)
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


/** Default
 * Trivially copyable types
 * Composite types with save() and load() implemented
 */

template<typename T>
void pack(std::vector<std::byte>& buf, const T& val)
{
    if constexpr (has_save<T>::value)
    {
        val.save(buf);
    }
    else
    {
        static_assert(std::is_trivially_copyable_v<T>);
        const auto* bytes = reinterpret_cast<const std::byte*>(&val);
        buf.insert(buf.end(), bytes, bytes + sizeof(T));
    }
}

template<typename T>
void unpack(const std::byte*& cursor, T& var)
{
    if constexpr (has_save<T>::value)
    {
        var.load(cursor);
    }
    else
    {
        static_assert(std::is_trivially_copyable_v<T>);
        std::memcpy(&var, cursor, sizeof(T));
        cursor += sizeof(T);
    }
}

#endif // __PACK_HPP