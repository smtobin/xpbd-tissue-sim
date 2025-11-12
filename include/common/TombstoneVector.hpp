#ifndef __TOMBSTONE_VECTOR_HPP
#define __TOMBSTONE_VECTOR_HPP

#include "common/CompactOptional.hpp"

#include <vector>
#include <queue>
#include <exception>

/** A specialized vector class which performs O(1) removal by keeping track of "tombstones", i.e. indices of elements in the vector
 * that are removed and no longer valid.
 * 
 * Note: to achieve this, a vector of std::optional<T> is used. Due to alignment, for "small" types (i.e. int, float, etc.) the memory 
 * requirement is doubled, which is bad for cache locality. An option is to use a "CompactOptional" which marks a special value
 * as "empty". This removes the need for an additional boolean.
 */
template <class T>
class TombstoneVector
{
public:
    // using optional_type = CompactOptional<T, InvalidValue_>;
    using optional_type = std::optional<T>;

    TombstoneVector()
        : _empty_indices(), _data(), _num_valid(0)
    {}

    /** Initializes the underlying vector with the specified size.
     * However, all of the elements are initialized to their invalid values, so the reported size is still 0.
     */
    TombstoneVector(size_t size)
        : _empty_indices(), _data(size), _num_valid(0)
    {}

    /** Initializes the underlying vector with the specified size, and all elements have the default value.
     * Unless the initialized value is the invalid value, the reported size is the specified size.
     */
    TombstoneVector(size_t size, const T& initial_value)
        : _empty_indices(), _data(size, initial_value), _num_valid(size)
    {}

    /** Initializes the underlying vector from another vector. */
    TombstoneVector(const std::vector<T>& vec)
        : _empty_indices(), _data(vec.size()), _num_valid(vec.size())
    {
        // copy over data
        for (size_t i = 0; i < vec.size(); i++)
        {
            _data[i] = vec[i];
        }
    }

    /** Accessing elements */
    T& at(size_t index)
    {
        if (index >= _data.size() || !_data[index].has_value())
        {
            throw std::out_of_range("Invalid index or tombstone");
        }

        return *_data[index];
    }

    const T& at(size_t index) const
    {
        if (index >= _data.size() || !_data[index].has_value())
        {
            throw std::out_of_range("Invalid index or tombstone");
        }

        return *_data[index];
    }

    T& operator[](size_t index)
    {
        return *_data[index];
    }

    const T& operator[](size_t index) const
    {
        return *_data[index];
    }

    bool indexValid(size_t index) const
    {
        return (index < _data.size() && _data[index].has_value());
    }

    /** Adding elements */
    size_t push_back(const T& value)
    {
        if (!_empty_indices.empty())
        {
            size_t index = _empty_indices.front();
            _empty_indices.pop();
            _data[index] = value;
            _num_valid++;
            return index;
        }
        else
        {
            _data.push_back(value);
            _num_valid++;
            return _data.size()-1;
        }
    }

    size_t push_back(T&& value)
    {
        if (!_empty_indices.empty())
        {
            size_t index = _empty_indices.front();
            _empty_indices.pop();
            _data[index] = std::move(value);
            _num_valid++;
            return index;
        }
        else
        {
            _data.push_back(std::move(value));
            _num_valid++;
            return _data.size()-1;
        }
    }

    template<typename... Args>
    size_t emplace_back(Args&&... args)
    {
        if (!_empty_indices.empty())
        {
            size_t index = _empty_indices.front();
            _empty_indices.pop();
            _data[index].emplace(std::forward<Args>(args)...);
            _num_valid++;
            return index;
        }
        else
        {
            _data.emplace_back(std::in_place, std::forward<Args>(args)...);
            _num_valid++;
            return _data.size()-1;
        }
    }

    /** Removing elements */
    void erase(size_t index)
    {
        if (index < _data.size() && _data[index].has_value())
        {
            _data[index].reset();
            _empty_indices.push(index);
            _num_valid--;
        }
    }

    /** Size operations */
    size_t size() const { return _num_valid; }
    size_t totalSize() const { return _data.size(); }
    size_t capacity() const { return _data.capacity(); }
    bool empty() const { return (_num_valid == 0); }

    void reserve(size_t n)
    {
        _data.reserve(n);
    }
    
    void clear()
    {
        _data.clear();
        _empty_indices = std::queue<size_t>();
        _num_valid = 0;
    }

    /** Fragmentation operations */

    /** Returns the amount of fragmentation, as a fraction.
     * A returned value of 0 indicates no fragmentation (no invalid elements in the vector), while
     * a returned value of 0.3 indicates that 30% of the elements in the vector are invalid.
     */
    double fragmentation() const
    {
        return _data.empty() ? 0.0 : 1.0 - (double)size() / totalSize();
    }

    /** Copies all valid data to a new vector without fragmentation. */
    void compact()
    {
        if (_empty_indices.empty())
            return;

        std::vector<optional_type> new_data;
        new_data.reserve(_num_valid);

        // copy over nonempty data
        for (auto& item : _data)
        {
            if (item.has_value())
            {
                new_data.emplace_back(std::move(item));
            }
        }

        _data = std::move(new_data);
        _empty_indices = std::queue<size_t>();  // clear empty indices
    }

    class iterator
    {
    public:
        iterator(typename std::vector<optional_type>::iterator it,
                 typename std::vector<optional_type>::iterator end)
            : _it(it), _end(end)
        {
            _skipInvalid();
        }

        T& operator*() { return **_it; }
        T* operator->() { return &(**_it); }

        iterator& operator++()
        {
            ++_it;
            _skipInvalid();
            return *this;
        }

        iterator& operator++(int)
        {
            iterator tmp = *this;
            ++(*this);
            return tmp;
        }

        bool operator==(const iterator& other) const
        {
            return _it == other._it;
        }

        bool operator!=(const iterator& other) const
        {
            return _it != other._it;
        }

    private:
        void _skipInvalid()
        {
            while(_it != _end && !_it->has_value())
                ++_it;
        }

        typename std::vector<optional_type>::iterator _it;
        typename std::vector<optional_type>::iterator _end;
    };

    class const_iterator
    {
    public:

        const_iterator(typename std::vector<optional_type>::const_iterator it,
                 typename std::vector<optional_type>::const_iterator end)
            : _it(it), _end(end)
        {
            _skipInvalid();
        }

        const T& operator*() { return **_it; }
        const T* operator->() { return &(**_it); }

        const_iterator& operator++()
        {
            ++_it;
            _skipInvalid();
            return *this;
        }

        const_iterator& operator++(int)
        {
            iterator tmp = *this;
            ++(*this);
            return tmp;
        }

        bool operator==(const const_iterator& other) const
        {
            return _it == other._it;
        }

        bool operator!=(const const_iterator& other) const
        {
            return _it != other._it;
        }

    private:
        void _skipInvalid()
        {
            while(_it != _end && !_it->has_value())
                ++_it;
        }

        typename std::vector<optional_type>::const_iterator _it;
        typename std::vector<optional_type>::const_iterator _end;
    };

    iterator begin()
    {
        return iterator(_data.begin(), _data.end());
    }

    iterator end()
    {
        return iterator(_data.end(), _data.end());
    }

    const_iterator begin() const
    {
        return const_iterator(_data.begin(), _data.end());
    }

    const_iterator end() const
    {
        return const_iterator(_data.end(), _data.end());
    }

    const_iterator cbegin() const
    {
        return const_iterator(_data.cbegin(), _data.cend());
    }

    const_iterator cend() const
    {
        return const_iterator(_data.cend(), _data.cend());
    }

    class ValidIndicesRange
    {
    public:
        ValidIndicesRange(const std::vector<optional_type>* data) : _data(data) {}

        class iterator
        {
        public:

            iterator(const std::vector<std::optional<T>>* data, size_t index)
                : _data(data), _index(index)
            {
                _skipInvalid();
            }

            size_t operator*() const { return _index; }

            iterator& operator++()
            {
                ++_index;
                _skipInvalid();
                return *this;
            }

            iterator operator++(int)
            {
                iterator temp = *this;
                ++(*this);
                return temp;
            }

            bool operator==(const iterator& other) const
            {
                return _index == other._index;
            }

            bool operator!=(const iterator& other) const
            {
                return _index != other._index;
            }

        private:
            void _skipInvalid()
            {
                while (_index < _data->size() && !(*_data)[_index].has_value())
                    ++_index;
            }

            const std::vector<std::optional<T>>* _data;
            size_t _index;
        };

        iterator begin() const { return iterator(_data, 0); }
        iterator end() const { return iterator(_data, _data->size()); }

    private:
        const std::vector<optional_type>* _data;
    };

    ValidIndicesRange validIndices() const
    {
        return ValidIndicesRange(&_data);
    }


private:
    /** A queue to keep track of empty indices that we can fill in */
    std::queue<size_t> _empty_indices;

    /** Storage of data */
    std::vector<optional_type> _data;

    /** Number of valid elements in the vector */
    size_t _num_valid;
};


#endif // __TOMBSTONE_VECTOR_HPP