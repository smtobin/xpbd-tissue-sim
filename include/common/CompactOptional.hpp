#ifndef __COMPACT_OPTIONAL_HPP
#define __COMPACT_OPTIONAL_HPP

#include <type_traits>
#include <optional>
#include <limits>
#include <stdexcept>

template <typename T, T Empty_>
class CompactOptional_Impl
{
    using compact_optional = CompactOptional_Impl<T, Empty_>;
    
public:
    constexpr static T empty_value = Empty_;

    /** Constructors */

    CompactOptional_Impl() {}

    explicit CompactOptional_Impl(const compact_optional& other)
    {
        _value = other._value;
    }

    explicit CompactOptional_Impl(compact_optional&& other)
    {
        _value = std::move(other._value);
    }

    template<class... Args>
    explicit CompactOptional_Impl(std::in_place_t, Args&&... args)
        : _value(std::forward<Args>(args)...)
    {}


    /** Define assignment operators */

    compact_optional& operator=(std::nullopt_t)
    {
        _value = Empty_;
        return *this;
    }

    compact_optional& operator=(const compact_optional& other)
    {
        _value = other._value;
        return *this;
    }

    compact_optional& operator=(compact_optional&& other)
    {
        _value = std::move(other._value);
        return *this;
    }

    compact_optional& operator=(const T& other)
    {
        _value = other;
        return *this;
    }

    compact_optional& operator=(T&& other)
    {
        _value = std::move(other);
        return *this;
    }


    /** Observers */
    const T* operator->() const
    {
        return &_value;
    }

    T* operator->()
    {
        return &_value;
    }

    const T& operator*() const&
    {
        return _value;
    }

    T& operator*() &
    {
        return _value;
    }

    const T&& operator*() const&&
    {
        return _value;
    }

    T&& operator*() &&
    {
        return _value;
    }

    operator bool() const
    {
        return has_value();
    }

    bool has_value() const
    {
        return (_value != Empty_);
    }

    void reset()
    {
        _value = Empty_;
    }


private:
    T _value = Empty_;
};

// check for Eigen::PlainObject to see if T is an Eigen matrix type
template<typename T, typename = void>
struct is_eigen_matrix_type : std::false_type {};

template<typename T>
struct is_eigen_matrix_type<T, std::void_t<typename T::PlainObject>> : std::true_type {};


template<typename T>
constexpr T get_empty_value()
{
    if constexpr (std::is_arithmetic_v<T>)
    {
        return std::numeric_limits<T>::max();
    }
    else if (is_eigen_matrix_type<T>::value)
    {
        return T::Constant(std::numeric_limits<typename T::Scalar>::max());
    }
    else
    {
        return T{};
    }
}

template <typename T>
// using CompactOptional = CompactOptional_Impl<T, get_empty_value<T>()>;
using CompactOptional = CompactOptional_Impl<T, T::Constant(std::numeric_limits<typename T::Scalar>::max())>;

#endif // __COMPACT_OPTIONAL_HPP