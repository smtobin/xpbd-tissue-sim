#ifndef __EIGEN_HASH_HPP
#define __EIGEN_HASH_HPP

#include "common/types.hpp"

template<typename EigenType>
struct EigenHash {
    std::size_t operator()(const EigenType& vec) const {
        std::size_t seed = 0;
        std::hash<typename EigenType::Scalar> hasher;
        
        for (const auto& elem : vec) {
            seed ^= hasher(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

#endif // __EIGEN_HASH_HPP