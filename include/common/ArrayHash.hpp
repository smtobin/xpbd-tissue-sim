#ifndef __ARRAY_HASH_HPP
#define __ARRAY_HASH_HPP

template<typename T, std::size_t N>
struct ArrayHash {
    std::size_t operator()(const std::array<T, N>& arr) const {
        std::size_t seed = 0;
        std::hash<T> hasher;
        
        for (const auto& elem : arr) {
            seed ^= hasher(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

#endif // __ARRAY_HASH_HPP