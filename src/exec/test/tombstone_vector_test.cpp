#include "common/types.hpp"
#include "common/TombstoneVector.hpp"

#include <iostream>

int main()
{
    TombstoneVector<int> vec3_vec;
    vec3_vec.emplace_back(1);
    vec3_vec.emplace_back(2);
    vec3_vec.emplace_back(3);

    std::cout << "Size: " << vec3_vec.size() << "  Total size: " << vec3_vec.totalSize() << std::endl;
    std::cout << "Vec[0]: " << vec3_vec[0] << std::endl;

    std::cout << "\nRemoving element at index 0..." << std::endl;
    vec3_vec.erase(0);
    std::cout << "Size: " << vec3_vec.size() << "  Total size: " << vec3_vec.totalSize() << std::endl;

    std::cout << "\nAdding a new element..." << std::endl;
    vec3_vec.emplace_back(4);
    std::cout << "Size: " << vec3_vec.size() << "  Total size: " << vec3_vec.totalSize() << std::endl;
    std::cout << "Vec[0]: " << vec3_vec[0] << std::endl;
}