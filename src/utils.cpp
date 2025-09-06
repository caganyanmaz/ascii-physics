#include "engine/utils.hpp"
#include <cstdlib>


template<typename T> 
T random_choice(const std::vector<T>& choices) {
    int n = choices.size();
    int idx = rand() % n;
    return choices[idx];

}

template char random_choice<char>(const std::vector<char>& choices);