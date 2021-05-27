#ifndef BP_RANDOM_HPP
#define BP_RANDOM_HPP

#include <chrono>
#include <stdlib.h>

class Random
{
private:
    unsigned int last_seed;
public:
    // use time based random seed
    Random() {
        auto t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        last_seed = (unsigned) t;
        std::srand(last_seed);
    }

    // use a specific seed
    Random(int seed) {
        last_seed = seed;
        std::srand(seed);
    }
    
    ~Random() {};

    // [0,upperBound)
    int nextInt(int upperBound) {
        if (upperBound == 0) return 0;
        else return std::rand() % upperBound;
    }

    // [lowerBound,upperBound)
    int nextInt(int lowerBound, int upperBound) { 
        if (upperBound == 0 || lowerBound >= upperBound) return 0;
        else return std::rand() % upperBound + lowerBound;
    }


    // between 0.0f and 1.0f
    float nextFloat() {
        return (float)(std::rand() % 100 + 0) / 100.f;
    }

    void reseed(){
        std::srand(last_seed);
    }

    void reseed(int new_seed){
        last_seed = new_seed;
        std::srand(last_seed);
    }

};

#endif