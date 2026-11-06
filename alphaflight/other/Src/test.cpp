#include <stdint.h>

// A small C++17 template test
template<typename T>
constexpr T cpp_test_add(T value)
{
    if constexpr (sizeof(T) == 4)
    {
        return value + 1;
    }
    else
    {
        return value;
    }
}

// Expose a C-compatible function
extern "C" uint32_t alphaflight_cpp_test(void)
{
    return cpp_test_add<uint32_t>(41);
}

class Scheduler{
    public:
    Scheduler(){

    }

};