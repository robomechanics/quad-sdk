#include <chrono>

using namespace std::chrono;

// Usage
// Timer(START);
// double elapsed_time = Timer(STOP);

// Speed benchmarking :).
enum {
    START,
    STOP
};

auto Timer(int set) -> double {

    double msec;

    // Set a timer.
    switch (set)
    {
        case START:
            static auto t1 = high_resolution_clock::now();
            msec = 0.;
            break;
        case STOP:
            static auto t2 = high_resolution_clock::now();
            msec = duration_cast<milliseconds>(t2 - t1).count();
            break;
        default:
            break;
    }
    return msec;
}