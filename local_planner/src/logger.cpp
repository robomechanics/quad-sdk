#include <iostream>
#include <string>
#include <local_planner/local_footstep_planner.h>
#include <pybind11/pybind11.h>


void cpp_print(std::string value)
{   
    std::cout << value << std::endl;
}

PYBIND11_MODULE(logger, m) {
    m.doc() = "pybind11 example plugin"; // optional module docstring

    m.def("cpp_print", &cpp_print, "A print function implemented in c++");
}
