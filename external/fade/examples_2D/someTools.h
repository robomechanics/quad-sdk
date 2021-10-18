#pragma once

// someTools.h: This file belongs to the example code of Fade2D. It
// defines a few useful development tools. Feel free to adapt them
// to your needs.

// Under Linux gRed() returns a red string
inline std::string gRed(const std::string& s)
{
#ifdef _WIN32
    return s;
#else
    return ("\033[0;31m"+s+"\033[m");
#endif
}

// The examples are in C++98 style to support also old compilers,
// thus a toString() helper is required.
inline std::string toString(int i)
{
    std::ostringstream oss;
    oss << i;
    return oss.str();
}



// GASSEX is an assertion-exception i.e., an improved assert()
// statement in two ways:
// 1. Its execution is independent from NDEBUG and thus it runs
// in any build type as long as GASSEX_ENABLED is defined below.
// 2. You can decide what shall happen when the CONDITION is
// false. Options are just-print, exit(1) or throw std::exception().
//



// * 1 *   GASSEX checks the condition when ASSEX_ENABLED is defined
#define GASSEX_ENABLED
// * 2 *   GBEHAVIOR specifies what to do when the CONDITION is false
enum GASSEX_BEHAVIOR{GB_JUST_PRINT,GB_EXCEPTION,GB_EXIT};
const GASSEX_BEHAVIOR GBEHAVIOR(GB_EXIT);

// Function name (you can use BOOST_CURRENT_FUNCTION) instead but
// here the dependency shall be avoided
#ifdef _WIN32
	#define FUNC   __FUNCTION__
#else
	#define FUNC   __func__
#endif

// The GASSEX command itself
#ifdef GASSEX_ENABLED
#define GASSEX(CONDITION) \
if((CONDITION)) \
{ \
} \
else \
{\
	std::cerr<<gRed("** Function: "+std::string(FUNC)+"()")<<std::endl; \
	std::cerr<<gRed("   Condition [ "+std::string(#CONDITION)+" ] failed")<<std::endl; \
	std::cerr<<"   "<<(__FILE__)<<":"<<(__LINE__); \
	switch(GBEHAVIOR) \
	{ \
		case GB_JUST_PRINT: std::cerr<<", continuing...\n"<<std::endl;break; \
		case GB_EXCEPTION: std::cerr<<", throwing...\n"<<std::endl;throw std::exception(); \
		default: std::cerr<<", exit(1)\n"<<std::endl;exit(1); \
	} \
}
#else
// Do not check the condition when not enabled
#define GASSEX(CONDITION)
#endif

