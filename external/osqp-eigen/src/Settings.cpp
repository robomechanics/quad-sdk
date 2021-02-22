/**
 * @file Settings.cpp
 * @author Giulio Romualdi
 * @copyright  Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include <OsqpEigen/Settings.hpp>
#include <iostream>

template <typename... Args> inline void unused(Args&&...) {}

OsqpEigen::Settings::Settings()
{
    m_settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
    osqp_set_default_settings(m_settings);
}

OsqpEigen::Settings::~Settings()
{
    c_free(m_settings);
}

void OsqpEigen::Settings::resetDefaultSettings()
{
    osqp_set_default_settings(m_settings);
}

void OsqpEigen::Settings::setRho(const double rho)
{
    m_settings->rho = rho;
}

void OsqpEigen::Settings::setSigma(const double sigma)
{
    m_settings->sigma = sigma;
}

void OsqpEigen::Settings::setScaling(const int scaling)
{
    m_settings->scaling = (c_int)scaling;
}

void OsqpEigen::Settings::setAdaptiveRho(const bool isRhoStepSizeAdactive)
{
# if EMBEDDED != 1
    m_settings->adaptive_rho = (c_int)isRhoStepSizeAdactive;
# else
    std::cerr<< "[OsqpEigen::Settings::setAdaptiveRho] OSPQ has been set to EMBEDDED, hence this setting is disabled." << std::endl;
    unused(isRhoStepSizeAdactive);
# endif
}

void OsqpEigen::Settings::setAdaptiveRhoInterval(const int rhoInterval)
{
# if EMBEDDED != 1
    m_settings->adaptive_rho_interval = (c_int)rhoInterval;
# else
    std::cerr<< "[OsqpEigen::Settings::setAdaptiveRhoInterval] OSPQ has been set to EMBEDDED, hence this setting is disabled." << std::endl;
    unused(rhoInterval);
# endif
}

void OsqpEigen::Settings::setAdaptiveRhoTolerance(const double adaptiveRhoTolerance)
{
# if EMBEDDED != 1
    m_settings->adaptive_rho_tolerance = (c_float)adaptiveRhoTolerance;
# else
    std::cerr<< "[OsqpEigen::Settings::setAdaptiveRhoTolerance] OSPQ has been set to EMBEDDED, hence this setting is disabled." << std::endl;
    unused(adaptiveRhoTolerance);
# endif
}

void OsqpEigen::Settings::setAdaptiveRhoFraction(const double adaptiveRhoFraction)
{
# if EMBEDDED != 1
# ifdef PROFILING
    m_settings->adaptive_rho_fraction = (c_float)adaptiveRhoFraction;
# else
    std::cerr<< "[OsqpEigen::Settings::setAdaptiveRhoFraction] OSPQ has been set without PROFILING, hence this setting is disabled." << std::endl;
    unused(adaptiveRhoFraction);
# endif //ifdef PROFILING
# else //# if EMBEDDED != 1
    std::cerr<< "[OsqpEigen::Settings::setAdaptiveRhoFraction] OSPQ has been set to EMBEDDED, hence this setting is disabled." << std::endl;
    unused(adaptiveRhoFraction);
# endif //# if EMBEDDED != 1
}

void OsqpEigen::Settings::setMaxIteraction(const int maxIteration)
{
    setMaxIteration(maxIteration);
}

void OsqpEigen::Settings::setMaxIteration(const int maxIteration)
{
    m_settings->max_iter = (c_int)maxIteration;
}

void OsqpEigen::Settings::setAbsoluteTolerance(const double absoluteTolerance)
{
    m_settings->eps_abs = (c_float)absoluteTolerance;
}

void OsqpEigen::Settings::setRelativeTolerance(const double relativeTolerance)
{
    m_settings->eps_rel = (c_float)relativeTolerance;
}

void OsqpEigen::Settings::setPrimalInfeasibilityTollerance(const double primalInfeasibilityTollerance)
{
    m_settings->eps_prim_inf = (c_float)primalInfeasibilityTollerance;
}

void OsqpEigen::Settings::setDualInfeasibilityTollerance(const double dualInfeasibilityTollerance)
{
    m_settings->eps_dual_inf = (c_float)dualInfeasibilityTollerance;
}

void OsqpEigen::Settings::setAlpha(const double alpha)
{
    m_settings->alpha = (c_float)alpha;
}

void OsqpEigen::Settings::setLinearSystemSolver(const int linsysSolver)
{
    m_settings->linsys_solver = (linsys_solver_type)linsysSolver;
}

void OsqpEigen::Settings::setDelta(const double delta)
{
# ifndef EMBEDDED
    m_settings->delta = (c_float)delta;
# else
    std::cerr<< "[OsqpEigen::Settings::setDelta] OSPQ has been set to EMBEDDED, hence this setting is disabled." << std::endl;
    unused(delta);
# endif
}

void OsqpEigen::Settings::setPolish(const bool polish)
{
# ifndef EMBEDDED
    m_settings->polish = (c_int)polish;
# else
    std::cerr<< "[OsqpEigen::Settings::setPolish] OSPQ has been set to EMBEDDED, hence this setting is disabled." << std::endl;
    unused(polish);
# endif
}

void OsqpEigen::Settings::setPolishRefineIter(const int polishRefineIter)
{
# ifndef EMBEDDED
    m_settings->polish_refine_iter = (c_int)polishRefineIter;
# else
    std::cerr<< "[OsqpEigen::Settings::setPolishRefineIter] OSPQ has been set to EMBEDDED, hence this setting is disabled." << std::endl;
    unused(polishRefineIter);
# endif
}

void OsqpEigen::Settings::setVerbosity(const bool isVerbose)
{
#ifndef EMBEDDED
    m_settings->verbose = (c_int)isVerbose;
#else
    std::cerr<< "[OsqpEigen::Settings::setVerbosity] OSPQ has been set to EMBEDDED, hence this setting is disabled." << std::endl;
    unused(isVerbose);
#endif
}

void OsqpEigen::Settings::setScaledTerimination(const bool scaledTermination)
{
    m_settings->scaled_termination = (c_int)scaledTermination;
}

void OsqpEigen::Settings::setCheckTermination(const int checkTermination)
{
    m_settings->check_termination = (c_int)checkTermination;
}

void OsqpEigen::Settings::setWarmStart(const bool warmStart)
{
    m_settings->warm_start = (c_int)warmStart;
}

void OsqpEigen::Settings::setTimeLimit(const double timeLimit)
{
# ifdef PROFILING
    m_settings->time_limit = (c_float)timeLimit;
# else
    std::cerr<< "[OsqpEigen::Settings::setTimeLimit] OSPQ has been set without PROFILING, hence this setting is disabled." << std::endl;
    unused(timeLimit);
# endif
}

OSQPSettings* const & OsqpEigen::Settings::getSettings() const
{
    return m_settings;
}
