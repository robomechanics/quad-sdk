/**
 * @file Settings.hpp
 * @author Giulio Romualdi
 * @copyright  Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef OSQPEIGEN_SETTINGS_HPP
#define OSQPEIGEN_SETTINGS_HPP

// OSQP
#include <osqp.h>

/**
 * OsqpEigen namespace.
 */
namespace OsqpEigen
{
    /**
     * settings class is a wrapper of the OSQP OSQPSettings struct.
     * All the setter methods refer to this particular kind of optimizer.
     * <a href="http://web.stanford.edu/~boyd/admm.html">Here</a>
     * you can find further information.
     */
    class Settings
    {
        OSQPSettings *m_settings; /**< OSQPSettings struct. */
    public:

        /**
         * Constructor.
         */
        Settings();

        /**
         * Deconstructor.
         */
        ~Settings();

        /**
         * Reset the default settings for the optimization problem.
         */
        void resetDefaultSettings();

        /**
         * Set the ADMM step rho.
         * @param rho a ADMM step constant.
         */
        void setRho(const double rho);

        /**
         * Set the ADMM step sigma.
         * @param sigma a ADMM step constant.
         */
        void setSigma(const double sigma);

        /**
         * Set the heuristic data scaling iterations. If 0, scaling disabled.
         * @param scaling is the heuristic data scaling iteration.
         */
        void setScaling(const int scaling);

        /**
         * Set if the rho step size adaptive feature is active.
         * @param isRhoStepSizeAdactive if True the feature is active.
         */
        void setAdaptiveRho(const bool isRhoStepSizeAdactive);

        /**
         * Set the number of iterations between rho adaptations rho. If 0, it is automatic.
         * @param rhoInterval number of iterations.
         */
        void setAdaptiveRhoInterval(const int rhoInterval);

        /**
         * Set the tolerance for adapting rho. The new rho has to be X times larger or 1/X times
         * smaller than the current one to trigger a new factorization.
         * @param adaptiveRhoTolerance is the tolerance.
         */
        void setAdaptiveRhoTolerance(const double adaptiveRhoTolerance);

        /**
         * Set the interval for adapting rho (fraction of the setup time).
         * @param adaptiveRhoFraction interval of the adapting rho.
         */
        void setAdaptiveRhoFraction(const double adaptiveRhoFraction);

        /**
         * Set the max number of iterations.
         * @param maxIteration max number of iteration
         */
        [[deprecated("Use setMaxIteration(int) instead.")]] void setMaxIteraction(const int maxIteration);

        /**
         * Set the max number of iterations.
         * @param maxIteration max number of iteration
         */
        void setMaxIteration(const int maxIteration);

        /**
         * Set the absolute convergence tolerance.
         * @param absoluteTolerance absoulte tolerance of the solver.
         */
        void setAbsoluteTolerance(const double absoluteTolerance);

        /**
         * Set the relative convergence tolerance.
         * @param relativeTolerance relative tolerance of the solver.
         */
        void setRelativeTolerance(const double relativeTolerance);

        /**
         * Set the primal infeasibility tolerance.
         * @param primalInfeasibilityTollerance tolerace of the primal variables.
         */
        void setPrimalInfeasibilityTollerance(const double primalInfeasibilityTollerance);

        /**
         * Set the dual infeasibility tolerance.
         * @param dualInfeasibilityTollerance tolerace of the dual variables.
         */
        void setDualInfeasibilityTollerance(const double dualInfeasibilityTollerance);

        /**
         * Set the relaxation parameter.
         * @param alpha is the relaxation parameter.
         */
        void setAlpha(const double alpha);

        /**
         * Set linear solver
         * @param linsysSolver is the name of the solver
         */
        void setLinearSystemSolver(const int linsysSolver);

        /**
         * Set the relaxation parameter for polish.
         * @param delta is the relaxation parameter.
         */
        void setDelta(const double delta);

        /**
         * Set if the polish feature is active.
         * @param polish if True the feature is active.
         */
        void setPolish(const bool polish);

        /**
         * Set the iterative refinement steps in polish.
         * @param polishRefineIter iterative refinement step.
         */
        void setPolishRefineIter(const int polishRefineIter);

        /**
         * Set the Verbose mode.
         * @param isVerbose if true the verbose mode is activate.
         */
        void setVerbosity(const bool isVerbose);

        /**
         * Set the scaled termination criteria.
         * @param scaledTermination if true the scaled termination criteria is used.
         */
        void setScaledTerimination(const bool scaledTermination);

        /**
         * Set check termination interval. If 0, termination checking is disabled.
         * @param checkTermination if 0 the termination checking is disabled.
         */
        void setCheckTermination(const int checkTermination);

        /**
         * Set warm start.
         * @param warmStart if true the warm start is set.
         */
        void setWarmStart(const bool warmStart);

        /**
        * Set the maximum number of seconds allowed to solve the problem.
        * @param timeLimit is the time limit in seconds. If 0, then disabled.
        */
        void setTimeLimit(const double timeLimit);

        /**
         * Get a pointer to Settings struct.
         * @return a const pointer to OSQPSettings struct.
         */
        OSQPSettings* const & getSettings() const;
    };
}

#endif
