function varargout = simpleIRK(varargin)
    %SIMPLEIRK [INTERNAL] 
    %
    %  Function = SIMPLEIRK(Function f, int N, int order, char scheme, char solver, struct solver_options)
    %
    %Construct an implicit Runge-Kutta integrator using a collocation
    % 
    %scheme.
    %
    %The constructed function has three inputs, corresponding to initial 
    %state 
    %(x0), parameter (p) and integration time (h) and one output, 
    %corresponding 
    %to final state (xf).
    %
    %Parameters:
    %-----------
    %
    %f: 
    %ODE function with two inputs (x and p) and one output (xdot)
    %
    %N: 
    %Number of integrator steps
    %
    %order: 
    %Order of interpolating polynomials
    %
    %scheme: 
    % Collocation scheme, as excepted by collocationPoints function.
    %
    %solver: 
    %Solver plugin
    %
    %solver_options: 
    %Options to be passed to the solver plugin
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1ss
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integration_tools.hpp#L289
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integration_tools.cpp#L289-L354
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(976, varargin{:});
end
