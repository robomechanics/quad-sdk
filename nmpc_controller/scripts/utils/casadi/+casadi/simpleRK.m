function varargout = simpleRK(varargin)
    %SIMPLERK [INTERNAL] 
    %
    %  Function = SIMPLERK(Function f, int N, int order)
    %
    %Construct an explicit Runge-Kutta integrator.
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
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1sr
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integration_tools.hpp#L128
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integration_tools.cpp#L128-L187
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(975, varargin{:});
end
