function varargout = collocation_points(varargin)
    %COLLOCATION_POINTS [INTERNAL] 
    %
    %  [double] = COLLOCATION_POINTS(int order, char scheme)
    %
    %Obtain collocation points of specific order and scheme.
    %
    %Parameters:
    %-----------
    %
    %order: 
    %Which order (1 to 9 supported)
    %
    %scheme: 
    %'radau' or 'legendre'
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1so
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integration_tools.hpp#L120
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integration_tools.cpp#L120-L122
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(972, varargin{:});
end
