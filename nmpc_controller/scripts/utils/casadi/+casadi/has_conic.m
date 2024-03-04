function varargout = has_conic(varargin)
    %HAS_CONIC [INTERNAL] 
    %
    %  bool = HAS_CONIC(char name)
    %
    %Check if a particular plugin is available.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/conic.hpp#L31
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/conic.cpp#L31-L33
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(825, varargin{:});
end
