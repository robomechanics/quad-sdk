function varargout = doc_interpolant(varargin)
    %DOC_INTERPOLANT [INTERNAL] 
    %
    %  char = DOC_INTERPOLANT(char name)
    %
    %Get the documentation string for a plugin.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/interpolant.hpp#L42
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/interpolant.cpp#L42-L44
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(886, varargin{:});
end
