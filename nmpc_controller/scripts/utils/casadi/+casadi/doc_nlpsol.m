function varargout = doc_nlpsol(varargin)
    %DOC_NLPSOL [INTERNAL] 
    %
    %  char = DOC_NLPSOL(char name)
    %
    %Get the documentation string for a plugin.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/nlpsol.hpp#L42
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/nlpsol.cpp#L42-L44
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(840, varargin{:});
end
