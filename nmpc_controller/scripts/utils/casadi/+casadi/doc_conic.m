function varargout = doc_conic(varargin)
    %DOC_CONIC [INTERNAL] 
    %
    %  char = DOC_CONIC(char name)
    %
    %Get the documentation string for a plugin.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/conic.hpp#L39
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/conic.cpp#L39-L41
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(827, varargin{:});
end
