function varargout = doc_dple(varargin)
    %DOC_DPLE [INTERNAL] 
    %
    %  char = DOC_DPLE(char name)
    %
    %Get the documentation string for a plugin.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dple.hpp#L39
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dple.cpp#L39-L41
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(876, varargin{:});
end
