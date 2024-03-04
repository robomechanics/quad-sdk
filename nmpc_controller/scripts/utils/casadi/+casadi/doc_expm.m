function varargout = doc_expm(varargin)
    %DOC_EXPM [INTERNAL] 
    %
    %  char = DOC_EXPM(char name)
    %
    %Get the documentation string for a plugin.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/expm.hpp#L40
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/expm.cpp#L40-L42
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(882, varargin{:});
end
