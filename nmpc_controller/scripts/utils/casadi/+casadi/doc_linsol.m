function varargout = doc_linsol(varargin)
    %DOC_LINSOL [INTERNAL] 
    %
    %  char = DOC_LINSOL(char name)
    %
    %Get the documentation string for a plugin.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/linsol.hpp#L213
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/linsol.cpp#L213-L215
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(868, varargin{:});
end
