function varargout = has_linsol(varargin)
    %HAS_LINSOL [INTERNAL] 
    %
    %  bool = HAS_LINSOL(char name)
    %
    %Check if a particular plugin is available.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/linsol.hpp#L205
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/linsol.cpp#L205-L207
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(866, varargin{:});
end
