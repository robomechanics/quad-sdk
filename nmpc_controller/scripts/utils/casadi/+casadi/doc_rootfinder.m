function varargout = doc_rootfinder(varargin)
    %DOC_ROOTFINDER [INTERNAL] 
    %
    %  char = DOC_ROOTFINDER(char name)
    %
    %Get the documentation string for a plugin.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/rootfinder.hpp#L92
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/rootfinder.cpp#L92-L94
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(851, varargin{:});
end
