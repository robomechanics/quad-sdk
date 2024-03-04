function varargout = lookupvector(varargin)
    %LOOKUPVECTOR [INTERNAL] 
    %
    %  [int] = LOOKUPVECTOR([int] v)
    %  [int] = LOOKUPVECTOR([int] v, int size)
    %
    %
    %.......
    %
    %::
    %
    %  LOOKUPVECTOR([int] v)
    %
    %
    %
    %[INTERNAL] 
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  LOOKUPVECTOR([int] v, int size)
    %
    %
    %
    %[INTERNAL] 
    %Returns a vector for quickly looking up entries of supplied 
    %list.
    %
    %lookupvector[i]!=-1 <=> v contains i v[lookupvector[i]] == i <=> v 
    %contains
    % i
    %
    %Duplicates are treated by looking up last occurrence
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1lg
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/casadi_misc.hpp#L154
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/casadi_misc.cpp#L154-L163
    %
    %
    %
    %.............
    %
    %
  [varargout{1:nargout}] = casadiMEX(36, varargin{:});
end
