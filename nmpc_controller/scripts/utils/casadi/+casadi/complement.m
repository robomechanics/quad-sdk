function varargout = complement(varargin)
    %COMPLEMENT [INTERNAL] 
    %
    %  [int] = COMPLEMENT([int] v, int size)
    %
    %Returns the list of all i in [0, size[ not found in supplied 
    %list.
    %
    %The supplied vector may contain duplicates and may be non-monotonous 
    %The 
    %supplied vector will be checked for bounds The result vector is 
    %guaranteed 
    %to be monotonously increasing
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1lf
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/casadi_misc.hpp#L136
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/casadi_misc.cpp#L136-L152
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(35, varargin{:});
end
