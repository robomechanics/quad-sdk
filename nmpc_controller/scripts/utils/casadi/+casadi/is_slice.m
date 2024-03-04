function varargout = is_slice(varargin)
    %IS_SLICE [INTERNAL] 
    %
    %  bool = IS_SLICE([int] v, bool ind1)
    %
    %Check if an index vector can be represented more efficiently as 
    %a 
    %slice.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.hpp#L170
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.cpp#L170-L200
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(213, varargin{:});
end
