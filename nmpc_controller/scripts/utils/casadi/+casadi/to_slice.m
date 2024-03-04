function varargout = to_slice(varargin)
    %TO_SLICE [INTERNAL] 
    %
    %  Slice = TO_SLICE([int] v, bool ind1)
    %
    %Construct from an index vector (requires is_slice(v) to be true)
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.hpp#L152
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.cpp#L152-L168
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(211, varargin{:});
end
