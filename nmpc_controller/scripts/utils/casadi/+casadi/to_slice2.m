function varargout = to_slice2(varargin)
    %TO_SLICE2 [INTERNAL] 
    %
    %  std::pair< casadi::Slice,casadi::Slice > = TO_SLICE2([int] v)
    %
    %Construct nested slices from an index vector (requires 
    %is_slice2(v) to
    % be true)
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.hpp#L255
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.cpp#L255-L289
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(212, varargin{:});
end
