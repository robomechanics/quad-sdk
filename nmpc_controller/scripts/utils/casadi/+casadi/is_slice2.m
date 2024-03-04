function varargout = is_slice2(varargin)
    %IS_SLICE2 [INTERNAL] 
    %
    %  bool = IS_SLICE2([int] v)
    %
    %Check if an index vector can be represented more efficiently as 
    %two 
    %nested slices.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.hpp#L202
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.cpp#L202-L253
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(214, varargin{:});
end
