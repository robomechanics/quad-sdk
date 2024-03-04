function varargout = hash_combine(varargin)
    %HASH_COMBINE 
    %
    %  HASH_COMBINE(std::size_t & seed, [int] v)
    %  HASH_COMBINE(std::size_t & seed, casadi_int const * v, std::size_t sz)
    %
    %
  [varargout{1:nargout}] = casadiMEX(188, varargin{:});
end
