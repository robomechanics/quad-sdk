function varargout = hash_sparsity(varargin)
    %HASH_SPARSITY 
    %
    %  std::size_t = HASH_SPARSITY(int nrow, int ncol, casadi_int const * colind, casadi_int const * row)
    %  std::size_t = HASH_SPARSITY(int nrow, int ncol, [int] colind, [int] row)
    %
    %
  [varargout{1:nargout}] = casadiMEX(189, varargin{:});
end
