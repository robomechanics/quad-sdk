function varargout = bspline(varargin)
    %BSPLINE 
    %
    %  MX = BSPLINE(MX x, DM coeffs, {[double]} knots, [int] degree, int m, struct opts)
    %  MX = BSPLINE(MX x, MX coeffs, {[double]} knots, [int] degree, int m, struct opts)
    %
    %
  [varargout{1:nargout}] = casadiMEX(902, varargin{:});
end
