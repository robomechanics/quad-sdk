function varargout = forward(varargin)
    %FORWARD Forward directional derivative.
    %
    %  {{DM}} = FORWARD({DM} ex, {DM} arg, {{DM}} v, struct opts)
    %  {{SX}} = FORWARD({SX} ex, {SX} arg, {{SX}} v, struct opts)
    %  {{MX}} = FORWARD({MX} ex, {MX} arg, {{MX}} v, struct opts)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1cx
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_matrix.hpp#L841
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_matrix.hpp#L841-L845
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(893, varargin{:});
end
