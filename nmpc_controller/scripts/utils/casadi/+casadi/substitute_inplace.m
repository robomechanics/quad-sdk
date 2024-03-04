function varargout = substitute_inplace(varargin)
    %SUBSTITUTE_INPLACE Inplace substitution with piggyback expressions.
    %
    %  [{DM} INOUT1, {DM} INOUT2] = SUBSTITUTE_INPLACE({DM} v, bool reverse)
    %  [{SX} INOUT1, {SX} INOUT2] = SUBSTITUTE_INPLACE({SX} v, bool reverse)
    %  [{MX} INOUT1, {MX} INOUT2] = SUBSTITUTE_INPLACE({MX} v, bool reverse)
    %
    %
    %Substitute variables v out of the expressions vdef sequentially, as 
    %well as
    % out of a number of other expressions piggyback
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1cn
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_matrix.hpp#L685
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_matrix.hpp#L685-L689
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(896, varargin{:});
end
