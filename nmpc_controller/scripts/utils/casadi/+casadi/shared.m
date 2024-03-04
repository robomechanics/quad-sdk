function varargout = shared(varargin)
    %SHARED Get a shared (owning) reference.
    %
    %  [{DM} OUTPUT1, {DM} OUTPUT2, {DM} OUTPUT3] = SHARED({DM} ex, char v_prefix, char v_suffix)
    %  [{SX} OUTPUT1, {SX} OUTPUT2, {SX} OUTPUT3] = SHARED({SX} ex, char v_prefix, char v_suffix)
    %  [{MX} OUTPUT1, {MX} OUTPUT2, {MX} OUTPUT3] = SHARED({MX} ex, char v_prefix, char v_suffix)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_b0
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/shared_object.hpp#L198
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/shared_object.cpp#L141-L147
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(898, varargin{:});
end
