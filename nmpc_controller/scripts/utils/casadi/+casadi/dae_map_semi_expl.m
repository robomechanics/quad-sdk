function varargout = dae_map_semi_expl(varargin)
    %DAE_MAP_SEMI_EXPL [INTERNAL] 
    %
    %  [struct:SX , Function OUTPUT, Function OUTPUT] = DAE_MAP_SEMI_EXPL(struct:SX dae, struct:SX dae_red)
    %  [struct:MX , Function OUTPUT, Function OUTPUT] = DAE_MAP_SEMI_EXPL(struct:MX dae, struct:MX dae_red)
    %
    %Turn a reduced DAE into a semi explicit form suitable for CasADi
    % 
    %integrator.
    %
    %Parameters:
    %-----------
    %
    %dae: 
    %Original (unreduced) DAE structure
    %
    %dae_red: 
    %Reduced DAE (see dae_reduce_index)
    %
    %state_to_orig: 
    %A mapping of integrator (semi explicit) states to states of 
    %the 
    %original DAE
    %
    %phi: 
    %A function to compute the invariants of the reduced DAE Inputs:
    %x and 
    %z: (semi explicit) integrator states; typically integrator 
    %outputs xf and 
    %zf
    %
    %p: parameters
    %
    %t: time
    %
    %Semi explicit DAE dictionary, suitable to pass to a CasADi integrator
    %
    %See: 
    % dae_reduce_index
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1su
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integration_tools.hpp#L1205
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integration_tools.cpp#L1205-L1208
    %
    %
    %
    %.......
    %
    %::
    %
    %  DAE_MAP_SEMI_EXPL(struct:SX dae, struct:SX dae_red)
    %
    %
    %
    %[INTERNAL] 
    %Turn a reduced DAE into a semi explicit form suitable for CasADi
    % 
    %integrator.
    %
    %Parameters:
    %-----------
    %
    %dae: 
    %Original (unreduced) DAE structure
    %
    %dae_red: 
    %Reduced DAE (see dae_reduce_index)
    %
    %state_to_orig: 
    %A mapping of integrator (semi explicit) states to states of 
    %the 
    %original DAE
    %
    %phi: 
    %A function to compute the invariants of the reduced DAE Inputs:
    %x and 
    %z: (semi explicit) integrator states; typically integrator 
    %outputs xf and 
    %zf
    %
    %p: parameters
    %
    %t: time
    %
    %Semi explicit DAE dictionary, suitable to pass to a CasADi integrator
    %
    %See: 
    % dae_reduce_index
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1su
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integration_tools.hpp#L1205
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integration_tools.cpp#L1205-L1208
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  DAE_MAP_SEMI_EXPL(struct:MX dae, struct:MX dae_red)
    %
    %
    %
    %[INTERNAL] 
    %Turn a reduced DAE into a semi explicit form suitable for CasADi
    % 
    %integrator.
    %
    %Parameters:
    %-----------
    %
    %dae: 
    %Original (unreduced) DAE structure
    %
    %dae_red: 
    %Reduced DAE (see dae_reduce_index)
    %
    %state_to_orig: 
    %A mapping of integrator (semi explicit) states to states of 
    %the 
    %original DAE
    %
    %phi: 
    %A function to compute the invariants of the reduced DAE Inputs:
    %x and 
    %z: (semi explicit) integrator states; typically integrator 
    %outputs xf and 
    %zf
    %
    %p: parameters
    %
    %t: time
    %
    %Semi explicit DAE dictionary, suitable to pass to a CasADi integrator
    %
    %See: 
    % dae_reduce_index
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1su
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integration_tools.hpp#L1200
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integration_tools.cpp#L1200-L1203
    %
    %
    %
    %.............
    %
    %
  [varargout{1:nargout}] = casadiMEX(979, varargin{:});
end
