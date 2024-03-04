function varargout = dae_init_gen(varargin)
    %DAE_INIT_GEN [INTERNAL] 
    %
    %  Function = DAE_INIT_GEN(struct:SX dae, struct:SX dae_red, char init_solver, struct:DM init_strength, struct init_solver_options)
    %  Function = DAE_INIT_GEN(struct:MX dae, struct:MX dae_red, char init_solver, struct:DM init_strength, struct init_solver_options)
    %
    %Obtain a generator  Function for producing consistent initial 
    %guesses of a reduced DAE.
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
    %init_solver: 
    %NLP solver plugin name for nlpsol used to construct an initial
    % guess
    %
    %init_strength: 
    %Influence the nature of the NLP Structure with keys x_impl, 
    %dx_impl, z
    % corresponding to inputs of init_gen Each key maps to a DM that 
    %should
    % match the variable size corresponding to that key. For each variable
    % 
    %the meaning of the corresponding DM value is as follows: When >=0, 
    %
    %indicates that the provided initial guess is used in a quadratic 
    %penalty 
    %(value used as weight) When -1, indicates that the provided 
    %initial guess 
    %must be observed (simple bound on variable)
    %
    %init_solver_options: 
    %NLP solver options to be passed to nlpsol
    %
    %init_gen A function to generate a consistent initial guess that can be
    % used
    % to pass to an integrator constructed from a semi explict reduced
    % DAE 
    %Inputs:
    %x_impl, dx_impl, z: initial guesses in the original DAE space
    %
    %p: parameters
    %
    %t: time Outputs:
    %
    %x0, z0: (semi explicit) integrator states and algebraic variables; 
    %
    %typically used as input for integrators
    %
    %See: 
    % dae_reduce_index
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1sv
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integration_tools.hpp#L1215
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integration_tools.cpp#L1215-L1218
    %
    %
    %
    %.......
    %
    %::
    %
    %  DAE_INIT_GEN(struct:SX dae, struct:SX dae_red, char init_solver, struct:DM init_strength, struct init_solver_options)
    %
    %
    %
    %[INTERNAL] 
    %Obtain a generator  Function for producing consistent initial 
    %guesses of a reduced DAE.
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
    %init_solver: 
    %NLP solver plugin name for nlpsol used to construct an initial
    % guess
    %
    %init_strength: 
    %Influence the nature of the NLP Structure with keys x_impl, 
    %dx_impl, z
    % corresponding to inputs of init_gen Each key maps to a DM that 
    %should
    % match the variable size corresponding to that key. For each variable
    % 
    %the meaning of the corresponding DM value is as follows: When >=0, 
    %
    %indicates that the provided initial guess is used in a quadratic 
    %penalty 
    %(value used as weight) When -1, indicates that the provided 
    %initial guess 
    %must be observed (simple bound on variable)
    %
    %init_solver_options: 
    %NLP solver options to be passed to nlpsol
    %
    %init_gen A function to generate a consistent initial guess that can be
    % used
    % to pass to an integrator constructed from a semi explict reduced
    % DAE 
    %Inputs:
    %x_impl, dx_impl, z: initial guesses in the original DAE space
    %
    %p: parameters
    %
    %t: time Outputs:
    %
    %x0, z0: (semi explicit) integrator states and algebraic variables; 
    %
    %typically used as input for integrators
    %
    %See: 
    % dae_reduce_index
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1sv
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integration_tools.hpp#L1215
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integration_tools.cpp#L1215-L1218
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
    %  DAE_INIT_GEN(struct:MX dae, struct:MX dae_red, char init_solver, struct:DM init_strength, struct init_solver_options)
    %
    %
    %
    %[INTERNAL] 
    %Obtain a generator  Function for producing consistent initial 
    %guesses of a reduced DAE.
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
    %init_solver: 
    %NLP solver plugin name for nlpsol used to construct an initial
    % guess
    %
    %init_strength: 
    %Influence the nature of the NLP Structure with keys x_impl, 
    %dx_impl, z
    % corresponding to inputs of init_gen Each key maps to a DM that 
    %should
    % match the variable size corresponding to that key. For each variable
    % 
    %the meaning of the corresponding DM value is as follows: When >=0, 
    %
    %indicates that the provided initial guess is used in a quadratic 
    %penalty 
    %(value used as weight) When -1, indicates that the provided 
    %initial guess 
    %must be observed (simple bound on variable)
    %
    %init_solver_options: 
    %NLP solver options to be passed to nlpsol
    %
    %init_gen A function to generate a consistent initial guess that can be
    % used
    % to pass to an integrator constructed from a semi explict reduced
    % DAE 
    %Inputs:
    %x_impl, dx_impl, z: initial guesses in the original DAE space
    %
    %p: parameters
    %
    %t: time Outputs:
    %
    %x0, z0: (semi explicit) integrator states and algebraic variables; 
    %
    %typically used as input for integrators
    %
    %See: 
    % dae_reduce_index
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1sv
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integration_tools.hpp#L1210
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integration_tools.cpp#L1210-L1213
    %
    %
    %
    %.............
    %
    %
  [varargout{1:nargout}] = casadiMEX(980, varargin{:});
end
