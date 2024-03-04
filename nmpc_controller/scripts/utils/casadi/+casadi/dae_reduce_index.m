function varargout = dae_reduce_index(varargin)
    %DAE_REDUCE_INDEX [INTERNAL] 
    %
    %  [struct:SX , struct OUTPUT] = DAE_REDUCE_INDEX(struct:SX dae, struct opts)
    %  [struct:MX , struct OUTPUT] = DAE_REDUCE_INDEX(struct:MX dae, struct opts)
    %
    %Reduce index.
    %
    %Index reduction leads to a new set of variables and equations.
    %
    %In the process, a set of constraints (algebraic equations or 
    %derivatives) 
    %a.k.a invariants is constructed that are invariant to the
    % problem: whenever
    % an initial point satisfies these constraints, the 
    %boundary-value-problem 
    %outcome will keep satisfying those constraints 
    %automatically, even though 
    %they are  not part of the reduced DAE.
    %
    %For any practical numerical integration method, there will be 
    %numerical 
    %drift away from satisfaction of those constraints. In other 
    %words, you will
    % see the value of invariants slowly moving away from 
    %original zero.
    %
    %A classic mitigation technique is Baumgarte stabilization: you add 
    %these 
    %invariants to the reduced DAE as a correction term that acts in 
    %a way to 
    %make small (numerical) perturbations to the invariants decay 
    %to the origin 
    %as a dampened linear system.
    %
    %in which a certain set of constraints (algebraic equations or 
    %derivatives) 
    %has been dropped in favour of
    %
    %Parameters:
    %-----------
    %
    %dae: 
    %Expression dictionary describing the DAE
    %
    %Each value must be a dense column vector.
    %
    %keys:
    %x_impl: symbol for implicit differential states
    %
    %dx_impl: symbol for implicit differential state derivatives
    %
    %z: symbol for algebraic variables
    %
    %alg: expression for algebraic equations
    %
    %t: symbol for time
    %
    %p: symbol for parameters
    %
    %Parameters:
    %-----------
    %
    %opts: 
    %Option dictionary
    %
    %'baumgarte_pole': double Poles (inverse time constants) of the 
    %Baumgarte 
    %invariant correction term. Must be <0 to dampen out 
    %perturbations 0 
    %(default) amounts to no correction. Corresponds to 
    %-gamma of equation (1.5)
    % in Ascher, Uri M., Hongsheng Chin, and 
    %Sebastian Reich. "Stabilization of
    % DAEs and invariant manifolds." 
    %Numerische Mathematik 67.2 (1994): 
    %131-149.
    %
    %Parameters:
    %-----------
    %
    %stats: 
    %Statistics
    %
    %Expression dictionary describing the reduced DAE
    %
    %In addition the fields allowed in the input DAE, the following keys 
    %occur:
    %
    %x: symbol for explicit differential states
    %
    %ode: expression for right-hand-side of explicit differential states
    %
    %I: expression for invariants
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_23h
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integration_tools.hpp#L1060
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integration_tools.cpp#L1060-L1062
    %
    %
    %
    %.......
    %
    %::
    %
    %  DAE_REDUCE_INDEX(struct:SX dae, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %Reduce index.
    %
    %Index reduction leads to a new set of variables and equations.
    %
    %In the process, a set of constraints (algebraic equations or 
    %derivatives) 
    %a.k.a invariants is constructed that are invariant to the
    % problem: whenever
    % an initial point satisfies these constraints, the 
    %boundary-value-problem 
    %outcome will keep satisfying those constraints 
    %automatically, even though 
    %they are  not part of the reduced DAE.
    %
    %For any practical numerical integration method, there will be 
    %numerical 
    %drift away from satisfaction of those constraints. In other 
    %words, you will
    % see the value of invariants slowly moving away from 
    %original zero.
    %
    %A classic mitigation technique is Baumgarte stabilization: you add 
    %these 
    %invariants to the reduced DAE as a correction term that acts in 
    %a way to 
    %make small (numerical) perturbations to the invariants decay 
    %to the origin 
    %as a dampened linear system.
    %
    %in which a certain set of constraints (algebraic equations or 
    %derivatives) 
    %has been dropped in favour of
    %
    %Parameters:
    %-----------
    %
    %dae: 
    %Expression dictionary describing the DAE
    %
    %Each value must be a dense column vector.
    %
    %keys:
    %x_impl: symbol for implicit differential states
    %
    %dx_impl: symbol for implicit differential state derivatives
    %
    %z: symbol for algebraic variables
    %
    %alg: expression for algebraic equations
    %
    %t: symbol for time
    %
    %p: symbol for parameters
    %
    %Parameters:
    %-----------
    %
    %opts: 
    %Option dictionary
    %
    %'baumgarte_pole': double Poles (inverse time constants) of the 
    %Baumgarte 
    %invariant correction term. Must be <0 to dampen out 
    %perturbations 0 
    %(default) amounts to no correction. Corresponds to 
    %-gamma of equation (1.5)
    % in Ascher, Uri M., Hongsheng Chin, and 
    %Sebastian Reich. "Stabilization of
    % DAEs and invariant manifolds." 
    %Numerische Mathematik 67.2 (1994): 
    %131-149.
    %
    %Parameters:
    %-----------
    %
    %stats: 
    %Statistics
    %
    %Expression dictionary describing the reduced DAE
    %
    %In addition the fields allowed in the input DAE, the following keys 
    %occur:
    %
    %x: symbol for explicit differential states
    %
    %ode: expression for right-hand-side of explicit differential states
    %
    %I: expression for invariants
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_23h
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integration_tools.hpp#L1060
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integration_tools.cpp#L1060-L1062
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
    %  DAE_REDUCE_INDEX(struct:MX dae, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %Reduce index.
    %
    %Index reduction leads to a new set of variables and equations.
    %
    %In the process, a set of constraints (algebraic equations or 
    %derivatives) 
    %a.k.a invariants is constructed that are invariant to the
    % problem: whenever
    % an initial point satisfies these constraints, the 
    %boundary-value-problem 
    %outcome will keep satisfying those constraints 
    %automatically, even though 
    %they are  not part of the reduced DAE.
    %
    %For any practical numerical integration method, there will be 
    %numerical 
    %drift away from satisfaction of those constraints. In other 
    %words, you will
    % see the value of invariants slowly moving away from 
    %original zero.
    %
    %A classic mitigation technique is Baumgarte stabilization: you add 
    %these 
    %invariants to the reduced DAE as a correction term that acts in 
    %a way to 
    %make small (numerical) perturbations to the invariants decay 
    %to the origin 
    %as a dampened linear system.
    %
    %in which a certain set of constraints (algebraic equations or 
    %derivatives) 
    %has been dropped in favour of
    %
    %Parameters:
    %-----------
    %
    %dae: 
    %Expression dictionary describing the DAE
    %
    %Each value must be a dense column vector.
    %
    %keys:
    %x_impl: symbol for implicit differential states
    %
    %dx_impl: symbol for implicit differential state derivatives
    %
    %z: symbol for algebraic variables
    %
    %alg: expression for algebraic equations
    %
    %t: symbol for time
    %
    %p: symbol for parameters
    %
    %Parameters:
    %-----------
    %
    %opts: 
    %Option dictionary
    %
    %'baumgarte_pole': double Poles (inverse time constants) of the 
    %Baumgarte 
    %invariant correction term. Must be <0 to dampen out 
    %perturbations 0 
    %(default) amounts to no correction. Corresponds to 
    %-gamma of equation (1.5)
    % in Ascher, Uri M., Hongsheng Chin, and 
    %Sebastian Reich. "Stabilization of
    % DAEs and invariant manifolds." 
    %Numerische Mathematik 67.2 (1994): 
    %131-149.
    %
    %Parameters:
    %-----------
    %
    %stats: 
    %Statistics
    %
    %Expression dictionary describing the reduced DAE
    %
    %In addition the fields allowed in the input DAE, the following keys 
    %occur:
    %
    %x: symbol for explicit differential states
    %
    %ode: expression for right-hand-side of explicit differential states
    %
    %I: expression for invariants
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_23h
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integration_tools.hpp#L1056
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integration_tools.cpp#L1056-L1058
    %
    %
    %
    %.............
    %
    %
  [varargout{1:nargout}] = casadiMEX(978, varargin{:});
end
