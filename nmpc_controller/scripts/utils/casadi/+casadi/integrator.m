function varargout = integrator(varargin)
    %INTEGRATOR [INTERNAL] 
    %
    %  Function = INTEGRATOR(char name, char solver, struct:SX dae, struct opts)
    %  Function = INTEGRATOR(char name, char solver, struct:MX dae, struct opts)
    %  Function = INTEGRATOR(char name, char solver, Function dae, struct opts)
    %  Function = INTEGRATOR(char name, char solver, struct:SX dae, double t0, [double] tout, struct opts)
    %  Function = INTEGRATOR(char name, char solver, struct:MX dae, double t0, [double] tout, struct opts)
    %  Function = INTEGRATOR(char name, char solver, Function dae, double t0, [double] tout, struct opts)
    %
    %
    %.......
    %
    %::
    %
    %  INTEGRATOR(char name, char solver, struct:SX dae, struct opts)
    %
    %
    %
    %
    %
    %Create an ODE/DAE integrator Solves an initial value problem (IVP) 
    %coupled 
    %to a terminal value problem with differential equation given 
    %as an implicit
    % ODE coupled to an algebraic equation and a set of 
    %quadratures:
    %
    %
    %
    %::
    %
    %  Initial conditions at t=t0
    %  x(t0)  = x0
    %  q(t0)  = 0
    %  
    %  Forward integration from t=t0 to t=tf
    %  der(x) = function(x, z, p, t)                  Forward ODE
    %  0 = fz(x, z, p, t)                  Forward algebraic equations
    %  der(q) = fq(x, z, p, t)                  Forward quadratures
    %  
    %  Terminal conditions at t=tf
    %  rx(tf)  = rx0
    %  rq(tf)  = 0
    %  
    %  Backward integration from t=tf to t=t0
    %  der(rx) = gx(rx, rz, rp, x, z, p, t)        Backward ODE
    %  0 = gz(rx, rz, rp, x, z, p, t)        Backward algebraic equations
    %  der(rq) = gq(rx, rz, rp, x, z, p, t)        Backward quadratures
    %  
    %  where we assume that both the forward and backwards integrations are index-1
    %  (i.e. dfz/dz, dgz/drz are invertible) and furthermore that
    %  gx, gz and gq have a linear dependency on rx, rz and rp.
    %
    %General information
    %
    %>List of available options
    %
    %+------------------+-----------------+------------------+------------------+
    %|        Id        |      Type       |   Description    |     Used in      |
    %+==================+=================+==================+==================+
    %| ad_weight        | OT_DOUBLE       | Weighting factor | casadi::Function |
    %|                  |                 | for derivative   | Internal         |
    %|                  |                 | calculation.When |                  |
    %|                  |                 | there is an      |                  |
    %|                  |                 | option of either |                  |
    %|                  |                 | using forward or |                  |
    %|                  |                 | reverse mode     |                  |
    %|                  |                 | directional      |                  |
    %|                  |                 | derivatives, the |                  |
    %|                  |                 | condition ad_wei |                  |
    %|                  |                 | ght*nf<=(1-      |                  |
    %|                  |                 | ad_weight)*na is |                  |
    %|                  |                 | used where nf    |                  |
    %|                  |                 | and na are       |                  |
    %|                  |                 | estimates of the |                  |
    %|                  |                 | number of        |                  |
    %|                  |                 | forward/reverse  |                  |
    %|                  |                 | mode directional |                  |
    %|                  |                 | derivatives      |                  |
    %|                  |                 | needed. By       |                  |
    %|                  |                 | default,         |                  |
    %|                  |                 | ad_weight is     |                  |
    %|                  |                 | calculated       |                  |
    %|                  |                 | automatically,   |                  |
    %|                  |                 | but this can be  |                  |
    %|                  |                 | overridden by    |                  |
    %|                  |                 | setting this     |                  |
    %|                  |                 | option. In       |                  |
    %|                  |                 | particular, 0    |                  |
    %|                  |                 | means forcing    |                  |
    %|                  |                 | forward mode and |                  |
    %|                  |                 | 1 forcing        |                  |
    %|                  |                 | reverse mode.    |                  |
    %|                  |                 | Leave unset for  |                  |
    %|                  |                 | (class specific) |                  |
    %|                  |                 | heuristics.      |                  |
    %+------------------+-----------------+------------------+------------------+
    %| ad_weight_sp     | OT_DOUBLE       | Weighting factor | casadi::Function |
    %|                  |                 | for sparsity     | Internal         |
    %|                  |                 | pattern          |                  |
    %|                  |                 | calculation calc |                  |
    %|                  |                 | ulation.Override |                  |
    %|                  |                 | s default        |                  |
    %|                  |                 | behavior. Set to |                  |
    %|                  |                 | 0 and 1 to force |                  |
    %|                  |                 | forward and      |                  |
    %|                  |                 | reverse mode     |                  |
    %|                  |                 | respectively.    |                  |
    %|                  |                 | Cf. option       |                  |
    %|                  |                 | "ad_weight".     |                  |
    %|                  |                 | When set to -1,  |                  |
    %|                  |                 | sparsity is      |                  |
    %|                  |                 | completely       |                  |
    %|                  |                 | ignored and      |                  |
    %|                  |                 | dense matrices   |                  |
    %|                  |                 | are used.        |                  |
    %+------------------+-----------------+------------------+------------------+
    %| always_inline    | OT_BOOL         | Force inlining.  | casadi::Function |
    %|                  |                 |                  | Internal         |
    %+------------------+-----------------+------------------+------------------+
    %| augmented_option | OT_DICT         | Options to be    | casadi::Integrat |
    %| s                |                 | passed down to   | or               |
    %|                  |                 | the augmented    |                  |
    %|                  |                 | integrator, if   |                  |
    %|                  |                 | one is           |                  |
    %|                  |                 | constructed.     |                  |
    %+------------------+-----------------+------------------+------------------+
    %| cache            | OT_DICT         | Prepopulate the  | casadi::Function |
    %|                  |                 | function cache.  | Internal         |
    %|                  |                 | Default: empty   |                  |
    %+------------------+-----------------+------------------+------------------+
    %| common_options   | OT_DICT         | Options for      | casadi::OracleFu |
    %|                  |                 | auto-generated   | nction           |
    %|                  |                 | functions        |                  |
    %+------------------+-----------------+------------------+------------------+
    %| compiler         | OT_STRING       | Just-in-time     | casadi::Function |
    %|                  |                 | compiler plugin  | Internal         |
    %|                  |                 | to be used.      |                  |
    %+------------------+-----------------+------------------+------------------+
    %| custom_jacobian  | OT_FUNCTION     | Override         | casadi::Function |
    %|                  |                 | CasADi's AD. Use | Internal         |
    %|                  |                 | together with    |                  |
    %|                  |                 | 'jac_penalty':   |                  |
    %|                  |                 | 0. Note: Highly  |                  |
    %|                  |                 | experimental.    |                  |
    %|                  |                 | Syntax may break |                  |
    %|                  |                 | often.           |                  |
    %+------------------+-----------------+------------------+------------------+
    %| der_options      | OT_DICT         | Default options  | casadi::Function |
    %|                  |                 | to be used to    | Internal         |
    %|                  |                 | populate         |                  |
    %|                  |                 | forward_options, |                  |
    %|                  |                 | reverse_options, |                  |
    %|                  |                 | and              |                  |
    %|                  |                 | jacobian_options |                  |
    %|                  |                 | before those     |                  |
    %|                  |                 | options are      |                  |
    %|                  |                 | merged in.       |                  |
    %+------------------+-----------------+------------------+------------------+
    %| derivative_of    | OT_FUNCTION     | The function is  | casadi::Function |
    %|                  |                 | a derivative of  | Internal         |
    %|                  |                 | another          |                  |
    %|                  |                 | function. The    |                  |
    %|                  |                 | type of          |                  |
    %|                  |                 | derivative       |                  |
    %|                  |                 | (directional     |                  |
    %|                  |                 | derivative,      |                  |
    %|                  |                 | Jacobian) is     |                  |
    %|                  |                 | inferred from    |                  |
    %|                  |                 | the function     |                  |
    %|                  |                 | name.            |                  |
    %+------------------+-----------------+------------------+------------------+
    %| dump             | OT_BOOL         | Dump function to | casadi::Function |
    %|                  |                 | file upon first  | Internal         |
    %|                  |                 | evaluation.      |                  |
    %|                  |                 | [false]          |                  |
    %+------------------+-----------------+------------------+------------------+
    %| dump_dir         | OT_STRING       | Directory to     | casadi::Function |
    %|                  |                 | dump             | Internal         |
    %|                  |                 | inputs/outputs   |                  |
    %|                  |                 | to. Make sure    |                  |
    %|                  |                 | the directory    |                  |
    %|                  |                 | exists [.]       |                  |
    %+------------------+-----------------+------------------+------------------+
    %| dump_format      | OT_STRING       | Choose file      | casadi::Function |
    %|                  |                 | format to dump   | Internal         |
    %|                  |                 | matrices. See    |                  |
    %|                  |                 | DM.from_file     |                  |
    %|                  |                 | [mtx]            |                  |
    %+------------------+-----------------+------------------+------------------+
    %| dump_in          | OT_BOOL         | Dump numerical   | casadi::Function |
    %|                  |                 | values of inputs | Internal         |
    %|                  |                 | to file          |                  |
    %|                  |                 | (readable with   |                  |
    %|                  |                 | DM.from_file )   |                  |
    %|                  |                 | [default: false] |                  |
    %+------------------+-----------------+------------------+------------------+
    %| dump_out         | OT_BOOL         | Dump numerical   | casadi::Function |
    %|                  |                 | values of        | Internal         |
    %|                  |                 | outputs to file  |                  |
    %|                  |                 | (readable with   |                  |
    %|                  |                 | DM.from_file )   |                  |
    %|                  |                 | [default: false] |                  |
    %+------------------+-----------------+------------------+------------------+
    %| enable_fd        | OT_BOOL         | Enable           | casadi::Function |
    %|                  |                 | derivative       | Internal         |
    %|                  |                 | calculation by   |                  |
    %|                  |                 | finite           |                  |
    %|                  |                 | differencing.    |                  |
    %|                  |                 | [default:        |                  |
    %|                  |                 | false]]          |                  |
    %+------------------+-----------------+------------------+------------------+
    %| enable_forward   | OT_BOOL         | Enable           | casadi::Function |
    %|                  |                 | derivative       | Internal         |
    %|                  |                 | calculation      |                  |
    %|                  |                 | using generated  |                  |
    %|                  |                 | functions for    |                  |
    %|                  |                 | Jacobian-times-  |                  |
    %|                  |                 | vector products  |                  |
    %|                  |                 | - typically      |                  |
    %|                  |                 | using forward    |                  |
    %|                  |                 | mode AD - if     |                  |
    %|                  |                 | available.       |                  |
    %|                  |                 | [default: true]  |                  |
    %+------------------+-----------------+------------------+------------------+
    %| enable_jacobian  | OT_BOOL         | Enable           | casadi::Function |
    %|                  |                 | derivative       | Internal         |
    %|                  |                 | calculation      |                  |
    %|                  |                 | using generated  |                  |
    %|                  |                 | functions for    |                  |
    %|                  |                 | Jacobians of all |                  |
    %|                  |                 | differentiable   |                  |
    %|                  |                 | outputs with     |                  |
    %|                  |                 | respect to all   |                  |
    %|                  |                 | differentiable   |                  |
    %|                  |                 | inputs - if      |                  |
    %|                  |                 | available.       |                  |
    %|                  |                 | [default: true]  |                  |
    %+------------------+-----------------+------------------+------------------+
    %| enable_reverse   | OT_BOOL         | Enable           | casadi::Function |
    %|                  |                 | derivative       | Internal         |
    %|                  |                 | calculation      |                  |
    %|                  |                 | using generated  |                  |
    %|                  |                 | functions for    |                  |
    %|                  |                 | transposed       |                  |
    %|                  |                 | Jacobian-times-  |                  |
    %|                  |                 | vector products  |                  |
    %|                  |                 | - typically      |                  |
    %|                  |                 | using reverse    |                  |
    %|                  |                 | mode AD - if     |                  |
    %|                  |                 | available.       |                  |
    %|                  |                 | [default: true]  |                  |
    %+------------------+-----------------+------------------+------------------+
    %| error_on_fail    | OT_BOOL         | Throw exceptions | casadi::ProtoFun |
    %|                  |                 | when function    | ction            |
    %|                  |                 | evaluation fails |                  |
    %|                  |                 | (default true).  |                  |
    %+------------------+-----------------+------------------+------------------+
    %| expand           | OT_BOOL         | Replace MX with  | casadi::Integrat |
    %|                  |                 | SX expressions   | or               |
    %|                  |                 | in problem       |                  |
    %|                  |                 | formulation      |                  |
    %|                  |                 | [false]          |                  |
    %+------------------+-----------------+------------------+------------------+
    %| fd_method        | OT_STRING       | Method for       | casadi::Function |
    %|                  |                 | finite           | Internal         |
    %|                  |                 | differencing     |                  |
    %|                  |                 | [default         |                  |
    %|                  |                 | 'central']       |                  |
    %+------------------+-----------------+------------------+------------------+
    %| fd_options       | OT_DICT         | Options to be    | casadi::Function |
    %|                  |                 | passed to the    | Internal         |
    %|                  |                 | finite           |                  |
    %|                  |                 | difference       |                  |
    %|                  |                 | instance         |                  |
    %+------------------+-----------------+------------------+------------------+
    %| forward_options  | OT_DICT         | Options to be    | casadi::Function |
    %|                  |                 | passed to a      | Internal         |
    %|                  |                 | forward mode     |                  |
    %|                  |                 | constructor      |                  |
    %+------------------+-----------------+------------------+------------------+
    %| gather_stats     | OT_BOOL         | Deprecated       | casadi::Function |
    %|                  |                 | option           | Internal         |
    %|                  |                 | (ignored):       |                  |
    %|                  |                 | Statistics are   |                  |
    %|                  |                 | now always       |                  |
    %|                  |                 | collected.       |                  |
    %+------------------+-----------------+------------------+------------------+
    %| grid             | OT_DOUBLEVECTOR | [DEPRECATED]     | casadi::Integrat |
    %|                  |                 | Time grid        | or               |
    %+------------------+-----------------+------------------+------------------+
    %| input_scheme     | OT_STRINGVECTOR | Deprecated       | casadi::Function |
    %|                  |                 | option (ignored) | Internal         |
    %+------------------+-----------------+------------------+------------------+
    %| inputs_check     | OT_BOOL         | Throw exceptions | casadi::Function |
    %|                  |                 | when the         | Internal         |
    %|                  |                 | numerical values |                  |
    %|                  |                 | of the inputs    |                  |
    %|                  |                 | don't make sense |                  |
    %+------------------+-----------------+------------------+------------------+
    %| is_diff_in       | OT_BOOLVECTOR   | Indicate for     | casadi::Function |
    %|                  |                 | each input if it | Internal         |
    %|                  |                 | should be        |                  |
    %|                  |                 | differentiable.  |                  |
    %+------------------+-----------------+------------------+------------------+
    %| is_diff_out      | OT_BOOLVECTOR   | Indicate for     | casadi::Function |
    %|                  |                 | each output if   | Internal         |
    %|                  |                 | it should be     |                  |
    %|                  |                 | differentiable.  |                  |
    %+------------------+-----------------+------------------+------------------+
    %| jac_penalty      | OT_DOUBLE       | When requested   | casadi::Function |
    %|                  |                 | for a number of  | Internal         |
    %|                  |                 | forward/reverse  |                  |
    %|                  |                 | directions, it   |                  |
    %|                  |                 | may be cheaper   |                  |
    %|                  |                 | to compute first |                  |
    %|                  |                 | the full         |                  |
    %|                  |                 | jacobian and     |                  |
    %|                  |                 | then multiply    |                  |
    %|                  |                 | with seeds,      |                  |
    %|                  |                 | rather than      |                  |
    %|                  |                 | obtain the       |                  |
    %|                  |                 | requested        |                  |
    %|                  |                 | directions in a  |                  |
    %|                  |                 | straightforward  |                  |
    %|                  |                 | manner. Casadi   |                  |
    %|                  |                 | uses a heuristic |                  |
    %|                  |                 | to decide which  |                  |
    %|                  |                 | is cheaper. A    |                  |
    %|                  |                 | high value of    |                  |
    %|                  |                 | 'jac_penalty'    |                  |
    %|                  |                 | makes it less    |                  |
    %|                  |                 | likely for the   |                  |
    %|                  |                 | heurstic to      |                  |
    %|                  |                 | chose the full   |                  |
    %|                  |                 | Jacobian         |                  |
    %|                  |                 | strategy. The    |                  |
    %|                  |                 | special value -1 |                  |
    %|                  |                 | indicates never  |                  |
    %|                  |                 | to use the full  |                  |
    %|                  |                 | Jacobian         |                  |
    %|                  |                 | strategy         |                  |
    %+------------------+-----------------+------------------+------------------+
    %| jacobian_options | OT_DICT         | Options to be    | casadi::Function |
    %|                  |                 | passed to a      | Internal         |
    %|                  |                 | Jacobian         |                  |
    %|                  |                 | constructor      |                  |
    %+------------------+-----------------+------------------+------------------+
    %| jit              | OT_BOOL         | Use just-in-time | casadi::Function |
    %|                  |                 | compiler to      | Internal         |
    %|                  |                 | speed up the     |                  |
    %|                  |                 | evaluation       |                  |
    %+------------------+-----------------+------------------+------------------+
    %| jit_cleanup      | OT_BOOL         | Cleanup up the   | casadi::Function |
    %|                  |                 | temporary source | Internal         |
    %|                  |                 | file that jit    |                  |
    %|                  |                 | creates.         |                  |
    %|                  |                 | Default: true    |                  |
    %+------------------+-----------------+------------------+------------------+
    %| jit_name         | OT_STRING       | The file name    | casadi::Function |
    %|                  |                 | used to write    | Internal         |
    %|                  |                 | out code. The    |                  |
    %|                  |                 | actual file      |                  |
    %|                  |                 | names used       |                  |
    %|                  |                 | depend on 'jit_t |                  |
    %|                  |                 | emp_suffix' and  |                  |
    %|                  |                 | include          |                  |
    %|                  |                 | extensions.      |                  |
    %|                  |                 | Default:         |                  |
    %|                  |                 | 'jit_tmp'        |                  |
    %+------------------+-----------------+------------------+------------------+
    %| jit_options      | OT_DICT         | Options to be    | casadi::Function |
    %|                  |                 | passed to the    | Internal         |
    %|                  |                 | jit compiler.    |                  |
    %+------------------+-----------------+------------------+------------------+
    %| jit_serialize    | OT_STRING       | Specify          | casadi::Function |
    %|                  |                 | behaviour when   | Internal         |
    %|                  |                 | serializing a    |                  |
    %|                  |                 | jitted function: |                  |
    %|                  |                 | SOURCE|link|embe |                  |
    %|                  |                 | d.               |                  |
    %+------------------+-----------------+------------------+------------------+
    %| jit_temp_suffix  | OT_BOOL         | Use a temporary  | casadi::Function |
    %|                  |                 | (seemingly       | Internal         |
    %|                  |                 | random) filename |                  |
    %|                  |                 | suffix for       |                  |
    %|                  |                 | generated code   |                  |
    %|                  |                 | and libraries.   |                  |
    %|                  |                 | This is desired  |                  |
    %|                  |                 | for thread-      |                  |
    %|                  |                 | safety. This     |                  |
    %|                  |                 | behaviour may    |                  |
    %|                  |                 | defeat caching   |                  |
    %|                  |                 | compiler         |                  |
    %|                  |                 | wrappers.        |                  |
    %|                  |                 | Default: true    |                  |
    %+------------------+-----------------+------------------+------------------+
    %| max_io           | OT_INT          | Acceptable       | casadi::Function |
    %|                  |                 | number of inputs | Internal         |
    %|                  |                 | and outputs.     |                  |
    %|                  |                 | Warn if          |                  |
    %|                  |                 | exceeded.        |                  |
    %+------------------+-----------------+------------------+------------------+
    %| max_num_dir      | OT_INT          | Specify the      | casadi::Function |
    %|                  |                 | maximum number   | Internal         |
    %|                  |                 | of directions    |                  |
    %|                  |                 | for derivative   |                  |
    %|                  |                 | functions.       |                  |
    %|                  |                 | Overrules the    |                  |
    %|                  |                 | builtin optimize |                  |
    %|                  |                 | d_num_dir.       |                  |
    %+------------------+-----------------+------------------+------------------+
    %| monitor          | OT_STRINGVECTOR | Set of user      | casadi::OracleFu |
    %|                  |                 | problem          | nction           |
    %|                  |                 | functions to be  |                  |
    %|                  |                 | monitored        |                  |
    %+------------------+-----------------+------------------+------------------+
    %| nadj             | OT_INT          | Number of        | casadi::Integrat |
    %|                  |                 | adjoint          | or               |
    %|                  |                 | sensitivities to |                  |
    %|                  |                 | be calculated    |                  |
    %|                  |                 | [0]              |                  |
    %+------------------+-----------------+------------------+------------------+
    %| never_inline     | OT_BOOL         | Forbid inlining. | casadi::Function |
    %|                  |                 |                  | Internal         |
    %+------------------+-----------------+------------------+------------------+
    %| nfwd             | OT_INT          | Number of        | casadi::Integrat |
    %|                  |                 | forward          | or               |
    %|                  |                 | sensitivities to |                  |
    %|                  |                 | be calculated    |                  |
    %|                  |                 | [0]              |                  |
    %+------------------+-----------------+------------------+------------------+
    %| number_of_finite | OT_INT          | Target number of | casadi::Integrat |
    %| _elements        |                 | finite elements. | or               |
    %|                  |                 | The actual       |                  |
    %|                  |                 | number may be    |                  |
    %|                  |                 | higher to        |                  |
    %|                  |                 | accommodate all  |                  |
    %|                  |                 | output times     |                  |
    %+------------------+-----------------+------------------+------------------+
    %| output_scheme    | OT_STRINGVECTOR | Deprecated       | casadi::Function |
    %|                  |                 | option (ignored) | Internal         |
    %+------------------+-----------------+------------------+------------------+
    %| output_t0        | OT_BOOL         | [DEPRECATED]     | casadi::Integrat |
    %|                  |                 | Output the state | or               |
    %|                  |                 | at the initial   |                  |
    %|                  |                 | time             |                  |
    %+------------------+-----------------+------------------+------------------+
    %| post_expand      | OT_BOOL         | After            | casadi::Function |
    %|                  |                 | construction,    | Internal         |
    %|                  |                 | expand this      |                  |
    %|                  |                 | Function .       |                  |
    %|                  |                 | Default: False   |                  |
    %+------------------+-----------------+------------------+------------------+
    %| post_expand_opti | OT_DICT         | Options to be    | casadi::Function |
    %| ons              |                 | passed to post-  | Internal         |
    %|                  |                 | construction     |                  |
    %|                  |                 | expansion.       |                  |
    %|                  |                 | Default: empty   |                  |
    %+------------------+-----------------+------------------+------------------+
    %| print_in         | OT_BOOL         | Print numerical  | casadi::Function |
    %|                  |                 | values of inputs | Internal         |
    %|                  |                 | [default: false] |                  |
    %+------------------+-----------------+------------------+------------------+
    %| print_out        | OT_BOOL         | Print numerical  | casadi::Function |
    %|                  |                 | values of        | Internal         |
    %|                  |                 | outputs          |                  |
    %|                  |                 | [default: false] |                  |
    %+------------------+-----------------+------------------+------------------+
    %| print_stats      | OT_BOOL         | Print out        | casadi::Integrat |
    %|                  |                 | statistics after | or               |
    %|                  |                 | integration      |                  |
    %+------------------+-----------------+------------------+------------------+
    %| print_time       | OT_BOOL         | print            | casadi::ProtoFun |
    %|                  |                 | information      | ction            |
    %|                  |                 | about execution  |                  |
    %|                  |                 | time. Implies    |                  |
    %|                  |                 | record_time.     |                  |
    %+------------------+-----------------+------------------+------------------+
    %| record_time      | OT_BOOL         | record           | casadi::ProtoFun |
    %|                  |                 | information      | ction            |
    %|                  |                 | about execution  |                  |
    %|                  |                 | time, for        |                  |
    %|                  |                 | retrieval with   |                  |
    %|                  |                 | stats().         |                  |
    %+------------------+-----------------+------------------+------------------+
    %| regularity_check | OT_BOOL         | Throw exceptions | casadi::ProtoFun |
    %|                  |                 | when NaN or Inf  | ction            |
    %|                  |                 | appears during   |                  |
    %|                  |                 | evaluation       |                  |
    %+------------------+-----------------+------------------+------------------+
    %| reverse_options  | OT_DICT         | Options to be    | casadi::Function |
    %|                  |                 | passed to a      | Internal         |
    %|                  |                 | reverse mode     |                  |
    %|                  |                 | constructor      |                  |
    %+------------------+-----------------+------------------+------------------+
    %| rootfinder       | OT_STRING       | An implicit      | casadi::Integrat |
    %|                  |                 | function solver  | or               |
    %+------------------+-----------------+------------------+------------------+
    %| rootfinder_optio | OT_DICT         | Options to be    | casadi::Integrat |
    %| ns               |                 | passed to the    | or               |
    %|                  |                 | NLP Solver       |                  |
    %+------------------+-----------------+------------------+------------------+
    %| show_eval_warnin | OT_BOOL         | Show warnings    | casadi::OracleFu |
    %| gs               |                 | generated from   | nction           |
    %|                  |                 | function         |                  |
    %|                  |                 | evaluations      |                  |
    %|                  |                 | [true]           |                  |
    %+------------------+-----------------+------------------+------------------+
    %| simplify         | OT_BOOL         | Implement as MX  | casadi::Integrat |
    %|                  |                 | Function (codege | or               |
    %|                  |                 | neratable/serial |                  |
    %|                  |                 | izable) default: |                  |
    %|                  |                 | false            |                  |
    %+------------------+-----------------+------------------+------------------+
    %| simplify_options | OT_DICT         | Any options to   | casadi::Integrat |
    %|                  |                 | pass to          | or               |
    %|                  |                 | simplified form  |                  |
    %|                  |                 | Function         |                  |
    %|                  |                 | constructor      |                  |
    %+------------------+-----------------+------------------+------------------+
    %| specific_options | OT_DICT         | Options for      | casadi::OracleFu |
    %|                  |                 | specific auto-   | nction           |
    %|                  |                 | generated        |                  |
    %|                  |                 | functions,       |                  |
    %|                  |                 | overwriting the  |                  |
    %|                  |                 | defaults from    |                  |
    %|                  |                 | common_options.  |                  |
    %|                  |                 | Nested           |                  |
    %|                  |                 | dictionary.      |                  |
    %+------------------+-----------------+------------------+------------------+
    %| t0               | OT_DOUBLE       | [DEPRECATED]     | casadi::Integrat |
    %|                  |                 | Beginning of the | or               |
    %|                  |                 | time horizon     |                  |
    %+------------------+-----------------+------------------+------------------+
    %| tf               | OT_DOUBLE       | [DEPRECATED] End | casadi::Integrat |
    %|                  |                 | of the time      | or               |
    %|                  |                 | horizon          |                  |
    %+------------------+-----------------+------------------+------------------+
    %| user_data        | OT_VOIDPTR      | A user-defined   | casadi::Function |
    %|                  |                 | field that can   | Internal         |
    %|                  |                 | be used to       |                  |
    %|                  |                 | identify the     |                  |
    %|                  |                 | function or pass |                  |
    %|                  |                 | additional       |                  |
    %|                  |                 | information      |                  |
    %+------------------+-----------------+------------------+------------------+
    %| verbose          | OT_BOOL         | Verbose          | casadi::ProtoFun |
    %|                  |                 | evaluation  for  | ction            |
    %|                  |                 | debugging        |                  |
    %+------------------+-----------------+------------------+------------------+
    %
    %>Input scheme: casadi::IntegratorInput (INTEGRATOR_NUM_IN = 7)
    %
    %+-------------------+--------+---------------------------------------------+
    %|     Full name     | Short  |                 Description                 |
    %+===================+========+=============================================+
    %| INTEGRATOR_X0     | x0     | Differential state at the initial time.     |
    %+-------------------+--------+---------------------------------------------+
    %| INTEGRATOR_Z0     | z0     | Initial guess for the algebraic variable at |
    %|                   |        | the initial time.                           |
    %+-------------------+--------+---------------------------------------------+
    %| INTEGRATOR_P      | p      | Parameters.                                 |
    %+-------------------+--------+---------------------------------------------+
    %| INTEGRATOR_U      | u      | Piecewise constant control, a new control   |
    %|                   |        | interval starts at each output time.        |
    %+-------------------+--------+---------------------------------------------+
    %| INTEGRATOR_ADJ_XF | adj_xf | Adjoint seeds corresponding to the states   |
    %|                   |        | at the output times.                        |
    %+-------------------+--------+---------------------------------------------+
    %| INTEGRATOR_ADJ_ZF | adj_zf | Adjoint seeds corresponding to the          |
    %|                   |        | algebraic variables at the output times.    |
    %+-------------------+--------+---------------------------------------------+
    %| INTEGRATOR_ADJ_QF | adj_qf | Adjoint seeds corresponding to the          |
    %|                   |        | quadratures at the output times.            |
    %+-------------------+--------+---------------------------------------------+
    %
    %>Output scheme: casadi::IntegratorOutput (INTEGRATOR_NUM_OUT = 7)
    %
    %+-------------------+--------+---------------------------------------------+
    %|     Full name     | Short  |                 Description                 |
    %+===================+========+=============================================+
    %| INTEGRATOR_XF     | xf     | Differential state at all output times.     |
    %+-------------------+--------+---------------------------------------------+
    %| INTEGRATOR_ZF     | zf     | Algebraic variable at all output times.     |
    %+-------------------+--------+---------------------------------------------+
    %| INTEGRATOR_QF     | qf     | Quadrature state at all output times.       |
    %+-------------------+--------+---------------------------------------------+
    %| INTEGRATOR_ADJ_X0 | adj_x0 | Adjoint sensitivities corresponding to the  |
    %|                   |        | initial state.                              |
    %+-------------------+--------+---------------------------------------------+
    %| INTEGRATOR_ADJ_Z0 | adj_z0 | Adjoint sensitivities corresponding to the  |
    %|                   |        | algebraic variable guess.                   |
    %+-------------------+--------+---------------------------------------------+
    %| INTEGRATOR_ADJ_P  | adj_p  | Adjoint sensitivities corresponding to the  |
    %|                   |        | parameter vector.                           |
    %+-------------------+--------+---------------------------------------------+
    %| INTEGRATOR_ADJ_U  | adj_u  | Adjoint sensitivities corresponding to the  |
    %|                   |        | control vector.                             |
    %+-------------------+--------+---------------------------------------------+
    %
    %List of plugins
    %- cvodes
    %
    %- idas
    %
    %- collocation
    %
    %- rk
    %
    %Note: some of the plugins in this list might not be available on your 
    %
    %system.  Also, there might be extra plugins available to you that are 
    %not 
    %listed here. You can obtain their documentation with  
    %Integrator.doc("myextraplugin")
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %cvodes
    %------
    %
    %
    %
    %Interface to CVodes from the Sundials suite.
    %
    %A call to evaluate will integrate to the end.
    %
    %You can retrieve the entire state trajectory as follows, after the 
    %evaluate
    % call: Call reset. Then call integrate(t_i) and getOuput for a
    % series of 
    %times t_i.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_228
    %
    %>List of available options
    %
    %+-----------------------------+-----------+--------------------------------+
    %|             Id              |   Type    |          Description           |
    %+=============================+===========+================================+
    %| abstol                      | OT_DOUBLE | Absolute tolerence for the IVP |
    %|                             |           | solution                       |
    %+-----------------------------+-----------+--------------------------------+
    %| always_recalculate_jacobian | OT_BOOL   | Recalculate Jacobian before    |
    %|                             |           | factorizations, even if        |
    %|                             |           | Jacobian is current [default:  |
    %|                             |           | true]                          |
    %+-----------------------------+-----------+--------------------------------+
    %| disable_internal_warnings   | OT_BOOL   | Disable SUNDIALS internal      |
    %|                             |           | warning messages               |
    %+-----------------------------+-----------+--------------------------------+
    %| fsens_all_at_once           | OT_BOOL   | Calculate all right hand sides |
    %|                             |           | of the sensitivity equations   |
    %|                             |           | at once                        |
    %+-----------------------------+-----------+--------------------------------+
    %| fsens_err_con               | OT_BOOL   | include the forward            |
    %|                             |           | sensitivities in all error     |
    %|                             |           | controls                       |
    %+-----------------------------+-----------+--------------------------------+
    %| interpolation_type          | OT_STRING | Type of interpolation for the  |
    %|                             |           | adjoint sensitivities          |
    %+-----------------------------+-----------+--------------------------------+
    %| linear_multistep_method     | OT_STRING | Integrator scheme: BDF|adams   |
    %+-----------------------------+-----------+--------------------------------+
    %| linear_solver               | OT_STRING | A custom linear solver creator |
    %|                             |           | function [default: qr]         |
    %+-----------------------------+-----------+--------------------------------+
    %| linear_solver_options       | OT_DICT   | Options to be passed to the    |
    %|                             |           | linear solver                  |
    %+-----------------------------+-----------+--------------------------------+
    %| max_krylov                  | OT_INT    | Maximum Krylov subspace size   |
    %+-----------------------------+-----------+--------------------------------+
    %| max_multistep_order         | OT_INT    | Maximum order for the          |
    %|                             |           | (variable-order) multistep     |
    %|                             |           | method                         |
    %+-----------------------------+-----------+--------------------------------+
    %| max_num_steps               | OT_INT    | Maximum number of integrator   |
    %|                             |           | steps                          |
    %+-----------------------------+-----------+--------------------------------+
    %| max_order                   | OT_DOUBLE | Maximum order                  |
    %+-----------------------------+-----------+--------------------------------+
    %| max_step_size               | OT_DOUBLE | Max step size [default: 0/inf] |
    %+-----------------------------+-----------+--------------------------------+
    %| min_step_size               | OT_DOUBLE | Min step size [default: 0/0.0] |
    %+-----------------------------+-----------+--------------------------------+
    %| newton_scheme               | OT_STRING | Linear solver scheme in the    |
    %|                             |           | Newton method:                 |
    %|                             |           | DIRECT|gmres|bcgstab|tfqmr     |
    %+-----------------------------+-----------+--------------------------------+
    %| nonlin_conv_coeff           | OT_DOUBLE | Coefficient in the nonlinear   |
    %|                             |           | convergence test               |
    %+-----------------------------+-----------+--------------------------------+
    %| nonlinear_solver_iteration  | OT_STRING | Nonlinear solver type:         |
    %|                             |           | NEWTON|functional              |
    %+-----------------------------+-----------+--------------------------------+
    %| quad_err_con                | OT_BOOL   | Should the quadratures affect  |
    %|                             |           | the step size control          |
    %+-----------------------------+-----------+--------------------------------+
    %| reltol                      | OT_DOUBLE | Relative tolerence for the IVP |
    %|                             |           | solution                       |
    %+-----------------------------+-----------+--------------------------------+
    %| scale_abstol                | OT_BOOL   | Scale absolute tolerance by    |
    %|                             |           | nominal value                  |
    %+-----------------------------+-----------+--------------------------------+
    %| second_order_correction     | OT_BOOL   | Second order correction in the |
    %|                             |           | augmented system Jacobian      |
    %|                             |           | [true]                         |
    %+-----------------------------+-----------+--------------------------------+
    %| sensitivity_method          | OT_STRING | Sensitivity method:            |
    %|                             |           | SIMULTANEOUS|staggered         |
    %+-----------------------------+-----------+--------------------------------+
    %| step0                       | OT_DOUBLE | initial step size [default:    |
    %|                             |           | 0/estimated]                   |
    %+-----------------------------+-----------+--------------------------------+
    %| steps_per_checkpoint        | OT_INT    | Number of steps between two    |
    %|                             |           | consecutive checkpoints        |
    %+-----------------------------+-----------+--------------------------------+
    %| stop_at_end                 | OT_BOOL   | [DEPRECATED] Stop the          |
    %|                             |           | integrator at the end of the   |
    %|                             |           | interval                       |
    %+-----------------------------+-----------+--------------------------------+
    %| use_preconditioner          | OT_BOOL   | Precondition the iterative     |
    %|                             |           | solver [default: true]         |
    %+-----------------------------+-----------+--------------------------------+
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %idas
    %----
    %
    %
    %
    %Interface to IDAS from the Sundials suite.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_225
    %
    %>List of available options
    %
    %+---------------------------+-----------------+----------------------------+
    %|            Id             |      Type       |        Description         |
    %+===========================+=================+============================+
    %| abstol                    | OT_DOUBLE       | Absolute tolerence for the |
    %|                           |                 | IVP solution               |
    %+---------------------------+-----------------+----------------------------+
    %| abstolv                   | OT_DOUBLEVECTOR | Absolute tolerarance for   |
    %|                           |                 | each component             |
    %+---------------------------+-----------------+----------------------------+
    %| calc_ic                   | OT_BOOL         | Use IDACalcIC to get       |
    %|                           |                 | consistent initial         |
    %|                           |                 | conditions.                |
    %+---------------------------+-----------------+----------------------------+
    %| calc_icB                  | OT_BOOL         | Use IDACalcIC to get       |
    %|                           |                 | consistent initial         |
    %|                           |                 | conditions for backwards   |
    %|                           |                 | system [default: equal to  |
    %|                           |                 | calc_ic].                  |
    %+---------------------------+-----------------+----------------------------+
    %| cj_scaling                | OT_BOOL         | IDAS scaling on cj for the |
    %|                           |                 | user-defined linear solver |
    %|                           |                 | module                     |
    %+---------------------------+-----------------+----------------------------+
    %| constraints               | OT_INTVECTOR    | Constrain the solution     |
    %|                           |                 | y=[x,z]. 0 (default): no   |
    %|                           |                 | constraint on yi, 1: yi >= |
    %|                           |                 | 0.0, -1: yi <= 0.0, 2: yi  |
    %|                           |                 | > 0.0, -2: yi < 0.0.       |
    %+---------------------------+-----------------+----------------------------+
    %| disable_internal_warnings | OT_BOOL         | Disable SUNDIALS internal  |
    %|                           |                 | warning messages           |
    %+---------------------------+-----------------+----------------------------+
    %| first_time                | OT_DOUBLE       | First requested time as a  |
    %|                           |                 | fraction of the time       |
    %|                           |                 | interval                   |
    %+---------------------------+-----------------+----------------------------+
    %| fsens_err_con             | OT_BOOL         | include the forward        |
    %|                           |                 | sensitivities in all error |
    %|                           |                 | controls                   |
    %+---------------------------+-----------------+----------------------------+
    %| init_xdot                 | OT_DOUBLEVECTOR | Initial values for the     |
    %|                           |                 | state derivatives          |
    %+---------------------------+-----------------+----------------------------+
    %| interpolation_type        | OT_STRING       | Type of interpolation for  |
    %|                           |                 | the adjoint sensitivities  |
    %+---------------------------+-----------------+----------------------------+
    %| linear_solver             | OT_STRING       | A custom linear solver     |
    %|                           |                 | creator function [default: |
    %|                           |                 | qr]                        |
    %+---------------------------+-----------------+----------------------------+
    %| linear_solver_options     | OT_DICT         | Options to be passed to    |
    %|                           |                 | the linear solver          |
    %+---------------------------+-----------------+----------------------------+
    %| max_krylov                | OT_INT          | Maximum Krylov subspace    |
    %|                           |                 | size                       |
    %+---------------------------+-----------------+----------------------------+
    %| max_multistep_order       | OT_INT          | Maximum order for the      |
    %|                           |                 | (variable-order) multistep |
    %|                           |                 | method                     |
    %+---------------------------+-----------------+----------------------------+
    %| max_num_steps             | OT_INT          | Maximum number of          |
    %|                           |                 | integrator steps           |
    %+---------------------------+-----------------+----------------------------+
    %| max_order                 | OT_DOUBLE       | Maximum order              |
    %+---------------------------+-----------------+----------------------------+
    %| max_step_size             | OT_DOUBLE       | Maximim step size          |
    %+---------------------------+-----------------+----------------------------+
    %| newton_scheme             | OT_STRING       | Linear solver scheme in    |
    %|                           |                 | the Newton method:         |
    %|                           |                 | DIRECT|gmres|bcgstab|tfqmr |
    %+---------------------------+-----------------+----------------------------+
    %| nonlin_conv_coeff         | OT_DOUBLE       | Coefficient in the         |
    %|                           |                 | nonlinear convergence test |
    %+---------------------------+-----------------+----------------------------+
    %| quad_err_con              | OT_BOOL         | Should the quadratures     |
    %|                           |                 | affect the step size       |
    %|                           |                 | control                    |
    %+---------------------------+-----------------+----------------------------+
    %| reltol                    | OT_DOUBLE       | Relative tolerence for the |
    %|                           |                 | IVP solution               |
    %+---------------------------+-----------------+----------------------------+
    %| scale_abstol              | OT_BOOL         | Scale absolute tolerance   |
    %|                           |                 | by nominal value           |
    %+---------------------------+-----------------+----------------------------+
    %| second_order_correction   | OT_BOOL         | Second order correction in |
    %|                           |                 | the augmented system       |
    %|                           |                 | Jacobian [true]            |
    %+---------------------------+-----------------+----------------------------+
    %| sensitivity_method        | OT_STRING       | Sensitivity method:        |
    %|                           |                 | SIMULTANEOUS|staggered     |
    %+---------------------------+-----------------+----------------------------+
    %| step0                     | OT_DOUBLE       | initial step size          |
    %|                           |                 | [default: 0/estimated]     |
    %+---------------------------+-----------------+----------------------------+
    %| steps_per_checkpoint      | OT_INT          | Number of steps between    |
    %|                           |                 | two consecutive            |
    %|                           |                 | checkpoints                |
    %+---------------------------+-----------------+----------------------------+
    %| stop_at_end               | OT_BOOL         | [DEPRECATED] Stop the      |
    %|                           |                 | integrator at the end of   |
    %|                           |                 | the interval               |
    %+---------------------------+-----------------+----------------------------+
    %| suppress_algebraic        | OT_BOOL         | Suppress algebraic         |
    %|                           |                 | variables in the error     |
    %|                           |                 | testing                    |
    %+---------------------------+-----------------+----------------------------+
    %| use_preconditioner        | OT_BOOL         | Precondition the iterative |
    %|                           |                 | solver [default: true]     |
    %+---------------------------+-----------------+----------------------------+
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %collocation
    %-----------
    %
    %
    %
    %Fixed-step implicit Runge-Kutta integrator ODE/DAE integrator based 
    %on 
    %collocation schemes
    %
    %The method is still under development
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_234
    %
    %>List of available options
    %
    %+---------------------------+-----------+----------------------------------+
    %|            Id             |   Type    |           Description            |
    %+===========================+===========+==================================+
    %| collocation_scheme        | OT_STRING | Collocation scheme:              |
    %|                           |           | radau|legendre                   |
    %+---------------------------+-----------+----------------------------------+
    %| interpolation_order       | OT_INT    | Order of the interpolating       |
    %|                           |           | polynomials                      |
    %+---------------------------+-----------+----------------------------------+
    %| number_of_finite_elements | OT_INT    | Target number of finite          |
    %|                           |           | elements. The actual number may  |
    %|                           |           | be higher to accommodate all     |
    %|                           |           | output times                     |
    %+---------------------------+-----------+----------------------------------+
    %| rootfinder                | OT_STRING | An implicit function solver      |
    %+---------------------------+-----------+----------------------------------+
    %| rootfinder_options        | OT_DICT   | Options to be passed to the NLP  |
    %|                           |           | Solver                           |
    %+---------------------------+-----------+----------------------------------+
    %| simplify                  | OT_BOOL   | Implement as MX  Function        |
    %|                           |           | (codegeneratable/serializable)   |
    %|                           |           | default: false                   |
    %+---------------------------+-----------+----------------------------------+
    %| simplify_options          | OT_DICT   | Any options to pass to           |
    %|                           |           | simplified form Function         |
    %|                           |           | constructor                      |
    %+---------------------------+-----------+----------------------------------+
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %rk
    %--
    %
    %
    %
    %Fixed-step explicit Runge-Kutta integrator for ODEs Currently 
    %implements 
    %RK4.
    %
    %The method is still under development
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_23a
    %
    %Joel Andersson
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_21k
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.hpp#L109
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.cpp#L109-L112
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
    %  INTEGRATOR(char name, char solver, struct:MX dae, struct opts)
    %  INTEGRATOR(char name, char solver, Function dae, struct opts)
    %  INTEGRATOR(char name, char solver, struct:SX dae, double t0, [double] tout, struct opts)
    %  INTEGRATOR(char name, char solver, struct:MX dae, double t0, [double] tout, struct opts)
    %  INTEGRATOR(char name, char solver, Function dae, double t0, [double] tout, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %
    %.............
    %
    %
  [varargout{1:nargout}] = casadiMEX(804, varargin{:});
end
