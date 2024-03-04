function varargout = nlpsol(varargin)
    %NLPSOL [INTERNAL] 
    %
    %  Function = NLPSOL(char name, char solver, Importer compiler, struct opts)
    %  Function = NLPSOL(char name, char solver, NlpBuilder nl, struct opts)
    %  Function = NLPSOL(char name, char solver, struct:SX nlp, struct opts)
    %  Function = NLPSOL(char name, char solver, struct:MX nlp, struct opts)
    %  Function = NLPSOL(char name, char solver, char fname, struct opts)
    %  Function = NLPSOL(char name, char solver, Function nlp, struct opts)
    %
    %
    %.......
    %
    %::
    %
    %  NLPSOL(char name, char solver, struct:SX nlp, struct opts)
    %
    %
    %
    %[INTERNAL]
    %
    %Create an NLP solver Creates a solver for the following parametric 
    %
    %nonlinear program (NLP):
    %
    %::
    %
    %  min          F(x, p)
    %  x
    %  
    %  subject to
    %  LBX <=   x    <= UBX
    %  LBG <= G(x, p) <= UBG
    %  p  == P
    %  
    %  nx: number of decision variables
    %  ng: number of constraints
    %  np: number of parameters
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
    %| bound_consistenc | OT_BOOL         | Ensure that      | casadi::Nlpsol   |
    %| y                |                 | primal-dual      |                  |
    %|                  |                 | solution is      |                  |
    %|                  |                 | consistent with  |                  |
    %|                  |                 | the bounds       |                  |
    %+------------------+-----------------+------------------+------------------+
    %| cache            | OT_DICT         | Prepopulate the  | casadi::Function |
    %|                  |                 | function cache.  | Internal         |
    %|                  |                 | Default: empty   |                  |
    %+------------------+-----------------+------------------+------------------+
    %| calc_f           | OT_BOOL         | Calculate 'f' in | casadi::Nlpsol   |
    %|                  |                 | the Nlpsol base  |                  |
    %|                  |                 | class            |                  |
    %+------------------+-----------------+------------------+------------------+
    %| calc_g           | OT_BOOL         | Calculate 'g' in | casadi::Nlpsol   |
    %|                  |                 | the Nlpsol base  |                  |
    %|                  |                 | class            |                  |
    %+------------------+-----------------+------------------+------------------+
    %| calc_lam_p       | OT_BOOL         | Calculate        | casadi::Nlpsol   |
    %|                  |                 | 'lam_p' in the   |                  |
    %|                  |                 | Nlpsol base      |                  |
    %|                  |                 | class            |                  |
    %+------------------+-----------------+------------------+------------------+
    %| calc_lam_x       | OT_BOOL         | Calculate        | casadi::Nlpsol   |
    %|                  |                 | 'lam_x' in the   |                  |
    %|                  |                 | Nlpsol base      |                  |
    %|                  |                 | class            |                  |
    %+------------------+-----------------+------------------+------------------+
    %| calc_multipliers | OT_BOOL         | Calculate        | casadi::Nlpsol   |
    %|                  |                 | Lagrange         |                  |
    %|                  |                 | multipliers in   |                  |
    %|                  |                 | the Nlpsol base  |                  |
    %|                  |                 | class            |                  |
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
    %| detect_simple_bo | OT_BOOL         | Automatically    | casadi::Nlpsol   |
    %| unds             |                 | detect simple    |                  |
    %|                  |                 | bounds (lbx/ubx) |                  |
    %|                  |                 | (default false). |                  |
    %|                  |                 | This is          |                  |
    %|                  |                 | hopefully        |                  |
    %|                  |                 | beneficial to    |                  |
    %|                  |                 | speed and        |                  |
    %|                  |                 | robustness but   |                  |
    %|                  |                 | may also have    |                  |
    %|                  |                 | adverse affects: |                  |
    %|                  |                 | 1) Subtleties in |                  |
    %|                  |                 | heuristics and   |                  |
    %|                  |                 | stopping         |                  |
    %|                  |                 | criteria may     |                  |
    %|                  |                 | change the       |                  |
    %|                  |                 | solution, 2)     |                  |
    %|                  |                 | IPOPT may lie    |                  |
    %|                  |                 | about            |                  |
    %|                  |                 | multipliers of   |                  |
    %|                  |                 | simple equality  |                  |
    %|                  |                 | bounds unless 'f |                  |
    %|                  |                 | ixed_variable_tr |                  |
    %|                  |                 | eatment' is set  |                  |
    %|                  |                 | to               |                  |
    %|                  |                 | 'relax_bounds'.  |                  |
    %+------------------+-----------------+------------------+------------------+
    %| detect_simple_bo | OT_BOOLVECTOR   | For internal use | casadi::Nlpsol   |
    %| unds_is_simple   |                 | only.            |                  |
    %+------------------+-----------------+------------------+------------------+
    %| detect_simple_bo | OT_FUNCTION     | For internal use | casadi::Nlpsol   |
    %| unds_parts       |                 | only.            |                  |
    %+------------------+-----------------+------------------+------------------+
    %| detect_simple_bo | OT_INTVECTOR    | For internal use | casadi::Nlpsol   |
    %| unds_target_x    |                 | only.            |                  |
    %+------------------+-----------------+------------------+------------------+
    %| discrete         | OT_BOOLVECTOR   | Indicates which  | casadi::Nlpsol   |
    %|                  |                 | of the variables |                  |
    %|                  |                 | are discrete,    |                  |
    %|                  |                 | i.e. integer-    |                  |
    %|                  |                 | valued           |                  |
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
    %| eval_errors_fata | OT_BOOL         | When errors      | casadi::Nlpsol   |
    %| l                |                 | occur during     |                  |
    %|                  |                 | evaluation of    |                  |
    %|                  |                 | f,g,...,stop the |                  |
    %|                  |                 | iterations       |                  |
    %+------------------+-----------------+------------------+------------------+
    %| expand           | OT_BOOL         | Replace MX with  | casadi::OracleFu |
    %|                  |                 | SX expressions   | nction           |
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
    %| ignore_check_vec | OT_BOOL         | If set to true,  | casadi::Nlpsol   |
    %|                  |                 | the input shape  |                  |
    %|                  |                 | of F will not be |                  |
    %|                  |                 | checked.         |                  |
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
    %| iteration_callba | OT_FUNCTION     | A function that  | casadi::Nlpsol   |
    %| ck               |                 | will be called   |                  |
    %|                  |                 | at each          |                  |
    %|                  |                 | iteration with   |                  |
    %|                  |                 | the solver as    |                  |
    %|                  |                 | input. Check     |                  |
    %|                  |                 | documentation of |                  |
    %|                  |                 | Callback .       |                  |
    %+------------------+-----------------+------------------+------------------+
    %| iteration_callba | OT_BOOL         | If set to true,  | casadi::Nlpsol   |
    %| ck_ignore_errors |                 | errors thrown by |                  |
    %|                  |                 | iteration_callba |                  |
    %|                  |                 | ck will be       |                  |
    %|                  |                 | ignored.         |                  |
    %+------------------+-----------------+------------------+------------------+
    %| iteration_callba | OT_INT          | Only call the    | casadi::Nlpsol   |
    %| ck_step          |                 | callback         |                  |
    %|                  |                 | function every   |                  |
    %|                  |                 | few iterations.  |                  |
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
    %| min_lam          | OT_DOUBLE       | Minimum allowed  | casadi::Nlpsol   |
    %|                  |                 | multiplier value |                  |
    %+------------------+-----------------+------------------+------------------+
    %| monitor          | OT_STRINGVECTOR | Set of user      | casadi::OracleFu |
    %|                  |                 | problem          | nction           |
    %|                  |                 | functions to be  |                  |
    %|                  |                 | monitored        |                  |
    %+------------------+-----------------+------------------+------------------+
    %| never_inline     | OT_BOOL         | Forbid inlining. | casadi::Function |
    %|                  |                 |                  | Internal         |
    %+------------------+-----------------+------------------+------------------+
    %| no_nlp_grad      | OT_BOOL         | Prevent the      | casadi::Nlpsol   |
    %|                  |                 | creation of the  |                  |
    %|                  |                 | 'nlp_grad'       |                  |
    %|                  |                 | function         |                  |
    %+------------------+-----------------+------------------+------------------+
    %| oracle_options   | OT_DICT         | Options to be    | casadi::Nlpsol   |
    %|                  |                 | passed to the    |                  |
    %|                  |                 | oracle function  |                  |
    %+------------------+-----------------+------------------+------------------+
    %| output_scheme    | OT_STRINGVECTOR | Deprecated       | casadi::Function |
    %|                  |                 | option (ignored) | Internal         |
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
    %| sens_linsol      | OT_STRING       | Linear solver    | casadi::Nlpsol   |
    %|                  |                 | used for         |                  |
    %|                  |                 | parametric       |                  |
    %|                  |                 | sensitivities    |                  |
    %|                  |                 | (default 'qr').  |                  |
    %+------------------+-----------------+------------------+------------------+
    %| sens_linsol_opti | OT_DICT         | Linear solver    | casadi::Nlpsol   |
    %| ons              |                 | options used for |                  |
    %|                  |                 | parametric       |                  |
    %|                  |                 | sensitivities.   |                  |
    %+------------------+-----------------+------------------+------------------+
    %| show_eval_warnin | OT_BOOL         | Show warnings    | casadi::OracleFu |
    %| gs               |                 | generated from   | nction           |
    %|                  |                 | function         |                  |
    %|                  |                 | evaluations      |                  |
    %|                  |                 | [true]           |                  |
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
    %| verbose_init     | OT_BOOL         | Print out timing | casadi::Nlpsol   |
    %|                  |                 | information      |                  |
    %|                  |                 | about the        |                  |
    %|                  |                 | different stages |                  |
    %|                  |                 | of               |                  |
    %|                  |                 | initialization   |                  |
    %+------------------+-----------------+------------------+------------------+
    %| warn_initial_bou | OT_BOOL         | Warn if the      | casadi::Nlpsol   |
    %| nds              |                 | initial guess    |                  |
    %|                  |                 | does not satisfy |                  |
    %|                  |                 | LBX and UBX      |                  |
    %+------------------+-----------------+------------------+------------------+
    %
    %>Input scheme: casadi::NlpsolInput (NLPSOL_NUM_IN = 8)
    %
    %+---------------+--------+-------------------------------------------------+
    %|   Full name   | Short  |                   Description                   |
    %+===============+========+=================================================+
    %| NLPSOL_X0     | x0     | Decision variables, initial guess (nx x 1)      |
    %+---------------+--------+-------------------------------------------------+
    %| NLPSOL_P      | p      | Value of fixed parameters (np x 1)              |
    %+---------------+--------+-------------------------------------------------+
    %| NLPSOL_LBX    | lbx    | Decision variables lower bound (nx x 1),        |
    %|               |        | default -inf.                                   |
    %+---------------+--------+-------------------------------------------------+
    %| NLPSOL_UBX    | ubx    | Decision variables upper bound (nx x 1),        |
    %|               |        | default +inf.                                   |
    %+---------------+--------+-------------------------------------------------+
    %| NLPSOL_LBG    | lbg    | Constraints lower bound (ng x 1), default -inf. |
    %+---------------+--------+-------------------------------------------------+
    %| NLPSOL_UBG    | ubg    | Constraints upper bound (ng x 1), default +inf. |
    %+---------------+--------+-------------------------------------------------+
    %| NLPSOL_LAM_X0 | lam_x0 | Lagrange multipliers for bounds on X, initial   |
    %|               |        | guess (nx x 1)                                  |
    %+---------------+--------+-------------------------------------------------+
    %| NLPSOL_LAM_G0 | lam_g0 | Lagrange multipliers for bounds on G, initial   |
    %|               |        | guess (ng x 1)                                  |
    %+---------------+--------+-------------------------------------------------+
    %
    %>Output scheme: casadi::NlpsolOutput (NLPSOL_NUM_OUT = 6)
    %
    %+--------------+-------+---------------------------------------------------+
    %|  Full name   | Short |                    Description                    |
    %+==============+=======+===================================================+
    %| NLPSOL_X     | x     | Decision variables at the optimal solution (nx x  |
    %|              |       | 1)                                                |
    %+--------------+-------+---------------------------------------------------+
    %| NLPSOL_F     | f     | Cost function value at the optimal solution (1 x  |
    %|              |       | 1)                                                |
    %+--------------+-------+---------------------------------------------------+
    %| NLPSOL_G     | g     | Constraints function at the optimal solution (ng  |
    %|              |       | x 1)                                              |
    %+--------------+-------+---------------------------------------------------+
    %| NLPSOL_LAM_X | lam_x | Lagrange multipliers for bounds on X at the       |
    %|              |       | solution (nx x 1)                                 |
    %+--------------+-------+---------------------------------------------------+
    %| NLPSOL_LAM_G | lam_g | Lagrange multipliers for bounds on G at the       |
    %|              |       | solution (ng x 1)                                 |
    %+--------------+-------+---------------------------------------------------+
    %| NLPSOL_LAM_P | lam_p | Lagrange multipliers for bounds on P at the       |
    %|              |       | solution (np x 1)                                 |
    %+--------------+-------+---------------------------------------------------+
    %
    %List of plugins
    %- AmplInterface
    %
    %- blocksqp
    %
    %- bonmin
    %
    %- ipopt
    %
    %- knitro
    %
    %- snopt
    %
    %- worhp
    %
    %- feasiblesqpmethod
    %
    %- qrsqp
    %
    %- scpgen
    %
    %- sqpmethod
    %
    %Note: some of the plugins in this list might not be available on your 
    %
    %system.  Also, there might be extra plugins available to you that are 
    %not 
    %listed here. You can obtain their documentation with  
    %Nlpsol.doc("myextraplugin")
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %AmplInterface
    %-------------
    %
    %
    %
    %>List of available options
    %
    %+--------+-----------+--------------------+
    %|   Id   |   Type    |    Description     |
    %+========+===========+====================+
    %| solver | OT_STRING | AMPL solver binary |
    %+--------+-----------+--------------------+
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %blocksqp
    %--------
    %
    %
    %
    %This is a modified version of blockSQP by Janka et al.
    %
    %Dennis Janka, Joel Andersson
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_224
    %
    %>List of available options
    %
    %+----------------------------+-----------+---------------------------------+
    %|             Id             |   Type    |           Description           |
    %+============================+===========+=================================+
    %| block_hess                 | OT_INT    | Blockwise Hessian               |
    %|                            |           | approximation?                  |
    %+----------------------------+-----------+---------------------------------+
    %| col_eps                    | OT_DOUBLE | Epsilon for COL scaling         |
    %|                            |           | strategy                        |
    %+----------------------------+-----------+---------------------------------+
    %| col_tau1                   | OT_DOUBLE | tau1 for COL scaling strategy   |
    %+----------------------------+-----------+---------------------------------+
    %| col_tau2                   | OT_DOUBLE | tau2 for COL scaling strategy   |
    %+----------------------------+-----------+---------------------------------+
    %| conv_strategy              | OT_INT    | Convexification strategy        |
    %+----------------------------+-----------+---------------------------------+
    %| delta                      | OT_DOUBLE | Filter line search parameter,   |
    %|                            |           | cf. IPOPT paper                 |
    %+----------------------------+-----------+---------------------------------+
    %| delta_h0                   | OT_DOUBLE | Filter line search parameter,   |
    %|                            |           | cf. IPOPT paper                 |
    %+----------------------------+-----------+---------------------------------+
    %| eps                        | OT_DOUBLE | Values smaller than this are    |
    %|                            |           | regarded as numerically zero    |
    %+----------------------------+-----------+---------------------------------+
    %| eta                        | OT_DOUBLE | Filter line search parameter,   |
    %|                            |           | cf. IPOPT paper                 |
    %+----------------------------+-----------+---------------------------------+
    %| fallback_scaling           | OT_INT    | If indefinite update is used,   |
    %|                            |           | the type of fallback strategy   |
    %+----------------------------+-----------+---------------------------------+
    %| fallback_update            | OT_INT    | If indefinite update is used,   |
    %|                            |           | the type of fallback strategy   |
    %+----------------------------+-----------+---------------------------------+
    %| gamma_f                    | OT_DOUBLE | Filter line search parameter,   |
    %|                            |           | cf. IPOPT paper                 |
    %+----------------------------+-----------+---------------------------------+
    %| gamma_theta                | OT_DOUBLE | Filter line search parameter,   |
    %|                            |           | cf. IPOPT paper                 |
    %+----------------------------+-----------+---------------------------------+
    %| globalization              | OT_BOOL   | Enable globalization            |
    %+----------------------------+-----------+---------------------------------+
    %| hess_damp                  | OT_INT    | Activate Powell damping for     |
    %|                            |           | BFGS                            |
    %+----------------------------+-----------+---------------------------------+
    %| hess_damp_fac              | OT_DOUBLE | Damping factor for BFGS Powell  |
    %|                            |           | modification                    |
    %+----------------------------+-----------+---------------------------------+
    %| hess_lim_mem               | OT_INT    | Full or limited memory          |
    %+----------------------------+-----------+---------------------------------+
    %| hess_memsize               | OT_INT    | Memory size for L-BFGS updates  |
    %+----------------------------+-----------+---------------------------------+
    %| hess_scaling               | OT_INT    | Scaling strategy for Hessian    |
    %|                            |           | approximation                   |
    %+----------------------------+-----------+---------------------------------+
    %| hess_update                | OT_INT    | Type of Hessian approximation   |
    %+----------------------------+-----------+---------------------------------+
    %| ini_hess_diag              | OT_DOUBLE | Initial Hessian guess: diagonal |
    %|                            |           | matrix diag(iniHessDiag)        |
    %+----------------------------+-----------+---------------------------------+
    %| kappa_f                    | OT_DOUBLE | Filter line search parameter,   |
    %|                            |           | cf. IPOPT paper                 |
    %+----------------------------+-----------+---------------------------------+
    %| kappa_minus                | OT_DOUBLE | Filter line search parameter,   |
    %|                            |           | cf. IPOPT paper                 |
    %+----------------------------+-----------+---------------------------------+
    %| kappa_plus                 | OT_DOUBLE | Filter line search parameter,   |
    %|                            |           | cf. IPOPT paper                 |
    %+----------------------------+-----------+---------------------------------+
    %| kappa_plus_max             | OT_DOUBLE | Filter line search parameter,   |
    %|                            |           | cf. IPOPT paper                 |
    %+----------------------------+-----------+---------------------------------+
    %| kappa_soc                  | OT_DOUBLE | Filter line search parameter,   |
    %|                            |           | cf. IPOPT paper                 |
    %+----------------------------+-----------+---------------------------------+
    %| linsol                     | OT_STRING | The linear solver to be used by |
    %|                            |           | the QP method                   |
    %+----------------------------+-----------+---------------------------------+
    %| max_consec_reduced_steps   | OT_INT    | Maximum number of consecutive   |
    %|                            |           | reduced steps                   |
    %+----------------------------+-----------+---------------------------------+
    %| max_consec_skipped_updates | OT_INT    | Maximum number of consecutive   |
    %|                            |           | skipped updates                 |
    %+----------------------------+-----------+---------------------------------+
    %| max_conv_qp                | OT_INT    | How many additional QPs may be  |
    %|                            |           | solved for convexification per  |
    %|                            |           | iteration?                      |
    %+----------------------------+-----------+---------------------------------+
    %| max_it_qp                  | OT_INT    | Maximum number of QP iterations |
    %|                            |           | per SQP iteration               |
    %+----------------------------+-----------+---------------------------------+
    %| max_iter                   | OT_INT    | Maximum number of SQP           |
    %|                            |           | iterations                      |
    %+----------------------------+-----------+---------------------------------+
    %| max_line_search            | OT_INT    | Maximum number of steps in line |
    %|                            |           | search                          |
    %+----------------------------+-----------+---------------------------------+
    %| max_soc_iter               | OT_INT    | Maximum number of SOC line      |
    %|                            |           | search iterations               |
    %+----------------------------+-----------+---------------------------------+
    %| max_time_qp                | OT_DOUBLE | Maximum number of time in       |
    %|                            |           | seconds per QP solve per SQP    |
    %|                            |           | iteration                       |
    %+----------------------------+-----------+---------------------------------+
    %| nlinfeastol                | OT_DOUBLE | Nonlinear feasibility tolerance |
    %+----------------------------+-----------+---------------------------------+
    %| obj_lo                     | OT_DOUBLE | Lower bound on objective        |
    %|                            |           | function [-inf]                 |
    %+----------------------------+-----------+---------------------------------+
    %| obj_up                     | OT_DOUBLE | Upper bound on objective        |
    %|                            |           | function [inf]                  |
    %+----------------------------+-----------+---------------------------------+
    %| opttol                     | OT_DOUBLE | Optimality tolerance            |
    %+----------------------------+-----------+---------------------------------+
    %| print_header               | OT_BOOL   | Print solver header at startup  |
    %+----------------------------+-----------+---------------------------------+
    %| print_iteration            | OT_BOOL   | Print SQP iterations            |
    %+----------------------------+-----------+---------------------------------+
    %| print_maxit_reached        | OT_BOOL   | Print error when maximum number |
    %|                            |           | of SQP iterations reached       |
    %+----------------------------+-----------+---------------------------------+
    %| qp_init                    | OT_BOOL   | Use warmstarting                |
    %+----------------------------+-----------+---------------------------------+
    %| qpsol                      | OT_STRING | The QP solver to be used by the |
    %|                            |           | SQP method                      |
    %+----------------------------+-----------+---------------------------------+
    %| qpsol_options              | OT_DICT   | Options to be passed to the QP  |
    %|                            |           | solver                          |
    %+----------------------------+-----------+---------------------------------+
    %| restore_feas               | OT_BOOL   | Use feasibility restoration     |
    %|                            |           | phase                           |
    %+----------------------------+-----------+---------------------------------+
    %| rho                        | OT_DOUBLE | Feasibility restoration phase   |
    %|                            |           | parameter                       |
    %+----------------------------+-----------+---------------------------------+
    %| s_f                        | OT_DOUBLE | Filter line search parameter,   |
    %|                            |           | cf. IPOPT paper                 |
    %+----------------------------+-----------+---------------------------------+
    %| s_theta                    | OT_DOUBLE | Filter line search parameter,   |
    %|                            |           | cf. IPOPT paper                 |
    %+----------------------------+-----------+---------------------------------+
    %| schur                      | OT_BOOL   | Use qpOASES Schur compliment    |
    %|                            |           | approach                        |
    %+----------------------------+-----------+---------------------------------+
    %| skip_first_globalization   | OT_BOOL   | No globalization strategy in    |
    %|                            |           | first iteration                 |
    %+----------------------------+-----------+---------------------------------+
    %| theta_max                  | OT_DOUBLE | Filter line search parameter,   |
    %|                            |           | cf. IPOPT paper                 |
    %+----------------------------+-----------+---------------------------------+
    %| theta_min                  | OT_DOUBLE | Filter line search parameter,   |
    %|                            |           | cf. IPOPT paper                 |
    %+----------------------------+-----------+---------------------------------+
    %| warmstart                  | OT_BOOL   | Use warmstarting                |
    %+----------------------------+-----------+---------------------------------+
    %| which_second_derv          | OT_INT    | For which block should second   |
    %|                            |           | derivatives be provided by the  |
    %|                            |           | user                            |
    %+----------------------------+-----------+---------------------------------+
    %| zeta                       | OT_DOUBLE | Feasibility restoration phase   |
    %|                            |           | parameter                       |
    %+----------------------------+-----------+---------------------------------+
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %bonmin
    %------
    %
    %
    %
    %When in warmstart mode, output NLPSOL_LAM_X may be used as input
    %
    %NOTE: Even when max_iter == 0, it is not guaranteed that 
    %input(NLPSOL_X0) 
    %== output(NLPSOL_X). Indeed if bounds on X or 
    %constraints are unmet, they 
    %will differ.
    %
    %For a good tutorial on BONMIN, see 
    %http://drops.dagstuhl.de/volltexte/2009/2089/pdf/09061.WaechterAndreas.Paper.2089.pdf
    %
    %A good resource about the algorithms in BONMIN is: Wachter and L. T. 
    %
    %Biegler, On the Implementation of an Interior-Point Filter Line-Search
    % 
    %Algorithm for Large-Scale Nonlinear Programming, Mathematical 
    %Programming 
    %106(1), pp. 25-57, 2006 (As Research Report RC 23149, IBM 
    %T. J. Watson 
    %Research Center, Yorktown, USA
    %
    %Caveats:
    %with default options, multipliers for the decision variables are 
    %wrong
    % for equality constraints. Change the 'fixed_variable_treatment' to 
    %
    %'make_constraint' or 'relax_bounds' to obtain correct results.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_223
    %
    %>List of available options
    %
    %+-------------------------+-----------------------+------------------------+
    %|           Id            |         Type          |      Description       |
    %+=========================+=======================+========================+
    %| bonmin                  | OT_DICT               | Options to be passed   |
    %|                         |                       | to BONMIN              |
    %+-------------------------+-----------------------+------------------------+
    %| con_integer_md          | OT_DICT               | Integer metadata (a    |
    %|                         |                       | dictionary with lists  |
    %|                         |                       | of integers) about     |
    %|                         |                       | constraints to be      |
    %|                         |                       | passed to BONMIN       |
    %+-------------------------+-----------------------+------------------------+
    %| con_numeric_md          | OT_DICT               | Numeric metadata (a    |
    %|                         |                       | dictionary with lists  |
    %|                         |                       | of reals) about        |
    %|                         |                       | constraints to be      |
    %|                         |                       | passed to BONMIN       |
    %+-------------------------+-----------------------+------------------------+
    %| con_string_md           | OT_DICT               | String metadata (a     |
    %|                         |                       | dictionary with lists  |
    %|                         |                       | of strings) about      |
    %|                         |                       | constraints to be      |
    %|                         |                       | passed to BONMIN       |
    %+-------------------------+-----------------------+------------------------+
    %| grad_f                  | OT_FUNCTION           | Function for           |
    %|                         |                       | calculating the        |
    %|                         |                       | gradient of the        |
    %|                         |                       | objective (column,     |
    %|                         |                       | autogenerated by       |
    %|                         |                       | default)               |
    %+-------------------------+-----------------------+------------------------+
    %| grad_f_options          | OT_DICT               | Options for the        |
    %|                         |                       | autogenerated gradient |
    %|                         |                       | of the objective.      |
    %+-------------------------+-----------------------+------------------------+
    %| hess_lag                | OT_FUNCTION           | Function for           |
    %|                         |                       | calculating the        |
    %|                         |                       | Hessian of the         |
    %|                         |                       | Lagrangian             |
    %|                         |                       | (autogenerated by      |
    %|                         |                       | default)               |
    %+-------------------------+-----------------------+------------------------+
    %| hess_lag_options        | OT_DICT               | Options for the        |
    %|                         |                       | autogenerated Hessian  |
    %|                         |                       | of the Lagrangian.     |
    %+-------------------------+-----------------------+------------------------+
    %| jac_g                   | OT_FUNCTION           | Function for           |
    %|                         |                       | calculating the        |
    %|                         |                       | Jacobian of the        |
    %|                         |                       | constraints            |
    %|                         |                       | (autogenerated by      |
    %|                         |                       | default)               |
    %+-------------------------+-----------------------+------------------------+
    %| jac_g_options           | OT_DICT               | Options for the        |
    %|                         |                       | autogenerated Jacobian |
    %|                         |                       | of the constraints.    |
    %+-------------------------+-----------------------+------------------------+
    %| pass_nonlinear_constrai | OT_BOOL               | Pass list of           |
    %| nts                     |                       | constraints entering   |
    %|                         |                       | nonlinearly to BONMIN  |
    %+-------------------------+-----------------------+------------------------+
    %| pass_nonlinear_variable | OT_BOOL               | Pass list of variables |
    %| s                       |                       | entering nonlinearly   |
    %|                         |                       | to BONMIN              |
    %+-------------------------+-----------------------+------------------------+
    %| sos1_groups             | OT_INTVECTORVECTOR    | Options for the        |
    %|                         |                       | autogenerated gradient |
    %|                         |                       | of the objective.      |
    %+-------------------------+-----------------------+------------------------+
    %| sos1_priorities         | OT_INTVECTOR          | Options for the        |
    %|                         |                       | autogenerated gradient |
    %|                         |                       | of the objective.      |
    %+-------------------------+-----------------------+------------------------+
    %| sos1_weights            | OT_DOUBLEVECTORVECTOR | Options for the        |
    %|                         |                       | autogenerated gradient |
    %|                         |                       | of the objective.      |
    %+-------------------------+-----------------------+------------------------+
    %| var_integer_md          | OT_DICT               | Integer metadata (a    |
    %|                         |                       | dictionary with lists  |
    %|                         |                       | of integers) about     |
    %|                         |                       | variables to be passed |
    %|                         |                       | to BONMIN              |
    %+-------------------------+-----------------------+------------------------+
    %| var_numeric_md          | OT_DICT               | Numeric metadata (a    |
    %|                         |                       | dictionary with lists  |
    %|                         |                       | of reals) about        |
    %|                         |                       | variables to be passed |
    %|                         |                       | to BONMIN              |
    %+-------------------------+-----------------------+------------------------+
    %| var_string_md           | OT_DICT               | String metadata (a     |
    %|                         |                       | dictionary with lists  |
    %|                         |                       | of strings) about      |
    %|                         |                       | variables to be passed |
    %|                         |                       | to BONMIN              |
    %+-------------------------+-----------------------+------------------------+
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %ipopt
    %-----
    %
    %
    %
    %When in warmstart mode, output NLPSOL_LAM_X may be used as input
    %
    %NOTE: Even when max_iter == 0, it is not guaranteed that 
    %input(NLPSOL_X0) 
    %== output(NLPSOL_X). Indeed if bounds on X or 
    %constraints are unmet, they 
    %will differ.
    %
    %For a good tutorial on IPOPT, see 
    %http://drops.dagstuhl.de/volltexte/2009/2089/pdf/09061.WaechterAndreas.Paper.2089.pdf
    %
    %A good resource about the algorithms in IPOPT is: Wachter and L. T. 
    %
    %Biegler, On the Implementation of an Interior-Point Filter Line-Search
    % 
    %Algorithm for Large-Scale Nonlinear Programming, Mathematical 
    %Programming 
    %106(1), pp. 25-57, 2006 (As Research Report RC 23149, IBM 
    %T. J. Watson 
    %Research Center, Yorktown, USA
    %
    %Caveats:
    %with default options, multipliers for the decision variables are 
    %wrong
    % for equality constraints. Change the 'fixed_variable_treatment' to 
    %
    %'make_constraint' or 'relax_bounds' to obtain correct results.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_21y
    %
    %>List of available options
    %
    %+--------------------------+-------------+---------------------------------+
    %|            Id            |    Type     |           Description           |
    %+==========================+=============+=================================+
    %| clip_inactive_lam        | OT_BOOL     | Explicitly set Lagrange         |
    %|                          |             | multipliers to 0 when bound is  |
    %|                          |             | deemed inactive (default:       |
    %|                          |             | false).                         |
    %+--------------------------+-------------+---------------------------------+
    %| con_integer_md           | OT_DICT     | Integer metadata (a dictionary  |
    %|                          |             | with lists of integers) about   |
    %|                          |             | constraints to be passed to     |
    %|                          |             | IPOPT                           |
    %+--------------------------+-------------+---------------------------------+
    %| con_numeric_md           | OT_DICT     | Numeric metadata (a dictionary  |
    %|                          |             | with lists of reals) about      |
    %|                          |             | constraints to be passed to     |
    %|                          |             | IPOPT                           |
    %+--------------------------+-------------+---------------------------------+
    %| con_string_md            | OT_DICT     | String metadata (a dictionary   |
    %|                          |             | with lists of strings) about    |
    %|                          |             | constraints to be passed to     |
    %|                          |             | IPOPT                           |
    %+--------------------------+-------------+---------------------------------+
    %| convexify_margin         | OT_DOUBLE   | When using a convexification    |
    %|                          |             | strategy, make sure that the    |
    %|                          |             | smallest eigenvalue is at least |
    %|                          |             | this (default: 1e-7).           |
    %+--------------------------+-------------+---------------------------------+
    %| convexify_strategy       | OT_STRING   | NONE|regularize|eigen-          |
    %|                          |             | reflect|eigen-clip. Strategy to |
    %|                          |             | convexify the Lagrange Hessian  |
    %|                          |             | before passing it to the        |
    %|                          |             | solver.                         |
    %+--------------------------+-------------+---------------------------------+
    %| grad_f                   | OT_FUNCTION | Function for calculating the    |
    %|                          |             | gradient of the objective       |
    %|                          |             | (column, autogenerated by       |
    %|                          |             | default)                        |
    %+--------------------------+-------------+---------------------------------+
    %| hess_lag                 | OT_FUNCTION | Function for calculating the    |
    %|                          |             | Hessian of the Lagrangian       |
    %|                          |             | (autogenerated by default)      |
    %+--------------------------+-------------+---------------------------------+
    %| inactive_lam_strategy    | OT_STRING   | Strategy to detect if a bound   |
    %|                          |             | is inactive. RELTOL: use        |
    %|                          |             | solver-defined constraint       |
    %|                          |             | tolerance *                     |
    %|                          |             | inactive_lam_value|abstol: use  |
    %|                          |             | inactive_lam_value              |
    %+--------------------------+-------------+---------------------------------+
    %| inactive_lam_value       | OT_DOUBLE   | Value used in                   |
    %|                          |             | inactive_lam_strategy (default: |
    %|                          |             | 10).                            |
    %+--------------------------+-------------+---------------------------------+
    %| ipopt                    | OT_DICT     | Options to be passed to IPOPT   |
    %+--------------------------+-------------+---------------------------------+
    %| jac_g                    | OT_FUNCTION | Function for calculating the    |
    %|                          |             | Jacobian of the constraints     |
    %|                          |             | (autogenerated by default)      |
    %+--------------------------+-------------+---------------------------------+
    %| max_iter_eig             | OT_DOUBLE   | Maximum number of iterations to |
    %|                          |             | compute an eigenvalue           |
    %|                          |             | decomposition (default: 50).    |
    %+--------------------------+-------------+---------------------------------+
    %| pass_nonlinear_variables | OT_BOOL     | Pass list of variables entering |
    %|                          |             | nonlinearly to IPOPT            |
    %+--------------------------+-------------+---------------------------------+
    %| var_integer_md           | OT_DICT     | Integer metadata (a dictionary  |
    %|                          |             | with lists of integers) about   |
    %|                          |             | variables to be passed to IPOPT |
    %+--------------------------+-------------+---------------------------------+
    %| var_numeric_md           | OT_DICT     | Numeric metadata (a dictionary  |
    %|                          |             | with lists of reals) about      |
    %|                          |             | variables to be passed to IPOPT |
    %+--------------------------+-------------+---------------------------------+
    %| var_string_md            | OT_DICT     | String metadata (a dictionary   |
    %|                          |             | with lists of strings) about    |
    %|                          |             | variables to be passed to IPOPT |
    %+--------------------------+-------------+---------------------------------+
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %knitro
    %------
    %
    %
    %
    %KNITRO interface
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_22c
    %
    %>List of available options
    %
    %+--------------------------+--------------------+--------------------------+
    %|            Id            |        Type        |       Description        |
    %+==========================+====================+==========================+
    %| complem_variables        | OT_INTVECTORVECTOR | List of complementary    |
    %|                          |                    | constraints on simple    |
    %|                          |                    | bounds. Pair (i, j)      |
    %|                          |                    | encodes complementarity  |
    %|                          |                    | between the bounds on    |
    %|                          |                    | variable i and variable  |
    %|                          |                    | j.                       |
    %+--------------------------+--------------------+--------------------------+
    %| contype                  | OT_INTVECTOR       | Type of constraint       |
    %+--------------------------+--------------------+--------------------------+
    %| detect_linear_constraint | OT_BOOL            | Detect type of           |
    %| s                        |                    | constraints              |
    %+--------------------------+--------------------+--------------------------+
    %| knitro                   | OT_DICT            | Options to be passed to  |
    %|                          |                    | KNITRO                   |
    %+--------------------------+--------------------+--------------------------+
    %| options_file             | OT_STRING          | Read options from file   |
    %|                          |                    | (solver specific)        |
    %+--------------------------+--------------------+--------------------------+
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %snopt
    %-----
    %
    %
    %
    %SNOPT interface
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_22m
    %
    %>List of available options
    %
    %+-------+-----------+---------------------------------------------+
    %|  Id   |   Type    |                 Description                 |
    %+=======+===========+=============================================+
    %| snopt | OT_DICT   | Options to be passed to SNOPT               |
    %+-------+-----------+---------------------------------------------+
    %| start | OT_STRING | Warm-start options for Worhp: cold|warm|hot |
    %+-------+-----------+---------------------------------------------+
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %worhp
    %-----
    %
    %
    %
    %WORHP interface
    %
    %Designed for Worhp 1.12
    %
    %>List of available options
    %
    %+-------+---------+-------------------------------+
    %|  Id   |  Type   |          Description          |
    %+=======+=========+===============================+
    %| worhp | OT_DICT | Options to be passed to WORHP |
    %+-------+---------+-------------------------------+
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %feasiblesqpmethod
    %-----------------
    %
    %
    %
    %A textbook FeasibleSQPMethod
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_241
    %
    %>List of available options
    %
    %+----------------------------+-----------------+---------------------------+
    %|             Id             |      Type       |        Description        |
    %+============================+=================+===========================+
    %| anderson_memory            | OT_INT          | Anderson memory. If       |
    %|                            |                 | Anderson is used default  |
    %|                            |                 | is 1, else default is 0.  |
    %+----------------------------+-----------------+---------------------------+
    %| contraction_acceptance_val | OT_DOUBLE       | If the empirical          |
    %| ue                         |                 | contraction rate in the   |
    %|                            |                 | feasibility iterations is |
    %|                            |                 | above this value in the   |
    %|                            |                 | heuristics the iterations |
    %|                            |                 | are aborted.              |
    %+----------------------------+-----------------+---------------------------+
    %| convexify_margin           | OT_DOUBLE       | When using a              |
    %|                            |                 | convexification strategy, |
    %|                            |                 | make sure that the        |
    %|                            |                 | smallest eigenvalue4 is   |
    %|                            |                 | at least this (default:   |
    %|                            |                 | 1e-7).                    |
    %+----------------------------+-----------------+---------------------------+
    %| convexify_strategy         | OT_STRING       | NONE|regularize|eigen-    |
    %|                            |                 | reflect|eigen-clip.       |
    %|                            |                 | Strategy to convexify the |
    %|                            |                 | Lagrange Hessian before   |
    %|                            |                 | passing it to the solver. |
    %+----------------------------+-----------------+---------------------------+
    %| f                          | OT_FUNCTION     | Function for calculating  |
    %|                            |                 | the objective function    |
    %|                            |                 | (autogenerated by         |
    %|                            |                 | default)                  |
    %+----------------------------+-----------------+---------------------------+
    %| feas_tol                   | OT_DOUBLE       | Feasibility tolerance.    |
    %|                            |                 | Below this tolerance an   |
    %|                            |                 | iterate is considered to  |
    %|                            |                 | be feasible.              |
    %+----------------------------+-----------------+---------------------------+
    %| g                          | OT_FUNCTION     | Function for calculating  |
    %|                            |                 | the constraints           |
    %|                            |                 | (autogenerated by         |
    %|                            |                 | default)                  |
    %+----------------------------+-----------------+---------------------------+
    %| grad_f                     | OT_FUNCTION     | Function for calculating  |
    %|                            |                 | the gradient of the       |
    %|                            |                 | objective (autogenerated  |
    %|                            |                 | by default)               |
    %+----------------------------+-----------------+---------------------------+
    %| hess_lag                   | OT_FUNCTION     | Function for calculating  |
    %|                            |                 | the Hessian of the        |
    %|                            |                 | Lagrangian (autogenerated |
    %|                            |                 | by default)               |
    %+----------------------------+-----------------+---------------------------+
    %| hessian_approximation      | OT_STRING       | limited-memory|exact      |
    %+----------------------------+-----------------+---------------------------+
    %| init_feasible              | OT_BOOL         | Initialize the QP         |
    %|                            |                 | subproblems with a        |
    %|                            |                 | feasible initial value    |
    %|                            |                 | (default: false).         |
    %+----------------------------+-----------------+---------------------------+
    %| jac_g                      | OT_FUNCTION     | Function for calculating  |
    %|                            |                 | the Jacobian of the       |
    %|                            |                 | constraints               |
    %|                            |                 | (autogenerated by         |
    %|                            |                 | default)                  |
    %+----------------------------+-----------------+---------------------------+
    %| lbfgs_memory               | OT_INT          | Size of L-BFGS memory.    |
    %+----------------------------+-----------------+---------------------------+
    %| max_inner_iter             | OT_DOUBLE       | Maximum number of inner   |
    %|                            |                 | iterations.               |
    %+----------------------------+-----------------+---------------------------+
    %| max_iter                   | OT_INT          | Maximum number of SQP     |
    %|                            |                 | iterations                |
    %+----------------------------+-----------------+---------------------------+
    %| max_iter_eig               | OT_DOUBLE       | Maximum number of         |
    %|                            |                 | iterations to compute an  |
    %|                            |                 | eigenvalue decomposition  |
    %|                            |                 | (default: 50).            |
    %+----------------------------+-----------------+---------------------------+
    %| merit_memory               | OT_INT          | Size of memory to store   |
    %|                            |                 | history of merit function |
    %|                            |                 | values                    |
    %+----------------------------+-----------------+---------------------------+
    %| min_iter                   | OT_INT          | Minimum number of SQP     |
    %|                            |                 | iterations                |
    %+----------------------------+-----------------+---------------------------+
    %| optim_tol                  | OT_DOUBLE       | Optimality tolerance.     |
    %|                            |                 | Below this value an       |
    %|                            |                 | iterate is considered to  |
    %|                            |                 | be optimal.               |
    %+----------------------------+-----------------+---------------------------+
    %| print_header               | OT_BOOL         | Print the header with     |
    %|                            |                 | problem statistics        |
    %+----------------------------+-----------------+---------------------------+
    %| print_iteration            | OT_BOOL         | Print the iterations      |
    %+----------------------------+-----------------+---------------------------+
    %| print_status               | OT_BOOL         | Print a status message    |
    %|                            |                 | after solving             |
    %+----------------------------+-----------------+---------------------------+
    %| qpsol                      | OT_STRING       | The QP solver to be used  |
    %|                            |                 | by the SQP method         |
    %|                            |                 | [qpoases]                 |
    %+----------------------------+-----------------+---------------------------+
    %| qpsol_options              | OT_DICT         | Options to be passed to   |
    %|                            |                 | the QP solver             |
    %+----------------------------+-----------------+---------------------------+
    %| solve_type                 | OT_STRING       | The solver type: Either   |
    %|                            |                 | SQP or SLP. Defaults to   |
    %|                            |                 | SQP                       |
    %+----------------------------+-----------------+---------------------------+
    %| tol_du                     | OT_DOUBLE       | Stopping criterion for    |
    %|                            |                 | dual infeasability        |
    %+----------------------------+-----------------+---------------------------+
    %| tol_pr                     | OT_DOUBLE       | Stopping criterion for    |
    %|                            |                 | primal infeasibility      |
    %+----------------------------+-----------------+---------------------------+
    %| tr_acceptance              | OT_DOUBLE       | Is the trust-region ratio |
    %|                            |                 | above this value, the     |
    %|                            |                 | step is accepted.         |
    %+----------------------------+-----------------+---------------------------+
    %| tr_alpha1                  | OT_DOUBLE       | Lower alpha in trust-     |
    %|                            |                 | region size criterion.    |
    %+----------------------------+-----------------+---------------------------+
    %| tr_alpha2                  | OT_DOUBLE       | Upper alpha in trust-     |
    %|                            |                 | region size criterion.    |
    %+----------------------------+-----------------+---------------------------+
    %| tr_eta1                    | OT_DOUBLE       | Lower eta in trust-region |
    %|                            |                 | acceptance criterion.     |
    %+----------------------------+-----------------+---------------------------+
    %| tr_eta2                    | OT_DOUBLE       | Upper eta in trust-region |
    %|                            |                 | acceptance criterion.     |
    %+----------------------------+-----------------+---------------------------+
    %| tr_rad0                    | OT_DOUBLE       | Initial trust-region      |
    %|                            |                 | radius.                   |
    %+----------------------------+-----------------+---------------------------+
    %| tr_rad_max                 | OT_DOUBLE       | Maximum trust-region      |
    %|                            |                 | radius.                   |
    %+----------------------------+-----------------+---------------------------+
    %| tr_rad_min                 | OT_DOUBLE       | Minimum trust-region      |
    %|                            |                 | radius.                   |
    %+----------------------------+-----------------+---------------------------+
    %| tr_scale_vector            | OT_DOUBLEVECTOR | Vector that tells where   |
    %|                            |                 | trust-region is applied.  |
    %+----------------------------+-----------------+---------------------------+
    %| tr_tol                     | OT_DOUBLE       | Trust-region tolerance.   |
    %|                            |                 | Below this value another  |
    %|                            |                 | scalar is equal to the    |
    %|                            |                 | trust region radius.      |
    %+----------------------------+-----------------+---------------------------+
    %| use_anderson               | OT_BOOL         | Use Anderson              |
    %|                            |                 | Acceleration. (default    |
    %|                            |                 | false)                    |
    %+----------------------------+-----------------+---------------------------+
    %| watchdog                   | OT_INT          | Number of watchdog        |
    %|                            |                 | iterations in feasibility |
    %|                            |                 | iterations. After this    |
    %|                            |                 | amount of iterations, it  |
    %|                            |                 | is checked with the       |
    %|                            |                 | contraction acceptance    |
    %|                            |                 | value, if iterations are  |
    %|                            |                 | converging.               |
    %+----------------------------+-----------------+---------------------------+
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %qrsqp
    %-----
    %
    %
    %
    %A textbook SQPMethod
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_22u
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %scpgen
    %------
    %
    %
    %
    %A structure-exploiting sequential quadratic programming (to be come 
    %
    %sequential convex programming) method for nonlinear programming.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_232
    %
    %>List of available options
    %
    %+-----------------------+-----------------+--------------------------------+
    %|          Id           |      Type       |          Description           |
    %+=======================+=================+================================+
    %| beta                  | OT_DOUBLE       | Line-search parameter,         |
    %|                       |                 | restoration factor of stepsize |
    %+-----------------------+-----------------+--------------------------------+
    %| c1                    | OT_DOUBLE       | Armijo condition, coefficient  |
    %|                       |                 | of decrease in merit           |
    %+-----------------------+-----------------+--------------------------------+
    %| codegen               | OT_BOOL         | C-code generation              |
    %+-----------------------+-----------------+--------------------------------+
    %| hessian_approximation | OT_STRING       | gauss-newton|exact             |
    %+-----------------------+-----------------+--------------------------------+
    %| lbfgs_memory          | OT_INT          | Size of L-BFGS memory.         |
    %+-----------------------+-----------------+--------------------------------+
    %| max_iter              | OT_INT          | Maximum number of SQP          |
    %|                       |                 | iterations                     |
    %+-----------------------+-----------------+--------------------------------+
    %| max_iter_ls           | OT_INT          | Maximum number of linesearch   |
    %|                       |                 | iterations                     |
    %+-----------------------+-----------------+--------------------------------+
    %| merit_memsize         | OT_INT          | Size of memory to store        |
    %|                       |                 | history of merit function      |
    %|                       |                 | values                         |
    %+-----------------------+-----------------+--------------------------------+
    %| merit_start           | OT_DOUBLE       | Lower bound for the merit      |
    %|                       |                 | function parameter             |
    %+-----------------------+-----------------+--------------------------------+
    %| name_x                | OT_STRINGVECTOR | Names of the variables.        |
    %+-----------------------+-----------------+--------------------------------+
    %| print_header          | OT_BOOL         | Print the header with problem  |
    %|                       |                 | statistics                     |
    %+-----------------------+-----------------+--------------------------------+
    %| print_x               | OT_INTVECTOR    | Which variables to print.      |
    %+-----------------------+-----------------+--------------------------------+
    %| qpsol                 | OT_STRING       | The QP solver to be used by    |
    %|                       |                 | the SQP method                 |
    %+-----------------------+-----------------+--------------------------------+
    %| qpsol_options         | OT_DICT         | Options to be passed to the QP |
    %|                       |                 | solver                         |
    %+-----------------------+-----------------+--------------------------------+
    %| reg_threshold         | OT_DOUBLE       | Threshold for the              |
    %|                       |                 | regularization.                |
    %+-----------------------+-----------------+--------------------------------+
    %| regularize            | OT_BOOL         | Automatic regularization of    |
    %|                       |                 | Lagrange Hessian.              |
    %+-----------------------+-----------------+--------------------------------+
    %| tol_du                | OT_DOUBLE       | Stopping criterion for dual    |
    %|                       |                 | infeasability                  |
    %+-----------------------+-----------------+--------------------------------+
    %| tol_pr                | OT_DOUBLE       | Stopping criterion for primal  |
    %|                       |                 | infeasibility                  |
    %+-----------------------+-----------------+--------------------------------+
    %| tol_pr_step           | OT_DOUBLE       | Stopping criterion for the     |
    %|                       |                 | step size                      |
    %+-----------------------+-----------------+--------------------------------+
    %| tol_reg               | OT_DOUBLE       | Stopping criterion for         |
    %|                       |                 | regularization                 |
    %+-----------------------+-----------------+--------------------------------+
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %sqpmethod
    %---------
    %
    %
    %
    %A textbook SQPMethod
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_22x
    %
    %>List of available options
    %
    %+--------------------------+-------------+---------------------------------+
    %|            Id            |    Type     |           Description           |
    %+==========================+=============+=================================+
    %| beta                     | OT_DOUBLE   | Line-search parameter,          |
    %|                          |             | restoration factor of stepsize  |
    %+--------------------------+-------------+---------------------------------+
    %| c1                       | OT_DOUBLE   | Armijo condition, coefficient   |
    %|                          |             | of decrease in merit            |
    %+--------------------------+-------------+---------------------------------+
    %| convexify_margin         | OT_DOUBLE   | When using a convexification    |
    %|                          |             | strategy, make sure that the    |
    %|                          |             | smallest eigenvalue is at least |
    %|                          |             | this (default: 1e-7).           |
    %+--------------------------+-------------+---------------------------------+
    %| convexify_strategy       | OT_STRING   | NONE|regularize|eigen-          |
    %|                          |             | reflect|eigen-clip. Strategy to |
    %|                          |             | convexify the Lagrange Hessian  |
    %|                          |             | before passing it to the        |
    %|                          |             | solver.                         |
    %+--------------------------+-------------+---------------------------------+
    %| elastic_mode             | OT_BOOL     | Enable the elastic mode which   |
    %|                          |             | is used when the QP is          |
    %|                          |             | infeasible (default: false).    |
    %+--------------------------+-------------+---------------------------------+
    %| gamma_0                  | OT_DOUBLE   | Starting value for the penalty  |
    %|                          |             | parameter of elastic mode       |
    %|                          |             | (default: 1).                   |
    %+--------------------------+-------------+---------------------------------+
    %| gamma_1_min              | OT_DOUBLE   | Minimum value for gamma_1       |
    %|                          |             | (default: 1e-5).                |
    %+--------------------------+-------------+---------------------------------+
    %| gamma_max                | OT_DOUBLE   | Maximum value for the penalty   |
    %|                          |             | parameter of elastic mode       |
    %|                          |             | (default: 1e20).                |
    %+--------------------------+-------------+---------------------------------+
    %| hess_lag                 | OT_FUNCTION | Function for calculating the    |
    %|                          |             | Hessian of the Lagrangian       |
    %|                          |             | (autogenerated by default)      |
    %+--------------------------+-------------+---------------------------------+
    %| hessian_approximation    | OT_STRING   | limited-memory|exact            |
    %+--------------------------+-------------+---------------------------------+
    %| init_feasible            | OT_BOOL     | Initialize the QP subproblems   |
    %|                          |             | with a feasible initial value   |
    %|                          |             | (default: false).               |
    %+--------------------------+-------------+---------------------------------+
    %| jac_fg                   | OT_FUNCTION | Function for calculating the    |
    %|                          |             | gradient of the objective and   |
    %|                          |             | Jacobian of the constraints     |
    %|                          |             | (autogenerated by default)      |
    %+--------------------------+-------------+---------------------------------+
    %| lbfgs_memory             | OT_INT      | Size of L-BFGS memory.          |
    %+--------------------------+-------------+---------------------------------+
    %| max_iter                 | OT_INT      | Maximum number of SQP           |
    %|                          |             | iterations                      |
    %+--------------------------+-------------+---------------------------------+
    %| max_iter_eig             | OT_DOUBLE   | Maximum number of iterations to |
    %|                          |             | compute an eigenvalue           |
    %|                          |             | decomposition (default: 50).    |
    %+--------------------------+-------------+---------------------------------+
    %| max_iter_ls              | OT_INT      | Maximum number of linesearch    |
    %|                          |             | iterations                      |
    %+--------------------------+-------------+---------------------------------+
    %| merit_memory             | OT_INT      | Size of memory to store history |
    %|                          |             | of merit function values        |
    %+--------------------------+-------------+---------------------------------+
    %| min_iter                 | OT_INT      | Minimum number of SQP           |
    %|                          |             | iterations                      |
    %+--------------------------+-------------+---------------------------------+
    %| min_step_size            | OT_DOUBLE   | The size (inf-norm) of the step |
    %|                          |             | size should not become smaller  |
    %|                          |             | than this.                      |
    %+--------------------------+-------------+---------------------------------+
    %| print_header             | OT_BOOL     | Print the header with problem   |
    %|                          |             | statistics                      |
    %+--------------------------+-------------+---------------------------------+
    %| print_iteration          | OT_BOOL     | Print the iterations            |
    %+--------------------------+-------------+---------------------------------+
    %| print_status             | OT_BOOL     | Print a status message after    |
    %|                          |             | solving                         |
    %+--------------------------+-------------+---------------------------------+
    %| qpsol                    | OT_STRING   | The QP solver to be used by the |
    %|                          |             | SQP method [qpoases]            |
    %+--------------------------+-------------+---------------------------------+
    %| qpsol_options            | OT_DICT     | Options to be passed to the QP  |
    %|                          |             | solver                          |
    %+--------------------------+-------------+---------------------------------+
    %| second_order_corrections | OT_BOOL     | Enable second order             |
    %|                          |             | corrections. These are used     |
    %|                          |             | when a step is considered bad   |
    %|                          |             | by the merit function and       |
    %|                          |             | constraint norm (default:       |
    %|                          |             | false).                         |
    %+--------------------------+-------------+---------------------------------+
    %| tol_du                   | OT_DOUBLE   | Stopping criterion for dual     |
    %|                          |             | infeasability                   |
    %+--------------------------+-------------+---------------------------------+
    %| tol_pr                   | OT_DOUBLE   | Stopping criterion for primal   |
    %|                          |             | infeasibility                   |
    %+--------------------------+-------------+---------------------------------+
    %
    %Joel Andersson
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_21q
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/nlpsol.hpp#L115
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/nlpsol.cpp#L115-L118
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
    %  NLPSOL(char name, char solver, Importer compiler, struct opts)
    %  NLPSOL(char name, char solver, NlpBuilder nl, struct opts)
    %  NLPSOL(char name, char solver, struct:MX nlp, struct opts)
    %  NLPSOL(char name, char solver, char fname, struct opts)
    %  NLPSOL(char name, char solver, Function nlp, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %
    %.............
    %
    %
  [varargout{1:nargout}] = casadiMEX(829, varargin{:});
end
