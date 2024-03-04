function varargout = rootfinder(varargin)
    %ROOTFINDER [INTERNAL] 
    %
    %  Function = ROOTFINDER(char name, char solver, struct:SX rfp, struct opts)
    %  Function = ROOTFINDER(char name, char solver, struct:MX rfp, struct opts)
    %  Function = ROOTFINDER(char name, char solver, Function f, struct opts)
    %
    %
    %.......
    %
    %::
    %
    %  ROOTFINDER(char name, char solver, struct:SX rfp, struct opts)
    %
    %
    %
    %[INTERNAL]
    %
    %Create a solver for rootfinding problems Takes a function where one 
    %of the 
    %inputs is unknown and one of the outputs is a residual function
    % that is 
    %always zero, defines a new function where the the unknown 
    %input has been 
    %replaced by a  guess for the unknown and the residual output has been 
    %replaced by the 
    %calculated value for the input.
    %
    %For a function [y0, y1, ...,yi, .., yn] = F(x0, x1, ..., xj, ..., xm),
    % 
    %where xj is unknown and yi=0, defines a new function [y0, y1, ...,xj,
    % .., 
    %yn] = G(x0, x1, ..., xj_guess, ..., xm),
    %
    %xj and yi must have the same dimension and d(yi)/d(xj) must be 
    %invertable.
    %
    %By default, the first input is unknown and the first output is the 
    %
    %residual.
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
    %| constraints      | OT_INTVECTOR    | Constrain the    | casadi::Rootfind |
    %|                  |                 | unknowns. 0      | er               |
    %|                  |                 | (default): no    |                  |
    %|                  |                 | constraint on    |                  |
    %|                  |                 | ui, 1: ui >=     |                  |
    %|                  |                 | 0.0, -1: ui <=   |                  |
    %|                  |                 | 0.0, 2: ui >     |                  |
    %|                  |                 | 0.0, -2: ui <    |                  |
    %|                  |                 | 0.0.             |                  |
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
    %| implicit_input   | OT_INT          | Index of the     | casadi::Rootfind |
    %|                  |                 | input that       | er               |
    %|                  |                 | corresponds to   |                  |
    %|                  |                 | the actual root- |                  |
    %|                  |                 | finding          |                  |
    %+------------------+-----------------+------------------+------------------+
    %| implicit_output  | OT_INT          | Index of the     | casadi::Rootfind |
    %|                  |                 | output that      | er               |
    %|                  |                 | corresponds to   |                  |
    %|                  |                 | the actual root- |                  |
    %|                  |                 | finding          |                  |
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
    %| jacobian_functio | OT_FUNCTION     | Function object  | casadi::Rootfind |
    %| n                |                 | for calculating  | er               |
    %|                  |                 | the Jacobian     |                  |
    %|                  |                 | (autogenerated   |                  |
    %|                  |                 | by default)      |                  |
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
    %| linear_solver    | OT_STRING       | User-defined     | casadi::Rootfind |
    %|                  |                 | linear solver    | er               |
    %|                  |                 | class. Needed    |                  |
    %|                  |                 | for              |                  |
    %|                  |                 | sensitivities.   |                  |
    %+------------------+-----------------+------------------+------------------+
    %| linear_solver_op | OT_DICT         | Options to be    | casadi::Rootfind |
    %| tions            |                 | passed to the    | er               |
    %|                  |                 | linear solver.   |                  |
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
    %| never_inline     | OT_BOOL         | Forbid inlining. | casadi::Function |
    %|                  |                 |                  | Internal         |
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
    %
    %>Input scheme: casadi::RootfinderInput (ROOTFINDER_NUM_IN = 2)
    %
    %+---------------+-------+---------------------------------+
    %|   Full name   | Short |           Description           |
    %+===============+=======+=================================+
    %| ROOTFINDER_X0 | x0    | Initial guess for the solution. |
    %+---------------+-------+---------------------------------+
    %| ROOTFINDER_P  | p     | Parameters.                     |
    %+---------------+-------+---------------------------------+
    %
    %>Output scheme: casadi::RootfinderOutput (ROOTFINDER_NUM_OUT = 1)
    %
    %+--------------+-------+--------------------------------------+
    %|  Full name   | Short |             Description              |
    %+==============+=======+======================================+
    %| ROOTFINDER_X | x     | Solution to the system of equations. |
    %+--------------+-------+--------------------------------------+
    %
    %List of plugins
    %- kinsol
    %
    %- fast_newton
    %
    %- nlpsol
    %
    %- newton
    %
    %Note: some of the plugins in this list might not be available on your 
    %
    %system.  Also, there might be extra plugins available to you that are 
    %not 
    %listed here. You can obtain their documentation with  
    %Rootfinder.doc("myextraplugin")
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %kinsol
    %------
    %
    %
    %
    %KINSOL interface from the Sundials suite
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_226
    %
    %>List of available options
    %
    %+---------------------------+-----------------+----------------------------+
    %|            Id             |      Type       |        Description         |
    %+===========================+=================+============================+
    %| abstol                    | OT_DOUBLE       | Stopping criterion         |
    %|                           |                 | tolerance                  |
    %+---------------------------+-----------------+----------------------------+
    %| disable_internal_warnings | OT_BOOL         | Disable KINSOL internal    |
    %|                           |                 | warning messages           |
    %+---------------------------+-----------------+----------------------------+
    %| exact_jacobian            | OT_BOOL         | Use exact Jacobian         |
    %|                           |                 | information                |
    %+---------------------------+-----------------+----------------------------+
    %| f_scale                   | OT_DOUBLEVECTOR | Equation scaling factors   |
    %+---------------------------+-----------------+----------------------------+
    %| iterative_solver          | OT_STRING       | gmres|bcgstab|tfqmr        |
    %+---------------------------+-----------------+----------------------------+
    %| linear_solver_type        | OT_STRING       | dense|banded|iterative|use |
    %|                           |                 | r_defined                  |
    %+---------------------------+-----------------+----------------------------+
    %| lower_bandwidth           | OT_INT          | Lower bandwidth for banded |
    %|                           |                 | linear solvers             |
    %+---------------------------+-----------------+----------------------------+
    %| max_iter                  | OT_INT          | Maximum number of Newton   |
    %|                           |                 | iterations. Putting 0 sets |
    %|                           |                 | the default value of       |
    %|                           |                 | KinSol.                    |
    %+---------------------------+-----------------+----------------------------+
    %| max_krylov                | OT_INT          | Maximum Krylov space       |
    %|                           |                 | dimension                  |
    %+---------------------------+-----------------+----------------------------+
    %| pretype                   | OT_STRING       | Type of preconditioner     |
    %+---------------------------+-----------------+----------------------------+
    %| print_level               | OT_INT          | Verbosity level            |
    %+---------------------------+-----------------+----------------------------+
    %| strategy                  | OT_STRING       | Globalization strategy     |
    %+---------------------------+-----------------+----------------------------+
    %| u_scale                   | OT_DOUBLEVECTOR | Variable scaling factors   |
    %+---------------------------+-----------------+----------------------------+
    %| upper_bandwidth           | OT_INT          | Upper bandwidth for banded |
    %|                           |                 | linear solvers             |
    %+---------------------------+-----------------+----------------------------+
    %| use_preconditioner        | OT_BOOL         | Precondition an iterative  |
    %|                           |                 | solver                     |
    %+---------------------------+-----------------+----------------------------+
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %fast_newton
    %-----------
    %
    %
    %
    %Implements simple newton iterations to solve an implicit function.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_237
    %
    %>List of available options
    %
    %+------------+-----------+-------------------------------------------------+
    %|     Id     |   Type    |                   Description                   |
    %+============+===========+=================================================+
    %| abstol     | OT_DOUBLE | Stopping criterion tolerance on ||g||__inf)     |
    %+------------+-----------+-------------------------------------------------+
    %| abstolStep | OT_DOUBLE | Stopping criterion tolerance on step size       |
    %+------------+-----------+-------------------------------------------------+
    %| max_iter   | OT_INT    | Maximum number of Newton iterations to perform  |
    %|            |           | before returning.                               |
    %+------------+-----------+-------------------------------------------------+
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %nlpsol
    %------
    %
    %
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %newton
    %------
    %
    %
    %
    %Implements simple newton iterations to solve an implicit function.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_236
    %
    %>List of available options
    %
    %+-----------------+-----------+--------------------------------------------+
    %|       Id        |   Type    |                Description                 |
    %+=================+===========+============================================+
    %| abstol          | OT_DOUBLE | Stopping criterion tolerance on max(|F|)   |
    %+-----------------+-----------+--------------------------------------------+
    %| abstolStep      | OT_DOUBLE | Stopping criterion tolerance on step size  |
    %+-----------------+-----------+--------------------------------------------+
    %| line_search     | OT_BOOL   | Enable line-search (default: true)         |
    %+-----------------+-----------+--------------------------------------------+
    %| max_iter        | OT_INT    | Maximum number of Newton iterations to     |
    %|                 |           | perform before returning.                  |
    %+-----------------+-----------+--------------------------------------------+
    %| print_iteration | OT_BOOL   | Print information about each iteration     |
    %+-----------------+-----------+--------------------------------------------+
    %
    %Joel Andersson
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_21r
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/rootfinder.hpp#L96
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/rootfinder.cpp#L96-L99
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
    %  ROOTFINDER(char name, char solver, struct:MX rfp, struct opts)
    %  ROOTFINDER(char name, char solver, Function f, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %
    %.............
    %
    %
  [varargout{1:nargout}] = casadiMEX(841, varargin{:});
end
