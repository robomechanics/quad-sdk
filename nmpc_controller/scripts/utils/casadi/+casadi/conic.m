function varargout = conic(varargin)
    %CONIC [INTERNAL]
    %
    %  Function = CONIC(char name, char solver, struct:Sparsity qp, struct opts)
    %
    %
    %Create a QP solver Solves the following strictly convex problem:
    %
    %
    %
    %::
    %
    %  min          1/2 x' H x + g' x
    %  x
    %  
    %  subject to
    %  LBA <= A x <= UBA
    %  LBX <= x   <= UBX
    %  
    %  resize(Q x, np, np) + P >= 0 (psd)
    %  
    %  with :
    %  H sparse (n x n) positive definite
    %  g dense  (n x 1)
    %  A sparse (nc x n)
    %  Q sparse symmetric (np^2 x n)
    %  P sparse symmetric (np x nq)
    %  
    %  n: number of decision variables (x)
    %  nc: number of constraints (A)
    %  nq: shape of psd constraint matrix
    %
    %
    %
    %If H is not positive-definite, the solver should throw an error.
    %
    %Second-order cone constraints can be added as psd constraints through 
    %a 
    %helper function 'soc':
    %
    %x in R^n y in R
    %
    %|| x ||_2 <= y
    %
    %<=>
    %
    %soc(x, y) psd
    %
    %This can be proven with soc(x, y)=[y*I x; x' y] using the Shur 
    %complement.
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
    %| cache            | OT_DICT         | Prepopulate the  | casadi::Function |
    %|                  |                 | function cache.  | Internal         |
    %|                  |                 | Default: empty   |                  |
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
    %| discrete         | OT_BOOLVECTOR   | Indicates which  | casadi::Conic    |
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
    %| print_problem    | OT_BOOL         | Print a numeric  | casadi::Conic    |
    %|                  |                 | description of   |                  |
    %|                  |                 | the problem      |                  |
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
    %>Input scheme: casadi::ConicInput (CONIC_NUM_IN = 12)
    %
    %+--------------+--------+--------------------------------------------------+
    %|  Full name   | Short  |                   Description                    |
    %+==============+========+==================================================+
    %| CONIC_H      | h      | The square matrix H: sparse, (n x n). Only the   |
    %|              |        | lower triangular part is actually used. The      |
    %|              |        | matrix is assumed to be symmetrical.             |
    %+--------------+--------+--------------------------------------------------+
    %| CONIC_G      | g      | The vector g: dense, (n x 1)                     |
    %+--------------+--------+--------------------------------------------------+
    %| CONIC_A      | a      | The matrix A: sparse, (nc x n) - product with x  |
    %|              |        | must be dense.                                   |
    %+--------------+--------+--------------------------------------------------+
    %| CONIC_LBA    | lba    | dense, (nc x 1)                                  |
    %+--------------+--------+--------------------------------------------------+
    %| CONIC_UBA    | uba    | dense, (nc x 1)                                  |
    %+--------------+--------+--------------------------------------------------+
    %| CONIC_LBX    | lbx    | dense, (n x 1)                                   |
    %+--------------+--------+--------------------------------------------------+
    %| CONIC_UBX    | ubx    | dense, (n x 1)                                   |
    %+--------------+--------+--------------------------------------------------+
    %| CONIC_X0     | x0     | dense, (n x 1)                                   |
    %+--------------+--------+--------------------------------------------------+
    %| CONIC_LAM_X0 | lam_x0 | dense                                            |
    %+--------------+--------+--------------------------------------------------+
    %| CONIC_LAM_A0 | lam_a0 | dense                                            |
    %+--------------+--------+--------------------------------------------------+
    %| CONIC_Q      | q      | The matrix Q: sparse symmetric, (np^2 x n)       |
    %+--------------+--------+--------------------------------------------------+
    %| CONIC_P      | p      | The matrix P: sparse symmetric, (np x np)        |
    %+--------------+--------+--------------------------------------------------+
    %
    %>Output scheme: casadi::ConicOutput (CONIC_NUM_OUT = 4)
    %
    %+-------------+-------+---------------------------------------------------+
    %|  Full name  | Short |                    Description                    |
    %+=============+=======+===================================================+
    %| CONIC_X     | x     | The primal solution.                              |
    %+-------------+-------+---------------------------------------------------+
    %| CONIC_COST  | cost  | The optimal cost.                                 |
    %+-------------+-------+---------------------------------------------------+
    %| CONIC_LAM_A | lam_a | The dual solution corresponding to linear bounds. |
    %+-------------+-------+---------------------------------------------------+
    %| CONIC_LAM_X | lam_x | The dual solution corresponding to simple bounds. |
    %+-------------+-------+---------------------------------------------------+
    %
    %List of plugins
    %- cbc
    %
    %- clp
    %
    %- cplex
    %
    %- fatrop
    %
    %- gurobi
    %
    %- highs
    %
    %- hpipm
    %
    %- hpmpc
    %
    %- ooqp
    %
    %- osqp
    %
    %- proxqp
    %
    %- qpoases
    %
    %- sqic
    %
    %- superscs
    %
    %- ipqp
    %
    %- nlpsol
    %
    %- qrqp
    %
    %Note: some of the plugins in this list might not be available on your 
    %
    %system.  Also, there might be extra plugins available to you that are 
    %not 
    %listed here. You can obtain their documentation with  
    %Conic.doc("myextraplugin")
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %cbc
    %---
    %
    %
    %
    %Interface to Cbc solver for sparse Quadratic Programs
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_221
    %
    %>List of available options
    %
    %+-------------+-----------------------+------------------------------------+
    %|     Id      |         Type          |            Description             |
    %+=============+=======================+====================================+
    %| cbc         | OT_DICT               | Options to be passed to CBC.Three  |
    %|             |                       | sets of options are supported. The |
    %|             |                       | first can be found in              |
    %|             |                       | OsiSolverParameters.hpp. The       |
    %|             |                       | second can be found in             |
    %|             |                       | CbcModel.hpp. The third are        |
    %|             |                       | options that can be passed to      |
    %|             |                       | CbcMain1.                          |
    %+-------------+-----------------------+------------------------------------+
    %| hot_start   | OT_BOOL               | Hot start with x0 [Default false]. |
    %+-------------+-----------------------+------------------------------------+
    %| sos_groups  | OT_INTVECTORVECTOR    | Definition of SOS groups by        |
    %|             |                       | indices.                           |
    %+-------------+-----------------------+------------------------------------+
    %| sos_types   | OT_INTVECTOR          | Specify 1 or 2 for each SOS group. |
    %+-------------+-----------------------+------------------------------------+
    %| sos_weights | OT_DOUBLEVECTORVECTOR | Weights corresponding to SOS       |
    %|             |                       | entries.                           |
    %+-------------+-----------------------+------------------------------------+
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %clp
    %---
    %
    %
    %
    %Interface to Clp solver for sparse Quadratic Programs
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_22d
    %
    %>List of available options
    %
    %+-----+---------+----------------------------------------------------------+
    %| Id  |  Type   |                       Description                        |
    %+=====+=========+==========================================================+
    %| clp | OT_DICT | Options to be passed to CLP. A first set of options can  |
    %|     |         | be found in ClpParameters.hpp. eg. 'PrimalTolerance'.    |
    %|     |         | There are other options in additions. 'AutomaticScaling' |
    %|     |         | (bool) is recognised. 'initial_solve' (default off)      |
    %|     |         | activates the use of Clp's initialSolve.                 |
    %|     |         | 'initial_solve_options' takes a dictionary with          |
    %|     |         | following keys (see ClpSolve.hpp): SolveType (string),   |
    %|     |         | PresolveType (string), NumberPasses, SpecialOptions      |
    %|     |         | (intvectorvector), IndependentOptions (intvectorvector). |
    %+-----+---------+----------------------------------------------------------+
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %cplex
    %-----
    %
    %
    %
    %Interface to Cplex solver for sparse Quadratic Programs
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_22a
    %
    %>List of available options
    %
    %+----------------+-----------------------+---------------------------------+
    %|       Id       |         Type          |           Description           |
    %+================+=======================+=================================+
    %| cplex          | OT_DICT               | Options to be passed to CPLEX   |
    %+----------------+-----------------------+---------------------------------+
    %| dep_check      | OT_INT                | Detect redundant constraints.   |
    %+----------------+-----------------------+---------------------------------+
    %| dump_filename  | OT_STRING             | The filename to dump to.        |
    %+----------------+-----------------------+---------------------------------+
    %| dump_to_file   | OT_BOOL               | Dumps QP to file in CPLEX       |
    %|                |                       | format.                         |
    %+----------------+-----------------------+---------------------------------+
    %| mip_start      | OT_BOOL               | Hot start integers with x0      |
    %|                |                       | [Default false].                |
    %+----------------+-----------------------+---------------------------------+
    %| qp_method      | OT_INT                | Determines which CPLEX          |
    %|                |                       | algorithm to use.               |
    %+----------------+-----------------------+---------------------------------+
    %| sos_groups     | OT_INTVECTORVECTOR    | Definition of SOS groups by     |
    %|                |                       | indices.                        |
    %+----------------+-----------------------+---------------------------------+
    %| sos_types      | OT_INTVECTOR          | Specify 1 or 2 for each SOS     |
    %|                |                       | group.                          |
    %+----------------+-----------------------+---------------------------------+
    %| sos_weights    | OT_DOUBLEVECTORVECTOR | Weights corresponding to SOS    |
    %|                |                       | entries.                        |
    %+----------------+-----------------------+---------------------------------+
    %| tol            | OT_DOUBLE             | Tolerance of solver             |
    %+----------------+-----------------------+---------------------------------+
    %| version_suffix | OT_STRING             | Specify version of cplex to     |
    %|                |                       | load. We will attempt to load l |
    %|                |                       | ibcplex<version_suffix>.[so|dll |
    %|                |                       | |dylib]. Default value is taken |
    %|                |                       | from CPLEX_VERSION env          |
    %|                |                       | variable.                       |
    %+----------------+-----------------------+---------------------------------+
    %| warm_start     | OT_BOOL               | Use warm start with simplex     |
    %|                |                       | methods (affects only the       |
    %|                |                       | simplex methods).               |
    %+----------------+-----------------------+---------------------------------+
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %fatrop
    %------
    %
    %
    %
    %Interface to Fatrop Solver
    %
    %In order to use this interface, you must:
    %
    %Decision variables must only by state and control, and the variable 
    %
    %ordering must be [x0 u0 x1 u1 ...]
    %
    %The constraints must be in order: [ gap0 lincon0 gap1 lincon1 ]
    %
    %gap: Ak+1 = Ak xk + Bk uk lincon: yk= Ck xk + Dk uk
    %
    %
    %
    %::
    %
    %         A0 B0 -I
    %         C0 D0
    %                A1 B1 -I
    %                C1 D1
    %
    %
    %
    %where I must be a diagonal sparse matrix
    %Either supply all of N, nx, ng, nu 
    %options or rely on automatic 
    %detection
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_27g
    %
    %>List of available options
    %
    %+--------+--------------+-----------------------------------------------+
    %|   Id   |     Type     |                  Description                  |
    %+========+==============+===============================================+
    %| N      | OT_INT       | OCP horizon                                   |
    %+--------+--------------+-----------------------------------------------+
    %| fatrop | OT_DICT      | Options to be passed to fatrop                |
    %+--------+--------------+-----------------------------------------------+
    %| ng     | OT_INTVECTOR | Number of non-dynamic constraints, length N+1 |
    %+--------+--------------+-----------------------------------------------+
    %| nu     | OT_INTVECTOR | Number of controls, length N                  |
    %+--------+--------------+-----------------------------------------------+
    %| nx     | OT_INTVECTOR | Number of states, length N+1                  |
    %+--------+--------------+-----------------------------------------------+
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %gurobi
    %------
    %
    %
    %
    %Interface to the GUROBI Solver for quadratic programming
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_22q
    %
    %>List of available options
    %
    %+-------------+-----------------------+------------------------------------+
    %|     Id      |         Type          |            Description             |
    %+=============+=======================+====================================+
    %| gurobi      | OT_DICT               | Options to be passed to gurobi.    |
    %+-------------+-----------------------+------------------------------------+
    %| sos_groups  | OT_INTVECTORVECTOR    | Definition of SOS groups by        |
    %|             |                       | indices.                           |
    %+-------------+-----------------------+------------------------------------+
    %| sos_types   | OT_INTVECTOR          | Specify 1 or 2 for each SOS group. |
    %+-------------+-----------------------+------------------------------------+
    %| sos_weights | OT_DOUBLEVECTORVECTOR | Weights corresponding to SOS       |
    %|             |                       | entries.                           |
    %+-------------+-----------------------+------------------------------------+
    %| vtype       | OT_STRINGVECTOR       | Type of variables: [CONTINUOUS|bin |
    %|             |                       | ary|integer|semicont|semiint]      |
    %+-------------+-----------------------+------------------------------------+
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %highs
    %-----
    %
    %
    %
    %Interface to HiGHS solver for sparse Quadratic Programs, see 
    %highs.dev for 
    %more information and https://www.maths.ed.ac.uk/hall/HiGHS/HighsOptions.html
    %  for a list of options.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_22f
    %
    %>List of available options
    %
    %+-------+---------+--------------------------------+
    %|  Id   |  Type   |          Description           |
    %+=======+=========+================================+
    %| highs | OT_DICT | Options to be passed to HiGHS. |
    %+-------+---------+--------------------------------+
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %hpipm
    %-----
    %
    %
    %
    %Interface to HPIPM Solver
    %
    %In order to use this interface, you must:
    %
    %Decision variables must only by state and control, and the variable 
    %
    %ordering must be [x0 u0 x1 u1 ...]
    %
    %The constraints must be in order: [ gap0 lincon0 gap1 lincon1 ]
    %
    %gap: Ak+1 = Ak xk + Bk uk lincon: yk= Ck xk + Dk uk
    %
    %
    %
    %::
    %
    %         A0 B0 -I
    %         C0 D0
    %                A1 B1 -I
    %                C1 D1
    %
    %
    %
    %where I must be a diagonal sparse matrix
    %Either supply all of N, nx, ng, nu 
    %options or rely on automatic 
    %detection
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_242
    %
    %>List of available options
    %
    %+-------+--------------+--------------------------------------------------+
    %|  Id   |     Type     |                   Description                    |
    %+=======+==============+==================================================+
    %| N     | OT_INT       | OCP horizon                                      |
    %+-------+--------------+--------------------------------------------------+
    %| hpipm | OT_DICT      | Options to be passed to hpipm                    |
    %+-------+--------------+--------------------------------------------------+
    %| inf   | OT_DOUBLE    | Replace infinities by this amount [default: 1e8] |
    %+-------+--------------+--------------------------------------------------+
    %| ng    | OT_INTVECTOR | Number of non-dynamic constraints, length N+1    |
    %+-------+--------------+--------------------------------------------------+
    %| nu    | OT_INTVECTOR | Number of controls, length N                     |
    %+-------+--------------+--------------------------------------------------+
    %| nx    | OT_INTVECTOR | Number of states, length N+1                     |
    %+-------+--------------+--------------------------------------------------+
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %hpmpc
    %-----
    %
    %
    %
    %Interface to HMPC Solver
    %
    %In order to use this interface, you must:
    %
    %Decision variables must only by state and control, and the variable 
    %
    %ordering must be [x0 u0 x1 u1 ...]
    %
    %The constraints must be in order: [ gap0 lincon0 gap1 lincon1 ]
    %
    %gap: Ak+1 = Ak xk + Bk uk lincon: yk= Ck xk + Dk uk
    %
    %
    %
    %::
    %
    %         A0 B0 -I
    %         C0 D0
    %                A1 B1 -I
    %                C1 D1
    %
    %
    %
    %where I must be a diagonal sparse matrix
    %Either supply all of N, nx, ng, nu 
    %options or rely on automatic 
    %detection
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_22p
    %
    %>List of available options
    %
    %+----------------+--------------+------------------------------------------+
    %|       Id       |     Type     |               Description                |
    %+================+==============+==========================================+
    %| N              | OT_INT       | OCP horizon                              |
    %+----------------+--------------+------------------------------------------+
    %| blasfeo_target | OT_STRING    | hpmpc target                             |
    %+----------------+--------------+------------------------------------------+
    %| inf            | OT_DOUBLE    | HPMPC cannot handle infinities.          |
    %|                |              | Infinities will be replaced by this      |
    %|                |              | option's value.                          |
    %+----------------+--------------+------------------------------------------+
    %| max_iter       | OT_INT       | Max number of iterations                 |
    %+----------------+--------------+------------------------------------------+
    %| mu0            | OT_DOUBLE    | Max element in cost function as estimate |
    %|                |              | of max multiplier                        |
    %+----------------+--------------+------------------------------------------+
    %| ng             | OT_INTVECTOR | Number of non-dynamic constraints,       |
    %|                |              | length N+1                               |
    %+----------------+--------------+------------------------------------------+
    %| nu             | OT_INTVECTOR | Number of controls, length N             |
    %+----------------+--------------+------------------------------------------+
    %| nx             | OT_INTVECTOR | Number of states, length N+1             |
    %+----------------+--------------+------------------------------------------+
    %| print_level    | OT_INT       | Amount of diagnostic printing [Default:  |
    %|                |              | 1].                                      |
    %+----------------+--------------+------------------------------------------+
    %| target         | OT_STRING    | hpmpc target                             |
    %+----------------+--------------+------------------------------------------+
    %| tol            | OT_DOUBLE    | Tolerance in the duality measure         |
    %+----------------+--------------+------------------------------------------+
    %| warm_start     | OT_BOOL      | Use warm-starting                        |
    %+----------------+--------------+------------------------------------------+
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %ooqp
    %----
    %
    %
    %
    %Interface to the OOQP Solver for quadratic programming The current 
    %
    %implementation assumes that OOQP is configured with the MA27 sparse 
    %linear 
    %solver.
    %
    %NOTE: when doing multiple calls to evaluate(), check if you need to 
    %
    %reInit();
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_222
    %
    %>List of available options
    %
    %+-------------+-----------+------------------------------------------------+
    %|     Id      |   Type    |                  Description                   |
    %+=============+===========+================================================+
    %| artol       | OT_DOUBLE | tolerance as provided with setArTol to OOQP    |
    %+-------------+-----------+------------------------------------------------+
    %| mutol       | OT_DOUBLE | tolerance as provided with setMuTol to OOQP    |
    %+-------------+-----------+------------------------------------------------+
    %| print_level | OT_INT    | Print level. OOQP listens to print_level 0, 10 |
    %|             |           | and 100                                        |
    %+-------------+-----------+------------------------------------------------+
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %osqp
    %----
    %
    %
    %
    %Interface to the OSQP Solver for quadratic programming
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_220
    %
    %Interface to the PROXQP Solver for quadratic programming
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_243
    %
    %>List of available options
    %
    %+-------------------+---------+--------------------------------------------+
    %|        Id         |  Type   |                Description                 |
    %+===================+=========+============================================+
    %| osqp              | OT_DICT | const Options to be passed to osqp.        |
    %+-------------------+---------+--------------------------------------------+
    %| warm_start_dual   | OT_BOOL | Use lam_a0 and lam_x0 input to warmstart   |
    %|                   |         | [Default: truw].                           |
    %+-------------------+---------+--------------------------------------------+
    %| warm_start_primal | OT_BOOL | Use x0 input to warmstart [Default: true]. |
    %+-------------------+---------+--------------------------------------------+
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %proxqp
    %------
    %
    %
    %
    %>List of available options
    %
    %+-------------------+---------+--------------------------------------------+
    %|        Id         |  Type   |                Description                 |
    %+===================+=========+============================================+
    %| proxqp            | OT_DICT | const proxqp options.                      |
    %+-------------------+---------+--------------------------------------------+
    %| warm_start_dual   | OT_BOOL | Use y and z input to warmstart [Default:   |
    %|                   |         | true].                                     |
    %+-------------------+---------+--------------------------------------------+
    %| warm_start_primal | OT_BOOL | Use x input to warmstart [Default: true].  |
    %+-------------------+---------+--------------------------------------------+
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %qpoases
    %-------
    %
    %
    %
    %Interface to QPOases Solver for quadratic programming
    %
    %::
    %
    %  Extra doc: https://github.com/casadi/casadi/wiki/L_22o 
    %  
    %
    %
    %
    %>List of available options
    %
    %+-------------------------------+-----------+------------------------------+
    %|              Id               |   Type    |         Description          |
    %+===============================+===========+==============================+
    %| CPUtime                       | OT_DOUBLE | The maximum allowed CPU time |
    %|                               |           | in seconds for the whole     |
    %|                               |           | initialisation (and the      |
    %|                               |           | actually required one on     |
    %|                               |           | output). Disabled if unset.  |
    %+-------------------------------+-----------+------------------------------+
    %| boundRelaxation               | OT_DOUBLE | Initial relaxation of bounds |
    %|                               |           | to start homotopy and        |
    %|                               |           | initial value for far        |
    %|                               |           | bounds.                      |
    %+-------------------------------+-----------+------------------------------+
    %| boundTolerance                | OT_DOUBLE | If upper and lower bounds    |
    %|                               |           | differ less than this        |
    %|                               |           | tolerance, they are regarded |
    %|                               |           | equal, i.e. as equality      |
    %|                               |           | constraint.                  |
    %+-------------------------------+-----------+------------------------------+
    %| enableCholeskyRefactorisation | OT_INT    | Specifies the frequency of a |
    %|                               |           | full re-factorisation of     |
    %|                               |           | projected Hessian matrix: 0: |
    %|                               |           | turns them off, 1: uses them |
    %|                               |           | at each iteration etc.       |
    %+-------------------------------+-----------+------------------------------+
    %| enableDriftCorrection         | OT_INT    | Specifies the frequency of   |
    %|                               |           | drift corrections: 0: turns  |
    %|                               |           | them off.                    |
    %+-------------------------------+-----------+------------------------------+
    %| enableEqualities              | OT_BOOL   | Specifies whether equalities |
    %|                               |           | should be treated as always  |
    %|                               |           | active (True) or not (False) |
    %+-------------------------------+-----------+------------------------------+
    %| enableFarBounds               | OT_BOOL   | Enables the use of far       |
    %|                               |           | bounds.                      |
    %+-------------------------------+-----------+------------------------------+
    %| enableFlippingBounds          | OT_BOOL   | Enables the use of flipping  |
    %|                               |           | bounds.                      |
    %+-------------------------------+-----------+------------------------------+
    %| enableFullLITests             | OT_BOOL   | Enables condition-hardened   |
    %|                               |           | (but more expensive) LI      |
    %|                               |           | test.                        |
    %+-------------------------------+-----------+------------------------------+
    %| enableInertiaCorrection       | OT_BOOL   | Should working set be        |
    %|                               |           | repaired when negative       |
    %|                               |           | curvature is discovered      |
    %|                               |           | during hotstart.             |
    %+-------------------------------+-----------+------------------------------+
    %| enableNZCTests                | OT_BOOL   | Enables nonzero curvature    |
    %|                               |           | tests.                       |
    %+-------------------------------+-----------+------------------------------+
    %| enableRamping                 | OT_BOOL   | Enables ramping.             |
    %+-------------------------------+-----------+------------------------------+
    %| enableRegularisation          | OT_BOOL   | Enables automatic Hessian    |
    %|                               |           | regularisation.              |
    %+-------------------------------+-----------+------------------------------+
    %| epsDen                        | OT_DOUBLE | Denominator tolerance for    |
    %|                               |           | ratio tests.                 |
    %+-------------------------------+-----------+------------------------------+
    %| epsFlipping                   | OT_DOUBLE | Tolerance of squared         |
    %|                               |           | Cholesky diagonal factor     |
    %|                               |           | which triggers flipping      |
    %|                               |           | bound.                       |
    %+-------------------------------+-----------+------------------------------+
    %| epsIterRef                    | OT_DOUBLE | Early termination tolerance  |
    %|                               |           | for iterative refinement.    |
    %+-------------------------------+-----------+------------------------------+
    %| epsLITests                    | OT_DOUBLE | Tolerance for linear         |
    %|                               |           | independence tests.          |
    %+-------------------------------+-----------+------------------------------+
    %| epsNZCTests                   | OT_DOUBLE | Tolerance for nonzero        |
    %|                               |           | curvature tests.             |
    %+-------------------------------+-----------+------------------------------+
    %| epsNum                        | OT_DOUBLE | Numerator tolerance for      |
    %|                               |           | ratio tests.                 |
    %+-------------------------------+-----------+------------------------------+
    %| epsRegularisation             | OT_DOUBLE | Scaling factor of identity   |
    %|                               |           | matrix used for Hessian      |
    %|                               |           | regularisation.              |
    %+-------------------------------+-----------+------------------------------+
    %| finalRamping                  | OT_DOUBLE | Final value for ramping      |
    %|                               |           | strategy.                    |
    %+-------------------------------+-----------+------------------------------+
    %| growFarBounds                 | OT_DOUBLE | Factor to grow far bounds.   |
    %+-------------------------------+-----------+------------------------------+
    %| hessian_type                  | OT_STRING | Type of Hessian - see        |
    %|                               |           | qpOASES documentation [UNKNO |
    %|                               |           | WN|posdef|semidef|indef|zero |
    %|                               |           | |identity]]                  |
    %+-------------------------------+-----------+------------------------------+
    %| initialFarBounds              | OT_DOUBLE | Initial size for far bounds. |
    %+-------------------------------+-----------+------------------------------+
    %| initialRamping                | OT_DOUBLE | Start value for ramping      |
    %|                               |           | strategy.                    |
    %+-------------------------------+-----------+------------------------------+
    %| initialStatusBounds           | OT_STRING | Initial status of bounds at  |
    %|                               |           | first iteration.             |
    %+-------------------------------+-----------+------------------------------+
    %| linsol_plugin                 | OT_STRING | Linear solver plugin         |
    %+-------------------------------+-----------+------------------------------+
    %| maxDualJump                   | OT_DOUBLE | Maximum allowed jump in dual |
    %|                               |           | variables in linear          |
    %|                               |           | independence tests.          |
    %+-------------------------------+-----------+------------------------------+
    %| maxPrimalJump                 | OT_DOUBLE | Maximum allowed jump in      |
    %|                               |           | primal variables in nonzero  |
    %|                               |           | curvature tests.             |
    %+-------------------------------+-----------+------------------------------+
    %| max_schur                     | OT_INT    | Maximal number of Schur      |
    %|                               |           | updates [75]                 |
    %+-------------------------------+-----------+------------------------------+
    %| nWSR                          | OT_INT    | The maximum number of        |
    %|                               |           | working set recalculations   |
    %|                               |           | to be performed during the   |
    %|                               |           | initial homotopy. Default is |
    %|                               |           | 5(nx + nc)                   |
    %+-------------------------------+-----------+------------------------------+
    %| numRefinementSteps            | OT_INT    | Maximum number of iterative  |
    %|                               |           | refinement steps.            |
    %+-------------------------------+-----------+------------------------------+
    %| numRegularisationSteps        | OT_INT    | Maximum number of successive |
    %|                               |           | regularisation steps.        |
    %+-------------------------------+-----------+------------------------------+
    %| printLevel                    | OT_STRING | Defines the amount of text   |
    %|                               |           | output during QP solution,   |
    %|                               |           | see Section 5.7              |
    %+-------------------------------+-----------+------------------------------+
    %| schur                         | OT_BOOL   | Use Schur Complement         |
    %|                               |           | Approach [false]             |
    %+-------------------------------+-----------+------------------------------+
    %| sparse                        | OT_BOOL   | Formulate the QP using       |
    %|                               |           | sparse matrices. [false]     |
    %+-------------------------------+-----------+------------------------------+
    %| terminationTolerance          | OT_DOUBLE | Relative termination         |
    %|                               |           | tolerance to stop homotopy.  |
    %+-------------------------------+-----------+------------------------------+
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %sqic
    %----
    %
    %
    %
    %Interface to the SQIC solver for quadratic programming
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_21s
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %superscs
    %--------
    %
    %
    %
    %Interface to the SuperSCS solver for conic programming
    %
    %Joris Gillis, 2019
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_21z
    %
    %>List of available options
    %
    %+----------+---------+-----------------------------------+
    %|    Id    |  Type   |            Description            |
    %+==========+=========+===================================+
    %| superscs | OT_DICT | Options to be passed to superscs. |
    %+----------+---------+-----------------------------------+
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %ipqp
    %----
    %
    %
    %
    %Solves QPs using a Mehrotra predictor-corrector interior point method
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_23c
    %
    %>List of available options
    %
    %+-----------------------+-----------+--------------------------------------+
    %|          Id           |   Type    |             Description              |
    %+=======================+===========+======================================+
    %| constr_viol_tol       | OT_DOUBLE | Constraint violation tolerance       |
    %|                       |           | [1e-8].                              |
    %+-----------------------+-----------+--------------------------------------+
    %| dual_inf_tol          | OT_DOUBLE | Dual feasibility violation tolerance |
    %|                       |           | [1e-8]                               |
    %+-----------------------+-----------+--------------------------------------+
    %| linear_solver         | OT_STRING | A custom linear solver creator       |
    %|                       |           | function [default: ldl]              |
    %+-----------------------+-----------+--------------------------------------+
    %| linear_solver_options | OT_DICT   | Options to be passed to the linear   |
    %|                       |           | solver                               |
    %+-----------------------+-----------+--------------------------------------+
    %| max_iter              | OT_INT    | Maximum number of iterations [1000]. |
    %+-----------------------+-----------+--------------------------------------+
    %| min_lam               | OT_DOUBLE | Smallest multiplier treated as       |
    %|                       |           | inactive for the initial active set  |
    %|                       |           | [0].                                 |
    %+-----------------------+-----------+--------------------------------------+
    %| print_header          | OT_BOOL   | Print header [true].                 |
    %+-----------------------+-----------+--------------------------------------+
    %| print_info            | OT_BOOL   | Print info [true].                   |
    %+-----------------------+-----------+--------------------------------------+
    %| print_iter            | OT_BOOL   | Print iterations [true].             |
    %+-----------------------+-----------+--------------------------------------+
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
    %Solve QPs using an  Nlpsol Use the 'nlpsol' option to specify the NLP solver
    % to use.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_235
    %
    %>List of available options
    %
    %+----------------+-----------+---------------------------------+
    %|       Id       |   Type    |           Description           |
    %+================+===========+=================================+
    %| nlpsol         | OT_STRING | Name of solver.                 |
    %+----------------+-----------+---------------------------------+
    %| nlpsol_options | OT_DICT   | Options to be passed to solver. |
    %+----------------+-----------+---------------------------------+
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %qrqp
    %----
    %
    %
    %
    %Solve QPs using an active-set method
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_22y
    %
    %>List of available options
    %
    %+-----------------+-----------+--------------------------------------------+
    %|       Id        |   Type    |                Description                 |
    %+=================+===========+============================================+
    %| constr_viol_tol | OT_DOUBLE | Constraint violation tolerance [1e-8].     |
    %+-----------------+-----------+--------------------------------------------+
    %| dual_inf_tol    | OT_DOUBLE | Dual feasibility violation tolerance       |
    %|                 |           | [1e-8]                                     |
    %+-----------------+-----------+--------------------------------------------+
    %| max_iter        | OT_INT    | Maximum number of iterations [1000].       |
    %+-----------------+-----------+--------------------------------------------+
    %| min_lam         | OT_DOUBLE | Smallest multiplier treated as inactive    |
    %|                 |           | for the initial active set [0].            |
    %+-----------------+-----------+--------------------------------------------+
    %| print_header    | OT_BOOL   | Print header [true].                       |
    %+-----------------+-----------+--------------------------------------------+
    %| print_info      | OT_BOOL   | Print info [true].                         |
    %+-----------------+-----------+--------------------------------------------+
    %| print_iter      | OT_BOOL   | Print iterations [true].                   |
    %+-----------------+-----------+--------------------------------------------+
    %| print_lincomb   | OT_BOOL   | Print dependant linear combinations of     |
    %|                 |           | constraints [false]. Printed numbers are   |
    %|                 |           | 0-based indices into the vector of [simple |
    %|                 |           | bounds;linear bounds]                      |
    %+-----------------+-----------+--------------------------------------------+
    %
    %Joel Andersson
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_21n
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/conic.hpp#L43
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/conic.cpp#L43-L46
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(816, varargin{:});
end
