function varargout = interpolant(varargin)
    %INTERPOLANT [INTERNAL] 
    %
    %  Function = INTERPOLANT(char name, char solver, {[double]} grid, int m, struct opts)
    %  Function = INTERPOLANT(char name, char solver, [int] grid_dims, int m, struct opts)
    %  Function = INTERPOLANT(char name, char solver, {[double]} grid, [double] values, struct opts)
    %  Function = INTERPOLANT(char name, char solver, [int] grid_dims, [double] values, struct opts)
    %
    %Parametric variant of interpolant.
    %
    %The resulting function will have an additional argument for the grid
    %
    %By default, derivatives wrt the coefficients are not supported (zero).
    % Some
    % interpolant plugins may support the  inline=true which enables correct 
    %derivatives
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1p5
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/interpolant.hpp#L171
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/interpolant.cpp#L171-L187
    %
    %
    %
    %.......
    %
    %::
    %
    %  INTERPOLANT(char name, char solver, {[double]} grid, [double] values, struct opts)
    %
    %
    %
    %[INTERNAL]
    %
    %An interpolant function for lookup table data
    %
    %Parameters:
    %-----------
    %
    %name: 
    %label for the resulting  Function
    %
    %solver: 
    %name of the plugin
    %
    %grid: 
    %collection of 1D grids whose outer product defines the full N-D 
    %
    %rectangular grid
    %
    %values: 
    %flattened vector of all values for all gridpoints
    %
    %Syntax 1D
    %
    %::
    %
    %  * # Python
    %  * xgrid = np.linspace(1,6,6)
    %  * V = [-1,-1,-2,-3,0,2]
    %  * LUT = casadi.interpolant("LUT","bspline",[xgrid],V)
    %  * print(LUT(2.5))
    %  * 
    %
    %
    %
    %::
    %
    %  * % Matlab
    %  * xgrid = 1:6;
    %  * V = [-1 -1 -2 -3 0 2];
    %  * LUT = casadi.interpolant('LUT','bspline',{xgrid},V);
    %  * LUT(2.5)
    %  * 
    %
    %
    %
    %Syntax 2D
    %
    %::
    %
    %  * # Python
    %  * xgrid = np.linspace(-5,5,11)
    %  * ygrid = np.linspace(-4,4,9)
    %  * X,Y = np.meshgrid(xgrid,ygrid,indexing='ij')
    %  * R = np.sqrt(5*X**2 + Y**2)+ 1
    %  * data = np.sin(R)/R
    %  * data_flat = data.ravel(order='F')
    %  * LUT = casadi.interpolant('name','bspline',[xgrid,ygrid],data_flat)
    %  * print(LUT([0.5,1]))
    %  * \\enverbatim
    %  * \\verbatim
    %  * % Matlab
    %  * xgrid = -5:1:5;
    %  * ygrid = -4:1:4;
    %  * R = sqrt(5*X.^2 + Y.^2)+ 1;
    %  * V = sin(R)./(R);
    %  * LUT = interpolant('LUT','bspline',{xgrid, ygrid},V(:));
    %  * LUT([0.5 1])
    %  * 
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
    %| batch_x          | OT_INT          | Evaluate a batch | casadi::Interpol |
    %|                  |                 | of different     | ant              |
    %|                  |                 | inputs at once   |                  |
    %|                  |                 | (default 1).     |                  |
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
    %| inline           | OT_BOOL         | Implement the    | casadi::Interpol |
    %|                  |                 | lookup table in  | ant              |
    %|                  |                 | MX primitives.   |                  |
    %|                  |                 | Useful when you  |                  |
    %|                  |                 | need derivatives |                  |
    %|                  |                 | with respect to  |                  |
    %|                  |                 | grid and/or      |                  |
    %|                  |                 | coefficients.    |                  |
    %|                  |                 | Such derivatives |                  |
    %|                  |                 | are              |                  |
    %|                  |                 | fundamentally    |                  |
    %|                  |                 | dense, so use    |                  |
    %|                  |                 | with caution.    |                  |
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
    %| lookup_mode      | OT_STRINGVECTOR | Specifies, for   | casadi::Interpol |
    %|                  |                 | each grid        | ant              |
    %|                  |                 | dimension, the   |                  |
    %|                  |                 | lookup algorithm |                  |
    %|                  |                 | used to find the |                  |
    %|                  |                 | correct index.   |                  |
    %|                  |                 | 'linear' uses a  |                  |
    %|                  |                 | for-loop +       |                  |
    %|                  |                 | break; (default  |                  |
    %|                  |                 | when             |                  |
    %|                  |                 | #knots<=100),    |                  |
    %|                  |                 | 'exact' uses     |                  |
    %|                  |                 | floored division |                  |
    %|                  |                 | (only for        |                  |
    %|                  |                 | uniform grids),  |                  |
    %|                  |                 | 'binary' uses a  |                  |
    %|                  |                 | binary search.   |                  |
    %|                  |                 | (default when    |                  |
    %|                  |                 | #knots>100).     |                  |
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
    %List of plugins
    %- bspline
    %
    %- linear
    %
    %Note: some of the plugins in this list might not be available on your 
    %
    %system.  Also, there might be extra plugins available to you that are 
    %not 
    %listed here. You can obtain their documentation with  
    %Interpolant.doc("myextraplugin")
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %bspline
    %-------
    %
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_239
    %
    %>List of available options
    %
    %+-----------------------+--------------+-----------------------------------+
    %|          Id           |     Type     |            Description            |
    %+=======================+==============+===================================+
    %| algorithm             | OT_STRING    | Algorithm used for fitting the    |
    %|                       |              | data: 'not_a_knot' (default, same |
    %|                       |              | as Matlab), 'smooth_linear'.      |
    %+-----------------------+--------------+-----------------------------------+
    %| degree                | OT_INTVECTOR | Sets, for each grid dimension,    |
    %|                       |              | the degree of the spline.         |
    %+-----------------------+--------------+-----------------------------------+
    %| linear_solver         | OT_STRING    | Solver used for constructing the  |
    %|                       |              | coefficient tensor.               |
    %+-----------------------+--------------+-----------------------------------+
    %| linear_solver_options | OT_DICT      | Options to be passed to the       |
    %|                       |              | linear solver.                    |
    %+-----------------------+--------------+-----------------------------------+
    %| smooth_linear_frac    | OT_DOUBLE    | When 'smooth_linear' algorithm is |
    %|                       |              | active, determines sharpness      |
    %|                       |              | between 0 (sharp, as linear       |
    %|                       |              | interpolation) and 0.5            |
    %|                       |              | (smooth).Default value is 0.1.    |
    %+-----------------------+--------------+-----------------------------------+
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %linear
    %------
    %
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_23b
    %
    %>List of available options
    %
    %+-------------+-----------------+------------------------------------------+
    %|     Id      |      Type       |               Description                |
    %+=============+=================+==========================================+
    %| lookup_mode | OT_STRINGVECTOR | Sets, for each grid dimenion, the lookup |
    %|             |                 | algorithm used to find the correct       |
    %|             |                 | index. 'linear' uses a for-loop + break; |
    %|             |                 | 'exact' uses floored division (only for  |
    %|             |                 | uniform grids).                          |
    %+-------------+-----------------+------------------------------------------+
    %
    %Joel Andersson
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_21p
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/interpolant.hpp#L125
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/interpolant.cpp#L125-L147
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
    %  INTERPOLANT(char name, char solver, {[double]} grid, int m, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %Parametric variant of interpolant.
    %
    %The resulting function will have an additional argument for the 
    %
    %coefficients
    %
    %By default, derivatives wrt the coefficients are not supported (zero).
    % Some
    % interpolant plugins may support the  inline=true which enables correct 
    %derivatives
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1p3
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/interpolant.hpp#L189
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/interpolant.cpp#L189-L203
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
    %  INTERPOLANT(char name, char solver, [int] grid_dims, int m, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %Parametric variant of interpolant.
    %
    %The resulting function will have additional arguments for the grid and
    % 
    %coefficients
    %
    %By default, derivatives wrt the coefficients are not supported (zero).
    % Some
    % interpolant plugins may support the  inline=true which enables correct 
    %derivatives
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1p4
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/interpolant.hpp#L205
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/interpolant.cpp#L205-L213
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
    %  INTERPOLANT(char name, char solver, [int] grid_dims, [double] values, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %Parametric variant of interpolant.
    %
    %The resulting function will have an additional argument for the grid
    %
    %By default, derivatives wrt the coefficients are not supported (zero).
    % Some
    % interpolant plugins may support the  inline=true which enables correct 
    %derivatives
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1p5
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/interpolant.hpp#L171
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/interpolant.cpp#L171-L187
    %
    %
    %
    %.............
    %
    %
  [varargout{1:nargout}] = casadiMEX(883, varargin{:});
end
