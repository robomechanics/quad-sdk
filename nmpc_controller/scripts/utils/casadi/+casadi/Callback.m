classdef  Callback < casadi.Function
    %CALLBACK [INTERNAL] 
    %
    %
    % Callback function functionality.
    %
    %This class provides a public API to the FunctionInternal class that 
    %can be 
    %subclassed by the user, who is then able to implement the 
    %different virtual
    % method. Note that the  Function class also provides a public API to 
    %FunctionInternal, but only allows
    % calling, not being called.
    %
    %The user is responsible for not deleting this class for the lifetime 
    %of the
    % internal function object.
    %
    %Joris Gillis, Joel Andersson
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_o0
    %
    %C++ includes: callback.hpp
    %
    %
  methods
    function self = Callback(varargin)
      self@casadi.Function(SwigRef.Null);
    %CALLBACK [INTERNAL] 
    %
    %  new_obj = CALLBACK(self)
    %
    %Copy constructor (throws an error)
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_o3
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.hpp#L64
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.cpp#L32-L34
    %
    %
    %
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        if strcmp(class(self),'director_basic.Callback')
          tmp = casadiMEX(924, 0, varargin{:});
        else
          tmp = casadiMEX(924, self, varargin{:});
        end
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(925, self);
          self.SwigClear();
        end
    end
    function varargout = construct(self,varargin)
    %CONSTRUCT [INTERNAL] 
    %
    %  CONSTRUCT(self, char name, struct opts)
    %
    %Construct internal object.
    %
    %This is the step that actually construct the internal object, as the 
    %class 
    %constructor only creates a null pointer. It should be called 
    %from the user 
    %constructor.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_o5
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.hpp#L78
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.cpp#L36-L42
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(926, self, varargin{:});
    end
    function varargout = init(self,varargin)
    %INIT [INTERNAL] 
    %
    %  INIT(self)
    %
    %Initialize the object.
    %
    %This function is called after the object construction (for the whole 
    %class 
    %hierarchy) is complete, but before the finalization step. It is 
    %called 
    %recursively for the whole class hierarchy, starting with the 
    %lowest level.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_o6
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.hpp#L88
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.hpp#L88-L88
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(927, self, varargin{:});
    end
    function varargout = finalize(self,varargin)
    %FINALIZE [INTERNAL] 
    %
    %  FINALIZE(self)
    %
    %Finalize the object.
    %
    %This function is called after the construction and init steps are 
    %
    %completed, but before user functions are called. It is called 
    %recursively 
    %for the whole class hierarchy, starting with the highest 
    %level.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_o7
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.hpp#L98
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.hpp#L98-L98
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(928, self, varargin{:});
    end
    function varargout = eval(self,varargin)
    %EVAL [INTERNAL] 
    %
    %  {DM} = EVAL(self, {DM} arg)
    %
    %Evaluate numerically, using temporary matrices and work vectors.
    %
    %This signature is not thread-safe. For guaranteed thread-safety, use  
    %eval_buffer
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_o8
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.hpp#L106
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.cpp#L60-L62
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(929, self, varargin{:});
    end
    function varargout = eval_buffer(self,varargin)
    %EVAL_BUFFER [INTERNAL] 
    %
    %  int = EVAL_BUFFER(self, double const ** arg, [int] sizes_arg, double ** res, [int] sizes_res)
    %
    %A copy-free low level interface.
    %
    %In Python, you will be passed two tuples of memoryview objects Note 
    %that 
    %only the structural nonzeros are present in the memoryview 
    %objects/buffers.
    %
    %Make sure to override  has_eval_buffer() to indicate support for this 
    %method.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_o9
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.hpp#L116
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.cpp#L53-L56
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(930, self, varargin{:});
    end
    function varargout = has_eval_buffer(self,varargin)
    %HAS_EVAL_BUFFER [INTERNAL] 
    %
    %  bool = HAS_EVAL_BUFFER(self)
    %
    %Does the  Callback class support a copy-free low level interface
    % ?
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_265
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.hpp#L122
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.cpp#L57-L59
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(931, self, varargin{:});
    end
    function varargout = get_n_in(self,varargin)
    %GET_N_IN [INTERNAL] 
    %
    %  int = GET_N_IN(self)
    %
    %Get the number of inputs.
    %
    %This function is called during construction.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_oa
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.hpp#L129
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.cpp#L64-L66
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(932, self, varargin{:});
    end
    function varargout = get_n_out(self,varargin)
    %GET_N_OUT [INTERNAL] 
    %
    %  int = GET_N_OUT(self)
    %
    %Get the number of outputs.
    %
    %This function is called during construction.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_ob
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.hpp#L136
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.cpp#L68-L70
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(933, self, varargin{:});
    end
    function varargout = get_sparsity_in(self,varargin)
    %GET_SPARSITY_IN [INTERNAL] 
    %
    %  Sparsity = GET_SPARSITY_IN(self, int i)
    %
    %Get the sparsity of an input.
    %
    %This function is called during construction.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_oc
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.hpp#L143
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.cpp#L72-L74
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(934, self, varargin{:});
    end
    function varargout = get_sparsity_out(self,varargin)
    %GET_SPARSITY_OUT [INTERNAL] 
    %
    %  Sparsity = GET_SPARSITY_OUT(self, int i)
    %
    %Get the sparsity of an output.
    %
    %This function is called during construction.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_od
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.hpp#L150
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.cpp#L76-L78
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(935, self, varargin{:});
    end
    function varargout = get_name_in(self,varargin)
    %GET_NAME_IN [INTERNAL] 
    %
    %  char = GET_NAME_IN(self, int i)
    %
    %Get the name of an input.
    %
    %This function is called during construction.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_oe
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.hpp#L157
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.cpp#L80-L82
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(936, self, varargin{:});
    end
    function varargout = get_name_out(self,varargin)
    %GET_NAME_OUT [INTERNAL] 
    %
    %  char = GET_NAME_OUT(self, int i)
    %
    %Get the name of an output.
    %
    %This function is called during construction.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_of
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.hpp#L164
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.cpp#L84-L86
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(937, self, varargin{:});
    end
    function varargout = uses_output(self,varargin)
    %USES_OUTPUT [INTERNAL] 
    %
    %  bool = USES_OUTPUT(self)
    %
    %Do the derivative functions need nondifferentiated outputs?
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_og
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.hpp#L169
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.cpp#L88-L90
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(938, self, varargin{:});
    end
    function varargout = has_jacobian(self,varargin)
    %HAS_JACOBIAN [INTERNAL] 
    %
    %  bool = HAS_JACOBIAN(self)
    %
    %Return Jacobian of all input elements with respect to all output
    % 
    %elements.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_oh
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.hpp#L175
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.cpp#L92-L94
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(939, self, varargin{:});
    end
    function varargout = get_jacobian(self,varargin)
    %GET_JACOBIAN [INTERNAL] 
    %
    %  Function = GET_JACOBIAN(self, char name, {char} inames, {char} onames, struct opts)
    %
    %Return Jacobian of all input elements with respect to all output
    % 
    %elements.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_oh
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.hpp#L176
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.cpp#L97-L102
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(940, self, varargin{:});
    end
    function varargout = has_forward(self,varargin)
    %HAS_FORWARD [INTERNAL] 
    %
    %  bool = HAS_FORWARD(self, int nfwd)
    %
    %Return function that calculates forward derivatives.
    %
    %forward(nfwd) returns a cached instance if available, and calls   Function 
    %get_forward(casadi_int nfwd) if no cached version is available.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_oi
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.hpp#L190
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.cpp#L112-L114
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(941, self, varargin{:});
    end
    function varargout = get_forward(self,varargin)
    %GET_FORWARD [INTERNAL] 
    %
    %  Function = GET_FORWARD(self, int nfwd, char name, {char} inames, {char} onames, struct opts)
    %
    %Return function that calculates forward derivatives.
    %
    %forward(nfwd) returns a cached instance if available, and calls   Function 
    %get_forward(casadi_int nfwd) if no cached version is available.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_oi
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.hpp#L191
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.cpp#L105-L110
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(942, self, varargin{:});
    end
    function varargout = has_reverse(self,varargin)
    %HAS_REVERSE [INTERNAL] 
    %
    %  bool = HAS_REVERSE(self, int nadj)
    %
    %Return function that calculates adjoint derivatives.
    %
    %reverse(nadj) returns a cached instance if available, and calls   Function 
    %get_reverse(casadi_int nadj) if no cached version is available.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_oj
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.hpp#L205
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.cpp#L124-L126
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(943, self, varargin{:});
    end
    function varargout = get_reverse(self,varargin)
    %GET_REVERSE [INTERNAL] 
    %
    %  Function = GET_REVERSE(self, int nadj, char name, {char} inames, {char} onames, struct opts)
    %
    %Return function that calculates adjoint derivatives.
    %
    %reverse(nadj) returns a cached instance if available, and calls   Function 
    %get_reverse(casadi_int nadj) if no cached version is available.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_oj
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.hpp#L206
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.cpp#L117-L122
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(944, self, varargin{:});
    end
    function varargout = has_jac_sparsity(self,varargin)
    %HAS_JAC_SPARSITY [INTERNAL] 
    %
    %  bool = HAS_JAC_SPARSITY(self, int oind, int iind)
    %
    %Return sparsity of Jacobian of all input elements.
    %
    %with respect to all output elements
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_ok
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.hpp#L218
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.hpp#L218-L218
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(945, self, varargin{:});
    end
    function varargout = get_jac_sparsity(self,varargin)
    %GET_JAC_SPARSITY [INTERNAL] 
    %
    %  Sparsity = GET_JAC_SPARSITY(self, int oind, int iind, bool symmetric)
    %
    %Return sparsity of Jacobian of all input elements.
    %
    %with respect to all output elements
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_ok
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.hpp#L219
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/callback.hpp#L219-L220
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(946, self, varargin{:});
    end
  end
  methods(Static)
    function varargout = type_name(varargin)
    %TYPE_NAME 
    %
    %  char = TYPE_NAME()
    %
    %
     [varargout{1:nargout}] = casadiMEX(923, varargin{:});
    end
  end
end
