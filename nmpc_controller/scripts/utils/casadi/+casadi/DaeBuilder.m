classdef  DaeBuilder < casadi.SharedObject & casadi.PrintableCommon
    %DAEBUILDER [INTERNAL] 
    %
    %
    %A symbolic representation of a differential-algebraic equations 
    %model.
    %
    %Variables:
    %==========
    %
    %
    %
    %
    %
    %::
    %
    %  t:      independent variable (usually time)
    %  c:      constants
    %  p:      parameters
    %  d:      dependent parameters (time independent)
    %  u:      controls
    %  w:      dependent variables  (time dependent)
    %  x:      differential states
    %  z:      algebraic variables
    %  q:      quadrature states
    %  y:      outputs
    %  
    %
    %
    %
    %Equations:
    %==========
    %
    %
    %
    %
    %
    %::
    %
    %  differential equations: \\dot{x} ==  ode(...)
    %  algebraic equations:          0 ==  alg(...)
    %  quadrature equations:   \\dot{q} == quad(...)
    %  dependent parameters:         d == ddef(d_prev,p)
    %  dependent variables:          w == wdef(w_prev,x,z,u,p,t)
    %  output equations:             y == ydef(...)
    %  initial equations:     init_lhs == init_rhs(...)
    %  events:      when when_cond < 0: when_lhs := when_rhs
    %  
    %
    %
    %
    %Joel Andersson
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_5c
    %
    %C++ includes: dae_builder.hpp
    %
    %
  methods
    function this = swig_this(self)
      this = casadiMEX(3, self);
    end
    function varargout = type_name(self,varargin)
    %TYPE_NAME [INTERNAL] 
    %
    %  char = TYPE_NAME(self)
    %
    %Readable name of the class.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L74
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L74-L74
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(998, self, varargin{:});
    end
    function varargout = name(self,varargin)
    %NAME [INTERNAL] 
    %
    %  char = NAME(self)
    %
    %Name of instance.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_5d
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L86
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L59-L61
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(999, self, varargin{:});
    end
    function varargout = t(self,varargin)
    %T [INTERNAL] 
    %
    %  MX = T(self)
    %
    %Independent variable (usually time)
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_5e
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L93
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L63-L65
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1000, self, varargin{:});
    end
    function varargout = x(self,varargin)
    %X [INTERNAL] 
    %
    %  {char} = X(self)
    %
    %Differential states.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_5f
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L98
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L67-L69
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1001, self, varargin{:});
    end
    function varargout = ode(self,varargin)
    %ODE [INTERNAL] 
    %
    %  {MX} = ODE(self)
    %
    %Ordinary differential equations (ODE)
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_5g
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L103
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L71-L73
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1002, self, varargin{:});
    end
    function varargout = z(self,varargin)
    %Z [INTERNAL] 
    %
    %  {char} = Z(self)
    %
    %Algebraic variables.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_5h
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L108
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L75-L77
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1003, self, varargin{:});
    end
    function varargout = alg(self,varargin)
    %ALG [INTERNAL] 
    %
    %  {MX} = ALG(self)
    %
    %Algebraic equations.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_5i
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L113
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L79-L81
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1004, self, varargin{:});
    end
    function varargout = q(self,varargin)
    %Q [INTERNAL] 
    %
    %  {char} = Q(self)
    %
    %Quadrature states.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_5j
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L118
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L83-L85
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1005, self, varargin{:});
    end
    function varargout = quad(self,varargin)
    %QUAD [INTERNAL] 
    %
    %  {MX} = QUAD(self)
    %
    %Quadrature equations.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_5k
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L123
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L87-L89
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1006, self, varargin{:});
    end
    function varargout = y(self,varargin)
    %Y [INTERNAL] 
    %
    %  {char} = Y(self)
    %
    % Output variables.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_5l
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L128
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L91-L93
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1007, self, varargin{:});
    end
    function varargout = ydef(self,varargin)
    %YDEF [INTERNAL] 
    %
    %  {MX} = YDEF(self)
    %
    %Definitions of output variables.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_5m
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L133
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L95-L97
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1008, self, varargin{:});
    end
    function varargout = u(self,varargin)
    %U [INTERNAL] 
    %
    %  {char} = U(self)
    %
    %Free controls.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_5n
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L138
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L99-L101
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1009, self, varargin{:});
    end
    function varargout = p(self,varargin)
    %P [INTERNAL] 
    %
    %  {char} = P(self)
    %
    %Parameters.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_5o
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L143
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L103-L105
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1010, self, varargin{:});
    end
    function varargout = c(self,varargin)
    %C [INTERNAL] 
    %
    %  {char} = C(self)
    %
    %Named constants.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_5p
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L148
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L107-L109
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1011, self, varargin{:});
    end
    function varargout = cdef(self,varargin)
    %CDEF [INTERNAL] 
    %
    %  {MX} = CDEF(self)
    %
    %Definitions of named constants.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_5q
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L153
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L111-L113
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1012, self, varargin{:});
    end
    function varargout = d(self,varargin)
    %D [INTERNAL] 
    %
    %  {char} = D(self)
    %
    %Dependent parameters.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_5r
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L158
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L115-L117
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1013, self, varargin{:});
    end
    function varargout = ddef(self,varargin)
    %DDEF [INTERNAL] 
    %
    %  {MX} = DDEF(self)
    %
    %Definitions of dependent parameters.
    %
    %Interdependencies are allowed but must be non-cyclic.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_5s
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L165
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L119-L121
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1014, self, varargin{:});
    end
    function varargout = w(self,varargin)
    %W [INTERNAL] 
    %
    %  {char} = W(self)
    %
    %Dependent variables.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_5t
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L170
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L123-L125
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1015, self, varargin{:});
    end
    function varargout = wdef(self,varargin)
    %WDEF [INTERNAL] 
    %
    %  {MX} = WDEF(self)
    %
    %Dependent variables and corresponding definitions.
    %
    %Interdependencies are allowed but must be non-cyclic.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_5u
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L177
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L127-L129
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1016, self, varargin{:});
    end
    function varargout = aux(self,varargin)
    %AUX [INTERNAL] 
    %
    %  {MX} = AUX(self)
    %
    %Auxiliary variables: Used e.g. to define functions.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_5v
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L182
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L131-L133
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1017, self, varargin{:});
    end
    function varargout = init_lhs(self,varargin)
    %INIT_LHS [INTERNAL] 
    %
    %  {MX} = INIT_LHS(self)
    %
    %Initial conditions, left-hand-side.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_5w
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L187
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L135-L137
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1018, self, varargin{:});
    end
    function varargout = init_rhs(self,varargin)
    %INIT_RHS [INTERNAL] 
    %
    %  {MX} = INIT_RHS(self)
    %
    %Initial conditions, right-hand-side.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_5x
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L192
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L139-L141
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1019, self, varargin{:});
    end
    function varargout = when_cond(self,varargin)
    %WHEN_COND [INTERNAL] 
    %
    %  {MX} = WHEN_COND(self)
    %
    %When statement: triggering condition.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_5y
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L197
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L143-L145
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1020, self, varargin{:});
    end
    function varargout = when_lhs(self,varargin)
    %WHEN_LHS [INTERNAL] 
    %
    %  {MX} = WHEN_LHS(self)
    %
    %When statement: left-hand-side.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_5z
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L202
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L147-L149
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1021, self, varargin{:});
    end
    function varargout = when_rhs(self,varargin)
    %WHEN_RHS [INTERNAL] 
    %
    %  {MX} = WHEN_RHS(self)
    %
    %When statement: right-hand-side.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_60
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L207
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L151-L153
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1022, self, varargin{:});
    end
    function varargout = outputs(self,varargin)
    %OUTPUTS [INTERNAL] 
    %
    %  {char} = OUTPUTS(self)
    %
    %Model structure: outputs.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_61
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L213
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L155-L162
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1023, self, varargin{:});
    end
    function varargout = derivatives(self,varargin)
    %DERIVATIVES [INTERNAL] 
    %
    %  {char} = DERIVATIVES(self)
    %
    %Model structure: derivatives.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_62
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L218
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L164-L171
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1024, self, varargin{:});
    end
    function varargout = initial_unknowns(self,varargin)
    %INITIAL_UNKNOWNS [INTERNAL] 
    %
    %  {char} = INITIAL_UNKNOWNS(self)
    %
    %Model structure: initial unknowns.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_63
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L223
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L173-L180
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1025, self, varargin{:});
    end
    function varargout = has_t(self,varargin)
    %HAS_T [INTERNAL] 
    %
    %  bool = HAS_T(self)
    %
    %Is there a time variable?
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_64
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L231
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L182-L184
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1026, self, varargin{:});
    end
    function varargout = nx(self,varargin)
    %NX [INTERNAL] 
    %
    %  int = NX(self)
    %
    %Differential states.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_65
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L236
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L186-L188
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1027, self, varargin{:});
    end
    function varargout = nz(self,varargin)
    %NZ [INTERNAL] 
    %
    %  int = NZ(self)
    %
    %Algebraic variables.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_66
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L241
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L190-L192
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1028, self, varargin{:});
    end
    function varargout = nq(self,varargin)
    %NQ [INTERNAL] 
    %
    %  int = NQ(self)
    %
    %Quadrature states.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_67
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L246
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L194-L196
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1029, self, varargin{:});
    end
    function varargout = ny(self,varargin)
    %NY [INTERNAL] 
    %
    %  int = NY(self)
    %
    % Output variables.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_68
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L251
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L198-L200
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1030, self, varargin{:});
    end
    function varargout = nu(self,varargin)
    %NU [INTERNAL] 
    %
    %  int = NU(self)
    %
    %Free controls.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_69
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L256
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L202-L204
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1031, self, varargin{:});
    end
    function varargout = np(self,varargin)
    %NP [INTERNAL] 
    %
    %  int = NP(self)
    %
    %Parameters.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_6a
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L261
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L206-L208
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1032, self, varargin{:});
    end
    function varargout = nc(self,varargin)
    %NC [INTERNAL] 
    %
    %  int = NC(self)
    %
    %Named constants.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_6b
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L266
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L210-L212
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1033, self, varargin{:});
    end
    function varargout = nd(self,varargin)
    %ND [INTERNAL] 
    %
    %  int = ND(self)
    %
    %Dependent parameters.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_6c
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L271
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L214-L216
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1034, self, varargin{:});
    end
    function varargout = nw(self,varargin)
    %NW [INTERNAL] 
    %
    %  int = NW(self)
    %
    %Dependent variables.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_6d
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L276
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L218-L220
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1035, self, varargin{:});
    end
    function varargout = add_t(self,varargin)
    %ADD_T [INTERNAL] 
    %
    %  MX = ADD_T(self, char name)
    %
    %Add an independent variable (time)
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L284
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L430-L435
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1036, self, varargin{:});
    end
    function varargout = add_p(self,varargin)
    %ADD_P [INTERNAL] 
    %
    %  MX = ADD_P(self, char name)
    %
    %Add a new parameter.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L287
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L437-L444
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1037, self, varargin{:});
    end
    function varargout = add_u(self,varargin)
    %ADD_U [INTERNAL] 
    %
    %  MX = ADD_U(self, char name)
    %
    %Add a new control.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L290
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L446-L453
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1038, self, varargin{:});
    end
    function varargout = add_x(self,varargin)
    %ADD_X [INTERNAL] 
    %
    %  MX = ADD_X(self, char name)
    %
    %Add a new differential state.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L293
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L455-L462
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1039, self, varargin{:});
    end
    function varargout = add_z(self,varargin)
    %ADD_Z [INTERNAL] 
    %
    %  MX = ADD_Z(self, char name)
    %
    %Add a new algebraic variable.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L296
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L464-L471
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1040, self, varargin{:});
    end
    function varargout = add_q(self,varargin)
    %ADD_Q [INTERNAL] 
    %
    %  MX = ADD_Q(self, char name)
    %
    %Add a new quadrature state.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L299
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L473-L480
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1041, self, varargin{:});
    end
    function varargout = add_c(self,varargin)
    %ADD_C [INTERNAL] 
    %
    %  MX = ADD_C(self, char name, MX new_cdef)
    %
    %Add a new constant.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L302
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L482-L489
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1042, self, varargin{:});
    end
    function varargout = add_d(self,varargin)
    %ADD_D [INTERNAL] 
    %
    %  MX = ADD_D(self, char name, MX new_ddef)
    %
    %Add a new dependent parameter.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L305
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L491-L498
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1043, self, varargin{:});
    end
    function varargout = add_w(self,varargin)
    %ADD_W [INTERNAL] 
    %
    %  MX = ADD_W(self, char name, MX new_wdef)
    %
    %Add a new dependent variable.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L308
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L500-L507
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1044, self, varargin{:});
    end
    function varargout = add_y(self,varargin)
    %ADD_Y [INTERNAL] 
    %
    %  MX = ADD_Y(self, char name, MX new_ydef)
    %
    %Add a new output.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L311
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L509-L516
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1045, self, varargin{:});
    end
    function varargout = set_ode(self,varargin)
    %SET_ODE [INTERNAL] 
    %
    %  SET_ODE(self, char name, MX ode_rhs)
    %
    %Specify the ordinary differential equation for a state.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L314
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L535-L541
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1046, self, varargin{:});
    end
    function varargout = set_alg(self,varargin)
    %SET_ALG [INTERNAL] 
    %
    %  SET_ALG(self, char name, MX alg_rhs)
    %
    %Specificy the residual equation for an algebraic variable.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L317
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L543-L549
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1047, self, varargin{:});
    end
    function varargout = add_aux(self,varargin)
    %ADD_AUX [INTERNAL] 
    %
    %  MX = ADD_AUX(self, char name, int n)
    %
    %Add an auxiliary variable.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L320
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L518-L522
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1048, self, varargin{:});
    end
    function varargout = add_init(self,varargin)
    %ADD_INIT [INTERNAL] 
    %
    %  ADD_INIT(self, MX lhs, MX rhs)
    %
    %Add an initial equation.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L323
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L524-L527
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1049, self, varargin{:});
    end
    function varargout = add_when(self,varargin)
    %ADD_WHEN [INTERNAL] 
    %
    %  ADD_WHEN(self, MX cond, MX lhs, MX rhs)
    %
    %Add a when statement.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L326
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L529-L533
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1050, self, varargin{:});
    end
    function varargout = sanity_check(self,varargin)
    %SANITY_CHECK [INTERNAL] 
    %
    %  SANITY_CHECK(self)
    %
    %Check if dimensions match.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L329
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L551-L557
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1051, self, varargin{:});
    end
    function varargout = clear_all(self,varargin)
    %CLEAR_ALL [INTERNAL] 
    %
    %  CLEAR_ALL(self, char v)
    %
    %Clear all variables of a type.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L333
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L414-L420
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1052, self, varargin{:});
    end
    function varargout = set_all(self,varargin)
    %SET_ALL [INTERNAL] 
    %
    %  SET_ALL(self, char v, {char} name)
    %
    %Set all variables of a type.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L336
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L422-L428
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1053, self, varargin{:});
    end
    function varargout = register_t(self,varargin)
    %REGISTER_T [INTERNAL] 
    %
    %  REGISTER_T(self, char name)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1054, self, varargin{:});
    end
    function varargout = register_p(self,varargin)
    %REGISTER_P [INTERNAL] 
    %
    %  REGISTER_P(self, char name)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1055, self, varargin{:});
    end
    function varargout = register_u(self,varargin)
    %REGISTER_U [INTERNAL] 
    %
    %  REGISTER_U(self, char name)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1056, self, varargin{:});
    end
    function varargout = register_x(self,varargin)
    %REGISTER_X [INTERNAL] 
    %
    %  REGISTER_X(self, char name)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1057, self, varargin{:});
    end
    function varargout = register_z(self,varargin)
    %REGISTER_Z [INTERNAL] 
    %
    %  REGISTER_Z(self, char name)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1058, self, varargin{:});
    end
    function varargout = register_q(self,varargin)
    %REGISTER_Q [INTERNAL] 
    %
    %  REGISTER_Q(self, char name)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1059, self, varargin{:});
    end
    function varargout = register_c(self,varargin)
    %REGISTER_C [INTERNAL] 
    %
    %  REGISTER_C(self, char name)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1060, self, varargin{:});
    end
    function varargout = register_d(self,varargin)
    %REGISTER_D [INTERNAL] 
    %
    %  REGISTER_D(self, char name)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1061, self, varargin{:});
    end
    function varargout = register_w(self,varargin)
    %REGISTER_W [INTERNAL] 
    %
    %  REGISTER_W(self, char name)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1062, self, varargin{:});
    end
    function varargout = register_y(self,varargin)
    %REGISTER_Y [INTERNAL] 
    %
    %  REGISTER_Y(self, char name)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1063, self, varargin{:});
    end
    function varargout = set_u(self,varargin)
    %SET_U [INTERNAL] 
    %
    %  SET_U(self, {char} name)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1064, self, varargin{:});
    end
    function varargout = set_x(self,varargin)
    %SET_X [INTERNAL] 
    %
    %  SET_X(self, {char} name)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1065, self, varargin{:});
    end
    function varargout = set_z(self,varargin)
    %SET_Z [INTERNAL] 
    %
    %  SET_Z(self, {char} name, {char} alg)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1066, self, varargin{:});
    end
    function varargout = set_q(self,varargin)
    %SET_Q [INTERNAL] 
    %
    %  SET_Q(self, {char} name)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1067, self, varargin{:});
    end
    function varargout = set_y(self,varargin)
    %SET_Y [INTERNAL] 
    %
    %  SET_Y(self, {char} name)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1068, self, varargin{:});
    end
    function varargout = clear_in(self,varargin)
    %CLEAR_IN [DEPRECATED] Clear input variable: Replaced by clear_all
    %
    %  CLEAR_IN(self, char v)
    %
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L371
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L371-L371
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1069, self, varargin{:});
    end
    function varargout = eliminate_w(self,varargin)
    %ELIMINATE_W [INTERNAL] 
    %
    %  ELIMINATE_W(self)
    %
    %Eliminate all dependent variables.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L375
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L611-L617
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1070, self, varargin{:});
    end
    function varargout = lift(self,varargin)
    %LIFT [INTERNAL] 
    %
    %  LIFT(self, bool lift_shared, bool lift_calls)
    %
    %Lift problem formulation by extracting shared subexpressions.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L378
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L619-L625
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1071, self, varargin{:});
    end
    function varargout = eliminate_quad(self,varargin)
    %ELIMINATE_QUAD [INTERNAL] 
    %
    %  ELIMINATE_QUAD(self)
    %
    %Eliminate quadrature states and turn them into ODE states.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L381
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L248-L254
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1072, self, varargin{:});
    end
    function varargout = sort_d(self,varargin)
    %SORT_D [INTERNAL] 
    %
    %  SORT_D(self)
    %
    %Sort dependent parameters.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L384
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L256-L262
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1073, self, varargin{:});
    end
    function varargout = sort_w(self,varargin)
    %SORT_W [INTERNAL] 
    %
    %  SORT_W(self)
    %
    %Sort dependent variables.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L387
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L264-L270
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1074, self, varargin{:});
    end
    function varargout = sort_z(self,varargin)
    %SORT_Z [INTERNAL] 
    %
    %  SORT_Z(self, {char} z_order)
    %
    %Sort algebraic variables.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L390
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L272-L278
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1075, self, varargin{:});
    end
    function varargout = prune(self,varargin)
    %PRUNE [INTERNAL] 
    %
    %  PRUNE(self, bool prune_p, bool prune_u)
    %
    %Prune unused controls.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L393
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L280-L286
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1076, self, varargin{:});
    end
    function varargout = tear(self,varargin)
    %TEAR [INTERNAL] 
    %
    %  TEAR(self)
    %
    %Identify iteration variables and residual equations using naming
    % 
    %convention.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L396
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L288-L294
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1077, self, varargin{:});
    end
    function varargout = add_fun(self,varargin)
    %ADD_FUN [INTERNAL] 
    %
    %  Function = ADD_FUN(self, Function f)
    %  Function = ADD_FUN(self, char name, Importer compiler, struct opts)
    %  Function = ADD_FUN(self, char name, {char} arg, {char} res, struct opts)
    %
    %Add an external function.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L413
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L770-L773
    %
    %
    %
    %.......
    %
    %::
    %
    %  ADD_FUN(self, Function f)
    %
    %
    %
    %[INTERNAL] 
    %Add an already existing function.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L410
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L751-L758
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
    %  ADD_FUN(self, char name, {char} arg, {char} res, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %Add a function from loaded expressions.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L405
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L760-L768
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
    %  ADD_FUN(self, char name, Importer compiler, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %Add an external function.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L413
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L770-L773
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(1078, self, varargin{:});
    end
    function varargout = has_fun(self,varargin)
    %HAS_FUN [INTERNAL] 
    %
    %  bool = HAS_FUN(self, char name)
    %
    %Does a particular function already exist?
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L417
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L775-L782
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1079, self, varargin{:});
    end
    function varargout = fun(self,varargin)
    %FUN [INTERNAL] 
    %
    %  {Function} = FUN(self)
    %  Function = FUN(self, char name)
    %
    %Get all functions.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L423
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L814-L816
    %
    %
    %
    %.......
    %
    %::
    %
    %  FUN(self, char name)
    %
    %
    %
    %[INTERNAL] 
    %Get function by name.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L420
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L784-L791
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
    %  FUN(self)
    %
    %
    %
    %[INTERNAL] 
    %Get all functions.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L423
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L814-L816
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(1080, self, varargin{:});
    end
    function varargout = gather_fun(self,varargin)
    %GATHER_FUN [INTERNAL] 
    %
    %  GATHER_FUN(self, int max_depth)
    %
    %Collect embedded functions from the expression graph.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L426
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L793-L812
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1081, self, varargin{:});
    end
    function varargout = parse_fmi(self,varargin)
    %PARSE_FMI [INTERNAL] 
    %
    %  PARSE_FMI(self, char filename)
    %
    %Import existing problem from FMI/XML
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L433
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L433-L433
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1082, self, varargin{:});
    end
    function varargout = provides_directional_derivative(self,varargin)
    %PROVIDES_DIRECTIONAL_DERIVATIVE [INTERNAL] 
    %
    %  bool = PROVIDES_DIRECTIONAL_DERIVATIVE(self)
    %
    %Does the FMU provide support for analytic derivatives.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L436
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L230-L238
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1083, self, varargin{:});
    end
    function varargout = load_fmi_description(self,varargin)
    %LOAD_FMI_DESCRIPTION [INTERNAL] 
    %
    %  LOAD_FMI_DESCRIPTION(self, char filename)
    %
    %Import problem description from FMI or XML.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L439
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L222-L228
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1084, self, varargin{:});
    end
    function varargout = export_fmu(self,varargin)
    %EXPORT_FMU [INTERNAL] 
    %
    %  {char} = EXPORT_FMU(self, struct opts)
    %
    %Export instance into an FMU.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L442
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L240-L246
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1085, self, varargin{:});
    end
    function varargout = add_lc(self,varargin)
    %ADD_LC [INTERNAL] 
    %
    %  ADD_LC(self, char name, {char} f_out)
    %
    %Add a named linear combination of output expressions.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L445
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L711-L718
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1086, self, varargin{:});
    end
    function varargout = create(self,varargin)
    %CREATE [INTERNAL] 
    %
    %  Function = CREATE(self, char name, struct opts)
    %  Function = CREATE(self, char name, {char} name_in, {char} name_out, struct opts)
    %  Function = CREATE(self, char fname, {char} name_in, {char} name_out, bool sx, bool lifted_calls)
    %
    %Load a function from an FMU DLL, standard IO conforming with 
    %
    %simulator.
    %
    %Parameters:
    %-----------
    %
    %name: 
    %Name assigned to the resulting function object
    %
    %opts: 
    %Optional settings
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_6f
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L472
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L742-L749
    %
    %
    %
    %.......
    %
    %::
    %
    %  CREATE(self, char name, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %Load a function from an FMU DLL, standard IO conforming with 
    %
    %simulator.
    %
    %Parameters:
    %-----------
    %
    %name: 
    %Name assigned to the resulting function object
    %
    %opts: 
    %Optional settings
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_6f
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L472
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L742-L749
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
    %  CREATE(self, char name, {char} name_in, {char} name_out, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %Construct a function object, names provided.
    %
    %Parameters:
    %-----------
    %
    %name: 
    %Name assigned to the resulting function object
    %
    %name_in: 
    %Names of all the inputs
    %
    %name_out: 
    %Names of all the outputs
    %
    %opts: 
    %Optional settings
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_6e
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L460
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L731-L740
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
    %  CREATE(self, char fname, {char} name_in, {char} name_out, bool sx, bool lifted_calls)
    %
    %
    %
    %[INTERNAL] 
    %Construct a function object, legacy syntax.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L448
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L720-L729
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(1087, self, varargin{:});
    end
    function varargout = dependent_fun(self,varargin)
    %DEPENDENT_FUN [INTERNAL] 
    %
    %  Function = DEPENDENT_FUN(self, char fname, {char} s_in, {char} s_out)
    %
    %Construct a function for evaluating dependent parameters.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L475
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L827-L836
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1088, self, varargin{:});
    end
    function varargout = var(self,varargin)
    %VAR [INTERNAL] 
    %
    %  MX = VAR(self, char name)
    %
    %Get variable expression by name
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L481
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L559-L566
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1089, self, varargin{:});
    end
    function varargout = paren(self,varargin)
    %PAREN 
    %
    %  MX = PAREN(self, char name)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1090, self, varargin{:});
    end
    function varargout = der(self,varargin)
    %DER [INTERNAL] 
    %
    %  {char} = DER(self, {char} name)
    %
    %Get the time derivative of an expression.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L486
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L583-L592
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1091, self, varargin{:});
    end
    function varargout = beq(self,varargin)
    %BEQ [INTERNAL] 
    %
    %  MX = BEQ(self, char name)
    %
    %Get/set the binding equation for a variable
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L490
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L594-L601
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1092, self, varargin{:});
    end
    function varargout = set_beq(self,varargin)
    %SET_BEQ [INTERNAL] 
    %
    %  SET_BEQ(self, char name, MX val)
    %
    %Get/set the binding equation for a variable
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L491
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L603-L609
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1093, self, varargin{:});
    end
    function varargout = value_reference(self,varargin)
    %VALUE_REFERENCE [INTERNAL] 
    %
    %  int = VALUE_REFERENCE(self, char name)
    %
    %Get/set value reference
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L496
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L627-L629
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1094, self, varargin{:});
    end
    function varargout = set_value_reference(self,varargin)
    %SET_VALUE_REFERENCE [INTERNAL] 
    %
    %  SET_VALUE_REFERENCE(self, char name, int val)
    %
    %Get/set value reference
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L497
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L631-L633
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1095, self, varargin{:});
    end
    function varargout = description(self,varargin)
    %DESCRIPTION [INTERNAL] 
    %
    %  char = DESCRIPTION(self, char name)
    %
    %Get/set description
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L502
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L635-L637
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1096, self, varargin{:});
    end
    function varargout = set_description(self,varargin)
    %SET_DESCRIPTION [INTERNAL] 
    %
    %  SET_DESCRIPTION(self, char name, char val)
    %
    %Get/set description
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L503
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L639-L641
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1097, self, varargin{:});
    end
    function varargout = type(self,varargin)
    %TYPE [INTERNAL] 
    %
    %  char = TYPE(self, char name, int fmi_version)
    %
    %Get/set the type
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L508
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L643-L652
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1098, self, varargin{:});
    end
    function varargout = set_type(self,varargin)
    %SET_TYPE [INTERNAL] 
    %
    %  SET_TYPE(self, char name, char val)
    %
    %Get/set the type
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L509
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L654-L661
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1099, self, varargin{:});
    end
    function varargout = causality(self,varargin)
    %CAUSALITY [INTERNAL] 
    %
    %  char = CAUSALITY(self, char name)
    %
    %Get/set the causality
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L514
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L663-L665
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1100, self, varargin{:});
    end
    function varargout = set_causality(self,varargin)
    %SET_CAUSALITY [INTERNAL] 
    %
    %  SET_CAUSALITY(self, char name, char val)
    %
    %Get/set the causality
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L515
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L667-L669
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1101, self, varargin{:});
    end
    function varargout = variability(self,varargin)
    %VARIABILITY [INTERNAL] 
    %
    %  char = VARIABILITY(self, char name)
    %
    %Get/set the variability
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L520
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L671-L673
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1102, self, varargin{:});
    end
    function varargout = set_variability(self,varargin)
    %SET_VARIABILITY [INTERNAL] 
    %
    %  SET_VARIABILITY(self, char name, char val)
    %
    %Get/set the variability
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L521
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L675-L677
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1103, self, varargin{:});
    end
    function varargout = initial(self,varargin)
    %INITIAL [INTERNAL] 
    %
    %  char = INITIAL(self, char name)
    %
    %Get/set the initial property
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L526
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L679-L681
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1104, self, varargin{:});
    end
    function varargout = set_initial(self,varargin)
    %SET_INITIAL [INTERNAL] 
    %
    %  SET_INITIAL(self, char name, char val)
    %
    %Get/set the initial property
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L527
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L683-L685
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1105, self, varargin{:});
    end
    function varargout = unit(self,varargin)
    %UNIT [INTERNAL] 
    %
    %  char = UNIT(self, char name)
    %
    %Get/set the unit
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L532
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L687-L689
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1106, self, varargin{:});
    end
    function varargout = set_unit(self,varargin)
    %SET_UNIT [INTERNAL] 
    %
    %  SET_UNIT(self, char name, char val)
    %
    %Get/set the unit
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L533
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L691-L693
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1107, self, varargin{:});
    end
    function varargout = display_unit(self,varargin)
    %DISPLAY_UNIT [INTERNAL] 
    %
    %  char = DISPLAY_UNIT(self, char name)
    %
    %Get/set the display unit
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L538
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L695-L697
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1108, self, varargin{:});
    end
    function varargout = set_display_unit(self,varargin)
    %SET_DISPLAY_UNIT [INTERNAL] 
    %
    %  SET_DISPLAY_UNIT(self, char name, char val)
    %
    %Get/set the display unit
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L539
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L699-L701
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1109, self, varargin{:});
    end
    function varargout = numel(self,varargin)
    %NUMEL [INTERNAL] 
    %
    %  int = NUMEL(self, char name)
    %
    %Get the number of elements of a variable.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L543
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L703-L705
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1110, self, varargin{:});
    end
    function varargout = dimension(self,varargin)
    %DIMENSION [INTERNAL] 
    %
    %  [int] = DIMENSION(self, char name)
    %
    %Get the dimensions of a variable.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L546
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L707-L709
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1111, self, varargin{:});
    end
    function varargout = attribute(self,varargin)
    %ATTRIBUTE [INTERNAL] 
    %
    %  [double] = ATTRIBUTE(self, char a, {char} name)
    %
    %Get an attribute.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L599
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L949-L957
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1112, self, varargin{:});
    end
    function varargout = set_attribute(self,varargin)
    %SET_ATTRIBUTE [INTERNAL] 
    %
    %  SET_ATTRIBUTE(self, char a, {char} name, [double] val)
    %
    %Set an attribute.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L602
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L967-L974
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1113, self, varargin{:});
    end
    function varargout = min(self,varargin)
    %MIN [INTERNAL] 
    %
    %  [double] = MIN(self, {char} name)
    %
    %Get the lower bound.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L606
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L985-L992
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1114, self, varargin{:});
    end
    function varargout = set_min(self,varargin)
    %SET_MIN [INTERNAL] 
    %
    %  SET_MIN(self, {char} name, [double] val)
    %
    %Set the lower bound.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L609
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L1002-L1008
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1115, self, varargin{:});
    end
    function varargout = max(self,varargin)
    %MAX [INTERNAL] 
    %
    %  [double] = MAX(self, {char} name)
    %
    %Get the upper bound.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L612
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L1019-L1026
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1116, self, varargin{:});
    end
    function varargout = set_max(self,varargin)
    %SET_MAX [INTERNAL] 
    %
    %  SET_MAX(self, {char} name, [double] val)
    %
    %Set the upper bound.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L615
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L1036-L1042
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1117, self, varargin{:});
    end
    function varargout = nominal(self,varargin)
    %NOMINAL [INTERNAL] 
    %
    %  [double] = NOMINAL(self, {char} name)
    %
    %Get the nominal value.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L618
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L1053-L1060
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1118, self, varargin{:});
    end
    function varargout = set_nominal(self,varargin)
    %SET_NOMINAL [INTERNAL] 
    %
    %  SET_NOMINAL(self, {char} name, [double] val)
    %
    %Set the nominal value.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L621
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L1070-L1076
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1119, self, varargin{:});
    end
    function varargout = start(self,varargin)
    %START [INTERNAL] 
    %
    %  [double] = START(self, {char} name)
    %
    %Get the start attribute.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L624
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L1088-L1095
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1120, self, varargin{:});
    end
    function varargout = set_start(self,varargin)
    %SET_START [INTERNAL] 
    %
    %  SET_START(self, {char} name, [double] val)
    %
    %Set the start attribute.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L627
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L1105-L1111
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1121, self, varargin{:});
    end
    function varargout = set(self,varargin)
    %SET [INTERNAL] 
    %
    %  SET(self, {char} name, [double] val)
    %  SET(self, {char} name, {char} val)
    %
    %Set the current value (string)
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L633
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L1145-L1152
    %
    %
    %
    %.......
    %
    %::
    %
    %  SET(self, {char} name, {char} val)
    %
    %
    %
    %[INTERNAL] 
    %Set the current value (string)
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L633
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L1145-L1152
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
    %  SET(self, {char} name, [double] val)
    %
    %
    %
    %[INTERNAL] 
    %Set the current value.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L630
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L1137-L1143
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(1122, self, varargin{:});
    end
    function varargout = get(self,varargin)
    %GET [INTERNAL] 
    %
    %  {GenericType} = GET(self, {char} name)
    %
    %Evaluate the values for a set of variables at the initial time.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L636
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L1158-L1173
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1123, self, varargin{:});
    end
    function varargout = add_variable(self,varargin)
    %ADD_VARIABLE [INTERNAL] 
    %
    %  ADD_VARIABLE(self, MX new_v)
    %  MX = ADD_VARIABLE(self, char name, int n)
    %  MX = ADD_VARIABLE(self, char name, Sparsity sp)
    %
    %Add a new variable from symbolic expressions.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L645
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L332-L335
    %
    %
    %
    %.......
    %
    %::
    %
    %  ADD_VARIABLE(self, MX new_v)
    %
    %
    %
    %[INTERNAL] 
    %Add a new variable from symbolic expressions.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L645
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L332-L335
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
    %  ADD_VARIABLE(self, char name, int n)
    %
    %
    %
    %[INTERNAL] 
    %Add a new variable: returns corresponding symbolic expression.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L639
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L322-L324
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
    %  ADD_VARIABLE(self, char name, Sparsity sp)
    %
    %
    %
    %[INTERNAL] 
    %Add a new variable: returns corresponding symbolic expression.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L642
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L326-L330
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(1124, self, varargin{:});
    end
    function varargout = add_variable_new(self,varargin)
    %ADD_VARIABLE_NEW [INTERNAL] 
    %
    %  size_t = ADD_VARIABLE_NEW(self, MX new_v)
    %  size_t = ADD_VARIABLE_NEW(self, char name, int n)
    %  size_t = ADD_VARIABLE_NEW(self, char name, Sparsity sp)
    %
    %Add a new variable from symbolic expressions.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L654
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L347-L351
    %
    %
    %
    %.......
    %
    %::
    %
    %  ADD_VARIABLE_NEW(self, MX new_v)
    %
    %
    %
    %[INTERNAL] 
    %Add a new variable from symbolic expressions.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L654
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L347-L351
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
    %  ADD_VARIABLE_NEW(self, char name, int n)
    %
    %
    %
    %[INTERNAL] 
    %Add a new variable: returns corresponding symbolic expression.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L648
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L337-L339
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
    %  ADD_VARIABLE_NEW(self, char name, Sparsity sp)
    %
    %
    %
    %[INTERNAL] 
    %Add a new variable: returns corresponding symbolic expression.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L651
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L341-L345
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(1125, self, varargin{:});
    end
    function varargout = has_variable(self,varargin)
    %HAS_VARIABLE [INTERNAL] 
    %
    %  bool = HAS_VARIABLE(self, char name)
    %
    %Check if a particular variable exists.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L657
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L296-L303
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1126, self, varargin{:});
    end
    function varargout = all_variables(self,varargin)
    %ALL_VARIABLES [INTERNAL] 
    %
    %  {char} = ALL_VARIABLES(self)
    %
    %Get a list of all variables.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L660
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L305-L312
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1127, self, varargin{:});
    end
    function varargout = oracle(self,varargin)
    %ORACLE [INTERNAL] 
    %
    %  Function = ORACLE(self, bool sx, bool elim_w, bool lifted_calls)
    %
    %Get the (cached) oracle, SX or  MX.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L663
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L818-L825
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1128, self, varargin{:});
    end
    function varargout = jac_sparsity(self,varargin)
    %JAC_SPARSITY [INTERNAL] 
    %
    %  Sparsity = JAC_SPARSITY(self, {char} onames, {char} inames)
    %
    %Get Jacobian sparsity.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_6g
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L668
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L1175-L1183
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1129, self, varargin{:});
    end
    function self = DaeBuilder(varargin)
    %DAEBUILDER 
    %
    %  new_obj = DAEBUILDER()
    %  new_obj = DAEBUILDER(char name, char path, struct opts)
    %
    %
    %.......
    %
    %::
    %
    %  DAEBUILDER(char name, char path, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %Construct a  DaeBuilder instance.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L80
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L54-L57
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
    %  DAEBUILDER()
    %
    %
    %
    %[INTERNAL] 
    %Default constructor.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.hpp#L77
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dae_builder.cpp#L51-L52
    %
    %
    %
    %.............
    %
    %
      self@casadi.SharedObject(SwigRef.Null);
      self@casadi.PrintableCommon(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(1130, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(1131, self);
          self.SwigClear();
        end
    end
  end
  methods(Static)
  end
end
