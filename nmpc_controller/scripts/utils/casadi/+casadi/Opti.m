classdef  Opti < casadi.PrintableCommon & casadi.SharedObject
    %OPTI [INTERNAL] 
    %
    %
    %A simplified interface for NLP modeling/solving.
    %
    %This class offers a view with model description facilities The API is 
    %
    %guaranteed to be stable.
    %
    %Example NLP:
    %
    %::
    %
    %    opti = casadi.Opti();
    %  
    %    x = opti.variable();
    %    y = opti.variable();
    %  
    %    opti.minimize(  (y-x^2)^2   );
    %    opti.subject_to( x^2+y^2==1 );
    %    opti.subject_to(     x+y>=1 );
    %  
    %    opti.solver('ipopt');
    %    sol = opti.solve();
    %  
    %    sol.value(x)
    %    sol.value(y)
    %
    %
    %
    %Example parametric NLP:
    %
    %::
    %
    %    opti = casadi.Opti();
    %  
    %    x = opti.variable(2,1);
    %    p = opti.parameter();
    %  
    %    opti.minimize(  (p*x(2)-x(1)^2)^2   );
    %    opti.subject_to( 1<=sum(x)<=2 );
    %  
    %    opti.solver('ipopt');
    %  
    %    opti.set_value(p, 3);
    %    sol = opti.solve();
    %    sol.value(x)
    %  
    %    opti.set_value(p, 5);
    %    sol = opti.solve();
    %    sol.value(x)
    %
    %
    %
    %Joris Gillis, Erik Lambrechts, Joel Andersson
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_16
    %
    %C++ includes: optistack.hpp
    %
    %
  methods
    function this = swig_this(self)
      this = casadiMEX(3, self);
    end
    function varargout = internal_variable(self,varargin)
    %INTERNAL_VARIABLE [INTERNAL] 
    %
    %  MX = INTERNAL_VARIABLE(self, int n, int m, char attribute)
    %
    %Create a decision variable (symbol)
    %
    %The order of creation matters. The order will be reflected in the 
    %
    %optimization problem. It is not required for decision variables to 
    %actualy 
    %appear in the optimization problem.
    %
    %Parameters:
    %-----------
    %
    %n: 
    %number of rows (default 1)
    %
    %m: 
    %number of columnss (default 1)
    %
    %attribute: 
    %'full' (default) or 'symmetric'
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_18
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L112
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L51-L57
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1196, self, varargin{:});
    end
    function varargout = internal_parameter(self,varargin)
    %INTERNAL_PARAMETER [INTERNAL] 
    %
    %  MX = INTERNAL_PARAMETER(self, int n, int m, char attribute)
    %
    %Create a parameter (symbol); fixed during optimization.
    %
    %The order of creation does not matter. It is not required for 
    %parameter to 
    %actualy appear in the optimization problem. Parameters 
    %that do appear, must
    % be given a value before the problem can be 
    %solved.
    %
    %Parameters:
    %-----------
    %
    %n: 
    %number of rows (default 1)
    %
    %m: 
    %number of columnss (default 1)
    %
    %attribute: 
    %'full' (default) or 'symmetric'
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_19
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L125
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L74-L80
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1197, self, varargin{:});
    end
    function varargout = minimize(self,varargin)
    %MINIMIZE [INTERNAL] 
    %
    %  MINIMIZE(self, MX f)
    %
    %Set objective.
    %
    %Objective must be a scalar. Default objective: 0 When method is called
    % 
    %multiple times, the last call takes effect
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1a
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L133
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L82-L88
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1198, self, varargin{:});
    end
    function varargout = internal_subject_to(self,varargin)
    %INTERNAL_SUBJECT_TO [INTERNAL] 
    %
    %  INTERNAL_SUBJECT_TO(self)
    %  INTERNAL_SUBJECT_TO(self, MX g)
    %  INTERNAL_SUBJECT_TO(self, {MX} g)
    %
    %Clear constraints.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L166
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L102-L108
    %
    %
    %
    %.......
    %
    %::
    %
    %  INTERNAL_SUBJECT_TO(self, MX g)
    %
    %
    %
    %[INTERNAL] 
    %Add constraints.
    %
    %Examples:
    %
    %::
    %
    %  * \\begin{itemize}
    %  * opti.subject_to( sqrt(x+y) >= 1);
    %  * opti.subject_to( sqrt(x+y) > 1)}: same as above
    %  * opti.subject_to( 1<= sqrt(x+y) )}: same as above
    %  * opti.subject_to( 5*x+y==1 )}: equality
    %  *
    %  * Python
    %  * opti.subject_to([x*y>=1,x==3])
    %  * opti.subject_to(opti.bounded(0,x,1))
    %  *
    %  * MATLAB
    %  * opti.subject_to({x*y>=1,x==3})
    %  * opti.subject_to( 0<=x<=1 )
    %  * 
    %
    %
    %
    %Related functionalities:
    %opti.lbg,opti.g,opti.ubg represent the vector of 
    %flattened constraints
    %
    %opti.debug.show_infeasibilities() may be used to inspect which 
    %constraints 
    %are violated
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1b
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L161
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L90-L96
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
    %  INTERNAL_SUBJECT_TO(self, {MX} g)
    %
    %
    %
    %[INTERNAL] 
    %Add constraints.
    %
    %Examples:
    %
    %::
    %
    %  * \\begin{itemize}
    %  * opti.subject_to( sqrt(x+y) >= 1);
    %  * opti.subject_to( sqrt(x+y) > 1)}: same as above
    %  * opti.subject_to( 1<= sqrt(x+y) )}: same as above
    %  * opti.subject_to( 5*x+y==1 )}: equality
    %  *
    %  * Python
    %  * opti.subject_to([x*y>=1,x==3])
    %  * opti.subject_to(opti.bounded(0,x,1))
    %  *
    %  * MATLAB
    %  * opti.subject_to({x*y>=1,x==3})
    %  * opti.subject_to( 0<=x<=1 )
    %  * 
    %
    %
    %
    %Related functionalities:
    %opti.lbg,opti.g,opti.ubg represent the vector of 
    %flattened constraints
    %
    %opti.debug.show_infeasibilities() may be used to inspect which 
    %constraints 
    %are violated
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1b
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L162
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L98-L100
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
    %  INTERNAL_SUBJECT_TO(self)
    %
    %
    %
    %[INTERNAL] 
    %Clear constraints.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L166
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L102-L108
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(1199, self, varargin{:});
    end
    function varargout = solver(self,varargin)
    %SOLVER [INTERNAL] 
    %
    %  SOLVER(self, char solver, struct plugin_options, struct solver_options)
    %
    %Set a solver.
    %
    %Parameters:
    %-----------
    %
    %solver: 
    %any of the nlpsol plugins can be used here In practice, not all 
    %nlpsol
    % plugins may be supported yet
    %
    %options: 
    %passed on to nlpsol plugin No stability can be guaranteed about 
    %this 
    %part of the API
    %
    %options: 
    %to be passed to nlpsol solver No stability can be guaranteed about
    % 
    %this part of the API
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1c
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L178
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L111-L119
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1200, self, varargin{:});
    end
    function varargout = set_initial(self,varargin)
    %SET_INITIAL [INTERNAL] 
    %
    %  SET_INITIAL(self, {MX} assignments)
    %  SET_INITIAL(self, MX x, DM v)
    %
    %Set initial guess for decision variables
    %
    %::
    %
    %  * opti.set_initial(x, 2)
    %  * opti.set_initial(10*x(1), 2)
    %  * 
    %
    %
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L190
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L128-L134
    %
    %
    %
    %.......
    %
    %::
    %
    %  SET_INITIAL(self, {MX} assignments)
    %
    %
    %
    %[INTERNAL] 
    %Set initial guess for decision variables
    %
    %::
    %
    %  * opti.set_initial(x, 2)
    %  * opti.set_initial(10*x(1), 2)
    %  * 
    %
    %
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L190
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L128-L134
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
    %  SET_INITIAL(self, MX x, DM v)
    %
    %
    %
    %[INTERNAL] 
    %Set initial guess for decision variables
    %
    %::
    %
    %  * opti.set_initial(x, 2)
    %  * opti.set_initial(10*x(1), 2)
    %  * 
    %
    %
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L189
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L121-L127
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(1201, self, varargin{:});
    end
    function varargout = set_value(self,varargin)
    %SET_VALUE [INTERNAL] 
    %
    %  SET_VALUE(self, {MX} assignments)
    %  SET_VALUE(self, MX x, DM v)
    %
    %Set value of parameter.
    %
    %Each parameter must be given a value before 'solve' can be called
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1d
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L200
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L145-L151
    %
    %
    %
    %.......
    %
    %::
    %
    %  SET_VALUE(self, {MX} assignments)
    %
    %
    %
    %[INTERNAL] 
    %Set value of parameter.
    %
    %Each parameter must be given a value before 'solve' can be called
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1d
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L200
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L145-L151
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
    %  SET_VALUE(self, MX x, DM v)
    %
    %
    %
    %[INTERNAL] 
    %Set value of parameter.
    %
    %Each parameter must be given a value before 'solve' can be called
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1d
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L199
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L137-L143
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(1202, self, varargin{:});
    end
    function varargout = solve(self,varargin)
    %SOLVE [INTERNAL] 
    %
    %  OptiSol = SOLVE(self)
    %
    %Crunch the numbers; solve the problem.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L204
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L153-L159
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1203, self, varargin{:});
    end
    function varargout = solve_limited(self,varargin)
    %SOLVE_LIMITED [INTERNAL] 
    %
    %  OptiSol = SOLVE_LIMITED(self)
    %
    %Crunch the numbers; solve the problem.
    %
    %Allows the solver to return without error when an iteration or time 
    %limit 
    %is reached
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1e
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L212
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L161-L167
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1204, self, varargin{:});
    end
    function varargout = value(self,varargin)
    %VALUE [INTERNAL] 
    %
    %  double = VALUE(self, DM x, {MX} values)
    %  double = VALUE(self, SX x, {MX} values)
    %  double = VALUE(self, MX x, {MX} values)
    %
    %Obtain value of expression at the current value
    %
    %In regular mode, teh current value is the converged solution In debug 
    %mode,
    % the value can be non-converged
    %
    %Parameters:
    %-----------
    %
    %values: 
    %Optional assignment expressions (e.g. x==3) to overrule the current
    % 
    %value
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L225
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L186-L192
    %
    %
    %
    %.......
    %
    %::
    %
    %  VALUE(self, DM x, {MX} values)
    %
    %
    %
    %[INTERNAL] 
    %Obtain value of expression at the current value
    %
    %In regular mode, teh current value is the converged solution In debug 
    %mode,
    % the value can be non-converged
    %
    %Parameters:
    %-----------
    %
    %values: 
    %Optional assignment expressions (e.g. x==3) to overrule the current
    % 
    %value
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L224
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L178-L184
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
    %  VALUE(self, SX x, {MX} values)
    %
    %
    %
    %[INTERNAL] 
    %Obtain value of expression at the current value
    %
    %In regular mode, teh current value is the converged solution In debug 
    %mode,
    % the value can be non-converged
    %
    %Parameters:
    %-----------
    %
    %values: 
    %Optional assignment expressions (e.g. x==3) to overrule the current
    % 
    %value
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L225
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L186-L192
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
    %  VALUE(self, MX x, {MX} values)
    %
    %
    %
    %[INTERNAL] 
    %Obtain value of expression at the current value
    %
    %In regular mode, teh current value is the converged solution In debug 
    %mode,
    % the value can be non-converged
    %
    %Parameters:
    %-----------
    %
    %values: 
    %Optional assignment expressions (e.g. x==3) to overrule the current
    % 
    %value
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L223
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L169-L175
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(1205, self, varargin{:});
    end
    function varargout = stats(self,varargin)
    %STATS [INTERNAL] 
    %
    %  struct = STATS(self)
    %
    %Get statistics.
    %
    %nlpsol stats are passed as-is. No stability can be guaranteed about 
    %this 
    %part of the API
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1f
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L234
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L194-L200
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1206, self, varargin{:});
    end
    function varargout = return_status(self,varargin)
    %RETURN_STATUS [INTERNAL] 
    %
    %  char = RETURN_STATUS(self)
    %
    %Get return status of solver.
    %
    %
    %
    %::
    %
    %     passed as-is from nlpsol
    %  
    %
    %No stability can be guaranteed about this part of the API
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1g
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L242
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L202-L208
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1207, self, varargin{:});
    end
    function varargout = initial(self,varargin)
    %INITIAL [INTERNAL] 
    %
    %  {MX} = INITIAL(self)
    %
    %get assignment expressions for initial values
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_266
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L247
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L210-L216
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1208, self, varargin{:});
    end
    function varargout = value_variables(self,varargin)
    %VALUE_VARIABLES [INTERNAL] 
    %
    %  {MX} = VALUE_VARIABLES(self)
    %
    %get assignment expressions for latest values
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_267
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L252
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L218-L224
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1209, self, varargin{:});
    end
    function varargout = value_parameters(self,varargin)
    %VALUE_PARAMETERS [INTERNAL] 
    %
    %  {MX} = VALUE_PARAMETERS(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1210, self, varargin{:});
    end
    function varargout = dual(self,varargin)
    %DUAL [INTERNAL] 
    %
    %  MX = DUAL(self, MX m)
    %
    %get the dual variable
    %
    %m must be a constraint expression. The returned value is still a 
    %symbolic 
    %expression. Use  value on it to obtain the numerical value.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1h
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L262
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L234-L240
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1211, self, varargin{:});
    end
    function varargout = nx(self,varargin)
    %NX [INTERNAL] 
    %
    %  int = NX(self)
    %
    %Number of (scalarised) decision variables.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_268
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L267
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L242-L248
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1212, self, varargin{:});
    end
    function varargout = np(self,varargin)
    %NP [INTERNAL] 
    %
    %  int = NP(self)
    %
    %Number of (scalarised) parameters.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_269
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L272
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L250-L256
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1213, self, varargin{:});
    end
    function varargout = ng(self,varargin)
    %NG [INTERNAL] 
    %
    %  int = NG(self)
    %
    %Number of (scalarised) constraints.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_26a
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L277
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L258-L264
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1214, self, varargin{:});
    end
    function varargout = x(self,varargin)
    %X [INTERNAL] 
    %
    %  MX = X(self)
    %
    %Get all (scalarised) decision variables as a symbolic column 
    %vector.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_26b
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L282
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L266-L272
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1215, self, varargin{:});
    end
    function varargout = p(self,varargin)
    %P [INTERNAL] 
    %
    %  MX = P(self)
    %
    %Get all (scalarised) parameters as a symbolic column vector.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_26c
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L287
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L274-L280
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1216, self, varargin{:});
    end
    function varargout = g(self,varargin)
    %G [INTERNAL] 
    %
    %  MX = G(self)
    %
    %Get all (scalarised) constraint expressions as a column vector.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_26d
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L292
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L282-L288
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1217, self, varargin{:});
    end
    function varargout = f(self,varargin)
    %F [INTERNAL] 
    %
    %  MX = F(self)
    %
    %Get objective expression.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_26e
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L297
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L290-L296
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1218, self, varargin{:});
    end
    function varargout = lbg(self,varargin)
    %LBG [INTERNAL] 
    %
    %  MX = LBG(self)
    %
    %Get all (scalarised) bounds on constraints as a column vector.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_26f
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L302
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L298-L304
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1219, self, varargin{:});
    end
    function varargout = ubg(self,varargin)
    %UBG [INTERNAL] 
    %
    %  MX = UBG(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1220, self, varargin{:});
    end
    function varargout = lam_g(self,varargin)
    %LAM_G [INTERNAL] 
    %
    %  MX = LAM_G(self)
    %
    %Get all (scalarised) dual variables as a symbolic column vector.
    %
    %Useful for obtaining the Lagrange Hessian:
    %
    %::
    %
    %  * sol.value(hessian(opti.f+opti.lam_g'*opti.g,opti.x)) % MATLAB
    %  * sol.value(hessian(opti.f+dot(opti.lam_g,opti.g),opti.x)[0]) # Python
    %  * 
    %
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1i
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L314
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L315-L321
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1221, self, varargin{:});
    end
    function varargout = to_function(self,varargin)
    %TO_FUNCTION [INTERNAL] 
    %
    %  Function = TO_FUNCTION(self, char name, {MX} args, {MX} res, struct opts)
    %  Function = TO_FUNCTION(self, char name, struct:MX dict, {char} name_in, {char} name_out, struct opts)
    %  Function = TO_FUNCTION(self, char name, {MX} args, {MX} res, {char} name_in, {char} name_out, struct opts)
    %
    %Create a CasADi  Function from the  Opti solver.
    %
    %Parameters:
    %-----------
    %
    %name: 
    %Name of the resulting CasADi  Function
    %
    %args: 
    %List of parameters and decision/dual variables (which can be given an
    % 
    %initial guess) with the resulting  Function
    %
    %res: 
    %List of expressions that will get evaluated at the optimal solution
    %
    %opts: 
    %Standard CasADi Funcion options
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1j
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L336
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L341-L361
    %
    %
    %
    %.......
    %
    %::
    %
    %  TO_FUNCTION(self, char name, {MX} args, {MX} res, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %Create a CasADi  Function from the  Opti solver.
    %
    %Parameters:
    %-----------
    %
    %name: 
    %Name of the resulting CasADi  Function
    %
    %args: 
    %List of parameters and decision/dual variables (which can be given an
    % 
    %initial guess) with the resulting  Function
    %
    %res: 
    %List of expressions that will get evaluated at the optimal solution
    %
    %opts: 
    %Standard CasADi Funcion options
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1j
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L326
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L335-L339
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
    %  TO_FUNCTION(self, char name, struct:MX dict, {char} name_in, {char} name_out, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %Create a CasADi  Function from the  Opti solver.
    %
    %Parameters:
    %-----------
    %
    %name: 
    %Name of the resulting CasADi  Function
    %
    %args: 
    %List of parameters and decision/dual variables (which can be given an
    % 
    %initial guess) with the resulting  Function
    %
    %res: 
    %List of expressions that will get evaluated at the optimal solution
    %
    %opts: 
    %Standard CasADi Funcion options
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1j
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L336
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L341-L361
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
    %  TO_FUNCTION(self, char name, {MX} args, {MX} res, {char} name_in, {char} name_out, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %Create a CasADi  Function from the  Opti solver.
    %
    %Parameters:
    %-----------
    %
    %name: 
    %Name of the resulting CasADi  Function
    %
    %args: 
    %List of parameters and decision/dual variables (which can be given an
    % 
    %initial guess) with the resulting  Function
    %
    %res: 
    %List of expressions that will get evaluated at the optimal solution
    %
    %opts: 
    %Standard CasADi Funcion options
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1j
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L330
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L323-L333
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(1222, self, varargin{:});
    end
    function varargout = debug(self,varargin)
    %DEBUG [INTERNAL] 
    %
    %  OptiAdvanced = DEBUG(self)
    %
    %Get a copy with advanced functionality.
    %
    %You get access to more methods, but you have no guarantees about API 
    %
    %stability
    %
    %The copy is effectively a deep copy: Updating the state of the copy 
    %does 
    %not update the original.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1l
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L362
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L733-L735
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1223, self, varargin{:});
    end
    function varargout = advanced(self,varargin)
    %ADVANCED [INTERNAL] 
    %
    %  OptiAdvanced = ADVANCED(self)
    %
    %Get a copy with advanced functionality.
    %
    %You get access to more methods, but you have no guarantees about API 
    %
    %stability
    %
    %The copy is effectively a deep copy: Updating the state of the copy 
    %does 
    %not update the original.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1m
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L372
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L736-L738
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1224, self, varargin{:});
    end
    function varargout = copy(self,varargin)
    %COPY [INTERNAL] 
    %
    %  Opti = COPY(self)
    %
    %Get a copy of the.
    %
    %The copy is effectively a deep copy: Updating the state of the copy 
    %does 
    %not update the original.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1n
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L380
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L739-L741
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1225, self, varargin{:});
    end
    function varargout = update_user_dict(self,varargin)
    %UPDATE_USER_DICT [INTERNAL] 
    %
    %  UPDATE_USER_DICT(self, MX m, struct meta)
    %  UPDATE_USER_DICT(self, {MX} m, struct meta)
    %
    %
    %.......
    %
    %::
    %
    %  UPDATE_USER_DICT(self, MX m, struct meta)
    %
    %
    %
    %[INTERNAL] 
    %add user data
    %
    %Add arbitrary data in the form of a dictionary to symbols or 
    %constraints
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1o
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L388
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L379-L385
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
    %  UPDATE_USER_DICT(self, {MX} m, struct meta)
    %
    %
    %
    %[INTERNAL] 
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(1226, self, varargin{:});
    end
    function varargout = user_dict(self,varargin)
    %USER_DICT [INTERNAL] 
    %
    %  struct = USER_DICT(self, MX m)
    %
    %Get user data.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L391
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L392-L398
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1227, self, varargin{:});
    end
    function varargout = type_name(self,varargin)
    %TYPE_NAME [INTERNAL] 
    %
    %  char = TYPE_NAME(self)
    %
    %Readable name of the class.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L394
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L394-L394
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1228, self, varargin{:});
    end
    function varargout = disp(self,varargin)
    %DISP [INTERNAL] 
    %
    %  std::ostream & = DISP(self, bool more)
    %
    %Print representation.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L397
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L636-L656
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1229, self, varargin{:});
    end
    function varargout = str(self,varargin)
    %STR [INTERNAL] 
    %
    %  char = STR(self, bool more)
    %
    %Get string representation.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L400
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L658-L662
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1230, self, varargin{:});
    end
    function varargout = callback_class(self,varargin)
    %CALLBACK_CLASS [INTERNAL] 
    %
    %  CALLBACK_CLASS(self)
    %  CALLBACK_CLASS(self, OptiCallback callback)
    %
    %Helper methods for callback()
    %
    %Do not use directly.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1p
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L409
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L371-L377
    %
    %
    %
    %.......
    %
    %::
    %
    %  CALLBACK_CLASS(self)
    %
    %
    %
    %[INTERNAL] 
    %Helper methods for callback()
    %
    %Do not use directly.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1p
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L409
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L371-L377
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
    %  CALLBACK_CLASS(self, OptiCallback callback)
    %
    %
    %
    %[INTERNAL] 
    %Helper methods for callback()
    %
    %Do not use directly.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1p
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L408
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L363-L369
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(1231, self, varargin{:});
    end

      function out = variable(self, varargin)
        st = dbstack('-completenames',1);
        if length(st)>0
          meta = struct('stacktrace', st(1));
        else
          meta = struct;
        end
        out = self.internal_variable(varargin{:});
        self.update_user_dict(out, meta);
      end
      function out = parameter(self, varargin)
        st = dbstack('-completenames',1);
        if length(st)>0
          meta = struct('stacktrace', st(1));
        else
          meta = struct;
        end
        out = self.internal_parameter(varargin{:});
        self.update_user_dict(out, meta);
      end
      function [] = subject_to(self, varargin)
        if length(varargin)==0
          self.internal_subject_to();
          return;
        end
        st = dbstack('-completenames',1);
        if length(st)>0
          meta = struct('stacktrace', st(1));
        else
          meta = struct;
        end
        self.internal_subject_to(varargin{:});
        self.update_user_dict(varargin{1}, meta);
      end
    
    function [] = callback(self, varargin)
      casadi.OptiCallbackHelper.callback_setup(self, varargin{:})
    end
      function self = Opti(varargin)
    %OPTI 
    %
    %  new_obj = OPTI(char problem_type)
    %
    %
      self@casadi.PrintableCommon(SwigRef.Null);
      self@casadi.SharedObject(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(1232, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(1233, self);
          self.SwigClear();
        end
    end
  end
  methods(Static)
  end
end
