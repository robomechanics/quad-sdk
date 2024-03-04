classdef  OptiAdvanced < casadi.Opti
    %OPTIADVANCED [INTERNAL] C++ includes: optistack.hpp
    %
    %
    %
    %
  methods
    function delete(self)
        if self.swigPtr
          casadiMEX(1263, self);
          self.SwigClear();
        end
    end
    function varargout = solver(self,varargin)
    %SOLVER [INTERNAL] 
    %
    %  Function = SOLVER(self)
    %
    %Get the underlying CasADi solver of the  Opti stack.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L511
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L400-L406
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1264, self, varargin{:});
    end
    function varargout = is_parametric(self,varargin)
    %IS_PARAMETRIC [INTERNAL] 
    %
    %  bool = IS_PARAMETRIC(self, MX expr)
    %
    %return true if expression is only dependant on  Opti parameters,
    % not variables
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L514
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L408-L414
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1265, self, varargin{:});
    end
    function varargout = symvar(self,varargin)
    %SYMVAR [INTERNAL] 
    %
    %  {MX} = SYMVAR(self)
    %  {MX} = SYMVAR(self, MX expr)
    %  {MX} = SYMVAR(self, MX expr, casadi::VariableType type)
    %
    %Get symbols present in expression.
    %
    %Returned vector is ordered according to the order of  variable()/parameter()
    % calls used to create the variables
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1u
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L525
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L432-L438
    %
    %
    %
    %.......
    %
    %::
    %
    %  SYMVAR(self)
    %
    %
    %
    %[INTERNAL] 
    %Get symbols present in expression.
    %
    %Returned vector is ordered according to the order of  variable()/parameter()
    % calls used to create the variables
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1u
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L523
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L416-L422
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
    %  SYMVAR(self, MX expr)
    %
    %
    %
    %[INTERNAL] 
    %Get symbols present in expression.
    %
    %Returned vector is ordered according to the order of  variable()/parameter()
    % calls used to create the variables
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1u
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L524
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L424-L430
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
    %  SYMVAR(self, MX expr, casadi::VariableType type)
    %
    %
    %
    %[INTERNAL] 
    %Get symbols present in expression.
    %
    %Returned vector is ordered according to the order of  variable()/parameter()
    % calls used to create the variables
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1u
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L525
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L432-L438
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(1266, self, varargin{:});
    end
    function varargout = canon_expr(self,varargin)
    %CANON_EXPR [INTERNAL] 
    %
    %  MetaCon = CANON_EXPR(self, MX expr)
    %
    %Interpret an expression (for internal use only)
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L529
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L440-L446
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1267, self, varargin{:});
    end
    function varargout = get_meta(self,varargin)
    %GET_META [INTERNAL] 
    %
    %  MetaVar = GET_META(self, MX m)
    %
    %Get meta-data of symbol (for internal use only)
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L532
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L448-L454
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1268, self, varargin{:});
    end
    function varargout = get_meta_con(self,varargin)
    %GET_META_CON [INTERNAL] 
    %
    %  MetaCon = GET_META_CON(self, MX m)
    %
    %Get meta-data of symbol (for internal use only)
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L535
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L456-L462
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1269, self, varargin{:});
    end
    function varargout = set_meta(self,varargin)
    %SET_META [INTERNAL] 
    %
    %  SET_META(self, MX m, MetaVar meta)
    %
    %Set meta-data of an expression.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L538
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L464-L470
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1270, self, varargin{:});
    end
    function varargout = set_meta_con(self,varargin)
    %SET_META_CON [INTERNAL] 
    %
    %  SET_META_CON(self, MX m, MetaCon meta)
    %
    %Set meta-data of an expression.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L541
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L472-L478
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1271, self, varargin{:});
    end
    function varargout = assert_active_symbol(self,varargin)
    %ASSERT_ACTIVE_SYMBOL [INTERNAL] 
    %
    %  ASSERT_ACTIVE_SYMBOL(self, MX m)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1272, self, varargin{:});
    end
    function varargout = active_symvar(self,varargin)
    %ACTIVE_SYMVAR [INTERNAL] 
    %
    %  {MX} = ACTIVE_SYMVAR(self, casadi::VariableType type)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1273, self, varargin{:});
    end
    function varargout = active_values(self,varargin)
    %ACTIVE_VALUES [INTERNAL] 
    %
    %  {DM} = ACTIVE_VALUES(self, casadi::VariableType type)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1274, self, varargin{:});
    end
    function varargout = x_lookup(self,varargin)
    %X_LOOKUP [INTERNAL] 
    %
    %  MX = X_LOOKUP(self, index i)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1275, self, varargin{:});
    end
    function varargout = g_lookup(self,varargin)
    %G_LOOKUP [INTERNAL] 
    %
    %  MX = G_LOOKUP(self, index i)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1276, self, varargin{:});
    end
    function varargout = x_describe(self,varargin)
    %X_DESCRIBE [INTERNAL] 
    %
    %  char = X_DESCRIBE(self, index i)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1277, self, varargin{:});
    end
    function varargout = g_describe(self,varargin)
    %G_DESCRIBE [INTERNAL] 
    %
    %  char = G_DESCRIBE(self, index i)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1278, self, varargin{:});
    end
    function varargout = describe(self,varargin)
    %DESCRIBE [INTERNAL] 
    %
    %  char = DESCRIBE(self, MX x, index indent)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1279, self, varargin{:});
    end
    function varargout = show_infeasibilities(self,varargin)
    %SHOW_INFEASIBILITIES [INTERNAL] 
    %
    %  SHOW_INFEASIBILITIES(self, double tol)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1280, self, varargin{:});
    end
    function varargout = solve_prepare(self,varargin)
    %SOLVE_PREPARE [INTERNAL] 
    %
    %  SOLVE_PREPARE(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1281, self, varargin{:});
    end
    function varargout = solve_actual(self,varargin)
    %SOLVE_ACTUAL [INTERNAL] 
    %
    %  struct:DM = SOLVE_ACTUAL(self, struct:DM args)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1282, self, varargin{:});
    end
    function varargout = arg(self,varargin)
    %ARG [INTERNAL] 
    %
    %  struct:DM = ARG(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1283, self, varargin{:});
    end
    function varargout = res(self,varargin)
    %RES [INTERNAL] 
    %
    %  struct:DM = RES(self)
    %  RES(self, struct:DM res)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1284, self, varargin{:});
    end
    function varargout = constraints(self,varargin)
    %CONSTRAINTS [INTERNAL] 
    %
    %  {MX} = CONSTRAINTS(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1285, self, varargin{:});
    end
    function varargout = objective(self,varargin)
    %OBJECTIVE [INTERNAL] 
    %
    %  MX = OBJECTIVE(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1286, self, varargin{:});
    end
    function varargout = baked_copy(self,varargin)
    %BAKED_COPY [INTERNAL] 
    %
    %  OptiAdvanced = BAKED_COPY(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1287, self, varargin{:});
    end
    function varargout = assert_empty(self,varargin)
    %ASSERT_EMPTY [INTERNAL] 
    %
    %  ASSERT_EMPTY(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1288, self, varargin{:});
    end
    function varargout = bake(self,varargin)
    %BAKE [INTERNAL] 
    %
    %  BAKE(self)
    %
    %Fix the structure of the optimization problem.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L572
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L664-L670
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1289, self, varargin{:});
    end
    function v = problem_dirty_(self)
      v = casadiMEX(1290, self);
    end
    function varargout = mark_problem_dirty(self,varargin)
    %MARK_PROBLEM_DIRTY [INTERNAL] 
    %
    %  MARK_PROBLEM_DIRTY(self, bool flag)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1291, self, varargin{:});
    end
    function varargout = problem_dirty(self,varargin)
    %PROBLEM_DIRTY [INTERNAL] 
    %
    %  bool = PROBLEM_DIRTY(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1292, self, varargin{:});
    end
    function v = solver_dirty_(self)
      v = casadiMEX(1293, self);
    end
    function varargout = mark_solver_dirty(self,varargin)
    %MARK_SOLVER_DIRTY [INTERNAL] 
    %
    %  MARK_SOLVER_DIRTY(self, bool flag)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1294, self, varargin{:});
    end
    function varargout = solver_dirty(self,varargin)
    %SOLVER_DIRTY [INTERNAL] 
    %
    %  bool = SOLVER_DIRTY(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1295, self, varargin{:});
    end
    function v = solved_(self)
      v = casadiMEX(1296, self);
    end
    function varargout = mark_solved(self,varargin)
    %MARK_SOLVED [INTERNAL] 
    %
    %  MARK_SOLVED(self, bool flag)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1297, self, varargin{:});
    end
    function varargout = solved(self,varargin)
    %SOLVED [INTERNAL] 
    %
    %  bool = SOLVED(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1298, self, varargin{:});
    end
    function varargout = assert_solved(self,varargin)
    %ASSERT_SOLVED [INTERNAL] 
    %
    %  ASSERT_SOLVED(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1299, self, varargin{:});
    end
    function varargout = assert_baked(self,varargin)
    %ASSERT_BAKED [INTERNAL] 
    %
    %  ASSERT_BAKED(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1300, self, varargin{:});
    end
    function varargout = instance_number(self,varargin)
    %INSTANCE_NUMBER [INTERNAL] 
    %
    %  int = INSTANCE_NUMBER(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1301, self, varargin{:});
    end
    function self = OptiAdvanced(varargin)
    %OPTIADVANCED 
    %
    %  new_obj = OPTIADVANCED(Opti x)
    %
    %
      self@casadi.Opti(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(1302, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
