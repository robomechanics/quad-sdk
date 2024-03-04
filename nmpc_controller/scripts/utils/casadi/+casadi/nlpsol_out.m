function varargout = nlpsol_out(varargin)
    %NLPSOL_OUT [INTERNAL] 
    %
    %  {char} = NLPSOL_OUT()
    %  char = NLPSOL_OUT(int ind)
    %
    %Get output scheme name by index.
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
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1t1
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/nlpsol.hpp#L243
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/nlpsol.cpp#L243-L254
    %
    %
    %
    %.......
    %
    %::
    %
    %  NLPSOL_OUT()
    %
    %
    %
    %[INTERNAL] 
    %Get NLP solver output scheme of NLP solvers.
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
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1sz
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/nlpsol.hpp#L203
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/nlpsol.cpp#L203-L207
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
    %  NLPSOL_OUT(int ind)
    %
    %
    %
    %[INTERNAL] 
    %Get output scheme name by index.
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
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1t1
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/nlpsol.hpp#L243
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/nlpsol.cpp#L243-L254
    %
    %
    %
    %.............
    %
    %
  [varargout{1:nargout}] = casadiMEX(831, varargin{:});
end
