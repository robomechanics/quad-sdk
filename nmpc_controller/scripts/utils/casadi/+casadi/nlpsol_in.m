function varargout = nlpsol_in(varargin)
    %NLPSOL_IN [INTERNAL] 
    %
    %  {char} = NLPSOL_IN()
    %  char = NLPSOL_IN(int ind)
    %
    %Get NLP solver input scheme name by index.
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
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1t0
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/nlpsol.hpp#L228
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/nlpsol.cpp#L228-L241
    %
    %
    %
    %.......
    %
    %::
    %
    %  NLPSOL_IN()
    %
    %
    %
    %[INTERNAL] 
    %Get input scheme of NLP solvers.
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
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1sy
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/nlpsol.hpp#L197
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/nlpsol.cpp#L197-L201
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
    %  NLPSOL_IN(int ind)
    %
    %
    %
    %[INTERNAL] 
    %Get NLP solver input scheme name by index.
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
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1t0
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/nlpsol.hpp#L228
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/nlpsol.cpp#L228-L241
    %
    %
    %
    %.............
    %
    %
  [varargout{1:nargout}] = casadiMEX(830, varargin{:});
end
