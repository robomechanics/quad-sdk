function [ qpData ] = setupQpDataStruct( )

    qpData = struct(    'H', [], ...        % Hessian matrix
                        'g', [], ...        % gradient vector
                        'Aeq', [], ...      % equality constraints matrix
                        'beq', [], ...      % equality constraints vector
                        'lb', [], ...       % lower bound vector
                        'ub', [], ...       % upper bound vector
                        'Ain', [], ...      % inequality constraints matrix
                        'lbA', [], ...      % lower constraints vector
                        'ubA', [], ...      % upper constraints vector
                        'x0', [], ...       % primal initial guess
                        'options', [], ...  % QP solver options
                        'nV', 0, ...        % number of QP variables
                        'nC', 0 ...         % number of constraints
                        );

end
