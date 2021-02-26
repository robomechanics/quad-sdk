function [ successFlag ] = runTestSparse( doPrint )
    
    if ( nargin < 1 )
        doPrint = 0;
    end

    successFlag = 0;


    m = 50;
    n = 100;

    L = sprand(n, n, 0.03);
    H = L' * L;
    A = sprand(m, n, 0.05);

    lbA = -rand(m,1);
    ubA = rand(m,1); 
    ub = ones(n,1);
    lb = -ones(n,1);
    g = 10*rand(n,1);
    
    options = qpOASES_options( 'default', 'printLevel',2*doPrint );

    [x1,dummy1,exitflag1,iter1] = qpOASES(full(H), g, full(A), lb, ub, lbA, ubA, options);
    [x2,dummy1,exitflag2,iter2] = qpOASES(H, g, A, lb, ub, lbA, ubA, options);
    
    if ( ( exitflag1 == 0 ) && ( exitflag2 == 0 ) && ( iter1 == iter2 ) ...
            && ( norm(x1-x2) < 1e-10 ) )
        successFlag = 1;
    end

end
