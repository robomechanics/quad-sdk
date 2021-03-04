function [ successFlag ] = runTestSparse3( doPrint )
    
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
    
    
    [QP,dummy1,dummy2,exitflag1] = qpOASES_sequence( 'i', H, g, A, lb, ub, lbA, ubA, options );

    L = sprand(n, n, 0.03);
    H = L' * L;
    A = sprand(m, n, 0.05);

    [dummy1,dummy2,exitflag2] = qpOASES_sequence( 'm',QP, H, g*2, A, lb, ub, lbA, ubA );
    qpOASES_sequence( 'c',QP );
    
    if ( ( exitflag1 == 0 ) && ( exitflag2 == 0 ) )
        successFlag = 1;
    end
    
end
