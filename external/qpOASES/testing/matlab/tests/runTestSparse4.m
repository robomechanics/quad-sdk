function [ successFlag ] = runTestSparse4( doPrint )
    
    if ( nargin < 1 )
        doPrint = 0;
    end

    successFlag = 0;
    
    
    n = 100;

    L = sprand(n, n, 0.03);
    H = L' * L;

    ub = ones(n,1);
    lb = -ones(n,1);
    g = 10*rand(n,1);
    
    options = qpOASES_options( 'default', 'printLevel',2*doPrint );

    [QP,dummy1,dummy2,exitflag1] = qpOASES_sequence( 'i', H, g, lb, ub, options );
    [dummy1,dummy2,exitflag2] = qpOASES_sequence( 'h',QP, g*2, lb, ub );
    qpOASES_sequence( 'c',QP );

    if ( ( exitflag1 == 0 ) && ( exitflag2 == 0 ) )
        successFlag = 1;
    end

end
