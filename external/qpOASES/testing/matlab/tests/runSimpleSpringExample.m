function [ successFlag ] = runSimpleSpringExample( doPrint )
    
    if ( nargin < 1 )
        doPrint = 0;
    end

    successFlag = 0;
    
    % parameter
    k1 = 100;
    k2 = 100;

    m = 1;
    g = 9.81;

    % QP data
    H = [ k1,0; 0,k2 ];
    g = [ m*g; 0 ];

    lb = [ 0; 0 ];
    ub = [ 1; 1 ];

    A = [ 1,1 ];
    lbA = 1;
    ubA = 1;

    options = qpOASES_options( 'default', 'printLevel',2*doPrint );
    [x,fval,exitflag,iter] = qpOASES( H,g,A,lb,ub,lbA,ubA,options );

    if ( ( exitflag == 0 ) && ( iter < 5 ) )
        successFlag = 1;
    end
    
end
