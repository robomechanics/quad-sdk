function [ successFlag ] = runAlternativeX0Test( nV,nC, doPrint,seed )
    
    if ( nargin < 4 )
        seed = 4242;
        if ( nargin < 3 )
            doPrint = 0;
            if ( nargin < 2 )
                nC = 300;
                if ( nargin < 1 )
                    nV = 50;
                end
            end
        end
    end

    successFlag = 1;
    TOL = eps;
    
    qpData = generateExample( nV,nC, 0,0, 0,1,1,1, seed );
    
    H = qpData.H;
	g = qpData.g;
    A = [qpData.Aeq;qpData.Ain];
	lb = qpData.lb;
    ub = qpData.ub;
    lbA = [qpData.beq;qpData.lbA];
    ubA = [qpData.beq;qpData.ubA];
    
    x0 = nV*rand( nV,1 );
    
    options = qpOASES_options( 'default', 'printLevel',2*doPrint );
    auxInput = qpOASES_auxInput( 'x0',x0 );
    
    [ x1,f1,e1,i1,l1 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,[],x0 );
    [ x2,f2,e2,i2,l2 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,[],auxInput );
    [ x3,f3,e3,i3,l3 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,options,x0 );
    [ x4,f4,e4,i4,l4 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,options,auxInput );
    
    if ( ( norm(x1-x2) > TOL ) || ...
         ( norm(x1-x3) > TOL ) || ...
         ( norm(x1-x4) > TOL ) )
        if ( doPrint > 0 )
            disp('diff in x')
        end
        successFlag = 0;
    end
    
    if ( ( norm(f1-f2) > TOL ) || ...
         ( norm(f1-f3) > TOL ) || ...
         ( norm(f1-f4) > TOL ) )
        if ( doPrint > 0 )
            disp('diff in fval')
        end
        successFlag = 0;
    end
    
    if ( ( norm(e1-e2) > TOL ) || ...
         ( norm(e1-e3) > TOL ) || ...
         ( norm(e1-e4) > TOL ) )
        if ( doPrint > 0 )
            disp('diff in exitflag')
        end
        successFlag = 0;
    end
    
    if ( ( norm(i1-i2) > TOL ) || ...
         ( norm(i1-i3) > TOL ) || ...
         ( norm(i1-i4) > TOL ) )
        if ( doPrint > 0 )
            disp('diff in iter')
        end
        successFlag = 0;
    end
    
    if ( ( norm(l1-l2) > TOL ) || ...
         ( norm(l1-l3) > TOL ) || ...
         ( norm(l1-l4) > TOL ) )
        if ( doPrint > 0 )
            disp('diff in lambda')
        end
        successFlag = 0;
    end
		
end
