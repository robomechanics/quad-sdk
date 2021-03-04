function [ successFlag ] = runExternalCholeskyTests( doPrint )

    if ( nargin < 1 )
        doPrint = 0;
    end
    
    successFlag = 1;
    
    TOL = 1e4*eps;
	
    load 'benchmarkCHAIN1.mat';
    
	options  = qpOASES_options( 'MPC', 'printLevel',2*doPrint );
    auxInput = qpOASES_auxInput( 'R',chol(H) );
    
    %% test qpOASES
    [ x1,obj1,status1,nWSRout1,lambda1,auxOut1 ] = qpOASES( H,g(:,10),A,lb(:,10),ub(:,10),lbA(:,10),ubA(:,10),options );
    [ x2,obj2,status2,nWSRout2,lambda2,auxOut2 ] = qpOASES( H,g(:,10),A,lb(:,10),ub(:,10),lbA(:,10),ubA(:,10),options,auxInput );
    
    if doPrint
        disp( ['Runtime without R: ', num2str(auxOut1.cpuTime),'s'] )
        disp( ['Runtime with    R: ', num2str(auxOut2.cpuTime),'s'] )
        disp( ' ' );
    end
    
    if ( status1 ~= 0 ) || ( status2 ~= 0 )
        if doPrint
            disp( 'error' );
        end
        successFlag = 0;
    end
    
    if ( norm(x1-x2) > TOL )
        if doPrint
            disp( 'error in primal solution'  );
        end
        successFlag = 0;
    end
    
    if ( norm(obj1-obj2) > TOL )
        if doPrint
            disp( 'error in objective function value'  );
        end
        successFlag = 0;
    end
    
    if ( norm(nWSRout1-nWSRout2) > 0.5 )
        if doPrint
            disp( 'error in numer of iterations'  );
        end
        successFlag = 0;
    end
    
    if ( norm(lambda1-lambda2) > TOL )
        if doPrint
            disp( 'error in dual solution'  );
        end
        successFlag = 0;
    end
    
    
    %% test qpOASES_sequence
    [QP,x0,obj0,status0,nWSRout0,lambda0,auxOut1] = qpOASES_sequence( 'i',H,g(:,1),A,lb(:,1),ub(:,1),lbA(:,1),ubA(:,1),options );
	[x1,obj1,status1,nWSRout1,lambda1]            = qpOASES_sequence( 'h',QP,g(:,2),lb(:,2),ub(:,2),lbA(:,2),ubA(:,2),options );
    qpOASES_sequence( 'c',QP );
    
    [QP,x0,obj0,status0,nWSRout0,lambda0,auxOut2] = qpOASES_sequence( 'i',H,g(:,1),A,lb(:,1),ub(:,1),lbA(:,1),ubA(:,1),options,auxInput );
	[x2,obj2,status2,nWSRout2,lambda2]            = qpOASES_sequence( 'h',QP,g(:,2),lb(:,2),ub(:,2),lbA(:,2),ubA(:,2),options );
    qpOASES_sequence( 'c',QP );
    
    if doPrint
        disp( ['Runtime without R: ', num2str(auxOut1.cpuTime),'s'] )
        disp( ['Runtime with    R: ', num2str(auxOut2.cpuTime),'s'] )
        disp( ' ' )
    end
    
    if ( status1 ~= 0 ) || ( status2 ~= 0 )
        if doPrint
            disp( 'error'  );
        end
        successFlag = 0;
    end
    
    if ( norm(x1-x2) > TOL )
        if doPrint
            disp( 'error in primal solution'  );
        end
        successFlag = 0;
    end
    
    if ( norm(obj1-obj2) > TOL )
        if doPrint
            disp( 'error in objective function value'  );
        end
        successFlag = 0;
    end
    
    if ( norm(nWSRout1-nWSRout2) > 0.5 )
        if doPrint
            disp( 'error in numer of iterations'  );
        end
        successFlag = 0;
    end
    
    if ( norm(lambda1-lambda2) > TOL )
        if doPrint
            disp( 'error in dual solution'  );
        end
        successFlag = 0;
    end

        
    %% test simply bounded QP
    load 'benchmarkCHAIN1A.mat';
    
	options  = qpOASES_options( 'MPC', 'printLevel',0*doPrint );
    auxInput = qpOASES_auxInput( 'R',chol(H) );
    
    %% test qpOASES
    [ x1,obj1,status1,nWSRout1,lambda1,auxOut1 ] = qpOASES( H,g(:,10),lb(:,10),ub(:,10),options );
    [ x2,obj2,status2,nWSRout2,lambda2,auxOut2 ] = qpOASES( H,g(:,10),lb(:,10),ub(:,10),options,auxInput );
    
    if doPrint
        disp( ['Runtime without R: ', num2str(auxOut1.cpuTime),'s'] )
        disp( ['Runtime with    R: ', num2str(auxOut2.cpuTime),'s'] )
        disp( ' ' );
    end
    
    if ( status1 ~= 0 ) || ( status2 ~= 0 )
        if doPrint
            disp( 'error' );
        end
        successFlag = 0;
    end
    
    if ( norm(x1-x2) > TOL )
        if doPrint
            disp( 'error in primal solution'  );
        end
        successFlag = 0;
    end
    
    if ( norm(obj1-obj2) > TOL )
        if doPrint
            disp( 'error in objective function value'  );
        end
        successFlag = 0;
    end
    
    if ( norm(nWSRout1-nWSRout2) > 0.5 )
        if doPrint
            disp( 'error in numer of iterations'  );
        end
        successFlag = 0;
    end
    
    if ( norm(lambda1-lambda2) > TOL )
        if doPrint
            disp( 'error in dual solution'  );
        end
        successFlag = 0;
    end
    
    %% test qpOASES_sequence
    [QP,x0,obj0,status0,nWSRout0,lambda0,auxOut1] = qpOASES_sequence( 'i',H,g(:,1),lb(:,1),ub(:,1),options );
	[x1,obj1,status1,nWSRout1,lambda1]            = qpOASES_sequence( 'h',QP,g(:,2),lb(:,2),ub(:,2),options );
    qpOASES_sequence( 'c',QP );
    
    [QP,x0,obj0,status0,nWSRout0,lambda0,auxOut2] = qpOASES_sequence( 'i',H,g(:,1),lb(:,1),ub(:,1),options,auxInput );
	[x2,obj2,status2,nWSRout2,lambda2]            = qpOASES_sequence( 'h',QP,g(:,2),lb(:,2),ub(:,2),options );
    qpOASES_sequence( 'c',QP );
    
    if doPrint
        disp( ['Runtime without R: ', num2str(auxOut1.cpuTime),'s'] )
        disp( ['Runtime with    R: ', num2str(auxOut2.cpuTime),'s'] )
        disp( ' ' )
    end
    
    if ( status1 ~= 0 ) || ( status2 ~= 0 )
        if doPrint
            disp( 'error'  );
        end
        successFlag = 0;
    end
    
    if ( norm(x1-x2) > TOL )
        if doPrint
            disp( 'error in primal solution'  );
        end
        successFlag = 0;
    end
    
    if ( norm(obj1-obj2) > TOL )
        if doPrint
            disp( 'error in objective function value'  );
        end
        successFlag = 0;
    end
    
    if ( norm(nWSRout1-nWSRout2) > 0.5 )
        if doPrint
            disp( 'error in numer of iterations'  );
        end
        successFlag = 0;
    end
    
    if ( norm(lambda1-lambda2) > TOL )
        if doPrint
            disp( 'error in dual solution'  );
        end
        successFlag = 0;
    end

end
