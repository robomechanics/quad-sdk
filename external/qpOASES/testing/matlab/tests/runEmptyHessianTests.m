function [ successFlag ] = runEmptyHessianTests( doPrint )

    if ( nargin < 1 )
        doPrint = 0;
    end
    
    successFlag = 1;
    
    TOL = 100*eps;
	
	options = qpOASES_options( 'maxIter',100, 'enableEqualities',1, 'printLevel',-2*doPrint );
    auxInput0   = qpOASES_auxInput( 'hessianType',0 );
    auxInput1   = qpOASES_auxInput( 'hessianType',1 );
    auxInputErr = qpOASES_auxInput( 'hessianType',2 );

    
    load 'benchmarkEXAMPLE1.mat';
    
    %% test qpOASES with zero Hessian
    [ x1,obj1,status1,nWSRout1,lambda1 ] = qpOASES( 0*H,g(:,1),A,lb(:,1),ub(:,1),lbA(:,1),ubA(:,1),options );
    [ x2,obj2,status2,nWSRout2,lambda2 ] = qpOASES( [],g(:,1),A,lb(:,1),ub(:,1),lbA(:,1),ubA(:,1),options );
    [ x3,obj3,status3,nWSRout3,lambda3 ] = qpOASES( [],g(:,1),A,lb(:,1),ub(:,1),lbA(:,1),ubA(:,1),options,auxInput0 );
    [ x4,obj4,status4,nWSRout4,lambda4 ] = qpOASES( [],g(:,1),A,lb(:,1),ub(:,1),lbA(:,1),ubA(:,1),options,auxInputErr );
    
    if ( status1 ~= 0 ) || ( status2 ~= 0 ) || ( status3 ~= 0 ) || ( status4 ~= 0 )
        if doPrint
            disp( 'error'  );
        end
        successFlag = 0;
    end
    
    if ( norm(x1-x2) > TOL ) || ( norm(x1-x3) > TOL ) || ( norm(x1-x4) > TOL )
        if doPrint
            disp( 'error in primal solution'  );
        end
        successFlag = 0;
    end
    
    if ( norm(obj1-obj2) > TOL ) || ( norm(obj1-obj3) > TOL ) || ( norm(obj1-obj4) > TOL )
        if doPrint
            disp( 'error in objective function value'  );
        end
        successFlag = 0;
    end
    
    if ( norm(lambda1-lambda2) > TOL ) || ( norm(lambda1-lambda3) > TOL ) || ( norm(lambda1-lambda4) > TOL )
        if doPrint
            disp( 'error in dual solution'  );
        end
        successFlag = 0;
    end
    
    
    %% test qpOASES_sequence with zero Hessian
    [QP,x0,obj0,status0,nWSRout0,lambda0] = qpOASES_sequence( 'i',0*H,g(:,1),A,lb(:,1),ub(:,1),lbA(:,1),ubA(:,1),options ); %#ok<NASGU>
	[x1,obj1,status1,nWSRout1,lambda1]    = qpOASES_sequence( 'h',QP,g(:,2),lb(:,2),ub(:,2),lbA(:,2),ubA(:,2),options );
    qpOASES_sequence( 'c',QP );
    
    [QP,x0,obj0,status0,nWSRout0,lambda0] = qpOASES_sequence( 'i',[],g(:,1),A,lb(:,1),ub(:,1),lbA(:,1),ubA(:,1),options ); %#ok<NASGU>
	[x2,obj2,status2,nWSRout2,lambda2]    = qpOASES_sequence( 'h',QP,g(:,2),lb(:,2),ub(:,2),lbA(:,2),ubA(:,2),options );
    qpOASES_sequence( 'c',QP );
    
    [QP,x0,obj0,status0,nWSRout0,lambda0] = qpOASES_sequence( 'i',[],g(:,1),A,lb(:,1),ub(:,1),lbA(:,1),ubA(:,1),options,auxInput0 ); %#ok<NASGU>
	[x3,obj3,status3,nWSRout3,lambda3]    = qpOASES_sequence( 'h',QP,g(:,2),lb(:,2),ub(:,2),lbA(:,2),ubA(:,2),options );
    qpOASES_sequence( 'c',QP );
    
    [QP,x0,obj0,status0,nWSRout0,lambda0] = qpOASES_sequence( 'i',[],g(:,1),A,lb(:,1),ub(:,1),lbA(:,1),ubA(:,1),options,auxInputErr ); %#ok<NASGU>
	[x4,obj4,status4,nWSRout4,lambda4]    = qpOASES_sequence( 'h',QP,g(:,2),lb(:,2),ub(:,2),lbA(:,2),ubA(:,2),options );
    qpOASES_sequence( 'c',QP );
    
    if ( status1 ~= 0 ) || ( status2 ~= 0 ) || ( status3 ~= 0 ) || ( status4 ~= 0 )
        if doPrint
            disp( 'error'  );
        end
        successFlag = 0;
    end
    
    if ( norm(x1-x2) > TOL ) || ( norm(x1-x3) > TOL ) || ( norm(x1-x4) > TOL )
        if doPrint
            disp( 'error in primal solution'  );
        end
        successFlag = 0;
    end
    
    if ( norm(obj1-obj2) > TOL ) || ( norm(obj1-obj3) > TOL ) || ( norm(obj1-obj4) > TOL )
        if doPrint
            disp( 'error in objective function value'  );
        end
        successFlag = 0;
    end
    
    if ( norm(lambda1-lambda2) > TOL ) || ( norm(lambda1-lambda3) > TOL ) || ( norm(lambda1-lambda4) > TOL )
        if doPrint
            disp( 'error in dual solution'  );
        end
        successFlag = 0;
    end
    
    
    %% test qpOASES with ID Hessian
    [ x1,obj1,status1,nWSRout1,lambda1 ] = qpOASES( eye(2),g(:,1),A,lb(:,1),ub(:,1),lbA(:,1),ubA(:,1),options );
    [ x2,obj2,status2,nWSRout2,lambda2 ] = qpOASES( [],g(:,1),A,lb(:,1),ub(:,1),lbA(:,1),ubA(:,1),options,auxInput1 );
    
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
    
    
    %% test qpOASES_sequence with ID Hessian
    [QP,x0,obj0,status0,nWSRout0,lambda0] = qpOASES_sequence( 'i',eye(2),g(:,1),A,lb(:,1),ub(:,1),lbA(:,1),ubA(:,1),options ); %#ok<NASGU>
	[x1,obj1,status1,nWSRout1,lambda1]    = qpOASES_sequence( 'h',QP,g(:,2),lb(:,2),ub(:,2),lbA(:,2),ubA(:,2),options );
    qpOASES_sequence( 'c',QP );
    
    [QP,x0,obj0,status0,nWSRout0,lambda0] = qpOASES_sequence( 'i',[],g(:,1),A,lb(:,1),ub(:,1),lbA(:,1),ubA(:,1),options,auxInput1 ); %#ok<NASGU>
	[x2,obj2,status2,nWSRout2,lambda2]    = qpOASES_sequence( 'h',QP,g(:,2),lb(:,2),ub(:,2),lbA(:,2),ubA(:,2),options );
    qpOASES_sequence( 'c',QP );
    
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
    
    
    load 'benchmarkEXAMPLE1B.mat';
    
    %% test qpOASES (simply bounded) with zero Hessian
    [ x1,obj1,status1,nWSRout1,lambda1 ] = qpOASES( 0*H,g(:,1),lb(:,1),ub(:,1),options );
    [ x2,obj2,status2,nWSRout2,lambda2 ] = qpOASES( [],g(:,1),lb(:,1),ub(:,1),options );
    [ x3,obj3,status3,nWSRout3,lambda3 ] = qpOASES( [],g(:,1),lb(:,1),ub(:,1),options,auxInput0 );
    [ x4,obj4,status4,nWSRout4,lambda4 ] = qpOASES( [],g(:,1),lb(:,1),ub(:,1),options,auxInputErr );
    
    if ( status1 ~= 0 ) || ( status2 ~= 0 ) || ( status3 ~= 0 ) || ( status4 ~= 0 )
        if doPrint
            disp( 'error'  );
        end
        successFlag = 0;
    end
    
    if ( norm(x1-x2) > TOL ) || ( norm(x1-x3) > TOL ) || ( norm(x1-x4) > TOL )
        if doPrint
            disp( 'error in primal solution'  );
        end
        successFlag = 0;
    end
    
    if ( norm(obj1-obj2) > TOL ) || ( norm(obj1-obj3) > TOL ) || ( norm(obj1-obj4) > TOL )
        if doPrint
            disp( 'error in objective function value'  );
        end
        successFlag = 0;
    end
    
    if ( norm(lambda1-lambda2) > TOL ) || ( norm(lambda1-lambda3) > TOL ) || ( norm(lambda1-lambda4) > TOL )
        if doPrint
            disp( 'error in dual solution'  );
        end
        successFlag = 0;
    end
    
    
    %% test qpOASES (simply bounded) with ID Hessian
    [ x1,obj1,status1,nWSRout1,lambda1 ] = qpOASES( eye(2),g(:,1),lb(:,1),ub(:,1),options );
    [ x2,obj2,status2,nWSRout2,lambda2 ] = qpOASES( [],g(:,1),lb(:,1),ub(:,1),options,auxInput1 );
    
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
