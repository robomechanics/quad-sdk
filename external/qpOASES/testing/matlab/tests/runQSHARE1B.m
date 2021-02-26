function [ successFlag ] = runQSHARE1B( doPrint )
    
    if ( nargin < 1 )
        doPrint = 0;
    end

    successFlag = 0;

    try
        load 'QSHARE1B.mat';
    catch
        successFlag = -1;
        return;
    end
    
    options = qpOASES_options('default', 'maxIter',600, 'maxCpuTime',2.0, 'printLevel',-2*doPrint );
    auxInput = qpOASES_auxInput( 'hessianType',[] );
    tic
    [xD,fvalD,exitflagD,iterD,lambdaD] = qpOASES( H,g,A,lb,ub,lbA,ubA,options,auxInput );
    tD = toc;
    kktD = getKktResidual( H,g,A,lb,ub,lbA,ubA, xD,lambdaD );
    
    if ( doPrint > 0 )
        disp( ['dense kkt tol:        ',num2str(kktD, '%.3e')] );
        disp( ['dense solution time:  ',num2str(tD),' seconds'] );
		disp( ['dense #iterations:    ',num2str(iterD),] );
    end
    
    tic
    [xS,fvalS,exitflagS,iterS,lambdaS] = qpOASES( sparse(H),g,sparse(A),lb,ub,lbA,ubA,options );
    tS = toc;
    kktS = getKktResidual( H,g,A,lb,ub,lbA,ubA, xS,lambdaS );
    
    if ( doPrint > 0 )
        disp( ['sparse kkt tol:       ',num2str(kktS, '%.3e')] );
        disp( ['sparse solution time: ',num2str(tS),' seconds'] );
		disp( ['sparse #iterations:   ',num2str(iterS),] );
    end
    
    if ( ( exitflagD == 0 ) && ( kktD < 1e-6 ) && ( exitflagS == 0 ) && ( kktS < 1e-6 ) )
        successFlag = 1;
    end
    
end
