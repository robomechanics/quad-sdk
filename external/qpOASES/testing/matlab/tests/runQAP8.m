function [ successFlag ] = runQAP8( doPrint )
    
    if ( nargin < 1 )
        doPrint = 0;
    end

    successFlag = 0;

    try
        load 'QAP8.mat';
    catch
        successFlag = -1;
        return;
    end

    options = qpOASES_options('default', 'maxIter',30000, 'printLevel',-2*doPrint );
    tic
    [x, fval, status, nWSRout, y] = qpOASES(sparse(QP.H), QP.f, ...
        sparse(QP.C), QP.lb, QP.ub, QP.cl, QP.cu, options);
    t = toc;
    
    if ( doPrint > 0 )
        disp( ['solution time: ',num2str(t),' seconds'] );
    end
    
    if ( ( status == 0 ) && ( nWSRout < 23200 ) )
        successFlag = 1;
    end
    
    % check error and print
    if doPrint
        [stat, feas, cmpl] = qpresidual(S.B, S.b1, S.C, S.cl1, S.cu1, x, -y);
        fprintf( '%d iters in %.3fs to tolerance %.2e\n', nWSRout, t, max([stat,feas,cmpl]) );
        fprintf( 'Status: %d\n', status );
    end

end
