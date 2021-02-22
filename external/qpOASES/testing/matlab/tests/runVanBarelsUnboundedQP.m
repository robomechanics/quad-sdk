function [ successFlag ] = runVanBarelsUnboundedQP( doPrint )
    
    if ( nargin < 1 )
        doPrint = 0;
    end

    successFlag = 0;
    
    try
        data = load( 'vanBarelsUnboundedQP.mat' );
    catch
        successFlag = -1;
        return;
    end
    
    options1 = qpOASES_options( 'default', 'printLevel',-1*doPrint );
    
    [dummy1,dummy2,exitflag1,iter1] = qpOASES( data.H,data.g,data.lb,data.ub,options1 ); %#ok<*NASGU>

    % should return "QP unbounded"
    if ( exitflag1 == -3 )
        successFlag = 1;
    end
   
end
