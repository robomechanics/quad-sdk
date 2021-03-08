function [ successFlag ] = runAlexInfeas2( doPrint )
    
    if ( nargin < 1 )
        doPrint = 0;
    end

    successFlag = 0;
    
    try
        data = load( 'alexInfeas2.mat' );
    catch
        successFlag = -1;
        return;
    end
    
    options1 = qpOASES_options( 'default', 'printLevel',-2*doPrint,... 
                                           'maxIter',7000, 'enableFlippingBounds',0,...
                                           'terminationTolerance',1e7*eps);
                                       
    options2 = qpOASES_options( 'MPC', 'printLevel',-2*doPrint,... 
                                       'maxIter',3000, 'terminationTolerance',1e8*eps );
    
    [x1,dummy,exitflag1] = qpOASES( data.H,data.g,data.A, ...
                data.lb,data.ub,data.lbA,data.ubA, options1 ); %#ok<*NASGU>

    % should return "QP infeasible"
    if ( exitflag1 == -2 )
        successFlag = 1;
    else
        if ( doPrint > 0 )
            %disp( [data.lbA data.A*x1 data.ubA] );
        end
    end
   
end
