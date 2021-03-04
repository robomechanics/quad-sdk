function [ successFlag ] = runTestWorkingSetLI( doPrint )
    
    if ( nargin < 1 )
        doPrint = 0;
    end

    successFlag = 1;
    
    qpFeatures = setupQpFeaturesStruct( );
    
    qpFeatures.nV = 20;
    qpFeatures.nC = 100;
    
    qpFeatures.isSparseH = 0;
    qpFeatures.isSparseA = 0;
    
    qpFeatures.hasLowerB = 1;
    qpFeatures.hasUpperB = 1;
    qpFeatures.hasLowerC = 1;
    qpFeatures.hasUpperC = 1;
    
    qpFeatures.makeInfeas = 1;
    
    options = qpOASES_options( 'default', 'printLevel',2*doPrint, 'initialStatusBounds',0 );
    
    exitflag = 0;
    counter = 0;
    
    while ( ( exitflag ~= -42 ) && ( counter < 100 ) )
        
        counter = counter+1;
        
        qpData = generateRandomQp( qpFeatures );
        B = [ eye( qpFeatures.nV ); qpData.Ain ];
    
        [x,dummy1,exitflag,dummy2,dummy3,auxOutput] = qpOASES( qpData.H,qpData.g,qpData.Ain, ...
                qpData.lb,qpData.ub,qpData.lbA,qpData.ubA, options ); %#ok<*NASGU>
            
        WS = [auxOutput.workingSetB; auxOutput.workingSetC];
        nAct = sum( WS~=0 );
        Bact = B( WS~=0,: );

        if ( nAct ~= rank(Bact) )
            successFlag = 0;
            return;
        end
        
    end
   
end
