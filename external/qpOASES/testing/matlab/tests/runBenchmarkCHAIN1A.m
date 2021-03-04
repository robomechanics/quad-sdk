function [ successFlag ] = runBenchmarkCHAIN1A( nWSR,doPrint )

	if ( nargin < 2 )
		doPrint = 0;
	end

    successFlag = 0;
	maxViolation = 0;

    clear H g A lb ub lbA ubA;
    
    try
        load 'benchmarkCHAIN1A.mat';
    catch
        successFlag = -1;
        return;
    end
	
    if ( exist( 'A','var' ) )
        [nC,nV] = size(A);
    else
        nC = 0;
    end
    [nV,nP] = size(g);

    xOpt = zeros(nV,nP);
    yOpt = zeros(nV+nC,nP);
    objOpt = zeros(1,nP);
    
	options = qpOASES_options( 'fast','maxIter',nWSR, 'printLevel',2*doPrint );
    %options = qpOASES_options( 'reliable', 'maxIter',500,'printLevel',1 );
% ,'enableRamping',0,'enableFlippingBounds',0,'enableFullLITests',0,'enableDriftCorrection',0
  
  
	for jj=1:1
  
	for i=1:nP
		%disp(i);
		if ( i == 1 )
            [QP,x,obj,status,nWSRout,lambda] = qpOASES_sequence( 'i',H,g(:,i),lb(:,i),ub(:,i),options );
		else
            [x,obj,status,nWSRout,lambda] = qpOASES_sequence( 'h',QP,g(:,i),lb(:,i),ub(:,i),options );
        end

		[ maxViolationTMP ] = getKktResidual( H,g(:,i),[],lb(:,i),ub(:,i),[],[], x,lambda );
		maxViolation = max( [maxViolation,maxViolationTMP] );

        xOpt(:,i) = x;
        yOpt(:,i) = lambda;
        objOpt(:,i) = obj;
	end
	
	qpOASES_sequence( 'c',QP );

    end
    
    if ( ( maxViolation < 1e-14 ) && ( status == 0 ) )
        successFlag = 1;
    end   
	
end
