function [ successFlag ] = runTestAPrioriKnownSeq1( doPrint )

    if ( nargin < 1 )
        doPrint = 0;
    end

    successFlag = 1;
    TOL = eps;

    clear H g A lb ub lbA ubA;
	
    try
        load 'benchmarkCRANE1.mat';
    catch
        successFlag = -1;
        return;
    end

    [nC,nV] = size(A);
    [nV,nP] = size(g);

    xOptSeq = zeros(nV,nP);
    objOptSeq = zeros(1,nP);
    statusSeq = zeros(1,nP);
    iterSeq = zeros(1,nP);
    yOptSeq = zeros(nV+nC,nP);

	options = qpOASES_options( 'fast','maxIter',100, 'printLevel',2*doPrint );

	for i=1:nP
		%disp(i);

		if ( i == 1 )
			[QP,x,obj,status,nWSRout,lambda] = qpOASES_sequence( 'i',H,g(:,i),A,lb(:,i),ub(:,i),lbA(:,i),ubA(:,i),options );
		else
			[x,obj,status,nWSRout,lambda] = qpOASES_sequence( 'h',QP,g(:,i),lb(:,i),ub(:,i),lbA(:,i),ubA(:,i),options );
		end

        xOptSeq(:,i) = x;
        objOptSeq(:,i) = obj;
        statusSeq(:,i) = status;
        iterSeq(:,i) = nWSRout;
        yOptSeq(:,i) = lambda;
        
    end
	
	qpOASES_sequence( 'c',QP );
    
    
    [xOptAPKSeq,objOptAPKSeq,statusAPKSeq,iterAPKSeq,yOptAPKSeq] = qpOASES( H,g,A,lb,ub,lbA,ubA,options );
    
    if ( norm( xOptSeq-xOptAPKSeq ) > TOL )
        successFlag = 0;
        if ( doPrint > 0 )
            disp( 'xOpt error' )
        end
    end
    
    if ( norm( objOptSeq-objOptAPKSeq ) > TOL )
        successFlag = 0;
        if ( doPrint > 0 )
            disp( 'objOpt error' )
        end
    end
    
    if ( sum( statusSeq~=statusAPKSeq ) > 0 )
        successFlag = 0;
        if ( doPrint > 0 )
            disp( 'status error' )
        end
    end
    
    if ( sum( iterSeq~=iterAPKSeq ) > 0 )
        successFlag = 0;
        if ( doPrint > 0 )
            disp( 'iter error' )
        end
    end
    
    if ( norm( yOptSeq-yOptAPKSeq ) > TOL )
        successFlag = 0;
        if ( doPrint > 0 )
            disp( 'yOpt error' )
        end
    end
	
end
