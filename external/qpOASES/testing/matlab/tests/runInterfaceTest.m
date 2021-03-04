function [ successFlag ] = runInterfaceTest( nV,nC, doPrint,seed )

    if ( nargin < 4 )
        seed = 42;
        if ( nargin < 3 )
            doPrint = 1;
            if ( nargin < 2 )
                nC = 10;
                if ( nargin < 1 )
                    nV = 5;
                end
            end
        end
    end
    
    successFlag = 1;

    for isSparseH=0:1
        for isSparseA=0:1
            successFlag = runSeveralInterfaceTests( successFlag, nV,nC,isSparseH,isSparseA, doPrint,seed );
        end
    end
    
end



function [ successFLAG ] = runSeveralInterfaceTests( successFLAG, nV,nC, isSparseH,isSparseA, doPrint,seed )

    %% test without or empty A matrix
    for hasA=0:1
        for hasLowerB=0:1
            for hasUpperB=0:1
                for hasOptions=0:2
                    for hasX0=0:1
                        for hasWS=0:2
                            if ( ( hasWS ~= 2 ) || ( hasA ~= 0 ) || ( hasOptions == 1 ) || ( hasX0 ~= 0 ) )
                                curSuccessFLAG = runSingleInterfaceTest( nV,0,hasA,isSparseH,isSparseA, hasLowerB,hasUpperB,0,0,hasOptions,hasX0,hasWS, doPrint,seed );
                                successFLAG = min( successFLAG,curSuccessFLAG );
                            end
                        end
                    end
                end
            end
        end
    end
    
    %% test with non-empty A matrix
    for hasLowerB=0:1
        for hasUpperB=0:1
            for hasLowerC=0:1
                for hasUpperC=0:1
                    for hasOptions=0:2
                        for hasX0=0:1
                            for hasWS=0:2
                                curSuccessFLAG = runSingleInterfaceTest( nV,nC,1,isSparseH,isSparseA, hasLowerB,hasUpperB,hasLowerC,hasUpperC,hasOptions,hasX0,hasWS, doPrint,seed );
                                successFLAG = min( successFLAG,curSuccessFLAG );
                            end
                        end
                    end
                end
            end
        end
    end
    
end


function [ successFLAG ] = runSingleInterfaceTest( nV,nC,hasA,isSparseH,isSparseA, hasLowerB,hasUpperB,hasLowerC,hasUpperC,hasOptions,hasX0,hasWS, doPrint,seed )

    successFLAG = 0;

    qpData = generateExample( nV,nC, isSparseH,isSparseA, hasLowerB,hasUpperB,hasLowerC,hasUpperC, seed );

    string = 'Testing qpOASES( H';
    
    if ( isSparseH > 0 )
        string = [string,'s,g'];
    else
        string = [string,'d,g'];
    end
    
    if ( nC > 0 )
        if ( isSparseA > 0 )
            string = [string,',As'];
        else
            string = [string,',Ad'];
        end
    else
        if ( hasA > 0 )
            string = [string,',[]'];
        end
    end
    
    if ( hasLowerB > 0 )
        string = [string,',lb'];
    else
        string = [string,',[]'];
    end
    
    if ( hasUpperB > 0 )
        string = [string,',ub'];
    else
        string = [string,',[]'];
    end
    
    if ( hasLowerC > 0 )
        string = [string,',lbA'];
    else
        if ( hasA > 0 )
            string = [string,',[] '];
        end
    end
    
    if ( hasUpperC > 0 )
        string = [string,',ubA'];
    else
        if ( hasA > 0 )
            string = [string,',[] '];
        end
    end
    
    switch ( hasOptions )
        case 1
            string = [string,',opt'];
        
        case 2
            string = [string,',[] '];
            
        case 0
            if ( ( hasX0 > 0 ) || ( hasWS > 0 ) )
                string = [string,',[] '];
            end
    end

    switch ( hasX0 )
        case 1
            string = [string,',{x0'];
        
        case 2
            string = [string,',{[]'];
            
        case 0
            if ( hasWS > 0 )
                string = [string,',{[]'];
            end
    end
    
    switch ( hasWS )
        case 1
            string = [string,',WS}'];
        
        case 2
            string = [string,',[]}'];
    end

    string = [string,' )... '];
    if ( doPrint > 0 )
        %disp( string );
    end
    
    curSuccessFlag = callQpOases( qpData,hasA,hasOptions,hasX0,hasWS, 1 );
    if ( curSuccessFlag > 0 )
        string = [string,'pass!'];
        successFLAG = 1;
    else
        string = [string,'fail!'];
    end
    
    if ( doPrint > 0 )
        disp( string );
        if ( curSuccessFlag == 0 )
            pause;
        end
    end
    
end

function [ successFLAG ] = callQpOases( qpData,hasA,hasOptions,hasX0,hasWS, doPrint )

    if ( nargin < 6 )
        doPrint = 1;
    end

    TOL = 1e-15;
    KKTTOL = 1e-6;

    successFLAG = 0;
    
    H = qpData.H;
	g = qpData.g;
    A = [qpData.Aeq;qpData.Ain];
	lb = qpData.lb;
    ub = qpData.ub;
    lbA = [qpData.beq;qpData.lbA];
    ubA = [qpData.beq;qpData.ubA];

    [nV,dummy] = size(H); %#ok<NASGU>
    [nC,dummy] = size(A); %#ok<NASGU>
    
    if ( hasWS > 0 )
        if ( hasWS == 1 )
            wsB = 0 * ones( nV,1 );
            wsC = 0 * ones( nC,1 );
        else
            wsB = [];
            wsC = [];
        end
        
        if ( hasX0 > 0 )
            if ( hasX0 == 1 )
                x0 = -1e-3 * ones( nV,1 );
            else
                x0 = [];
            end
            
            auxInput = qpOASES_auxInput( 'x0',x0,'guessedWorkingSetB',wsB,'guessedWorkingSetC',wsC );

            if ( hasOptions > 0 )
                if ( hasOptions == 1 )
                    options = qpOASES_options();
                else
                    options = [];
                end

                if ( hasA > 0 )
                    [ x1 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,options,auxInput );
                    [ x2,f2 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,options,auxInput );
                    [ x3,f3,e3 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,options,auxInput );
                    [ x4,f4,e4,i4 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,options,auxInput );
                    [ x5,f5,e5,i5,l5 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,options,auxInput );
                    [ x6,f6,e6,i6,l6,w6 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,options,auxInput ); %#ok<NASGU>
                else                        
                    [ x1 ] = qpOASES( H,g,lb,ub,options,auxInput );
                    [ x2,f2 ] = qpOASES( H,g,lb,ub,options,auxInput );
                    [ x3,f3,e3 ] = qpOASES( H,g,lb,ub,options,auxInput );
                    [ x4,f4,e4,i4 ] = qpOASES( H,g,lb,ub,options,auxInput );
                    [ x5,f5,e5,i5,l5 ] = qpOASES( H,g,lb,ub,options,auxInput );
                    [ x6,f6,e6,i6,l6,w6 ] = qpOASES( H,g,lb,ub,options,auxInput ); %#ok<NASGU>
                end
            else
                if ( hasA > 0 )
                    [ x1 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,[],auxInput );
                    [ x2,f2 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,[],auxInput );
                    [ x3,f3,e3 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,[],auxInput );
                    [ x4,f4,e4,i4 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,[],auxInput );
                    [ x5,f5,e5,i5,l5 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,[],auxInput );
                    [ x6,f6,e6,i6,l6,w6 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,[],auxInput ); %#ok<NASGU>
                else
                    [ x1 ] = qpOASES( H,g,lb,ub,[],auxInput );
                    [ x2,f2 ] = qpOASES( H,g,lb,ub,[],auxInput );
                    [ x3,f3,e3 ] = qpOASES( H,g,lb,ub,[],auxInput );
                    [ x4,f4,e4,i4 ] = qpOASES( H,g,lb,ub,[],auxInput );
                    [ x5,f5,e5,i5,l5 ] = qpOASES( H,g,lb,ub,[],auxInput );
                    [ x6,f6,e6,i6,l6,w6 ] = qpOASES( H,g,lb,ub,[],auxInput ); %#ok<NASGU>
                end
            end

        else % hasX0 == 0

            %auxInput = qpOASES_auxInput( 'guessedWorkingSetB',wsB,'guessedWorkingSetC',wsC );
            auxInput = qpOASES_auxInput( 'guessedWorkingSetC',wsC );
            
            if ( hasOptions > 0 )
                if ( hasOptions == 1 )
                    options = qpOASES_options();
                else
                    options = [];
                end
                
                if ( hasA > 0 )
                    [ x1 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,options,auxInput );
                    [ x2,f2 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,options,auxInput );
                    [ x3,f3,e3 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,options,auxInput );
                    [ x4,f4,e4,i4 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,options,auxInput );
                    [ x5,f5,e5,i5,l5 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,options,auxInput );
                    [ x6,f6,e6,i6,l6,w6 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,options,auxInput ); %#ok<NASGU>
                else
                    [ x1 ] = qpOASES( H,g,lb,ub,options,auxInput );
                    [ x2,f2 ] = qpOASES( H,g,lb,ub,options,auxInput );
                    [ x3,f3,e3 ] = qpOASES( H,g,lb,ub,options,auxInput );
                    [ x4,f4,e4,i4 ] = qpOASES( H,g,lb,ub,options,auxInput );
                    [ x5,f5,e5,i5,l5 ] = qpOASES( H,g,lb,ub,options,auxInput );
                    [ x6,f6,e6,i6,l6,w6 ] = qpOASES( H,g,lb,ub,options,auxInput ); %#ok<NASGU>
                end
            else
                if ( hasA > 0 )
                    [ x1 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,[],auxInput );
                    [ x2,f2 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,[],auxInput );
                    [ x3,f3,e3 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,[],auxInput );
                    [ x4,f4,e4,i4 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,[],auxInput );
                    [ x5,f5,e5,i5,l5 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,[],auxInput );
                    [ x6,f6,e6,i6,l6,w6 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,[],auxInput ); %#ok<NASGU>
                else
                    [ x1 ] = qpOASES( H,g,lb,ub,[],auxInput );
                    [ x2,f2 ] = qpOASES( H,g,lb,ub,[],auxInput );
                    [ x3,f3,e3 ] = qpOASES( H,g,lb,ub,[],auxInput );
                    [ x4,f4,e4,i4 ] = qpOASES( H,g,lb,ub,[],auxInput );
                    [ x5,f5,e5,i5,l5 ] = qpOASES( H,g,lb,ub,[],auxInput );
                    [ x6,f6,e6,i6,l6,w6 ] = qpOASES( H,g,lb,ub,[],auxInput ); %#ok<NASGU>
                end
            end

        end % hasX0
        
    else % hasWS == 0
        
        if ( hasX0 > 0 )
            if ( hasX0 == 1 )
                x0 = -1e-3 * ones( nV,1 );
            else
                x0 = [];
            end
            
            auxInput = qpOASES_auxInput( 'x0',x0 );

            if ( hasOptions > 0 )
                if ( hasOptions == 1 )
                    options = qpOASES_options();
                else
                    options = [];
                end

                if ( hasA > 0 )
                    [ x1 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,options,auxInput );
                    [ x2,f2 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,options,auxInput );
                    [ x3,f3,e3 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,options,auxInput );
                    [ x4,f4,e4,i4 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,options,auxInput );
                    [ x5,f5,e5,i5,l5 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,options,auxInput );
                    [ x6,f6,e6,i6,l6,w6 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,options,auxInput ); %#ok<NASGU>
                else
                    [ x1 ] = qpOASES( H,g,lb,ub,options,auxInput );
                    [ x2,f2 ] = qpOASES( H,g,lb,ub,options,auxInput );
                    [ x3,f3,e3 ] = qpOASES( H,g,lb,ub,options,auxInput );
                    [ x4,f4,e4,i4 ] = qpOASES( H,g,lb,ub,options,auxInput );
                    [ x5,f5,e5,i5,l5 ] = qpOASES( H,g,lb,ub,options,auxInput );
                    [ x6,f6,e6,i6,l6,w6 ] = qpOASES( H,g,lb,ub,options,auxInput ); %#ok<NASGU>
                end
                    
            else
                if ( hasA > 0 )
                    [ x1 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,[],auxInput );
                    [ x2,f2 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,[],auxInput );
                    [ x3,f3,e3 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,[],auxInput );
                    [ x4,f4,e4,i4 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,[],auxInput );
                    [ x5,f5,e5,i5,l5 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,[],auxInput );
                    [ x6,f6,e6,i6,l6,w6 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,[],auxInput ); %#ok<NASGU>
                else
                    [ x1 ] = qpOASES( H,g,lb,ub,[],auxInput );
                    [ x2,f2 ] = qpOASES( H,g,lb,ub,[],auxInput );
                    [ x3,f3,e3 ] = qpOASES( H,g,lb,ub,[],auxInput );
                    [ x4,f4,e4,i4 ] = qpOASES( H,g,lb,ub,[],auxInput );
                    [ x5,f5,e5,i5,l5 ] = qpOASES( H,g,lb,ub,[],auxInput );
                    [ x6,f6,e6,i6,l6,w6 ] = qpOASES( H,g,lb,ub,[],auxInput ); %#ok<NASGU>
                end
            end

        else % hasX0 == 0
            
            if ( hasOptions > 0 )
                if ( hasOptions == 1 )
                    options = qpOASES_options();
                else
                    options = [];
                end

                if ( hasA > 0 )
                    [ x1 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,options );
                    [ x2,f2 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,options );
                    [ x3,f3,e3 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,options );
                    [ x4,f4,e4,i4 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,options );
                    [ x5,f5,e5,i5,l5 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,options );
                    [ x6,f6,e6,i6,l6,w6 ] = qpOASES( H,g,A,lb,ub,lbA,ubA,options ); %#ok<NASGU>
                else
                    [ x1 ] = qpOASES( H,g,lb,ub,options );
                    [ x2,f2 ] = qpOASES( H,g,lb,ub,options );
                    [ x3,f3,e3 ] = qpOASES( H,g,lb,ub,options );
                    [ x4,f4,e4,i4 ] = qpOASES( H,g,lb,ub,options );
                    [ x5,f5,e5,i5,l5 ] = qpOASES( H,g,lb,ub,options );
                    [ x6,f6,e6,i6,l6,w6 ] = qpOASES( H,g,lb,ub,options ); %#ok<NASGU>
                end
            else
                if ( hasA > 0 )
                    [ x1 ] = qpOASES( H,g,A,lb,ub,lbA,ubA );
                    [ x2,f2 ] = qpOASES( H,g,A,lb,ub,lbA,ubA );
                    [ x3,f3,e3 ] = qpOASES( H,g,A,lb,ub,lbA,ubA );
                    [ x4,f4,e4,i4 ] = qpOASES( H,g,A,lb,ub,lbA,ubA );
                    [ x5,f5,e5,i5,l5 ] = qpOASES( H,g,A,lb,ub,lbA,ubA );
                    [ x6,f6,e6,i6,l6,w6 ] = qpOASES( H,g,A,lb,ub,lbA,ubA ); %#ok<NASGU>
                else
                    [ x1 ] = qpOASES( H,g,lb,ub );
                    [ x2,f2 ] = qpOASES( H,g,lb,ub );
                    [ x3,f3,e3 ] = qpOASES( H,g,lb,ub );
                    [ x4,f4,e4,i4 ] = qpOASES( H,g,lb,ub );
                    [ x5,f5,e5,i5,l5 ] = qpOASES( H,g,lb,ub );
                    [ x6,f6,e6,i6,l6,w6 ] = qpOASES( H,g,lb,ub ); %#ok<NASGU>
                end
            end

        end % hasX0

    end % hasWS

    % check whether all calls lead to same optimal solution
    % independent from output arguments
    if ( ( norm(x1-x2) > TOL ) || ...
         ( norm(x1-x3) > TOL ) || ...
         ( norm(x1-x4) > TOL ) || ...
         ( norm(x1-x5) > TOL ) || ...
         ( norm(x1-x6) > TOL ) )
        if ( doPrint > 0 )
            disp('diff in x')
        end
        return;
    end
    
    if ( ( norm(f2-f3) > TOL ) || ...
         ( norm(f2-f4) > TOL ) || ...
         ( norm(f2-f5) > TOL ) || ...
         ( norm(f2-f6) > TOL ) )
        if ( doPrint > 0 )
            disp('diff in fval')
        end
        return;
    end

    if ( ( norm(e3-e4) > TOL ) || ...
         ( norm(e3-e5) > TOL ) || ...
         ( norm(e3-e6) > TOL ) )
        if ( doPrint > 0 )
            disp('diff in exitflag')
        end
        return;
    end

    if ( ( norm(i4-i5) > TOL ) || ...
         ( norm(i4-i6) > TOL ) )
        if ( doPrint > 0 )
            disp('diff in iter')
        end
        return;
    end
    
    if ( norm(l5-l6) > TOL )
        if ( doPrint > 0 )
            disp('diff in lambda')
        end
        return;
    end
    
    
    kktTol = getKktResidual( H,g,A,lb,ub,lbA,ubA, x6,l6 );
    
    if ( ( kktTol <= KKTTOL ) && ( e6 >= 0 ) )
        successFLAG = 1;
    else
        if ( doPrint > 0 )
            disp( ['kkt error: ',num2str(kktTol)] )
        end
    end
    
end
