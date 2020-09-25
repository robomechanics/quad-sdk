function [ successFlag ] = runInterfaceSeqTest( nV,nC, doPrint,seed )

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
            successFlag = runSeveralInterfaceSeqTests( successFlag, nV,nC,isSparseH,isSparseA, doPrint,seed );
        end
    end
    
end



function [ successFlag ] = runSeveralInterfaceSeqTests( successFlag, nV,nC, isSparseH,isSparseA, doPrint,seed )

    %% test without or empty A matrix
    for hasA=0:1
        for changeMat=0:hasA % cannot change matrices if QProblemB object is instantiated
            for hasLowerB=0:1
                for hasUpperB=0:1
                    for hasOptions=0:2
                        for hasX0=0:2
                            for hasWS=0:2
                                curSuccessFLAG = runSingleInterfaceSeqTest( nV,0,hasA,isSparseH,isSparseA, hasLowerB,hasUpperB,0,0,hasOptions,hasX0,hasWS,changeMat, doPrint,seed );
                                successFlag = min( successFlag,curSuccessFLAG );
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
                        for hasX0=0:2
                            for hasWS=0:2
                                for changeMat=0:1
                                    curSuccessFLAG = runSingleInterfaceSeqTest( nV,nC,1,isSparseH,isSparseA, hasLowerB,hasUpperB,hasLowerC,hasUpperC,hasOptions,hasX0,hasWS,changeMat, doPrint,seed );
                                    successFlag = min( successFlag,curSuccessFLAG );
                                end
                            end
                        end
                    end
                end
            end
        end
    end
    
end


function [ successFlag ] = runSingleInterfaceSeqTest( nV,nC,hasA,isSparseH,isSparseA, hasLowerB,hasUpperB,hasLowerC,hasUpperC,hasOptions,hasX0,hasWS,changeMat, doPrint,seed )

    successFlag = 0;

    qpData1 = generateExample( nV,nC, isSparseH,isSparseA, hasLowerB,hasUpperB,hasLowerC,hasUpperC, seed );
    if ( changeMat > 0 )
        qpData2 = generateExample( nV,nC, isSparseH,isSparseA, hasLowerB,hasUpperB,hasLowerC,hasUpperC, seed+1 );
    else
        qpData2 = generateExample( nV,nC, isSparseH,isSparseA, hasLowerB,hasUpperB,hasLowerC,hasUpperC, seed+1,qpData1.H,qpData1.Ain );
    end

    if ( changeMat == 0 )
        string = 'Testing qpOASES_sequence( ''i/h/c'',H';
    else
        string = 'Testing qpOASES_sequence( ''i/m/c'',H';
    end
    
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
        disp( string );
    end
    
    curSuccessFlag = callQpOasesSeq( qpData1,qpData2,hasA,hasOptions,hasX0,hasWS,changeMat );
    if ( curSuccessFlag > 0 )
        string = [string,'pass!'];
        successFlag = 1;
    else
        string = [string,'fail!'];
    end
    
    if ( doPrint > 0 )
        disp( string );
        if ( curSuccessFlag == 0 )
            pause(0.1);
        end
    end
    
end

function [ successFlag ] = callQpOasesSeq( qpData1,qpData2,hasA,hasOptions,hasX0,hasWS,changeMat, doPrint )

    if ( nargin < 8 )
        doPrint = 1;
    end

    %TOL = 1e-15;
    KKTTOL = 1e-6;

    successFlag = 0;
    
    H1 = qpData1.H;
	g1 = qpData1.g;
    A1 = [qpData1.Aeq;qpData1.Ain];
	lb1 = qpData1.lb;
    ub1 = qpData1.ub;
    lbA1 = [qpData1.beq;qpData1.lbA];
    ubA1 = [qpData1.beq;qpData1.ubA];
    
    if ( changeMat > 0 )
        H2 = qpData2.H;
    else
        H2 = H1;
    end
	g2 = qpData2.g;
    if ( changeMat > 0 )
        A2 = [qpData2.Aeq;qpData2.Ain];
    else
        A2 = A1;
    end
	lb2 = qpData2.lb;
    ub2 = qpData2.ub;
    lbA2 = [qpData2.beq;qpData2.lbA];
    ubA2 = [qpData2.beq;qpData2.ubA];

    [nV,dummy] = size(H1); %#ok<NASGU>
    [nC,dummy] = size(A1); %#ok<NASGU>
    
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
                    [ QP,x1,f1,e1,i1,l1,w1 ] = qpOASES_sequence( 'i',H1,g1,A1,lb1,ub1,lbA1,ubA1,options,auxInput ); %#ok<NASGU>
                    if ( changeMat > 0 )
                        [ x2,f2,e2,i2,l2,w2 ] = qpOASES_sequence( 'm',QP,H2,g2,A2,lb2,ub2,lbA2,ubA2,options ); %#ok<NASGU>
                    else
                        [ x2,f2,e2,i2,l2,w2 ] = qpOASES_sequence( 'h',QP,g2,lb2,ub2,lbA2,ubA2,options ); %#ok<NASGU>
                    end
                    qpOASES_sequence( 'c',QP );
                else
                    [ QP,x1,f1,e1,i1,l1,w1 ] = qpOASES_sequence( 'i',H1,g1,lb1,ub1,options,auxInput ); %#ok<NASGU>
                    [ x2,f2,e2,i2,l2,w2 ] = qpOASES_sequence( 'h',QP,g2,lb2,ub2,options ); %#ok<NASGU>
                    qpOASES_sequence( 'c',QP );
                end
            else
                if ( hasA > 0 )
                    [ QP,x1,f1,e1,i1,l1,w1 ] = qpOASES_sequence( 'i',H1,g1,A1,lb1,ub1,lbA1,ubA1,[],auxInput ); %#ok<NASGU>
                    if ( changeMat > 0 )
                        [ x2,f2,e2,i2,l2,w2 ] = qpOASES_sequence( 'm',QP,H2,g2,A2,lb2,ub2,lbA2,ubA2 ); %#ok<NASGU>
                    else
                        [ x2,f2,e2,i2,l2,w2 ] = qpOASES_sequence( 'h',QP,g2,lb2,ub2,lbA2,ubA2 ); %#ok<NASGU>
                    end
                    qpOASES_sequence( 'c',QP );
                else
                    [ QP,x1,f1,e1,i1,l1,w1 ] = qpOASES_sequence( 'i',H1,g1,lb1,ub1,[],auxInput ); %#ok<NASGU>
                    [ x2,f2,e2,i2,l2,w2 ] = qpOASES_sequence( 'h',QP,g2,lb2,ub2 ); %#ok<NASGU>
                    qpOASES_sequence( 'c',QP );
                end
            end

        else % hasX0 == 0

            %auxInput = qpOASES_auxInput( 'guessedWorkingSetB',wsB,'guessedWorkingSetC',wsC );
            auxInput = qpOASES_auxInput( 'guessedWorkingSetB',wsB );
            
            if ( hasOptions > 0 )
                if ( hasOptions == 1 )
                    options = qpOASES_options();
                else
                    options = [];
                end

                if ( hasA > 0 )
                    [ QP,x1,f1,e1,i1,l1,w1 ] = qpOASES_sequence( 'i',H1,g1,A1,lb1,ub1,lbA1,ubA1,options,auxInput ); %#ok<NASGU>
                    if ( changeMat > 0 )
                        [ x2,f2,e2,i2,l2,w2 ] = qpOASES_sequence( 'm',QP,H2,g2,A2,lb2,ub2,lbA2,ubA2,options ); %#ok<NASGU>
                    else
                        [ x2,f2,e2,i2,l2,w2 ] = qpOASES_sequence( 'h',QP,g2,lb2,ub2,lbA2,ubA2,options ); %#ok<NASGU>
                    end
                    qpOASES_sequence( 'c',QP );
                else
                    [ QP,x1,f1,e1,i1,l1,w1 ] = qpOASES_sequence( 'i',H1,g1,lb1,ub1,options,auxInput ); %#ok<NASGU>
                    [ x2,f2,e2,i2,l2,w2 ] = qpOASES_sequence( 'h',QP,g2,lb2,ub2,options ); %#ok<NASGU>
                    qpOASES_sequence( 'c',QP );
                end
            else
                if ( hasA > 0 )
                    [ QP,x1,f1,e1,i1,l1,w1 ] = qpOASES_sequence( 'i',H1,g1,A1,lb1,ub1,lbA1,ubA1,[],auxInput ); %#ok<NASGU>
                    if ( changeMat > 0 )
                        [ x2,f2,e2,i2,l2,w2 ] = qpOASES_sequence( 'm',QP,H2,g2,A2,lb2,ub2,lbA2,ubA2,[] ); %#ok<NASGU>
                    else
                        [ x2,f2,e2,i2,l2,w2 ] = qpOASES_sequence( 'h',QP,g2,lb2,ub2,lbA2,ubA2,[] ); %#ok<NASGU>
                    end
                    qpOASES_sequence( 'c',QP );
                else
                    [ QP,x1,f1,e1,i1,l1,w1 ] = qpOASES_sequence( 'i',H1,g1,lb1,ub1,[],auxInput ); %#ok<NASGU>
                    [ x2,f2,e2,i2,l2,w2 ] = qpOASES_sequence( 'h',QP,g2,lb2,ub2 ); %#ok<NASGU>
                    qpOASES_sequence( 'c',QP );
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
                    [ QP,x1,f1,e1,i1,l1,w1 ] = qpOASES_sequence( 'i',H1,g1,A1,lb1,ub1,lbA1,ubA1,options,auxInput ); %#ok<NASGU>
                    if ( changeMat > 0 )
                        [ x2,f2,e2,i2,l2,w2 ] = qpOASES_sequence( 'm',QP,H2,g2,A2,lb2,ub2,lbA2,ubA2,options ); %#ok<NASGU>
                    else
                        [ x2,f2,e2,i2,l2,w2 ] = qpOASES_sequence( 'h',QP,g2,lb2,ub2,lbA2,ubA2,options ); %#ok<NASGU>
                    end
                    qpOASES_sequence( 'c',QP );
                else
                    [ QP,x1,f1,e1,i1,l1,w1 ] = qpOASES_sequence( 'i',H1,g1,lb1,ub1,options,auxInput ); %#ok<NASGU>
                    [ x2,f2,e2,i2,l2,w2 ] = qpOASES_sequence( 'h',QP,g2,lb2,ub2,options ); %#ok<NASGU>
                    qpOASES_sequence( 'c',QP );
                end
                    
            else
                if ( hasA > 0 )
                    [ QP,x1,f1,e1,i1,l1,w1 ] = qpOASES_sequence( 'i',H1,g1,A1,lb1,ub1,lbA1,ubA1,[],auxInput ); %#ok<NASGU>
                    if ( changeMat > 0 )
                        [ x2,f2,e2,i2,l2,w2 ] = qpOASES_sequence( 'm',QP,H2,g2,A2,lb2,ub2,lbA2,ubA2 ); %#ok<NASGU>
                    else
                        [ x2,f2,e2,i2,l2,w2 ] = qpOASES_sequence( 'h',QP,g2,lb2,ub2,lbA2,ubA2 ); %#ok<NASGU>
                    end
                    qpOASES_sequence( 'c',QP );
                else
                    [ QP,x1,f1,e1,i1,l1,w1 ] = qpOASES_sequence( 'i',H1,g1,lb1,ub1,[],auxInput ); %#ok<NASGU>
                    [ x2,f2,e2,i2,l2,w2 ] = qpOASES_sequence( 'h',QP,g2,lb2,ub2 ); %#ok<NASGU>
                    qpOASES_sequence( 'c',QP );
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
                    [ QP,x1,f1,e1,i1,l1,w1 ] = qpOASES_sequence( 'i',H1,g1,A1,lb1,ub1,lbA1,ubA1,options ); %#ok<NASGU>
                    if ( changeMat > 0 )
                        [ x2,f2,e2,i2,l2,w2 ] = qpOASES_sequence( 'm',QP,H2,g2,A2,lb2,ub2,lbA2,ubA2,options ); %#ok<NASGU>
                    else
                        [ x2,f2,e2,i2,l2,w2 ] = qpOASES_sequence( 'h',QP,g2,lb2,ub2,lbA2,ubA2,options ); %#ok<NASGU>
                    end
                    qpOASES_sequence( 'c',QP );
                else
                    [ QP,x1,f1,e1,i1,l1,w1 ] = qpOASES_sequence( 'i',H1,g1,lb1,ub1,options ); %#ok<NASGU>
                    [ x2,f2,e2,i2,l2,w2 ] = qpOASES_sequence( 'h',QP,g2,lb2,ub2,options ); %#ok<NASGU>
                    qpOASES_sequence( 'c',QP );
                end
            else
                if ( hasA > 0 )
                    [ QP,x1,f1,e1,i1,l1,w1 ] = qpOASES_sequence( 'i',H1,g1,A1,lb1,ub1,lbA1,ubA1 ); %#ok<NASGU>
                    if ( changeMat > 0 )
                        [ x2,f2,e2,i2,l2,w2 ] = qpOASES_sequence( 'm',QP,H2,g2,A2,lb2,ub2,lbA2,ubA2 ); %#ok<NASGU>
                    else
                        [ x2,f2,e2,i2,l2,w2 ] = qpOASES_sequence( 'h',QP,g2,lb2,ub2,lbA2,ubA2 ); %#ok<NASGU>
                    end
                    qpOASES_sequence( 'c',QP );
                else
                    [ QP,x1,f1,e1,i1,l1,w1 ] = qpOASES_sequence( 'i',H1,g1,lb1,ub1 ); %#ok<NASGU>
                    [ x2,f2,e2,i2,l2,w2 ] = qpOASES_sequence( 'h',QP,g2,lb2,ub2 ); %#ok<NASGU>
                    qpOASES_sequence( 'c',QP );
                end
            end

        end % hasX0

    end % hasWS


    kktTol1 = getKktResidual( H1,g1,A1,lb1,ub1,lbA1,ubA1, x1,l1 );
    
    if ( changeMat > 0 )
		kktTol2 = getKktResidual( H2,g2,A2,lb2,ub2,lbA2,ubA2, x2,l2 );
    else
		kktTol2 = getKktResidual( H1,g2,A1,lb2,ub2,lbA2,ubA2, x2,l2 );
    end
    
    if ( ( kktTol1 <= KKTTOL ) && ( e1 >= 0 ) && ( kktTol2 <= KKTTOL ) && ( e2 >= 0 ) )
        successFlag = 1;
    else
        if ( doPrint > 0 )
            if ( ( kktTol1 > KKTTOL ) || ( kktTol2 > KKTTOL ) )
                disp( ['kkt error: ',num2str(kktTol1),'/',num2str(kktTol2)] )
            else
                disp('exitflag<0')
            end
        end
    end
    
end
