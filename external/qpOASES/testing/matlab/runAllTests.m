function [ successFlag ] = runAllTests( doPrint )
    
    if ( nargin < 1 )
        doPrint = 0;
    end
    
    successFlag = 1;
    
    curWarnLevel = warning;
    warning('off');
    
    % add sub-folders to Matlab path
	setupTestingPaths();
    
    clc;

    %% run interface tests
    fprintf( 'Running qpOASES interface tests... ' )
    successFlag = updateSuccessFlag( successFlag, runInterfaceTest( 10,20, doPrint,42 ) );

    fprintf( 'Running qpOASES_sequence interface tests... ' )
    successFlag = updateSuccessFlag( successFlag, runInterfaceSeqTest( 8,5, doPrint,42 ) );
    
    
    %% run functional tests
    fprintf( 'Running tests with random QPs having identity Hessian... ' )
    successFlag = updateSuccessFlag( successFlag, runRandomIdHessian( 12,12, doPrint,4242 ) );
    
    fprintf( 'Running tests with random QPs having zero Hessian... ' )
    successFlag = updateSuccessFlag( successFlag, runRandomZeroHessian( 11,41, doPrint,4242 ) );
    
    fprintf( 'Running qpOASES passing an empty Hessian matrix argument... ' )
    successFlag = updateSuccessFlag( successFlag, runEmptyHessianTests( doPrint ) );

    fprintf( 'Running alternativeX0 test... ' )
    successFlag = updateSuccessFlag( successFlag, runAlternativeX0Test( 50,300,doPrint,4242 ) );
    
    fprintf( 'Running testAPrioriKnownSeq1... ' )
    successFlag = updateSuccessFlag( successFlag, runTestAPrioriKnownSeq1( doPrint ) );
        
    fprintf( 'Running testSeq... ' )
    successFlag = updateSuccessFlag( successFlag, runTestSeq( doPrint ) );
    
    fprintf( 'Running testSparse... ' )
    successFlag = updateSuccessFlag( successFlag, runTestSparse( doPrint ) );
    
    fprintf( 'Running testSparse2... ' )
    successFlag = updateSuccessFlag( successFlag, runTestSparse2( doPrint ) );
    
    fprintf( 'Running testSparse3... ' )
    successFlag = updateSuccessFlag( successFlag, runTestSparse3( doPrint ) );
    
    fprintf( 'Running testSparse4... ' )
    successFlag = updateSuccessFlag( successFlag, runTestSparse4( doPrint ) );
    
    fprintf( 'Running simpleSpringExample... ' )
    successFlag = updateSuccessFlag( successFlag, runSimpleSpringExample( doPrint ) );
    
    fprintf( 'Running vanBarelsUnboundedQP... ' )
    successFlag = updateSuccessFlag( successFlag, runVanBarelsUnboundedQP( doPrint ) );
    
    fprintf( 'Running alexInfeas1... ' )
    successFlag = updateSuccessFlag( successFlag, runAlexInfeas1( doPrint ) );
    
    %fprintf( 'Running alexInfeas2... ' )
    %successFlag = updateSuccessFlag( successFlag, runAlexInfeas2( doPrint ) );

    %fprintf( 'Running QAP8... ' )
    %successFlag = updateSuccessFlag( successFlag, runQAP( doPrint ) );
    
    fprintf( 'Running testWorkingSetLI... ' )
    successFlag = updateSuccessFlag( successFlag, runTestWorkingSetLI( doPrint ) );
    
    fprintf( 'Running runExternalCholeskyTests... ' )
    successFlag = updateSuccessFlag( successFlag, runExternalCholeskyTests( doPrint ) );

    fprintf( 'Running EXAMPEL1... ' );
    successFlag = updateSuccessFlag( successFlag, runBenchmarkEXAMPLE1( 10,doPrint ) );
	
    fprintf( 'Running EXAMPLE1A... ' );
    successFlag = updateSuccessFlag( successFlag, runBenchmarkEXAMPLE1A( 10,doPrint ) );
	
    fprintf( 'Running EXAMPLE1B... ' );
    successFlag = updateSuccessFlag( successFlag, runBenchmarkEXAMPLE1B( 10,doPrint ) );

	fprintf( 'Running CHAIN1... ' );
    successFlag = updateSuccessFlag( successFlag, runBenchmarkCHAIN1( 20,doPrint ) );
	
    fprintf( 'Running CHAIN1A... ' );
    successFlag = updateSuccessFlag( successFlag, runBenchmarkCHAIN1A( 20,doPrint ) );

	fprintf( 'Running CRANE1... ' );
    successFlag = updateSuccessFlag( successFlag, runBenchmarkCRANE1( 100,doPrint ) );

    fprintf( 'Running CRANE2... ' );
    successFlag = updateSuccessFlag( successFlag, runBenchmarkCRANE2( 100,doPrint ) );

    fprintf( 'Running CRANE3... ' );
    successFlag = updateSuccessFlag( successFlag, runBenchmarkCRANE3( 100,doPrint ) );
	
	fprintf( 'Running EQUALITY1... ' );
    successFlag = updateSuccessFlag( successFlag, runBenchmarkEQUALITY1( 100,doPrint ) );

    fprintf( 'Running EQUALITY2... ' );
    successFlag = updateSuccessFlag( successFlag, runBenchmarkEQUALITY2( 3200,doPrint ) );

    %fprintf( 'Running IDHESSIAN1... ' );
	%successFlag = updateSuccessFlag( successFlag, runBenchmarkIDHESSIAN1( 1200,doPrint ) );

    fprintf( 'Running DIESEL... ' );
    successFlag = updateSuccessFlag( successFlag, runBenchmarkDIESEL( 230,doPrint ) );
    
    fprintf( 'Running QSHARE1B... ' )
    successFlag = updateSuccessFlag( successFlag, runQSHARE1B( doPrint ) );

    
    
    %% display results
    disp( ' ' );
    
    if ( successFlag == 0 )
        disp( 'At least one test failed!' );
    else
        disp( 'All available tests passed successfully!' );
    end

    warning( curWarnLevel );
    
end


function [ newSuccessFlag ] = updateSuccessFlag( curSuccessFlag,curResult )

    switch ( curResult )
        
        case 0
            newSuccessFlag = 0;
            fprintf( 'failed!\n' );
        
        case 1          
            newSuccessFlag = curSuccessFlag;
            fprintf( 'passed!\n' );
            
        case -1
            newSuccessFlag = curSuccessFlag;
            fprintf( 'problem data missing!\n' );
            
        otherwise
            error( 'Unknown success flag!' );
    end
    
end
