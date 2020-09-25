function [ qpData ] = generateRandomQp( qpFeatures,randSeed, givenH,givenA )

    if ( nargin < 4 )
        givenA = [];
        if ( nargin < 3 )
            givenH = [];
        end
    end
    
    if ( isoctave == 0 )
        if ( nargin < 2 )
            s = RandStream( 'mt19937ar', 'Seed','shuffle' );
            RandStream.setGlobalStream(s);
        else
            s = RandStream( 'mt19937ar', 'Seed',randSeed );
            RandStream.setGlobalStream(s);
        end
    end
    

    qpData = setupQpDataStruct( );
    qpData.nV = qpFeatures.nV;
    qpData.nC = qpFeatures.nC;
    
    
    % generate random optimal primal solution
    xFeas = rand( qpFeatures.nV,1 );
    
    % generate random optimal dual solution
    %yOpt = zeros( qpFeatures.nV+qpFeatures.nC,1 );
    
    if ( isempty(givenH) > 0 )
        
        switch ( qpFeatures.hessianType )

            case 0
                qpData.H = 100 * rand( qpData.nV,qpData.nV ) - 50;
                qpData.H = qpData.H' * qpData.H / 2;

            case 1
                qpData.H = 100 * rand( qpData.nV,round(qpData.nV/2) ) - 50;
                qpData.H = qpData.H' * qpData.H / 2;

            case 2
                qpData.H = eye( qpData.nV );

            case 3
                qpData.H = zeros( qpData.nV,qpData.nV );

        end
        
    else
        qpData.H = givenH;
    end
    
    qpData.g = 1000 * rand( qpFeatures.nV,1 ) - 500;
    
    if ( isempty(givenA) > 0 )
        
        if ( qpFeatures.nC > 0 )
            qpData.Ain = 100 * rand( qpData.nC,qpData.nV ) - 50;
        else
            qpData.Ain = [];
        end
        
    else
        qpData.Ain = givenA;
    end
    
    if ( qpFeatures.makeInfeas > 0 )
        alpha = -0.1;
        beta  = -0.001;
    else
        alpha = 1;
        beta  = 1;
    end
        
    if ( qpFeatures.hasLowerB > 0 )
        qpData.lb = xFeas - 3*rand( qpData.nV,1 );
    else
        qpData.lb = [];
    end
    
    if ( qpFeatures.hasUpperB > 0 )
        qpData.ub = xFeas + alpha*3*rand( qpData.nV,1 );
    else
        qpData.ub = [];
    end
    
    if ( ( qpFeatures.hasLowerC > 0 ) && ( qpData.nC > 0 ) )
        qpData.lbA = qpData.Ain*xFeas - 100*rand( qpData.nC,1 );
    else
        qpData.lbA = [];
    end
    
    if ( ( qpFeatures.hasUpperC > 0 ) && ( qpData.nC > 0 ) )
        qpData.ubA = qpData.Ain*xFeas + beta*100*rand( qpData.nC,1 );
    else
        qpData.ubA = [];
    end
    
    if ( qpFeatures.isSparseH > 0 )
        qpData.H = sparse( qpData.H );
    end
    
    if ( qpFeatures.isSparseA > 0 )
        qpData.Ain = sparse( qpData.Ain );
    end
    
end
