function [ qpData ] = generateExample( nV,nC, isSparseH,isSparseA, hasLowerB,hasUpperB,hasLowerC,hasUpperC, seed, givenH,givenA )

    if ( nargin < 11 )
        givenA = [];
        if ( nargin < 10 )
            givenH = [];
        end
    end

    qpFeatures = setupQpFeaturesStruct( );
    
    qpFeatures.nV = nV;
    qpFeatures.nC = nC;
    
    qpFeatures.isSparseH = isSparseH;
    qpFeatures.isSparseA = isSparseA;
    
    qpFeatures.hasLowerB = hasLowerB;
    qpFeatures.hasUpperB = hasUpperB;
    qpFeatures.hasLowerC = hasLowerC;
    qpFeatures.hasUpperC = hasUpperC;
    
    qpData = generateRandomQp( qpFeatures,seed, givenH,givenA );

end
