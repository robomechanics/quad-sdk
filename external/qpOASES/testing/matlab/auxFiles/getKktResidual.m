function [kkt,stat,feas,cmpl] = getKktResidual( H,g,A,lb,ub,lbA,ubA, x,y )

    % Tolerance for dual variables considered zero.
	dualActiveTolerance = 1.0e3 * eps;

	% Initialize residuals
    stat = 0.0;
    feas = 0.0;
    cmpl = 0.0;
    
    if ( isempty(H) == 0 )
        nV = size( H,1 );
    else
        nV = size( g,2 );
    end

    if ( isempty(A) == 0 )
        nC = size( A,1 );
    else
        nC = 0;
    end

   
	%% check stationarity
	for i=1:nV
        
		% g term and variable bounds dual term
        if ( isempty(g) == 0 )
            sum = g(i) - y(i);
        else
            sum = 0 - y(i);
        end

		% H*x term
        if ( isempty(H) == 0 )
            sum = sum + H(i,:) * x;
        end

		% A'*y term
        if ( isempty(A) == 0 )
            sum = sum - A(:,i)' * y(nV+1:nV+nC);
        end
		
		% update stat
        if (abs(sum) > stat)
            stat = abs(sum);
        end
        
	end

    %% check primal feasibility and complementarity
	% variable bounds
	for i=1:nV

		% feasibility
        if ( isempty(lb) == 0 )
            if (lb(i) - x(i) > feas) 
				feas = lb(i) - x(i);
            end
        end

        if ( isempty(ub) == 0 )
            if (x(i) - ub(i) > feas) 
				feas = x(i) - ub(i);
            end
        end

		% complementarity
		prod = 0.0;

        if ( isempty(lb) == 0 )
             if (y(i) > dualActiveTolerance) % lower bound
				prod = (x(i) - lb(i)) * y(i);
            end
        end

        if ( isempty(ub) == 0 )
            if (y(i) < -dualActiveTolerance) % upper bound
				prod = (x(i) - ub(i)) * y(i);
            end
        end

        if (abs(prod) > cmpl)
            cmpl = abs(prod);
        end
        
	end
    
	% A*x bounds
    for i=1:nC

        % compute sum = (A*x)_i
		sum = 0.0;
        if ( isempty(A) == 0 )
            sum = sum + A(i,:) * x;
        end

		% feasibility
        if ( isempty(lbA) == 0 )
            if (lbA(i) - sum > feas) 
				feas = lbA(i) - sum;
            end
        end

        if ( isempty(ubA) == 0 )
            if (sum - ubA(i) > feas) 
				feas = sum - ubA(i);
            end
        end

		% complementarity
		prod = 0.0;

        if ( isempty(lbA) == 0 )
            if (y(nV+i) > dualActiveTolerance) % lower bound
				prod = (sum - lbA(i)) * y(nV+i);
            end
        end
		
        if ( isempty(ubA) == 0 )
            if (y(nV+i) < -dualActiveTolerance) % upper bound
				prod = (sum - ubA(i)) * y(nV+i);
            end
        end

        if (abs(prod) > cmpl)
            cmpl = abs(prod);
        end
        
    end
    
    kkt = max( [stat,feas,cmpl] );

end
