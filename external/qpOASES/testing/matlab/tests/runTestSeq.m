function [ successFlag ] = runTestSeq( doPrint )
    
    if ( nargin < 1 )
        doPrint = 0;
    end

    successFlag = 1;
    

    % test case constants
    m = 50;
    n = 100;
    nMajSeq = 8;
    nMinSeq = 4;
    nSeq = nMajSeq * nMinSeq;
    p = 2; % interpolation monomial power
    fldim = 5; % feedback law dimension

    % generate start and end problem
    Ls = sprand(n, n, 0.03);
    Hs = Ls' * Ls + 1e-8 * eye(n);
    As = sprand(m, n, 0.05);

    % negative (!) definite Hessian
    Le = sprand(n, n, 0.03);
    He = -Le' * Le;
    Ae = sprand(m, n, 0.05);

    lbAs = -rand(m,1);
    ubAs = rand(m,1); 
    ubs = ones(n,1);
    lbs = -ones(n,1);
    gs = 10*rand(n,1);

    lbAe = -rand(m,1);
    ubAe = rand(m,1); 
    ube = ones(n,1);
    lbe = -ones(n,1);
    ge = 10*rand(n,1);

    % monomial interpolation
    tmaj = 1 - (1:1/(1-nMajSeq):0).^p;
    for i = 1:nMajSeq
        tau = tmaj(i);
        H{i} = (1-tau) * Hs + tau * He;
        A{i} = (1-tau) * As + tau * Ae;
    end
    t = 1 - (1:1/(1-nSeq):0).^p;
    for i = 1:nSeq
        tau = t(i);
        lbA{i} = (1-tau) * lbAs + tau * lbAe;
        ubA{i} = (1-tau) * ubAs + tau * ubAe;
        lb{i} = (1-tau) * lbs + tau * lbe;
        ub{i} = (1-tau) * ubs + tau * ube;
        g{i} = (1-tau) * gs + tau * ge;
    end

    x = zeros(n,1);

    if ~exist('cumIters', 'var')
        cumIters = zeros(nSeq, 1);
    end

    % solve sequence of QPs
    if ( doPrint > 0 )
        fprintf('%3s %5s\n', 'seq', 'iters')
    end
    
    for i = 1:nSeq
        if i == 1
            [QP, x, fval, exitflag, iter, lambda] = qpOASES_sequence('i', ...
                H{i}, g{i}, A{i}, lb{i}, ub{i}, lbA{i}, ubA{i}, x);
        else
            if mod(i-1, nMinSeq) == 0
                j = (i-1) / nMinSeq + 1;
                [x, fval, exitflag, iter, lambda] = qpOASES_sequence('m', QP, ...
                    H{j}, g{i}, A{j}, lb{i}, ub{i}, lbA{i}, ubA{i});
            else
                [x, fval, exitflag, iter, lambda] = qpOASES_sequence('h', QP, ...
                    g{i}, lb{i}, ub{i}, lbA{i}, ubA{i});
            end
        end
        
        if ( doPrint > 0 )
            fprintf('%3d %5d\n', i, iter)
        end
        
        cumIters(i) = cumIters(i) + iter;
        
        if ( exitflag ~= 0 )
            successFlag = 0;
        end
    end

    % solve EQP
    V0 = zeros(n, fldim);
    Lambda = eye(m, fldim);
    [X, Y] = qpOASES_sequence('e', QP, V0, V0, V0, Lambda, Lambda);
    FL = X(1:fldim,:);

    % clear
    qpOASES_sequence('c', QP)

end
