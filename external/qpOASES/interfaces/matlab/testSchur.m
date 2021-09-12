m = 2000;
n = 2*m+1;
H = 2*speye(n);
A = 0.5*H(1:m,:);
g = zeros(n,1);
lb = -ones(n,1);
ub = -lb;
lbA = 0.5*ones(m,1);
ubA = 0.5*ones(m,1);

options = qpOASES_options('default', 'printLevel', -1, 'maxIter', n+m+1, ...
	'enableEqualities', 0, 'initialStatusBounds', 1);

%[x,fval,exitflag,iter,lambda] = qpOASES(full(H),g,full(A),lb,ub,lbA,ubA,options);
[x,fval,exitflag,iter,lambda] = qpOASES(H,g,A,lb,ub,lbA,ubA,options);

