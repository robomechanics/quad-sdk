% Requires Lukas Schork's Matlab repository of the Maros Meszaros test set
QPsetDIR = '~/git/QPset/maros';

files = dir(QPsetDIR);

options = qpOASES_options('default', ...
	'printLevel', 0, 'maxIter', 1e8, 'maxCpuTime', 60, ...
	'initialStatusBounds', 0);
nQP = 0;
for i = 1:length(files)
	clear mex
	try
		load([QPsetDIR, '/', files(i).name]);
		H = sparse(H);
		A = sparse(A);
	catch
		continue
	end

	startTime = tic;
	try
		[x,fval,exitflag,iter,lambda] = qpOASES(H,g,A,xl,xu,al,au,options);
	catch
		exitflag = 666;
		iter = 666;
		fval = 666;
	end
	elapsedTime = toc(startTime);

	if mod(nQP, 20) == 0
		fprintf('\n%-10s %6s %6s %5s %7s %15s %9s\n', ...
			'name', 'nvar', 'ncon', 'eflag', 'iter', 'fval', 'time [s]');
	end
	fprintf('%-10s %6d %6d %5d %7d %15g %9g\n', ...
		files(i).name(1:findstr(files(i).name, '.')-1), ...
		size(A,2), size(A,1), exitflag, iter, fval, elapsedTime);

	nQP = nQP + 1;
end

