function [var] = elementwise_simplify(var)

name = inputname(1);

if isnumeric(var)
    fprintf("%s is numeric, no need to simplify \n", name)
else
    job = numel(var);
    for i=1:job
        fprintf("simplifying %s %d/%d \n", name, i, job)
        var(i) = simplify(var(i)) ;
    end
end

end
