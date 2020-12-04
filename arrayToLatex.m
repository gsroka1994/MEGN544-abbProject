function s = arrayToLatex(T, num_decimals, forCanvas )
s = '';
if isnumeric(T)
    T(abs(T)<sqrt(eps))=0;
end

if ~exist('num_decimals','var')
    num_decimals = 3;
end

if ~exist('forCanvas','var')
    forCanvas = false;
end

for i=1:size(T,1)
    for j = 1:(size(T,2)-1)
        if isnumeric(T(i,j))
            n_int = max(log10(abs(T(i,j)))+1,1);
            s = [s sprintf(['%' int2str(n_int) '.' int2str(num_decimals) 'g&'],T(i,j))];
        else
            s = [s sprintf('%s&',latex(T(i,j)))];
        end
    end
    if isnumeric(T(i,end))
        n_int = max(log10(T(i,end))+1,1);
        s = [s sprintf(['%' int2str(n_int) '.' int2str(num_decimals) 'g&'],T(i,end))];
    else
        s = [s sprintf('%s',latex(T(i,end)))];
    end
    if i ~= size(T,1)
        s = [s sprintf('\\\\')];
    end
end

if forCanvas
   s = ['\begin{bmatrix} ',s,' \end{bmatrix}'];
end
end