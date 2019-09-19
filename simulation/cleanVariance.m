function cleanMatrix = cleanVariance(dirtyMatrix)
    n_state = length(dirtyMatrix(1,:));
    cleanMatrix = nan(n_state);
    tol = single(1e-10);
    
    for i = 1:n_state
        if (dirtyMatrix(i,i) <tol)
            cleanMatrix(i,i) = 0;
        else
            cleanMatrix(i,i) = dirtyMatrix(i,i);
        end
        for j = i+1:n_state
            p = 0.5*(dirtyMatrix(i,j)+dirtyMatrix(j,i));
           if (abs(p)<tol)
               cleanMatrix(i,j) = 0;
               cleanMatrix(j,i) = 0;
           else
               cleanMatrix(i,j) = p;
               cleanMatrix(j,i) = p;
           end
        end
    end
end
