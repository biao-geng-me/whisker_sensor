function [nrow,ncol] = square_layout(n)
%SQUARE_LAYOUT set subplot layout close to a square

    nrow = floor(sqrt(n));

    ncol = round(n/nrow);

    if nrow*ncol<n
        ncol = ncol+1;
    end

end
