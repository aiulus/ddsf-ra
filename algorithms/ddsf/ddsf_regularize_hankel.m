function H = ddsf_regularize_hankel(H_u, H_y)
    ru = size(H_u, 1); ry = size(H_y, 1);
    U_svd = svdHankel(H_u, ru);
    Y_svd = svdHankel(H_y, ry);
    % This won't work for 
    H = [U_svd; Y_svd];
end

