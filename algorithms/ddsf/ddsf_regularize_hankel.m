function H = ddsf_regularize_hankel(H_u, H_y)
    ru = rank(H_u); ry = rank(H_y);
    U_svd = svdHankel(H_u, ru);
    Y_svd = svdHankel(H_y, ry);
    H = [U_svd; Y_svd];
end

