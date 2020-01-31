function M = getM(n_seg, n_order, ts)
    M = [];
    for k = 1:n_seg
        M_k = zeros(8,n_order+1);
        %#####################################################
        % STEP 1.1: calculate M_k of the k-th segment 
        % p_(0)
        M_k(1,1) = 1;
        % v_(0)
        M_k(2,2) = 1;
        % a_(0)
        M_k(3,3) = 2;
        % j_(0)
        M_k(4,4) = 6;
        % p_(T)
        for i=0:n_order
            M_k(5,i+1) = ts(k)^i;
        end
        % v_(T)
        for i=1:n_order
            M_k(6,i+1) = i*ts(k)^(i-1);
        end
        % a_(T)
        for i=2:n_order
            M_k(7,i+1) = i*(i-1)*ts(k)^(i-2);
        end
        % j_(T)
        for i=3:n_order
            M_k(8,i+1) = i*(i-1)*(i-2)*ts(k)^(i-3);
        end
        M = blkdiag(M, M_k);
    end
end