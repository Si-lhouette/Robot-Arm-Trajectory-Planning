function Ct = getCt(n_seg, n_order)
    %#####################################################
    % STEP 2.1: finish the expression of Ct
    Ct = zeros(8*n_seg,4*n_seg+4);
    % p0,v0,a0,j0
    for i=1:4
        Ct(i,i) = 1;
    end
    % p1-p_(k-1)
    for i=1:n_seg-1
        Ct(8*i-3,4+i) = 1;
        Ct(8*i+1,4+i) = 1;
    end
    % pk,vk,ak,jk
    for i=1:4
        Ct(8*n_seg-4+i,n_seg+3+i) = 1;
    end
    % v1-v_(k-1)
    for i=1:n_seg-1
        Ct(8*i-2,n_seg+7+(3*i-2)) = 1;
        Ct(8*i+2,n_seg+7+(3*i-2)) = 1;
    end
    % a1-a_(k-1)
    for i=1:n_seg-1
        Ct(8*i-1,n_seg+7+(3*i-1)) = 1;
        Ct(8*i+3,n_seg+7+(3*i-1)) = 1;
    end
    % j1-j_(k-1)
    for i=1:n_seg-1
        Ct(8*i,n_seg+7+(3*i)) = 1;
        Ct(8*i+4,n_seg+7+(3*i)) = 1;
    end
end