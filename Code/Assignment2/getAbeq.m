function [Aeq beq]= getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond)
    n_all_poly = n_seg*(n_order+1); % n_all_poly=num of cols of A=num of rows of P
    % Each Row of A and B presents a constraint
    %#####################################################
    % p,v,a,j constraint in start, 
    Aeq_start = zeros(4, n_all_poly);
    beq_start = zeros(4, 1);
    % STEP 2.1: write expression of Aeq_start and beq_start
    for k = 0:3
        for i = 0:n_order
            if i>=k
                Aeq_start(k+1,i+1) = factorial(i)/factorial(i-k) * 0^(i-k);
            end
        end
        beq_start(k+1) = start_cond(k+1);
    end
        
    %#####################################################
    % p,v,a constraint in end
    Aeq_end = zeros(4, n_all_poly);
    beq_end = zeros(4, 1);
    % STEP 2.2: write expression of Aeq_end and beq_end
	for k = 0:3
        for i = 0:n_order
            if i>=k
                Aeq_end(k+1,(i+1)+(n_order+1)*(n_seg-1)) = factorial(i)/factorial(i-k) * ts(end)^(i-k);
            end
        end
        beq_end(k+1) = end_cond(k+1);
	end
    
    %#####################################################
    % position constrain in all middle waypoints
    mid_points = waypoints(2:end-1);
    Aeq_wp = zeros(n_seg-1, n_all_poly); % Attention: num of cols of Aeq_wp = n_all_poly
    beq_wp = zeros(n_seg-1, 1);
    % STEP 2.3: write expression of Aeq_wp and beq_wp
    for j = 1:n_seg-1
        for i = 0:n_order
            Aeq_wp(j,(i+1)+(n_order+1)*(j-1)) = ts(j)^i;
        end
        beq_wp(j) = mid_points(j);
    end
    
    %#####################################################
    % position continuity constrain between each 2 segments
    Aeq_con_p = zeros(n_seg-1, n_all_poly);
    beq_con_p = zeros(n_seg-1, 1);
    % STEP 2.4: write expression of Aeq_con_p and beq_con_p
    for j = 1:n_seg-1
        for i = 0:n_order
            Aeq_con_p(j,(i+1)+(n_order+1)*(j-1)) = ts(j)^i;
        end
        for i = 0:n_order
            Aeq_con_p(j,(i+1)+(n_order+1)*(j)) = -0^i;
        end
    end
    
    %#####################################################
    % velocity continuity constrain between each 2 segments
    Aeq_con_v = zeros(n_seg-1, n_all_poly);
    beq_con_v = zeros(n_seg-1, 1);
    % STEP 2.5: write expression of Aeq_con_v and beq_con_v
    for j = 1:n_seg-1
        for i = 1:n_order
            Aeq_con_v(j,(i+1)+(n_order+1)*(j-1)) = i * ts(j)^(i-1);
        end
        for i = 1:n_order
            Aeq_con_v(j,(i+1)+(n_order+1)*(j)) = -i * 0^(i-1);
        end
    end

    %#####################################################
    % acceleration continuity constrain between each 2 segments
    Aeq_con_a = zeros(n_seg-1, n_all_poly);
    beq_con_a = zeros(n_seg-1, 1);
    % STEP 2.6: write expression of Aeq_con_a and beq_con_a
    for j = 1:n_seg-1
        for i = 2:n_order
            Aeq_con_a(j,(i+1)+(n_order+1)*(j-1)) = i*(i-1) * ts(j)^(i-2);
        end
        for i = 2:n_order
            Aeq_con_a(j,(i+1)+(n_order+1)*(j)) = -i*(i-1) * 0^(i-2);
        end
    end
    
    %#####################################################
    % jerk continuity constrain between each 2 segments
    Aeq_con_j = zeros(n_seg-1, n_all_poly);
    beq_con_j = zeros(n_seg-1, 1);
    % STEP 2.7: write expression of Aeq_con_j and beq_con_j
    for j = 1:n_seg-1
        for i = 3:n_order
            Aeq_con_j(j,(i+1)+(n_order+1)*(j-1)) = i*(i-1)*(i-2) * ts(j)^(i-3);
        end
        for i = 3:n_order
            Aeq_con_j(j,(i+1)+(n_order+1)*(j)) = -i*(i-1)*(i-2) * 0^(i-3);
        end
    end
    
    %#####################################################
    % combine all components to form Aeq and beq   
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a; Aeq_con_j];
    beq_con = [beq_con_p; beq_con_v; beq_con_a; beq_con_j];
    Aeq = [Aeq_start; Aeq_end; Aeq_wp; Aeq_con];
    beq = [beq_start; beq_end; beq_wp; beq_con];
end