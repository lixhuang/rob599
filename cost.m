function [J] = cost(u,param)

    HORIZON_STEP = param.HORIZON_STEP;
    T_STEP = param.T_STEP;
    C_STEP = param.C_STEP;
    %a = param.a;
    U1_SCALE = param.U1_SCALE;
    U2_SCALE = param.U2_SCALE;
    
    u(1:2:end) = u(1:2:end)/param.U1_SCALE;
    u(2:2:end) = u(2:2:end)/param.U2_SCALE;
    
    x = param.x0;
    prox_len = param.prox_len;
    bc = param.bc;
    
    x_log = [];
    id = 2;
    it = 1;
    d_vec = zeros([param.len,1]);
    %for i = 0:T_STEP:HORIZON_STEP*C_STEP-T_STEP
    for uid = 1:HORIZON_STEP
        %uid = floor(i/C_STEP)+1;
        x_log = [x_log,x];
        x = f_car(x,u(uid*2-1:uid*2),T_STEP);
        
        %% find distance
        
        it = it+1;
    end
    
    v = sqrt(x_log(2,:).^2 + x_log(4,:).^2);
    J = -0.5*sum((v).^2) + 0*sum(d_vec.^4) + 100*sum(u(1:2:end).^2);
end

