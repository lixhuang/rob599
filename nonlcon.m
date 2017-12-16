function [c,ceq] = nonlcon(u,param)

    HORIZON_STEP = param.HORIZON_STEP;
    T_STEP = param.T_STEP;
    C_STEP = param.C_STEP;
    %a = param.a;
    U1_SCALE = param.U1_SCALE;
    U2_SCALE = param.U2_SCALE;
    
    u(1:2:end) = u(1:2:end)/param.U1_SCALE;
    u(2:2:end) = u(2:2:end)/param.U2_SCALE;
    
    d_vec = zeros([param.len,1]);
    %theta_vec = zeros([param.len,1]);
    
    x = param.x0;
    bc = param.bc;
    %b_theta = param.theta;
    prox_len = param.prox_len;
    
    x_log = zeros([6,HORIZON_STEP]);
    id = 2;
    it = 1;
    %for i = 0:T_STEP:HORIZON_STEP*C_STEP-T_STEP
    for uid = 1:HORIZON_STEP
        %uid = floor(i/C_STEP)+1;
        x = f_car(x,u(uid*2-1:uid*2),T_STEP);
        x_log(:,uid) = x;
        
        %% find distance
        q = [x(1);x(3)];
        
        temp_id = id;
        d1 = sum((bc(:,temp_id)-q).^2);
        for i = 1:3
            d2 = sum((bc(:,temp_id+i)-q).^2);
            if(d2<d1)
                d1 = d2;
                temp_id = temp_id+1;
            end
        end
        if(id+1<prox_len)
            d1 = sum((bc(:,temp_id-1)-q).^2);
            d2 = sum((bc(:,temp_id+1)-q).^2);
            if(d2 < d1)
                id = temp_id + 1;
            else
                id = temp_id;
            end
        end
        v1 = bc(:,id) - bc(:,id-1);
        v2 = q - bc(:,id-1);
        d = v2'*v2-(v1'*v2)^2/(v1'*v1);
        
        d_vec(uid) = (sqrt(d)-11/2)^1;
        %theta_vec(uid) = wrapToPi(x(5) - (b_theta(id) + b_theta(id-1))/2);
        
        it = it+1;
    end
    %v = sqrt(x_log(2,:).^2 + x_log(4,:).^2);
    v = abs(x_log(4,:));
    c = 100*[d_vec;v'-3;-x_log(2,:)'];
    
    
    
    
    
    ceq = 0;
end

