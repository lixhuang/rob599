%global x0;
%global HORIZON_STEP;
%global T_STEP;


br = TestTrack.br;
bl = TestTrack.bl;
bc = TestTrack.cline;
b_theta = TestTrack.theta;

T_STEP = 0.1;
SIM_STEP = 0.01;
C_STEP = 0.2;
T_LENGTH = 70;
HORIZON_STEP = 50;
U1_SCALE = 30;
U2_SCALE = 0.00005;

a_max = 10;
kp_d = 1;
ki_d = 1;
kd_d = 1;
e_dis_sum = 0;
e_dis_past = 0;
kp_v = 1;
ki_v = 1;
kd_v = 1;
e_v_sum = 0;
e_v_past = 0;

u_w_max = 0.5;
u_w_min = -0.5;
u_v_max = 5000;
u_v_min = -10000;


% x, u, y, v, phi, r
x_init = [287; 5; -176; 0; 2; 0];
%x_init = [253.5; 10; -94; 0; 1.9; 0];
%x_init = [562.55;14.51;316.84;-2.09;1.01;0.24];
%x_init = [878.343380813432;24.6901208124630;458.223577742850;-0.885645635966873;0.836579999546011;0.114723547684054];

x = x_init;
id = 2;
%id = 7;
%id = 69;
%id = 140;

x_log = [];
u_log = [];

for t = 0:SIM_STEP:T_LENGTH
    
    x_log = [x_log,x];
    
    %% prepare initial guess
    %x0 = x;
        
    %% find the error
    id_d = id;
    id_v = id;
    
    v1 = bc(:,id_d) - bc(:,id_d-1);
    v2 = [x(1);x(3)] - bc(:,id_d-1);
    theta1 = atan2(v1(2), v1(1));
    theta2 = atan2(v2(2), v2(1));
    theta_d = theta1 - theta2;
    e_dis = sqrt(sum(v2.^2))*sin(theta_d);
    
    arc = sqrt(sum((bc(:,id_v) - bc(:,id_v-1)).^2));
    theta_arc = b_theta(id_v) - b_theta(id_v-1);
    v_d = sqrt(a_max*arc/abs(theta_arc));
    e_v = v_d - x(2);
    
    %% controller
    u_w = kp_d*e_dis + kd_d*(e_dis-e_dis_past) + ki_d*e_dis_sum;
    if(u_w > u_w_max)
        u_w = u_w_max;
    end
    if(u_w < u_w_min)
        u_w = u_w_min;
    end
    u_v = kp_v*e_v + kd_v*(e_v-e_v_past) + ki_v*e_v_sum;
    if(u_v > u_v_max)
        u_v = u_v_max;
    end
    if(u_v < u_v_min)
        u_v = u_v_min;
    end
    e_dis_past = e_dis;
    e_v_past = e_v;
    e_dis_sum = e_dis_sum + e_dis;
    e_v_sum = e_v_sum + e_v;
    u = [u_w;u_v];
    
    
    %% evolve system and found nearset point
    u_log = [u_log,[u(1)/U1_SCALE;u(2)/U2_SCALE]];
    x = f_car(x,[u(1)/U1_SCALE;u(2)/U2_SCALE], SIM_STEP);
    
    q = [x(1);x(3)];
    d = inf;
    for i = id:id+10
        if(i>length(bc))
            break
        end
        temp_d = sum(([x(1);x(3)]-bc(:,i)).^2);
        if(temp_d < d)
            d = temp_d;
            id = i;
        end
    end
    d1 = sum((bc(:,id-1)-q).^2);
    d2 = sum((bc(:,id+1)-q).^2);
    if(d2 < d1)
        id = id + 1;
    end
    
    %d_max_debug = max(c)
end

plot(x_log(1,:),x_log(3,:));
plot(br(1,:),br(2,:),'k');
plot(bl(1,:),bk(2,:),'k');
plot(bc(1,:),bc(2,:),'b');
