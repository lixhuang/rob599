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
T_LENGTH = 25;
HORIZON = 2;

%av_gain = @(v);

a_max = 1;
kp_d = 0.14;
ki_d = 0;
kd_d = 14;
e_dis_sum = 0;
e_dis_past = 0;
kp_v = 3000;
ki_v = 0;
kd_v = 100;
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
v_d_log = [];

for t = 0:SIM_STEP:T_LENGTH
    
    x_log = [x_log,x];
    
    %% prepare initial guess
    %x0 = x;
        
    %% find the error
    s = 0;
    increment = 0;
    for i = 1:40
        v = bc(:,id+i) - bc(:,id+i-1);
        s = s + sqrt(sum(v.^2));
        if(s < x(2)*HORIZON)
            increment = i;
        end
    end
    id_d = id;
    id_v = id+increment;
    
    v1 = bc(:,id_d) - bc(:,id_d-1);
    v2 = [x(1);x(3)] - bc(:,id_d-1);
    theta1 = atan2(v1(2), v1(1));
    theta2 = atan2(v2(2), v2(1));
    theta_d = theta1 - theta2;
    e_dis = sqrt(sum(v2.^2))*sin(theta_d);
    
    arc = sqrt(sum((bc(:,id_v) - bc(:,id_v-1)).^2));
    theta_arc = b_theta(id_v) - b_theta(id_v-1);
    v_d = sqrt(a_max*arc/abs(theta_arc));
    if(v_d > 40)
        v_d = 20;
    end
    v_d = 15;
    v_d_log = [v_d_log;v_d];
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
    u_log = [u_log,[u(1);u(2)]];
    if (t>0 && floor(t)*100==t*100)
        x = odefun(x_init,u_log);
        clf;
        hold on
        plot(x_log(1,:),x_log(3,:),'r');
        plot(br(1,:),br(2,:),'k');
        plot(bl(1,:),bl(2,:),'k');
        plot(bc(1,:),bc(2,:),'b');
        drawnow;
    else
        x=f_car(x,[u(1);u(2)], SIM_STEP);
    end
    
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
    
    t
    %d_max_debug = max(c)
end

%figure;
%hold on
%plot(x_log(2,:)');
%plot(v_d_log);