%global x0;
%global HORIZON_STEP;
%global T_STEP;
close all;

br = TestTrack.br;
bl = TestTrack.bl;
bc2 = TestTrack.cline;
b_theta = TestTrack.theta;

bc = bc2;
tv = bc(:,end)-bc(:,end-1);
for i = 1:30
    tbc = bc(:,end)+tv;
    bc = [bc,tbc];
end
xobs = generateRandomObstacles(15,TestTrack);

hold on;
plot(br(1,:),br(2,:),'k');
plot(bl(1,:),bl(2,:),'k');
plot(bc(1,:),bc(2,:),'b');
nobs = length(xobs);
st = 1;
for i = 1 : nobs
    d_min = inf;
    ob = xobs{i};
    plot(ob(:,1),ob(:,2));
    for id = st : length(bc2)
        d = ob(4,:)'-bc2(:,id);
        d = sum(d.^2);
        if(d < d_min)
            d_min = d;
            nst = id;
        end
    end
    d1 = sum((bc2(:,nst-1)-ob(4,:)').^2);
    d2 = sum((bc2(:,nst+1)-ob(4,:)').^2)
    if(d2 < d1)
        nst = nst + 1;
    end
    
    d_min = sum((ob(4,:)'-bc2(:,nst)).^2);
    d_min2 = sum((ob(3,:)'-bc2(:,nst)).^2);
    if (d_min < d_min2)
        %on right
        bc(:,nst-2) = 2/3*bc2(:,nst-2)+1/3*bl(:,nst-2);
        bc(:,nst-1) = 1/2*bc2(:,nst-1)+1/2*bl(:,nst-1);
        bc(:,nst) = 1/2*bc2(:,nst)+1/2*bl(:,nst);
        bc(:,nst+1) = 2/3*bc2(:,nst+1)+1/3*bl(:,nst+1);
        seq = [1,4];
    else
        bc(:,nst-2) = 2/3*bc2(:,nst-2)+1/3*br(:,nst-2);
        bc(:,nst-1) = 1/2*bc2(:,nst-1)+1/2*br(:,nst-1);
        bc(:,nst) = 1/2*bc2(:,nst)+1/2*br(:,nst);
        bc(:,nst+1) = 2/3*bc2(:,nst+1)+1/3*br(:,nst+1);
        seq = [2,3];
    end
    st = nst;
end

bc = bc3;
b_theta = zeros([1,length(bc)])
b_theta(1) = atan2(bc(2,2)-bc(2,1), bc(1,2)-bc(1,1));
for i = 2 : length(bc)
    b_theta(i) = atan2(bc(2,i)-bc(2,i-1), bc(1,i)-bc(1,i-1));
end

T_STEP = 0.1;
SIM_STEP = 0.01;
C_STEP = 0.2;
T_LENGTH = 150;
HORIZON = 4;

av_gain = @(v) v*1.5;

a_max = 1;
% kp_d = 0.1;
% ki_d = 0;
% kd_d = 50;
e_dis_sum = 0;
e_dis_past = 0;
kp_v = 10;
ki_v = 0.005;
kd_v = 1000;
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
%x_init = [405.810806875530;12.2407338323243;159.438023905822;0.136512947607889;0.532840817108712;0.0486906571205679];
x_init = [779.695656214899;10.9892978664049;444.177726337599;0.0639630834620582;-0.383941951260136;0.213299953790787];

x = x_init;
id = 2;
%id = 7;
%id = 69;
%id = 140;
%id = 34;
id = 119;

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
    
    ct = 1;
    arc = sqrt(sum((bc(:,id_v) - bc(:,id_v-1)).^2));
    theta_arc = b_theta(id_v) - b_theta(id_v-1);
    v_d = sqrt(a_max*arc/abs(theta_arc));
    arc_k = 0;
    for id_temp = id-3 : id_v
        arc = sqrt(sum((bc(:,id_temp) - bc(:,id_temp-1)).^2));
        theta_arc = b_theta(id_temp) - b_theta(id_temp-1);
        arc_k = max(abs(theta_arc)/arc,arc_k);
        %v_d = v_d + sqrt(a_max*arc/abs(theta_arc));
        ct = ct + 1;
    end
    
    
    
    
    %arc = sqrt(sum((bc(:,id_v) - bc(:,id_v-1)).^2));
    %theta_arc = b_theta(id_v) - b_theta(id_v-1);
    %arc_k1 = abs(theta_arc)/arc;
    
    %arc = sqrt(sum((bc(:,id) - bc(:,id-1)).^2));
%     theta_arc = b_theta(id) - b_theta(id-1);
%     arc_k2 = abs(theta_arc)/arc;
%     arc = sqrt(sum((bc(:,id-1) - bc(:,id-2)).^2));
%     theta_arc = b_theta(id-1) - b_theta(id-2);
%     arc_k3 = abs(theta_arc)/arc;
%     %v_d = sqrt(a_max*arc/abs(theta_arc));
%     
%     arc_k = max(arc_k1,arc_k2);
%     arc_k = max(arc_k,arc_k3);

    v_d = v_d/ct;
    if(v_d > 20)
        v_d = 20;
    end
    
    v_d = 24;
    if(e_dis > 0.5 || arc_k > 0.05)
        v_d = 8;
        kp_d = 0.5;
        ki_d = 0.001;
        kd_d = 20;
    elseif(e_dis > 0.4 || arc_k > 0.04)
        v_d = 10;
        kp_d = 0.5;
        ki_d = 0.001;
        kd_d = 20;
    elseif(e_dis > 0.3 || arc_k > 0.03)
        v_d = 12;
        kp_d = 0.5;
        ki_d = 0.001;
        kd_d = 20;
    elseif(e_dis > 0.2 )
        v_d = 15;
        kp_d = 0.5;
        ki_d = 0;
        kd_d = 50;
    else
        v_d = 20;
        kp_d = 0.5;
        ki_d = 0;
        kd_d = 50;
    end
    v_d_log = [v_d_log;v_d];
    e_v = v_d - x(2);
    
    if (t>10.34)
        asdasd = 1;
    end
    
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
    if (t>0 && floor(floor(t)/5)==t/5)
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

% figure;
% hold on
% plot(x_log(2,:)');
% plot(v_d_log);