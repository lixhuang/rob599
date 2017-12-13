%global x0;
%global HORIZON_STEP;
%global T_STEP;


br = TestTrack.br;
bl = TestTrack.bl;
bc = TestTrack.cline;
tv = bc(:,end)-bc(:,end-1);
bc2 = bc;
for i = 1:150
    tbc = bc2(:,end)+tv;
    bc2 = [bc2,tbc];
end
b_theta = TestTrack.theta;

T_STEP = 0.1;
SIM_STEP = 0.01;
C_STEP = 0.2;
T_LENGTH = 90;
HORIZON_STEP = 50;
U1_SCALE = 30;
U2_SCALE = 0.00005;


param.T_STEP = T_STEP;
param.C_STEP = C_STEP;
param.T_LENGTH = T_LENGTH;
param.HORIZON_STEP = HORIZON_STEP;
param.U1_SCALE = U1_SCALE;
param.U2_SCALE = U2_SCALE;

it = 1
for i = 0:T_STEP:HORIZON_STEP*C_STEP
    it = it + 1;
end
param.len = HORIZON_STEP;

% x, u, y, v, phi, r
x_init = [287; 5; -176; 0; 2; 0];
%x_init = [253.5; 10; -94; 0; 1.9; 0];
%x_init = [562.55;14.51;316.84;-2.09;1.01;0.24];
%x_init = [878.343380813432;24.6901208124630;458.223577742850;-0.885645635966873;0.836579999546011;0.114723547684054];
%x_init = [1158.20246078656;36.3882592898024;491.024703989237;-0.926198515281967;0.285013275433445;0.0100445678163843]
lb = ones([HORIZON_STEP*2,1]); 
lb(1:2:end) = lb(1:2:end)*-0.5*U1_SCALE;
lb(2:2:end) = lb(2:2:end)*-10000*U2_SCALE;
ub = ones([HORIZON_STEP*2,1]); 
ub(1:2:end) = ub(1:2:end)*0.5*U1_SCALE;
ub(2:2:end) = ub(2:2:end)*5000*U2_SCALE;

x = x_init;
id = 1;
%id = 7;
%id = 69;
%id = 140;
%id = 189
u = ones([HORIZON_STEP*2,1]);
u(1:2:end) = u(1:2:end)*0*U1_SCALE;
u(2:2:end) = u(2:2:end)*100*U2_SCALE;
x_log = [];
u_log = [];

for t = 0:SIM_STEP:T_LENGTH
    
    x_log = [x_log,x];
    
    %% prepare initial guess
    %x0 = x;
    prox_len =100;
    param.x0 = x;
    param.prox_len =prox_len;
    param.bc = bc2(:,id:id+prox_len-1);
    %param.theta = b_theta(:,id:id+prox_len-1);
    u0 = [u(3:end);u(end-1:end)];
    
    %px = bc(1,id:id+prox_len-1)-bc(1,id-1);
    %py = bc(2,id:id+prox_len-1)-bc(2,id-1);
    %plx = bl(1,id:id+prox_len-1)-bc(1,id-1);
    %ply = bl(2,id:id+prox_len-1)-bc(2,id-1);
    %prx = br(1,id:id+prox_len-1)-bc(1,id-1);
    %pry = br(2,id:id+prox_len-1)-bc(2,id-1);
    %A=[px;px.^2;py;py.^2;px.*py;px.^3;py.^3;px.*py.^2;py.*px.^2]';%;px.^4;px.^3.*py;px.^2.*py.^2;px.*py.^3;py.^4]';
    %Al=[plx;plx.^2;ply;ply.^2;plx.*ply;plx.^3;ply.^3;plx.*ply.^2;ply.*plx.^2]';
    %Ar=[prx;prx.^2;pry;pry.^2;prx.*pry;prx.^3;pry.^3;prx.*pry.^2;pry.*prx.^2]';
    %b = 1*ones([prox_len,1]);
    %a = (A'*A)^-1*A'*b; %overdetermined
    %al = (Al'*Al)^-1*Al'*b; %overdetermined
    %ar = (Ar'*Ar)^-1*Ar'*b; %overdetermined
    
    %a = A^-1*b; %exact
    %al = Al^-1*b; %exact
    %ar = Ar^-1*b; %exact
    
    %ar = Ar'*(Ar*Ar')^-1*b;
    %al = Al'*(Al*Al')^-1*b;
    %a = A'*(A*A')^-1*b;
    %param.a = a;
        
    option = optimoptions('fmincon','MaxFunctionEvaluations',25000);
    if(floor(t*10)==t*10)
        u = fmincon(@(x)cost(x,param), u0, [], [], [], [], lb, ub, @(x)nonlcon(x,param),option);
        t
    end
    
    debug_log=[];
    x_temp = x;
    it = 1;
    %for i = 0:T_STEP:HORIZON_STEP*C_STEP-T_STEP
    for uid = 1:HORIZON_STEP
        %uid = floor(i/C_STEP)+1;
        debug_log = [debug_log,x_temp];
        x_temp = f_car(x_temp,[u(uid*2-1)/U1_SCALE;u(uid*2)/U2_SCALE],T_STEP);
        it = it+1;
    end
    
    clf
    plot(bc(1,:),bc(2,:))
    hold on;
    plot(bl(1,:),bl(2,:),'k')
    plot(br(1,:),br(2,:),'k')
    %aprox_center = get_proxline([px(1);py(1)],[px(end);py(end)],a);
    %plot(aprox_center(1,:),aprox_center(2,:),'g-')
    plot(debug_log(1,:),debug_log(3,:),'r')
    drawnow;
      
    if(t>3.5)
        asdasd=0;
    end
    
    
    %% evolve system and found nearset point
    u_log = [u_log,[u(1)/U1_SCALE;u(2)/U2_SCALE]];
    if (t>0 && floor(t)*100==t*100)
        x = odefun(x_init,u_log);
    else
        x=f_car(x,[u(1)/U1_SCALE;u(2)/U2_SCALE], SIM_STEP);
    end
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
    
    [c,ceq] = nonlcon(u,param);
    %d_max_debug = max(c)
end


