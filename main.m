%global x0;
%global HORIZON_STEP;
%global T_STEP;


br = TestTrack.br;
bl = TestTrack.bl;
bc = TestTrack.cline;

T_STEP = 0.1;
C_STEP = 0.2;
T_LENGTH = 300;
HORIZON_STEP = 25;
U1_SCALE = 1;
U2_SCALE = 0.001;


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
%x_init = [287; 5; -176; 0; 2; 0];
x_init = [255; 10; -91; 0; 2; 0];
%x_init = [650.078934737117;9.96237194981144;461.327390311492;0.188743033523074;0.465875093881878;0.101631509480780];
lb = ones([HORIZON_STEP*2,1]); 
lb(1:2:end) = lb(1:2:end)*-0.5*U1_SCALE;
lb(2:2:end) = lb(2:2:end)*-10000*U2_SCALE;
ub = ones([HORIZON_STEP*2,1]); 
ub(1:2:end) = ub(1:2:end)*0.5*U1_SCALE;
ub(2:2:end) = ub(2:2:end)*5000*U2_SCALE;

x = x_init;
%id = 1;
id = 5;
%id = 99;
u = ones([HORIZON_STEP*2,1]);
u(1:2:end) = u(1:2:end)*0*U1_SCALE;
u(2:2:end) = u(2:2:end)*100*U2_SCALE;
x_log = [];

for t = 0:T_STEP:T_LENGTH
    
    x_log = [x_log,x];
    
    %% prepare initial guess
    %x0 = x;
    param.x0 = x;
    u0 = [u(3:end);u(end-1:end)];
    
    prox_len = 5;
    px = bc(1,id:id+prox_len-1);
    py = bc(2,id:id+prox_len-1);
    A=[px;px.^2;py;py.^2]';
    b = -1*ones([prox_len,1]);
    a = (A'*A)^-1*A'*b;
    param.a = a;
        
    option = optimoptions('fmincon','MaxFunctionEvaluations',10000);
    u = fmincon(@(x)cost(x,param), u0, [], [], [], [], lb, ub, @(x)nonlcon(x,param),option);
    
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
    %aprox_center = get_proxline([px(1);py(1)],[px(end);py(end)],a);
    %plot(aprox_center(1,:),aprox_center(2,:),'g-')
    plot(debug_log(1,:),debug_log(3,:),'r')
    drawnow;
      
    if(t>3.5)
        asdasd=0;
    end
    
    
    %% evolve system and found nearset point
    x = f_car(x,[u(1)/U1_SCALE;u(2)/U2_SCALE], T_STEP);
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
    t
    [c,ceq] = nonlcon(u,param);
    d_max_debug = max(c)
end


