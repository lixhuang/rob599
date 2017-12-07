function [c,ceq] = nonlcon(u,param)

    HORIZON_STEP = param.HORIZON_STEP;
    T_STEP = param.T_STEP;
    C_STEP = param.C_STEP;
    a = param.a;
    U1_SCALE = param.U1_SCALE;
    U2_SCALE = param.U2_SCALE;
    
    u(1:2:end) = u(1:2:end)/param.U1_SCALE;
    u(2:2:end) = u(2:2:end)/param.U2_SCALE;
    
    d_vec = zeros([param.len,1]);
    
    x = param.x0;
    x_log = [];
    it = 1;
    %for i = 0:T_STEP:HORIZON_STEP*C_STEP-T_STEP
    for uid = 1:HORIZON_STEP
        x_log = [x_log,x];
        %uid = floor(i/C_STEP)+1;
        x = f_car(x,u(uid*2-1:uid*2),T_STEP);
        
        %% find distance
        x0 = x(1);
        y0 = x(3);

        A1 = -(4*a(2)*y0+2*a(3));
        A2 = 2*a(3)*x0-2*a(1)*y0;
        A3 = 4*a(4)-4*a(2);
        A4 = -4*a(4)*x0-2*a(1);

        G4 = a(2)*A3^2;
        G3 = a(1)*A3^2 + 2*a(2)*A3*A4;
        G2 = a(2)*A4^2 + 2*a(1)*A3*A4 + a(3)*A1*A3 + a(4)*A1^2 + A3^2;
        G1 = a(1)*A4^2 + a(3)*A1*A4 + a(3)*A2*A3 + 2*a(4)*A1*A2 + 2*A3*A4;
        G0 = a(4)*A2^2 + a(3)*A2*A4 + A4^2;

        opx = x0;
        for opi = 1:20
            opx = opx - (G4*opx^4+G3*opx^3+G2*opx^2+G1*opx+G0)/(4*G4*opx^3+3*G3*opx^2+2*G2*opx^1+G1);
        end
        opy = (A1*opx+A2)/(A3*opx+A4);
        
        d = (x0-opx)^2+(y0-opy)^2;
        d_vec(uid) = (sqrt(d)-11.5/2)^1;
        
        it = it+1;
    end
    
    c = d_vec;
    
    
    
    
    
    ceq = 0;
end

