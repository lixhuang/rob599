function traj = get_proxline(xs, xe, a)
    x = xs;
    traj = [];
    for i = 1:30000
        dt = 0.02;
        v = [a(3)+2*a(4)*x(2);-a(1)+2*a(2)*x(1)];
        v = v/sqrt(sum(v.^2));
        x1 = x + v*dt;
        x2 = x - v*dt;
        if(v(2)>0)
            x = x1;
        else
            x = x2;
        end
        %if(sqrt(sum((x-xe).^2))<100)
        %    asdasdasd=0;
        %end
        traj = [traj,x];
    end
    
end

