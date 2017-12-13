function X = odefun(x0,U_log)
W = 13720 ;
Nw = 2 ;
f = 0.01 ;
Iz = 2667 ;
a = 1.35 ;
b = 1.45 ;
By = 0.27 ;
Cy = 1.2 ;
Dy = 2921 ;
Ey = -1.6 ;
Shy = 0 ;
Svy = 0 ;
m = 1400 ;
U=U_log';
% generate input functions

T = 0:0.01:(size(U,1)-1)*0.01 ;
d_f = @(t) interp1(T,U(:,1),t,'previous','extrap') ;
F_x = @(t) interp1(T,U(:,2),t,'previous','extrap') ;



% slip angle functions in degrees
a_f = @(t,x) rad2deg(d_f(t)-atan2(x(4)+a*x(6),x(2))) ;
a_r = @(t,x) rad2deg(-atan2((x(4)-b*x(6)),x(2))) ;

% Nonlinear Tire Dynamics
phi_yf = @(t,x) (1-Ey)*(a_f(t,x)+Shy)+(Ey/By)*atan(By*(a_f(t,x)+Shy)) ;
phi_yr = @(t,x) (1-Ey)*(a_r(t,x)+Shy)+(Ey/By)*atan(By*(a_r(t,x)+Shy)) ;

F_yf = @(t,x) Dy*sin(Cy*atan(By*phi_yf(t,x)))+Svy ;
F_yr = @(t,x) Dy*sin(Cy*atan(By*phi_yr(t,x)))+Svy ;

% vehicle dynamics
dX = @(t,x) [x(2)*cos(x(5))-x(4)*sin(x(5)) ;...
          (-f*W+Nw*F_x(t)-F_yf(t,x)*sin(d_f(t)))/m+x(4)*x(6) ;...
          x(2)*sin(x(5))+x(4)*cos(x(5)) ;...
          (F_yf(t,x)*cos(d_f(t))+F_yr(t,x))/m-x(2)*x(6) ;...
          x(6) ;...
          (F_yf(t,x)*a*cos(d_f(t))-F_yr(t,x)*b)/Iz] ;
    
 [~,Xvec] = ode45(dX,T,x0) ;
 X=Xvec(end,:)';
end