X_earth0 = 0;
Y_earth0 = 0;
Z_earth0 = -1000;
Vt0      = 150;
alpha0   = 0;
beta0    = 0;
phi0     = 0;
theta0   = 0;
psi0     = 0;
p_body0  = 0;
q_body0  = 0;
r_body0  = 0;
u_body0  = 150;
v_body0  = 0;
w_body0  = 0;

X0 = [X_earth0 Y_earth0 Z_earth0 Vt0 alpha0 beta0...
    phi0 theta0 psi0 p_body0 q_body0 r_body0...
    u_body0 v_body0 w_body0];
X=X0;


tic;
time_now=0;
iterasi=0;
Ts=1e-2;
simulation_time=2000;

iterasi_save = zeros(1,simulation_time);
pitch_save = zeros(1,simulation_time);
while iterasi<=simulation_time
    %if toc >= time_now+Ts
        time_now = time_now+Ts;
        % Runge-Kutta 4th
         k1 = Model_body(X);
         k2 = Model_body(X+0.5*Ts*k1);
         k3 = Model_body(X+0.5*Ts*k2);
         k4 = Model_body(X+Ts*k3);
         Y  = X+(1/6)*(k1+2*k2+2*k3+k4)*Ts;
         X  = Y;    
        
        iterasi=iterasi+1;
        iterasi_save(iterasi)=iterasi;
        pitch_save(iterasi)=X(8)*57.2958;        
    %end
end
toc;
figure
plot(iterasi_save,pitch_save)
grid on

