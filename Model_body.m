function Y = Model_body(X)
 F16parameter;%Parameter
 F16aerodata; %Koefisien aerodinamis

% State   = [0;0;-1000;175;0;0;0;0;0;0;0;0;0;50000]; %Inisialisasi State


X_earth = X(1); %Posisi sumbu x
Y_earth = X(2); %Posisi sumbu y
Z_earth = X(3); %Posisi sumbu z
Vt      = X(4); %Kecepatan total
alpha   = X(5); %Angle of Attack
beta    = X(6); %Angle of Sideslip
phi     = X(7); %Sudut roll
theta   = X(8); %Sudut pitch
psi     = X(9); %Sudut yaw
p_body  = X(10);%Kecepatan body x
q_body  = X(11);%Kecepatan body y
r_body  = X(12);%Kecepatan body z
u_body  = X(13);
v_body  = X(14);
w_body  = X(15);

da      = 0;%Sudut defleksi aileron
de      = 3;%Sudut defleksi elevator
dr      = 0;%Sudut defleksi rudder
thrust  = 40000;%Thrust

%Batas angle of attack
if alpha > 90*pi/180
    alpha = 90*pi/180;
elseif alpha < -20*pi/180
    alpha = -20*pi/180;
end

%Batas angle of sideslip
if beta > 30*pi/180
    beta = 30*pi/180;
elseif beta < -30*pi/180
    beta = -30*pi/180;
end

%Batas sudut aileron
if da > 21.5*pi/180
    da = 21.5*pi/180;
elseif da < -21.5*pi/180
    da = -21.5*pi/180;
end

%Batas sudut elevator
if de > 25*pi/180
    de = 25*pi/180;
elseif de < -25*pi/180
    de = -25*pi/180;
end

%Batas sudut rudder
if dr > 30*pi/180
    dr = 30*pi/180;
elseif dr< -30*pi/180
    dr = -30*pi/180;
end

%Batas throttle
% if dp > 1
%     dp = 1;
% elseif dp < 0
%     dp = 0;
% end

%Fungsi atmosfer
% [qbar,rho] = ISA_atmos(-Z_earth,Vt);
rho0 = 1.225;
R = 287.05;
T0 = 288.15;
g0 = 9.80665;
temp = 0;
if -Z_earth >= 11000.0
    temp = 216.65;
elseif -Z_earth < 11000.0
    temp = T0 - 0.0065 * -Z_earth;
end
rho = rho0 * exp((-g0 / (R * temp)) * -Z_earth);
qbar = 0.5 * rho * Vt * Vt;

%Kecepatan body
u_body = Vt*cos(alpha)*cos(beta);
v_body = Vt*sin(beta);
w_body = Vt*sin(alpha)*cos(beta);

%Transformasi radian ke degree
ALPHA = alpha*rtd;
BETA = beta*rtd;
DE = de*rtd;
DA = da*rtd;
DR = dr*rtd;
DLEF = 1.38*ALPHA-9.05*qbar/(101325*rho/1.225)+1.45;

%Batas sudut leading edge
if (DLEF > 25*pi/180)
    DLEF = 25*pi/180;
elseif (DLEF < 0)
    DLEF = 0;
end

%Batas tabel aerodinamis leading edge untuk alpha
if (ALPHA > 45)
    alpha_lef = 45;
else
    alpha_lef = ALPHA;
end

%Normalisasi aktuator
da_norm = DA/21.5;
dr_norm = DR/30.0;
dlef_norm = (1-DLEF/25.0);

%Aerodinamis
%CX
CX0 = interp3(f16data.beta,f16data.alpha1,f16data.de1,f16data.CX,BETA,ALPHA,DE);
delta_CX_lef = interp2(f16data.beta,f16data.alpha2,f16data.CX_lef,BETA,alpha_lef)-interp3(f16data.beta,f16data.alpha1,f16data.de1,f16data.CX,BETA,ALPHA,0);
CXq = interp1(f16data.alpha1,f16data.CXq,ALPHA);
dCXq_lef = interp1(f16data.alpha2,f16data.dCXq_lef,alpha_lef);


%CY
CY0 = interp2(f16data.beta,f16data.alpha1,f16data.CY,BETA,ALPHA);
delta_CY_lef= interp2(f16data.beta,f16data.alpha2,f16data.CY_lef,BETA,alpha_lef)-CY0;
delta_CY_da20 = interp2(f16data.beta,f16data.alpha1,f16data.CY_da20,BETA,ALPHA)-CY0;
delta_CY_da20lef = interp2(f16data.beta,f16data.alpha2,f16data.CY_da20lef,BETA,alpha_lef)- ...
    interp2(f16data.beta,f16data.alpha2,f16data.CY_lef,BETA,alpha_lef) - delta_CY_da20;
delta_CY_dr30= interp2(f16data.beta,f16data.alpha1,f16data.CY_dr30,BETA,ALPHA) - CY0;
CYr = interp1(f16data.alpha1,f16data.CYr,ALPHA);
dCYr_lef = interp1(f16data.alpha2,f16data.dCYr_lef,alpha_lef);
CYp = interp1(f16data.alpha1,f16data.CYp,ALPHA);
dCYp_lef = interp1(f16data.alpha2,f16data.dCYp_lef,alpha_lef);

%CZ
CZ0 = interp3(f16data.beta,f16data.alpha1,f16data.de1,f16data.CZ,BETA,ALPHA,DE);
delta_CZ_lef = interp2(f16data.beta,f16data.alpha2,f16data.CZ_lef,BETA,alpha_lef) - interp3(f16data.beta,f16data.alpha1,f16data.de1,f16data.CZ,BETA,ALPHA,0);
CZq = interp1(f16data.alpha1,f16data.CZq,ALPHA);
dCZq_lef = interp1(f16data.alpha2,f16data.dCZq_lef,alpha_lef);

%Cm
Cm0 = interp3(f16data.beta,f16data.alpha1,f16data.de1,f16data.Cm,BETA,ALPHA,DE);
delta_Cm_lef = interp2(f16data.beta,f16data.alpha2,f16data.Cm_lef,BETA,alpha_lef)- ...
    interp3(f16data.beta,f16data.alpha1,f16data.de1,f16data.Cm,BETA,ALPHA,0);
Cmq = interp1(f16data.alpha1,f16data.Cmq,ALPHA);
dCmq_lef = interp1(f16data.alpha2,f16data.dCmq_lef,alpha_lef);
dCm = interp1(f16data.alpha1,f16data.dCm,ALPHA);
dCm_ds = interp2(f16data.de3,f16data.alpha1,f16data.dCm_ds,DE,ALPHA);

%Cn
Cn0 = interp3(f16data.beta,f16data.alpha1,f16data.de2,f16data.Cn,BETA,ALPHA,DE);
delta_Cn_lef = interp2(f16data.beta,f16data.alpha2,f16data.Cn_lef,BETA,alpha_lef) - ...
   interp3(f16data.beta,f16data.alpha1,f16data.de2,f16data.Cn,BETA,ALPHA,0) ;
delta_Cn_da20 = interp2(f16data.beta,f16data.alpha1,f16data.Cn_da20,BETA,ALPHA) - ...
    interp3(f16data.beta,f16data.alpha1,f16data.de2,f16data.Cn,BETA,ALPHA,0);
delta_Cn_da20lef = interp2(f16data.beta,f16data.alpha2,f16data.Cn_da20lef,BETA,alpha_lef) - ...
     interp2(f16data.beta,f16data.alpha2,f16data.Cn_lef,BETA,alpha_lef) - delta_Cn_da20  ;
delta_Cn_dr30 = interp2(f16data.beta,f16data.alpha1,f16data.Cn_dr30,BETA,ALPHA) - ...
    interp3(f16data.beta,f16data.alpha1,f16data.de2,f16data.Cn,BETA,ALPHA,0) ;
Cnr = interp1(f16data.alpha1,f16data.Cnr,ALPHA);
dCnbeta = interp1(f16data.alpha1,f16data.dCnbeta,ALPHA);
dCnr_lef = interp1(f16data.alpha2,f16data.dCnr_lef,alpha_lef);
Cnp = interp1(f16data.alpha1,f16data.Cnp,ALPHA);
dCnp_lef = interp1(f16data.alpha2,f16data.dCnp_lef,alpha_lef);

%Cl
Cl0 = interp3(f16data.beta,f16data.alpha1,f16data.de2,f16data.Cl,BETA,ALPHA,DE);
delta_Cl_lef = interp2(f16data.beta,f16data.alpha2,f16data.Cl_lef,BETA,alpha_lef) - ...
    interp3(f16data.beta,f16data.alpha1,f16data.de2,f16data.Cl,BETA,ALPHA,0);
delta_Cl_da20 = interp2(f16data.beta,f16data.alpha1,f16data.Cl_da20,BETA,ALPHA) - ...
    interp3(f16data.beta,f16data.alpha1,f16data.de2,f16data.Cl,BETA,ALPHA,0);
delta_Cl_da20lef = interp2(f16data.beta,f16data.alpha2,f16data.Cl_da20lef,BETA,alpha_lef)- ...
   interp2(f16data.beta,f16data.alpha2,f16data.Cl_lef,BETA,alpha_lef) - delta_Cl_da20 ;
delta_Cl_dr30 = interp2(f16data.beta,f16data.alpha1,f16data.Cl_dr30,BETA,ALPHA) - ...
    interp3(f16data.beta,f16data.alpha1,f16data.de2,f16data.Cl,BETA,ALPHA,0);
Clr = interp1(f16data.alpha1,f16data.Clr,ALPHA);
dClbeta = interp1(f16data.alpha1,f16data.dClbeta,ALPHA);
dClr_lef = interp1(f16data.alpha2,f16data.dClr_lef,alpha_lef);
Clp = interp1(f16data.alpha1,f16data.Clp,ALPHA);
dClp_lef = interp1(f16data.alpha2,f16data.dClp_lef,alpha_lef);

%Perhitungan koefisien total aerodinamis
%Cx_tot 
CX_tot = CX0 + delta_CX_lef  * dlef_norm + ...
    (F16data.cref/(2*Vt))*(CXq + dCXq_lef * dlef_norm) * q_body;

%Cy_tot
CY_tot = CY0 + delta_CY_lef * dlef_norm+ ...
    (delta_CY_da20 + delta_CY_da20lef  * dlef_norm) * da_norm+ ...
    delta_CY_dr30 * dr_norm+ ...
    (F16data.bref / (2*Vt))*(CYr + dCYr_lef * dlef_norm) * r_body+ ...
    (F16data.bref/(2*Vt))*(CYp + dCYp_lef * dlef_norm) * p_body;

%Cz_tot 
CZ_tot = CZ0 + delta_CZ_lef * dlef_norm + ...
      (F16data.cref/(2*Vt))*(CZq + dCZq_lef * dlef_norm) * q_body;

%Cl_tot 
Cl_tot = Cl0 + delta_Cl_lef * dlef_norm ... 
     + (delta_Cl_da20 + delta_Cl_da20lef * dlef_norm) * da_norm ...
     + delta_Cl_dr30 * dr_norm ...
     + (F16data.bref / ((2*Vt))*(Clr + dClr_lef * dlef_norm)) * r_body ...
     + ((F16data.bref / (2*Vt)) * (Clp + dClp_lef * dlef_norm)) * p_body ...
     + dClbeta* BETA;

%Cm_tot
Cm_tot = Cm0 * 1.0 + CZ_tot * (F16data.xcgr - F16data.xcg) + delta_Cm_lef * dlef_norm ...
     + (F16data.cref / (2*Vt))*(Cmq + dCmq_lef  * dlef_norm) * q_body ...
     + dCm + dCm_ds ;
 
%Cn_tot
Cn_tot = Cn0 + delta_Cn_lef * dlef_norm ...
     - CY_tot * (F16data.xcgr - F16data.xcg)*(F16data.cref/F16data.bref) ...
     + (delta_Cn_da20 + delta_Cn_da20lef * dlef_norm) * da_norm ...
     + ((F16data.bref / (2*Vt)) * (Cnr + dCnr_lef * dlef_norm))* r_body ...
     + ((F16data.bref / (2*Vt)) * (Cnp + dCnp_lef * dlef_norm)) * p_body ...
     + delta_Cn_dr30  * dr_norm + dCnbeta * beta * rtd;
 
%Perhitungan total force
% Xbar = qbar*F16data.Sref*CX_tot;
% Ybar = qbar*F16data.Sref*CY_tot;
% Zbar = qbar*F16data.Sref*CZ_tot;
 
%Perhitungan total moment
Lbar = Cl_tot*qbar*F16data.Sref*F16data.bref;
Mbar = Cm_tot*qbar*F16data.Sref*F16data.cref;
Nbar = Cn_tot*qbar*F16data.Sref*F16data.bref;
 
%Perhitungan turunan
u_body_dot = r_body*v_body - q_body*w_body - g*sin(theta)...
    + qbar*F16data.Sref*CX_tot/F16data.mass + thrust/F16data.mass;
v_body_dot = p_body*w_body - r_body*u_body + g*cos(theta)*sin(phi)...
    + qbar*F16data.Sref*CY_tot/F16data.mass;
w_body_dot = q_body*u_body - p_body*v_body  + g*cos(theta)*cos(phi)...
    + qbar*F16data.Sref*CZ_tot/F16data.mass; 
 
Vt_dot = (u_body*u_body_dot + v_body*v_body_dot ...
    + w_body*w_body_dot)/Vt;
beta_dot = (v_body_dot*Vt - v_body*Vt_dot) ...
    / (Vt*Vt*cos(beta));
alpha_dot = (u_body*w_body_dot - w_body*u_body_dot) ...
    / (u_body*u_body + w_body*w_body);

%Kinetik
phi_dot = p_body + tan(theta)*(q_body*sin(phi) + r_body*cos(phi));
theta_dot = q_body*cos(phi) - r_body*sin(phi);
psi_dot = (q_body*sin(phi) + r_body*cos(phi)) / cos(theta);

p_body_dot  = (F16data.C1 * r_body + F16data.C2 * p_body) * q_body ...
    + F16data.C3 * Lbar + F16data.C4 * (Nbar + q_body * F16data.heng);
q_body_dot  = F16data.C5 * p_body * r_body ...
    - F16data.C6 * (p_body * p_body - r_body * r_body) ...
    + F16data.C7 * (Mbar - F16data.heng * r_body);
r_body_dot  = (F16data.C8 * p_body -F16data.C2 * r_body) * q_body ...
    + F16data.C4 * Lbar + F16data.C9 * (Nbar + q_body * F16data.heng);

X_earth_dot = u_body * (cos(theta)*cos(psi))...
    + v_body*(sin(phi)*cos(psi)*sin(theta) - cos(phi)*sin(psi))...
    + w_body*(cos(phi)*sin(theta)*cos(psi) - sin(phi)*sin(psi));
Y_earth_dot = u_body * (cos(theta)*sin(psi))...
    + v_body*(sin(phi)*sin(psi)*sin(theta) + cos(phi)*cos(psi))...
    + w_body*(cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi));
Z_earth_dot = u_body*sin(theta) - v_body*(sin(phi)*cos(theta))...
    - w_body*(cos(phi)*cos(theta));

% %Update state
% NewState(1) = X_earth;
% NewState(2) = Y_earth;
% NewState(3) = Z_earth;
% NewState(4) = Vt;
% NewState(5) = alpha;
% NewState(6) = beta;
% NewState(7) = phi;
% NewState(8) = theta;
% NewState(9) = psi;
% NewState(10) = p_body;
% NewState(11) = q_body;
% NewState(12) = r_body;
% NewState(13) = thrust;



Y = [X_earth_dot Y_earth_dot Z_earth_dot Vt_dot alpha_dot beta_dot...
    phi_dot theta_dot psi_dot p_body_dot q_body_dot r_body_dot...
    u_body_dot v_body_dot w_body_dot];

end
