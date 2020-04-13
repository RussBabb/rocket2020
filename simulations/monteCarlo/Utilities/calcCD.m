function C_D = calcCD(v, stage, rho_inf, c_inf, mu_inf)
A_wet = stage.A_wet;
A_ref = stage.A_ref;
l_ref = stage.l_ref;

A_surf = stage.fins.A_surf;
W = stage.fins.W;
t = stage.fins.t;
AR = stage.fins.AR;
theta_LE = stage.fins.theta_LE;
N_fins = stage.fins.N;

k = stage.k;
d = stage.tail.d;
D = stage.tail.D;

gamma = 1.4;
d_alpha = degtorad(3);

%Get atmospheric values
Re = rho_inf*v*l_ref/mu_inf;
M_inf = v/c_inf;

%Calculate Skin Friction Coefficient
C_F = calcCFblasius(Re);

%Calculate Skin Friction Drag Coefficient
C_D_F = A_wet/A_ref*C_F;

%Calculate Forebody Form (Pressure) Drag Coefficient
C_D_0 = k*A_wet/A_ref*C_F;

%Calculate Base Drag Coefficient
C_D_base = 0.029/sqrt(A_wet/A_ref*C_F);

%Calculate Boat Tail Effect
C_D_tail = C_D_base*(d/D)^2;

%Calculate Fin Dither Drag Coefficient
C_D_dither = N_fins*(A_surf/A_ref)*(4*pi*AR)/(2 + sqrt(4 + AR^2))^2*d_alpha^2;

%Calculate Fin Leading Edge Drag Coefficient
PartA = (1 + (gamma - 1)/2*(M_inf*cos(theta_LE))^2)^(gamma/(gamma - 1)) - 1;
PartB = gamma/2*M_inf*cos(theta_LE)^2;
C_D_LE = N_fins*(W*t/A_ref)*PartA/PartB;

%Add up drag coefficients
if v <= 0
    C_D_total_incomp = stage.C_D;
else
    C_D_total_incomp = C_D_F + C_D_0 + C_D_base - C_D_tail + C_D_dither + C_D_LE;
end

%Apply bulk compressibility correcton
C_D = calcCDcomp(C_D_total_incomp, M_inf);
end