% Fin Flutter 

function [FINAL_FOS] = FOS_finflutternext(t, Ls, Lt, Lr, h, otis)
    
arguments
    t 
    Ls 
    Lt 
    Lr 
    h 
    otis 
end

    % Simulation OpenRocket Variables (find a way to call this from OR), calling certain time points i believe
    V = 999; % Max Velocity [ft/s]
    Am = 725; % Altitude of Max Velocity [ft - AGL] 
    As = 4686; % Launch Site Altitude [ft - ASL] THIS FEELS LIKE ITS NOT FROM OPENROCKET
    P = 12.1; % Pressure at Max Velocity [psi]
    Vs = 1158; % Speed of Sound at Max Velocity [ft/s]
    
    % FIXED VARIABLES 
    % Shear Modulus [psi]
    G = 1450000;
    
    % Fixed Variables
    P0 = 14.696; % Sea Level Pressure [psi]
    k = 1.4; % Specific Heat Ratio for Air [unitless]
    Cm = 24; % Martin's Constant [unitless]
    
    % Fin Center of Mass Distance "Cx" [in]
    Cx = ((2 .* Ls .* Lt) + (Lt.^2) + (Lr .* Ls) + (Lt .* Lr) + (Lr^2))/(3 * (Lt+Lr));
    
    % Epsilon [unitless]
    eps = (Cx/Lr) - 0.25;
    
    % Denominator Constant "Dc" [psi]
    Dc = (Cm * k * eps * P0)/(pi);
    
    % Lamda [unitless]
    lambda = Lt/Lr;
    
    % Fin Area [in^2]
    A = h*((Lt + Lr)/2);
    
    % Fin Aspect Ratio [unitless]
    Ar = (h^2)/A;
    
    % Term 1 [psi]
    T1 = (Dc * Ar^3)/((t/Lr)^3 * (Ar + 2));
    
    % Term 2 [psi] 
    T2 = (lambda + 1)/2;
    
    % Term 3 [unitless]
    T3 = P/P0;
    
    % FIN FLUTTER VELOCITY [ft/s]
    Vf = Vs * (sqrt (G/ ( ((Dc*Ar^3)/((t/Lr)^3 * (Ar + 2))) * ((lambda + 1)/2) * (P/P0) )));
    
    % SAFTEY MARGIN [ft/s]
    safe_marg = Vf - V;
    
    % SAFTEY MARGIN [%]
    safe_marg_percent = (safe_marg/V);

    FINAL_FOS = safe_marg_percent + 1;
   
end