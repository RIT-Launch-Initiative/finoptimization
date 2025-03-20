% Fin Flutter 

function [FINAL_FOS] = FOS_finflutter(data, fins)
    arguments
        data timetable
        fins
    end

    t = fins.getThickness();
    Ls = fins.getSweep();
    Lt = fins.getTipChord();
    Lr = fins.getRootChord();
    h = fins.getHeight();
    
    [max_velocity, idx_max_velocity] = max(data.("Total velocity"));
    
    % Simulation OpenRocket Variables 
    % Max Velocity [m/s]
    V = max_velocity;

    % Pressure at Max Velocity [pascals]
    P = data{idx_max_velocity, "Air pressure"};

    % Speed of Sound at Max Velocity [m/s]
    Vs = data{idx_max_velocity, "Speed of sound"};
    
    % FIXED VARIABLES 
    % Shear Modulus [pascals]
    G = 9.997398e+9;
    
    % Fixed Variables
    P0 = 101325.3; % Sea Level Pressure [pascals]
    k = 1.4; % Specific Heat Ratio for Air [unitless]
    Cm = 24; % Martin's Constant [unitless]
    
    
    % Fin Center of Mass Distance "Cx"
    Cx = ((2 * Ls * Lt) + (Lt^2) + (Lr * Ls) + (Lt * Lr) + (Lr^2))/(3 * (Lt+Lr));
    
    % Epsilon [unitless]
    eps = (Cx/Lr) - 0.25;
    
    % Denominator Constant "Dc" 
    Dc = (Cm * k * eps * P0)/(pi);
    
    % Lamda [unitless]
    lambda = Lt/Lr;
    
    % Fin Area
    A = h *((Lt + Lr)/2);
    
    % Fin Aspect Ratio [unitless]
    Ar = (h^2)/A;
    
    % FIN FLUTTER VELOCITY 
    Vf = Vs * (sqrt (G/ ( ((Dc*Ar^3)/((t/Lr)^3 * (Ar + 2))) * ((lambda + 1)/2) * (P/P0) )));
    
    % SAFTEY MARGIN 
    safe_marg = Vf - V;
    
    % SAFTEY MARGIN [%]
    safe_marg_percent = (safe_marg/V);

    FINAL_FOS = safe_marg_percent + 1;
   
end