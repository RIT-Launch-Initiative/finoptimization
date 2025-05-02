%% FIN OPTIMIZATION 
% Created by Ares Bustinza-Nguyen
% Updated: 3/18/2025

clear; close all;

%% SETTING FLIGHT CONDITIONS, CONSTRAINTS ---------------------------------

otis_path = "data/OTIS.ork"; 
if ~isfile(otis_path)
    error("Error: not on path", otis_path);
end

otis = openrocket(otis_path);
sim = otis.sims("15MPH-TEXAS-36C-(TYP)");
 
fins = otis.component(class = "FinSet"); 
if ~isscalar(fins)
    error("Error: multiple fin sets found");
end

% calling fin flutter calculation function
f = @FOS_finflutter;

% allowing user to pick from predetermined stock thickness
t = user_thickness();


global cost_tracking
cost_tracking = [];
%% OPTIMIZATION

% calling the setup function and creating a var to pass into fminsearch
absolute_cost = costfunc_setup(otis, sim, fins, "Adjustable stability weight", f, t);

% initatilzing 
init_Ls = fins.getSweep();
init_Lt = fins.getTipChord();
init_Lr = fins.getRootChord();
init_h = fins.getHeight();
init_n = otis.component(name = "Adjustable stability weight").getComponentMass();

starting_vals = [init_Ls, init_Lt, init_Lr, init_h, init_n];

% displaying starting values
disp("INITIAL VALUES")
titles = {'Thickness', 'Sweep Length', 'Tip Chord', 'Root Chord', 'Height', 'Weight'};
fprintf('%-15s %-15s %-15s %-15s %-15s %-15s\n', titles{:});

disp_startvals = [t, init_Ls, init_Lt, init_Lr, init_h, init_n];
fprintf('%-15.4f %-15.4f %-15.4f %-15.4f %-15.4f %-15.4f\n', disp_startvals);

% minimize cost function using fminsearch
opts = optimset(Display = "iter");
opt_params = fminsearch(absolute_cost, starting_vals, opts);

%% FINAL SIM

% final optimized values passed to OR
fins.setThickness(t);
fins.setSweep(opt_params(1));
fins.setTipChord(opt_params(2));
fins.setRootChord(opt_params(3));
fins.setHeight(opt_params(4));
otis.component(name = "Adjustable stability weight").setComponentMass(opt_params(5));

sim.getOptions().setWindTurbulenceIntensity(0);
simdata = openrocket.simulate(sim, outputs = "ALL");

% apogee calculation (can use either)
final_apogee = max(simdata.Altitude);
%final_apogee = simdata{eventfilter("APOGEE"), "Altitude"};

% stability (could make this a calculation in the future, time seems fine otherwise)
data_range = timerange(eventfilter("LAUNCHROD"), eventfilter("BURNOUT"), "openleft");
simdata = simdata(data_range, :); 
final_stb_launchrod = simdata{1, "Stability margin"}; % launchrod
final_stb_burnout = simdata{end, "Stability margin"}; % burnout

% calling FOS calculation function
final_fos = f(simdata, fins);

% displaying final results
disp("FINAL VALUES")
results = [final_apogee, final_stb_launchrod, final_stb_burnout, final_fos];
titles = {'Apogee', 'STB (LR)', 'STB (BT)', 'FOS'};
fprintf('%-15s %-15s %-15s %-15s\n', titles{:});
fprintf('%-15.4f %-15.4f %-15.4f %-15.4f\n\n', results);


titles = {'Thickness', 'Sweep Length', 'Tip Chord', 'Root Chord', 'Height', 'Weight'};
fprintf('%-15s %-15s %-15s %-15s %-15s %-15s\n', titles{:});
disp_optparams = [t, opt_params(1), opt_params(2), opt_params(3), opt_params(4), opt_params(5)];
fprintf('%-15.4f %-15.4f %-15.4f %-15.4f %-15.4f %-15.4f\n', disp_optparams);


figure
plot(cost_tracking, 'LineWidth', 2);
xlabel('Iteration');
ylabel('Cost Value');
title('Evaluating Cost Evoluntion During Optimization');

%% OPTIMIZATION FUNCITONS

% setup for optimization routine
function func = costfunc_setup(rocket, sim, fins, opt_var, f, t)
    
    % setup
    sim.getOptions().setWindTurbulenceIntensity(0);
    fins = rocket.component(class = "FinSet"); 
    opt_mass = rocket.component(name = opt_var); 

    % set targets for mission parameters
    target_apg = 3078.48; 
    target_stbL = 1.55; 
    target_stbB = 3.6; 
    target_FOS = 1.54; 

    % define weights for each, modifies how the function behaves and affects the weight of the cost value
    % [greater number = more lenient above target, greater numer = less lenient below target]
    weights_apg = [2, 0.5];
    weights_stbL = [2.4, 0.3];
    weights_stbB = [0.5, 0.5];
    weights_FOS = [0.6, 0.5];
    
    func = @cost;

    % nested function for good practice (no global variables), no need to redefine/reinit either
    function cost_value = cost(x)
        % pull initial parameters
        Ls = x(1); 
        Lt = x(2); 
        Lr = x(3); 
        h = x(4); 
        n = x(5);

        % set openrocket variables
        fins.setThickness(t); 
        fins.setSweep(Ls); 
        fins.setTipChord(Lt); 
        fins.setRootChord(Lr); 
        fins.setHeight(h); 
        opt_mass.setComponentMass(n);
        
        % can change so you're only pulling necessary
        simdata = rocket.simulate(sim, outputs = "ALL");
        
        % apogee calculation
        apogee = max(simdata.Altitude); 
        %apogee = simdata{eventfilter("APOGEE"), "Altitude"};

        % stability (want to make this calculation for time save, might not be necessary)
        data_range = timerange(eventfilter("LAUNCHROD"), eventfilter("BURNOUT"), "openleft");
        simdata = simdata(data_range, :);
        stb_launchrod = simdata{1, "Stability margin"}; % launchrod
        stb_burnout = simdata{end, "Stability margin"}; % burnout

        % FOS calculation 
        fos_calc = f(simdata, fins);

        % deltas for all mission parameters
        delta_apg = (apogee/target_apg) * 100;
        delta_stbL = (stb_launchrod/target_stbL) * 100;
        delta_stbB = (stb_burnout/target_stbB) * 100;
        delta_FOS = (fos_calc/target_FOS) * 100;
        
        penalty = 0;

        if delta_apg < 98 
            penalty = penalty + 100;
        end

        % fos here

        if fins.getThickness() <= 0 || fins.getSweep() <= 0 || fins.getTipChord() <= 0 || fins.getRootChord() <= 0 || fins.getHeight() <= 0 || opt_mass.getComponentMass() < 0
            penalty = penalty + 100;
        end

        % function penalizes heavily if negative and less harshly if positive.
        apg_err = abs(-weights_apg(1) * (delta_apg - 100)^2 * (exp (-weights_apg(2) * (delta_apg - 100)) - 1)); 
        stbL_err = abs(-weights_stbL(1) * (delta_stbL - 100)^2 * (exp (-weights_stbL(2) * (delta_stbL - 100)) - 1)); 
        stbB_err = abs(-weights_stbB(1) * (delta_stbB - 100)^2 * (exp (weights_stbB(2) * (delta_stbB - 100)) - 1)); 
        FOS_err = abs(-weights_FOS(1) * (delta_FOS - 100)^2 * (exp (-weights_FOS(2) * (delta_FOS - 100)) - 1)); 
   
        cost_value = apg_err + stbL_err + stbB_err + FOS_err + penalty;
        
        %d ebug purposes
        global cost_tracking
        cost_tracking(end + 1) = cost_value;
    end
end
