%% FIN OPTIMIZATION 
% Created by Ares Bustinza-Nguyen
% Updated: 3/18/2025

clear; close all;

% NOTES: (want to update so its easier to use for any OR file, will pull the sim through a dropdown, etc)
% make sure no mass components being optimized are being overriden in the selected OR file
% ensure that "Adjustable stability weight" has the same calculation if used in OR 

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

% stopping sim when the func has a value of less than or equal to 0.1
% stop = @(~, optim_values, ~) optim_values.fval <= 0.1;
opts = optimset(Display = "iter");
% check for it numerically (OutputFcn) .x maybe??? look into plot, optm) 
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

% next step (add plots)
%plot_openrocket(opt_param(1), "Altitude", "Vertical velocity", labels = ["BURNOUT", "APOGEE", "MAIN"]);
% plot_openrocket(opt_param(1))
% callback for realtime plotting ??? what current dimensions are (do it every 10 steps for runtime, etc.) show current values/apg/stab/fos
% drawn version of what the fin looks like 

%% OPTIMIZATION FUNCITONS

% setup for optimization routine
function func = costfunc_setup(rocket, sim, fins, opt_var, f, t)
    
    % setup
    sim.getOptions().setWindTurbulenceIntensity(0);
    fins = rocket.component(class = "FinSet"); 
    opt_mass = rocket.component(name = opt_var); 

    % set targets
    target_apogee = 3078.48; 
    target_stb_launchrod = 1.6; 
    target_stb_burnout = 3.5; 
    target_FOS = 1.55; 

    % define weights for each, add up to 1. values can be modified as well
    weight_apogee = 0.3;
    weight_stb_launchrod = 0.4;
    weight_stb_burnout = 0.3;
    weight_FOS = 0.1;
    
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

        % penalization (can add more in the future if necessary)
        penalty = 0;
        if apogee < target_apogee
            penalty = penalty + abs((3048 - apogee) / target_apogee) * 2;
        end

        % error calulations & normalization

        % to capture the fact that we care about diff params in diff ways, use different funct for each error value instead of exp at the end (like step)
        apg_err = ((apogee - target_apogee)/target_apogee) * weight_apogee;
        stbL_err = ((stb_launchrod - target_stb_launchrod)/target_stb_launchrod) * weight_stb_launchrod;
        stbB_err = ((stb_burnout - target_stb_burnout)/target_stb_burnout) * weight_stb_burnout;
        FOS_err = ((fos_calc - target_FOS)/target_FOS)* weight_FOS;

        cost_value = exp(abs(apg_err) + abs(stbL_err) + abs(stbB_err) + abs(FOS_err)) + penalty;
        % plot cost function? pick a point -- vary the stability, how does the cost function change plot how the function changes 
        % reader that you have documented the cost function TECH REPORT MATERIAL 
    end
end
