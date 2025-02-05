%% FIN OPTIMIZATION 
% Created by Ares Bustinza-Nguyen (1/26/25)

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

opts = sim.getOptions;
opts.setWindSpeedDeviation(0);
opts.setTimeStep(0.05); % slower rate overnight, check against this

%% RANGE FOR VARIABLES ----------------------------------------------------

% Fin Thickness [in]
t = [0.003175, 0.0047625, 0.00635]; %0.125, 0.1875, 0.25 [in]

% Fin Sweep Length 
Ls_or = fins.getSweep();
Ls = Ls_or * (0.8:0.05:1.2); % 80% to 120%

% Tip Chord Length 
Lt_or = fins.getTipChord();
Lt = Lt_or * (0.8:0.05:1.2); % 80% to 120%

% Root Chord Length 
Lr_or = fins.getRootChord();
Lr = Lr_or * (0.8:0.05:1); % varying from 80% to 100% (Lr <= 12 in for stock size)

% Height 
h_or = fins.getHeight();
h = h_or * (0.8:0.05:1.2); % 80% to 120% 

% Variable Nosecone Weight
n_or = otis.component(name = "Adjustable stability weight").getComponentMass();
n = n_or * (0.8:0.05:1.2);

%% ITERATE (big 3) --------------------------------------------------------

[t_g, Ls_g, Lt_g, Lr_g, h_g, n_g] = ndgrid(t, Ls, Lt, Lr, h, n); % creating a grid of all possible combinations

num_elements = numel(t_g); % total # of combinations
disp(num_elements);

% preallocating for speed, dynamic to fit the total # of combinations
results = NaN(num_elements, 10); 
row_index = 1;

f = @FOS_finflutter; % fin flutter function


for i = 60617
    % values for this combination
    on_t = t_g(i); on_Ls = Ls_g(i); on_Lt = Lt_g(i); on_Lr = Lr_g(i); on_h = h_g(i); on_n = n_g(i);

    % call functions or simulations using the current values
    fins.setThickness(on_t); fins.setSweep(on_Ls); fins.setTipChord(on_Lt); fins.setRootChord(on_Lr); fins.setHeight(on_h); otis.component(name = "Adjustable stability weight").setComponentMass(on_n);

    sim = otis.sims("15MPH-TEXAS-36C-(TYP)"); % rerun sim
    ops = sim.getOptions;
    ops.setWindSpeedDeviation(0);
    
    openrocket.simulate(sim); 
    data = openrocket.get_data(sim);
    disp(i);

    % APOGEE --------------------------------------------------------------
    apogee = max(data.Altitude); 
    % has to be run before cut for launchrod and burnout
    if (apogee < 3030)
        continue
    end

    disp(apogee);

    % STABILITY -----------------------------------------------------------
    data_range = timerange(eventfilter("LAUNCHROD"), eventfilter("BURNOUT"), "openleft");
    data = data(data_range, :);

    stb_launchrod = data{1, "Stability margin"}; % launchrod
    stb_burnout = data{end, "Stability margin"}; % burnout
  
    if (stb_launchrod < 1.3)
        continue
    end

    disp(stb_launchrod); 
    disp(stb_burnout);

    % FOS -----------------------------------------------------------------
    % calling function FOS_finflutter

    FINAL_FOS = f(data, fins);
    disp(FINAL_FOS);
    
    % ---------------------------------------------------------------------
    % overarching check for acceptable geometry
    if (FINAL_FOS > 1.45) && (3030 < apogee) && (apogee < 3300) && (1.5 < stb_launchrod) && (stb_launchrod < 3) && (1.5 < stb_burnout) && (stb_burnout < 3.6)
        FOS_accept = FINAL_FOS; APG_accept = apogee; STB_accept_L = stb_launchrod; STB_accept_B = stb_burnout; % return acceptable values
        results(row_index, :)  = [fix(i), FOS_accept, APG_accept, STB_accept_L, STB_accept_B, on_t, on_h, on_Ls, on_Lt, on_Lr];
        row_index = row_index + 1; % append to array
    end
end

%% RESULTS ----------------------------------------------------------------

results = results(~any(isnan(results), 2), :); 
titles = {'# Iteration', 'FOS', 'Apogee', 'S-Launchrod', 'S-Burnout', 't', 'h', 'Ls', 'Lt', 'Lr'};

fprintf('%-15s %-15s %-15s %-15s %-15s %-15s %-15s %-15s %-15s %-15s\n', titles{:});
for i = 1:size(results, 1)
    fprintf('%-15d %-15.4f %-15.4f %-15.4f %-15.4f %-15.4f %-15.4f %-15.4f %-15.4f %-15.4f\n', results(i, :));
end

% write to file
results_file = array2table(results, 'VariableNames', titles);
writetable(results_file, 'finopt_results.csv');
