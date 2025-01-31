%% FIN OPTIMIZATION 
% Created by Ares Bustinza-Nguyen (Updated: 1/26/25)

clear; close all;

%% SETTING FLIGHT CONDITIONS, CONSTRAINTS ---------------------------------

otis_path = "data/OTIS.ork"; 
if ~isfile(otis_path)
    error("Error: not on path", otis_path);
end

otis = openrocket(otis_path);
sim = otis.sims("15MPH-SA-45DEG-36C");
 
fins = otis.component(class = "FinSet"); 
if ~isscalar(fins)
    error("Error: multiple fin sets found");
end

opts = sim.getOptions;
opts.setWindSpeedDeviation(0);
opts.setTimeStep(0.05); % slower rate overnight, check against this

%% RANGE FOR VARIABLES ----------------------------------------------------
in_m = 39.370079; %

% Fin Thickness [in]
t = [0.003175, 0.0047625, 0.00635]; %0.125, 0.1875, 0.25 [in]

% Fin Sweep Length [in]
Ls_or = fins.getSweep();
Ls = Ls_or * (0.8:0.05:1.2); % 80% to 120%

% Tip Chord Length [in]
Lt_or = fins.getTipChord();
Lt = Lt_or * (0.8:0.05:1.2); % 80% to 120%

% Root Chord Length [in]
Lr_or = fins.getRootChord();
Lr = Lr_or * (0.8:0.05:1); % varying from 80% to 100% (Lr <= 12 in for stock size)

% Height [in]
h_or = fins.getHeight();
h = h_or * (0.8:0.05:1.2); % 80% to 120% 

%% ITERATE (big 3) --------------------------------------------------------

[t_g, Ls_g, Lt_g, Lr_g, h_g] = ndgrid(t, Ls, Lt, Lr, h); % creating a grid of all possible combinations

num_elements = numel(t_g); % total # of combinations
disp(num_elements);

% preallocating for speed, dynamic to fit the total # of combinations
results = NaN(num_elements, 10); 
row_index = 1;

f = @FOS_finflutter; % fin flutter function

for i = 1:num_elements
    % values for this combination
    on_t = t_g(i); on_Ls = Ls_g(i); on_Lt = Lt_g(i); on_Lr = Lr_g(i); on_h = h_g(i);

    % call functions or simulations using the current values
    fins.setThickness(on_t); fins.setSweepAngle(on_Ls); fins.setTipChord(on_Lt); fins.setRootChord(on_Lr); fins.setHeight(on_h);
    
    sim = otis.sims("15MPH-SA-45DEG-36C"); % rerun sim
    ops = sim.getOptions;
    ops.setWindSpeedDeviation(0);
    
    openrocket.simulate(sim); 
    data = openrocket.get_data(sim);
    disp(i);

    % APOGEE --------------------------------------------------------------
    
    apogee = data{eventfilter("APOGEE"), "Altitude"};
    disp(apogee);

    % FOS -----------------------------------------------------------------
    
    % calling function FOS_finflutter
    FINAL_FOS = f(data, fins);
    disp(FINAL_FOS); % take out 

    % STABILITY -----------------------------------------------------------
   
    data_range = timerange(eventfilter("LAUNCHROD"), eventfilter("BURNOUT"), "openleft");
    data = data(data_range, :);

    stb_launchrod = data{1, "Stability margin"}; % launchrod
    stb_burnout = data{end, "Stability margin"}; % burnout
    disp(stb_launchrod);
    disp(stb_burnout);
    

    % overarching check for acceptable geometry
    % obviously this number is not correct, but i wanted to see if something would show up
    if (FINAL_FOS > 1.5) && (3019 < apogee) && (apogee < 3300) && (1 < stb_launchrod) && (stb_launchrod < 3) && (1 < stb_burnout) && (stb_burnout < 3.5)
        FOS_accept = FINAL_FOS;
        APG_accept = apogee;
        STB_accept_L = stb_launchrod;
        STB_accept_B = stb_burnout;
        results(row_index, :)  = [fix(i), FOS_accept, APG_accept, STB_accept_L, STB_accept_B, on_t, on_h, on_Ls, on_Lt, on_Lr];
        row_index = row_index + 1;
    end

end


%% RESULTS ----------------------------------------------------------------

results = results(~any(isnan(results), 2), :); % will have to change this for APG and STB
titles = {'# Iteration', 'FOS', 'Apogee', 'S-Launchrod', 'S-Burnout', 't', 'h', 'Ls', 'Lt', 'Lr'};

fprintf('%-15s %-15s %-15s %-15s %-15s %-15s %-15s %-15s %-15s %-15s\n', titles{:});

% Loop through each row of results and print 10 integer values per row
for i = 1:size(results, 1)
    fprintf('%-15d %-15.4f %-15.4f %-15.4f %-15.4f %-15.4f %-15.4f %-15.4f %-15.4f %-15.4f\n', results(i, :));
end


% ssm_reference =  NaN(num_elements, 1);
% % flight condition for reference
% ref_mach = 0.3; 
% ref_aoa = deg2rad(5);
% ref_fcond = otis.flight_condition(ref_mach, ref_aoa);
