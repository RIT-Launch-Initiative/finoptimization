function selected_thickness = user_thickness
    selected_thickness = 0;
    
    screen = get(0, 'ScreenSize');
    fig = uifigure('Position', [(screen(3)-200)/2, (screen(4)-125)/2, 200, 140], 'CloseRequestFcn', @(src, event) delete(fig));
    uilabel(fig, 'Text', 'Select Fin Thickness:', 'Position', [25, 110, 150, 25]);
    dropdown = uidropdown(fig, 'Items', {'1/8', '3/16', '1/4'}, 'Position', [25, 80, 150, 30]);
    uibutton(fig, 'Text', 'Select', 'Position', [25, 40, 150, 30], 'ButtonPushedFcn', @(src, event) set_selected(dropdown, fig));
    uiwait(fig);

    function set_selected(dropdown, fig)
        switch dropdown.Value
            case '1/8', selected_thickness = 0.003175;
            case '3/16', selected_thickness = 0.0047625;
            case '1/4', selected_thickness = 0.00635;
        end
        delete(fig);
    end
end
