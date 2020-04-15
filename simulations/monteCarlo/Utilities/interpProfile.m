%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function will obtain a thrust value for a given time. The user must
% load a motor profile from which the program will interpolate between
% time/thrust values.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [thrust] = interpProfile(t)
global motorProfile

if size(motorProfile) == [0, 0]
    error('Error: No motor thrust profile loaded. Please load a valid thrust profile into the global variable motorProfile.');  
end

if t >= motorProfile(end,1)
    thrust = 0;
else
    % Solve for the index of the Time value just greater than current Time
    i=2;
    Quit = 0;                       % Stop when Quit is not equal to 0
    while Quit == 0
        Check = motorProfile(i,1) - t;
        if Check >=0                 % If Time_Table(i) >= Time you are at the correct index
            Quit = 1;                % Quit value changed to stop while loop
        else
            i=i+1;                   % Increase i to check next value
        end
    end
    
    % Use solved index to interpolate Thrust Value
    thrust = (motorProfile(i,2) - motorProfile(i-1,2)) * (t - motorProfile(i-1,1))/(motorProfile(i,1) - motorProfile(i-1,1)) + motorProfile(i-1,2);
end

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%