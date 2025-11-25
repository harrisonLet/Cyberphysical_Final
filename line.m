clc
clear all
% for PC:% for Mac:
nb = nanobot('/dev/cu.usbmodem1101', 115200, 'serial');

mOffScale = 1.112;

% Initialize the reflectance array.
nb.initReflectance();

minReflectance = [131.3, 101.5, 88.2, 83.6, 95.3, 94.1];
% vals from advanced board minReflectance = [139.6, 103.7, 93.5,93.5,103.7,93.5];
maxReflectance = [1426.1, 987.7, 817.6, 757.7, 793.2, 581];
% harissons reflectance max maxReflectance = [2082.7,1631.7,1374.8,1380.7,1598,1235.8]; 


% 9 speed
% kp = .00255;
% ki = 0.00005;
% kd = 0.00089;

% 10 speed
% kp = 0.0025;
% ki = 0;
% kd = 0.0009;

%11 speed
% kp = 0.015;
% ki = 0;
% kd = 0.00095;

%13 speed
kp = 0.018;
ki = 0;
kd = 0.00165;

% Basic initialization
vals = 0;
prevError = 0;
prevTime = 0;
integral = 0;
derivative = 0;

whiteThresh = 200; % Max value detected for all white

motorBaseSpeed = 13;

tic % start time

nb.setMotor(1, 10 * mOffScale);
nb.setMotor(2, 10);
pause(0.03);

while (toc < 60)  % Adjust me if you want to stop your line following 
                 % earlier or let it run longer.

    % TIME STEP
    dt = toc - prevTime; % find the amount of time that has elapsed

    prevTime = toc; % set the current time to the previous time in the 
                    % next time step

    % take a reading                
    vals = nb.reflectanceRead();

    % change from a struct to a list for convenience
    vals = [vals.one, vals.two, vals.three, vals.four, vals.five, vals.six];
    
    % Calibrate sensor readings
    calibratedVals = zeros(6); % initialize to zero
    for i = 1:6
        calibratedVals(i) = (vals(i) - minReflectance(i))/(maxReflectance(i) - minReflectance(i));

        if vals(i) < minReflectance(i) 
            calibratedVals(i) = 0;
        end
        if vals(i) > maxReflectance(i) 
            calibratedVals(i) = maxReflectance(i);
        end
    end

    % Calculate the three errors to be used in the PID control 
    
    error = 3*vals(1) + 2*vals(2) + 1*vals(3) - 1*vals(4) - 2*vals(5) - 3*vals(6);

    integral = integral + prevError;

    derivative = (error- prevError)/dt;

    control = kp*error + ki*integral + kd*derivative;

    % STATE CHECKING 
    if (vals(1) < whiteThresh && ...
            vals(2) < whiteThresh && ...
            vals(3) < whiteThresh && ...
            vals(4) < whiteThresh && ...
            vals(5) < whiteThresh && ...
            vals(6) < whiteThresh)

        % ALL SENSORS READ WHITE (lost tracking):
        nb.setMotor(1, 0); % stop the motors
        nb.setMotor(2, 0);
        break; % exit the while loop

    else

        % LINE DETECTED:
        
        m1Duty = motorBaseSpeed + (control/2); %m1
        m2Duty = motorBaseSpeed*mOffScale - (control/2); %m2

        
        % if (heading > 1)
        %     m1Duty = right; 
        %     m2Duty = left; 
        % else
        %     m1Duty = motorBaseSpeed - control; 
        %     m2Duty = motorBaseSpeed*mOffScale; 
        % end

        nb.setMotor(1, m1Duty);
        nb.setMotor(2, m2Duty);
    end

    prevError = error;
end
nb.setMotor(1, 0);
nb.setMotor(2, 0);

%% EMERGENCY MOTOR SHUT OFF

% Clear motors
nb.setMotor(1, 0);
nb.setMotor(2, 0);

%% X. DISCONNECT

clc
delete(nb);
clear('nb');
clear all