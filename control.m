%%

clc
clear all
nb = nanobot('/dev/cu.usbmodem1101', 115200, 'serial');


mOffScale = 1.112;
    
% Initialize the reflectance array.
nb.initReflectance();

% Initialize ultrasonic
nb.initUltrasonic1('D4', 'D5')
nb.initUltrasonic2('D2', 'D3')

minReflectance = [131.3, 101.5, 88.2, 83.6, 95.3, 94.1];
maxReflectance = [1426.1, 987.7, 817.6, 757.7, 793.2, 581];

whiteThresh = 200; % Max value detected for all white

targetDistance = 200;
%%
% Call functions

% 1 is WallBungusfirst
% 2 is linefirst
gesture = 1;
switch(gesture)

    case 1
    LineFollowBungus()
    WallBungus()
    LineFollowBungus()
end

%%
function [successLine] = LineFollowBungus()

    motorBaseSpeed = 13;
    mOffScale = 1.112;
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
    
    m1Duty = 10 * mOffScale;
    m1Duty = 10;

    nb.setMotor(1, m1Duty);
    nb.setMotor(2, m2Duty);
    pause(0.03);
    
    while (~AllBlack(vals))  % Adjust me if you want to stop your line following 
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
            successLine = False;
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

    if(successLine == False) 
        return
    else
        successLine = True;
        return
    end
end

%%

function [successWallBungus] = WallBungus()
    
    distance = ApproachWallBungusBungusBungus();
    Right90();
    successWallBungus = WallBungusFollow(distance);


    return
end

%%

function [successWallBungus] = WallBungusFollow(currentDistance)
    baseSpeed = 8;

    kp = 0.047; % Proportional gain
    ki = 0.00003; % Integral gains
    kd = 0.0026; 

    prevError = 0;
    prevTime = 0;
    integral = 0;
    derivative = 0;


    while (~AllBlack(vals))  % Adjust me if you want to stop your line following 
                     % earlier or let it run longer.
    
        % TIME STEP
        dt = toc - prevTime; % find the amount of time that has elapsed
    
        prevTime = toc; % set the current time to the previous time in the 
                        % next time step
    
        
    
        % Calculate the three errors to be used in the PID control 
        
        side = nb.ultrasonicRead1();
    
        error = side - targetDistance;
    
    
        integral = integral + prevError;
    
        derivative = (error- prevError)/dt;
    
        % Create your PID controller output here using the previously defined 
        % gain values and the three errors computed above. 
        control = kp*error + ki*integral + kd*derivative;
    
    
        m1Speed = baseSpeed + control;
        m2Speed = baseSpeed * mOffScale;
    
        if(abs(m1Speed) > 300)
            nb.setMotor(1, 100);
        end
    
        nb.setMotor(1, m1Speed);
        nb.setMotor(2, m2Speed);
    
        prevError = error;
    end

    successWallBungus = True;
    return 
end

% Returns true if vals array detects all black
function [allBlack] = AllBlack(vals)

    if (vals(1) > whiteThresh && ...
                vals(2) > whiteThresh && ...
                vals(3) > whiteThresh && ...
                vals(4) > whiteThresh && ...
                vals(5) > whiteThresh && ...
                vals(6) > whiteThresh)
        allBlack = True;

    else
        allBlack = False;
    end

end

% distance returns front WallBungus distance
function [distance] = ApproachWallBungusBungus()

   motorBaseSpeed = 9;
   while (nb.ultrasonicRead2 > targetDistance)
       nb.setMotor(1, motorBaseSpeed);
       nb.setMotor(2, motorBaseSpeed * mOffScale);
   end
   nb.setMotor(1, 0);
   nb.setMotor(2, 0);

   sleep(0.3);

   distance = nb.ultrasonicRead2;
   return
end


%% 
% Helper

function Right90()
    % Turn 90 deg right
       tic
       while(toc < 2)
        nb.setMotor(1, -7);
        nb.setMotor(2, 7);
       end
end

Right90();





    