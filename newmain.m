%%

clc
clear all


%global nb minReflectance maxReflectance whiteThresh targetDistance

nb = nanobot('/dev/cu.usbmodem11101', 115200, 'serial');


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


function LineFollowBungus(nb)

    motorBaseSpeed = 13;
    mOffScale = 1.112;
    whiteThresh = 200; % Max value detected for all white


    minReflectance = [131.3, 101.5, 88.2, 83.6, 95.3, 94.1];
    maxReflectance = [1426.1, 987.7, 817.6, 757.7, 793.2, 581];


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
    m2Duty = 10;

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
        


end



function GetToWallBungus(nb)

    motorBaseSpeed = 9;
    mOffScale = 1.112;
    whiteThresh = 200; % Max value detected for all white


    minReflectance = [131.3, 101.5, 88.2, 83.6, 95.3, 94.1];
    maxReflectance = [1426.1, 987.7, 817.6, 757.7, 793.2, 581];


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
    %kp = 0.018;
    %ki = 0;
    %kd = 0.00165;

    kp = .00255;
    ki = 0.00005;
    kd = 0.00089;
    
    % Basic initialization
    vals = 0;
    prevError = 0;
    prevTime = 0;
    integral = 0;
    derivative = 0;
    
    whiteThresh = 200; % Max value detected for all white
    distThresh = 700;
        
    tic % start time
    
    m1Duty = 10 * mOffScale;
    m2Duty = 10;

    nb.setMotor(1, m1Duty);
    nb.setMotor(2, m2Duty);
    pause(0.03);
    
    while (nb.ultrasonicRead2 > distThresh)  % Adjust me if you want to stop your line following 
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
        
    
end




function [successWallBungus] = WallBungus(nb, mOffScale)
    disp("inside wal bungus\n");
    GetToWallBungus(nb);
    %distance = ApproachWallBungusBungus(nb, mOffScale);
    pause(0.5);
    fprintf("outside get to wall\n");

    Right90(nb);
    fprintf("after right turn, sleeping\n");
    pause(0.5);
    WallBungusFollow(nb);
    pause(0.5);
    Right90(nb);
    LineFollowBungus(nb);


    return
end


function WallBungusFollow(nb)
    baseSpeed = 8;
    fprintf("inside wall follow/n");

    kp = 0.047; % Proportional gain
    ki = 0.00003; % Integral gains
    kd = 0.0026; 

    prevError = 0;
    prevTime = 0;
    integral = 0;
    derivative = 0;
    targetDistance = 500;
    vals = zeros(1, 6);
    mOffScale = 1.112;


    while (~AnyBlack(vals))  % Adjust me if you want to stop your line following 
                     % earlier or let it run longer.
        
        disp("Entered wallfolow loop")
          % take a reading                
        vals = nb.reflectanceRead()
    
        % change from a struct to a list for convenience
        vals = [vals.one, vals.two, vals.three, vals.four, vals.five, vals.six];
    
        % TIME STEP
        dt = toc - prevTime; % find the amount of time that has elapsed
    
        prevTime = toc; % set the current time to the previous time in the 
                        % next time step
    
        
    
        % Calculate the three errors to be used in the PID control 
        
        side = nb.ultrasonicRead1()
    
        error = side - targetDistance
    
    
        integral = integral + prevError
    
        derivative = (error- prevError)/dt
    
        % Create your PID controller output here using the previously defined 
        % gain values and the three errors computed above. 
        control = kp*error + ki*integral + kd*derivative;
    
    
        m1Speed = baseSpeed + control
        m2Speed = baseSpeed * mOffScale
    
    
        nb.setMotor(1, m1Speed);
        nb.setMotor(2, m2Speed);
    
        prevError = error;
    end

    nb.setMotor(1, 0);
    nb.setMotor(2, 0);

    return 
end

% Returns true if vals array detects all black
function [allBlack] = AllBlack(vals)
    whiteThresh = 200;

    if (vals(1) > whiteThresh && ...
                vals(2) > whiteThresh && ...
                vals(3) > whiteThresh && ...
                vals(4) > whiteThresh && ...
                vals(5) > whiteThresh && ...
                vals(6) > whiteThresh)
        allBlack = 1;

    else
        allBlack = 0;
    end

end

% Returns true if vals array detects any black
function [anyBlack] = AnyBlack(vals)
    whiteThresh = 300;

    if (vals(2) > whiteThresh || ...
                vals(3) > whiteThresh || ...
                vals(4) > whiteThresh || ...
                vals(5) > whiteThresh || ...
                vals(6) > whiteThresh)
        anyBlack = 1

    else
        anyBlack = 0
    end

end

% distance returns front WallBungus distance
function [distance] = ApproachWallBungusBungus(nb, mOffScale)
   targetDistance = 700;

   motorBaseSpeed = 9;
   fprintf("inside approach wall bungus, off scale %f", mOffScale);
   while (nb.ultrasonicRead2 > targetDistance)
       fprintf("current distance %d\n", nb.ultrasonicRead2)
       nb.setMotor(1, motorBaseSpeed);
       nb.setMotor(2, motorBaseSpeed * mOffScale);
   end
   nb.setMotor(1, 0);
   nb.setMotor(2, 0);


   distance = nb.ultrasonicRead2;
   return
end



% Helper

function Right90(nb)
    % Turn 90 deg right
       tic
       nb.setMotor(1, 10);
       nb.setMotor(2, -10);
       while(toc < 1.3)
        nb.setMotor(1, -9);
        nb.setMotor(2, 9);
       end
       nb.setMotor(1, 0);
        nb.setMotor(2, 0);
end


function Right45(nb)
    % Turn 90 deg right
       tic
       nb.setMotor(1, 10);
       nb.setMotor(2, -10);
       while(toc < 0.625)
        nb.setMotor(1, -9);
        nb.setMotor(2, 9);
       end
       nb.setMotor(1, 0);
        nb.setMotor(2, 0);
end
% Call functions

% 1 is WallBungusfirst
% 2 is linefirst

%%
nb.setMotor(1, 0);
nb.setMotor(2, 0);

gesture = 1;
switch(gesture)

    case 1
    LineFollowBungus(nb);
    pause(3);
    WallBungus(nb);
    pause(3);

    LineFollowBungus(nb);
end

%%
WallBungus(nb, mOffScale);

%%

LineFollowBungus(nb)
%%
disp(nb.ultrasonicRead2());
Right90(nb);
%%
clc
clear all
nb = nanobot('/dev/cu.usbmodem1101', 115200, 'serial');

nb.setMotor(1, 0);
nb.setMotor(2, 0);

mOffScale = 1.112;

nb.initUltrasonic2('D2', 'D3')

while(1)
    nb.ultrasonicRead2()
end
% WallBungus(nb)
% Right45(nb)
% LineFollowBungus(nb)
