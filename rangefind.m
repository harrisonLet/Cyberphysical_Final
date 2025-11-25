clc
clear all
nb = nanobot('/dev/cu.usbmodem1101', 115200, 'serial');

% ULTRASONIC DISTANCE

%Initialize the ultrasonic sensor with TRIGPIN, ECHOPIN
% nb.initUltrasonic1('D2','D3')

nb.initUltrasonic1('D4', 'D5')
nb.initUltrasonic2('D2', 'D3')

targetDistance = nb.ultrasonicRead1; % target distance 
baseSpeed = 8;
mOffScale = 1.112;

prevError = 0;
prevTime = 0;
integral = 0;
derivative = 0;

%good values for 9 speed
%kp = 0.038; % Proportional gain
%ki = 0; % Integral gains
%kd = 0.0036; % Derivative gai

% good values for 8 speed
%kp = 0.047; % Proportional gain
%ki = 0.00003; % Integral gains
%kd = 0.0026; % Derivative gain


kp = 0.047; % Proportional gain
ki = 0.00003; % Integral gains
kd = 0.0026; % Derivative gain

tic
while (toc < 15)  % Adjust me if you want to stop your line following 
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

    if(abs(m1Speed) > 100)
        nb.setMotor(1, 100);
    end

    nb.setMotor(1, m1Speed);
    nb.setMotor(2, m2Speed);

    prevError = error;
end


nb.setMotor(1, 0)
nb.setMotor(2, 0)


%% 5. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.


nb.setMotor(1, 0);
nb.setMotor(2, 0);clc
delete(nb);
clear('nb');
clear all