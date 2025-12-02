%%

clc
clear all


%global nb minReflectance maxReflectance whiteThresh targetDistance

nb = nanobot('/dev/cu.usbmodem1101', 115200, 'serial');
nb2 = nanobot('/dev/cu.usbmodem4', 115200, 'serial');

main(nb, nb2);


%%

mOffScale = 1.112;
    
% Initialize the reflectance array.
nb.initReflectance();

% Initialize ultrasonic
nb.initUltrasonic1('D4', 'D5');
nb.initUltrasonic2('D2', 'D3');

minReflectance = [131.3, 101.5, 88.2, 83.6, 95.3, 94.1];
maxReflectance = [1426.1, 987.7, 817.6, 757.7, 793.2, 581];

whiteThresh = 200; % Max value detected for all white

targetDistance = 200;


%%


function LineFollowBungus(nb, mBaseSpeed, brake)

    mOffScale = 1.112;


    minReflectance = [131.3, 101.5, 88.2, 83.6, 95.3, 94.1];
    maxReflectance = [1426.1, 987.7, 817.6, 757.7, 793.2, 581];

switch mBaseSpeed
        
        case 9
            kp = 0.00255;
            ki = 0.00005;
            kd = 0.00089;

        case 10
            kp = 0.00250;
            ki = 0;
            kd = 0.00090;

        case 11
            kp = 0.015;
            ki = 0;
            kd = 0.00095;

        case 13
            kp = 0.018;
            ki = 0;
            kd = 0.00165;

        otherwise
            % default safe tuning
            kp = 0.01;
            ki = 0;
            kd = 0.001;
            disp("⚠ mBaseSpeed not recognized — using default PID values")
    end
    
    % Basic initialization
    vals = zeros(1,6);    
    prevError = 0;
    prevTime = 0;
    integral = 0;
    derivative = 0;
    
    whiteThresh = 200; % Max value detected for all white
    
    motorBaseSpeed = mBaseSpeed
    
    tic % start time
    
    m1Duty = 10 * mOffScale;
    m2Duty = 10;

    nb.setMotor(1, m1Duty);
    nb.setMotor(2, m2Duty);
    pause(0.03);
    
    while (AllBlack(vals, mBaseSpeed) == 0)  % Adjust me if you want to stop your line following 


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
            
             m1Duty = motorBaseSpeed + (control/2);
             m2Duty = motorBaseSpeed*mOffScale - (control/2);
           

    
            nb.setMotor(1, m1Duty);
            nb.setMotor(2, m2Duty);
        end
        
        
        prevError = error;
    end
    
    nb.setMotor(1, 0);
    nb.setMotor(2, 0);
        
    
    %Brakes if we going fast
    if(mBaseSpeed > 9 && brake)

        tic

        while(toc<.3)
            nb.setMotor(1, -9);
            nb.setMotor(2, -9);
        end
    end
   
    nb.setMotor(1, 0);
    nb.setMotor(2, 0);
        

end



function [frontDist] = GetToWallBungus(nb)

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
    distThresh = 450;
        
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
    
    frontDist = nb.ultrasonicRead2;

        
    
end




function [successWallBungus] = WallBungus(nb, mOffScale)
    disp("inside wal bungus\n");
    GetToWallBungus(nb);
    %distance = ApproachWallBungusBungus(nb, mOffScale);
    pause(0.5);
    disp("outside get to wall");

    %RightUltraSensor(nb, distance);
    distance = RightIterationCount(nb);
    disp("after right turn, pausing");
    pause(0.5);
    disp("entering wall follow");
    WallBungusFollow(nb, distance);

    disp("exited wall follow ");

    return
end

function main(nb, nb2)
       

    
    % Initialize the reflectance array.
    nb.initReflectance();
    
    % Initialize ultrasonic
    nb.initUltrasonic1('D4', 'D5');
    nb.initUltrasonic2('D2', 'D3');
    nb.initColor();
    
    
   
    myNeuralNetwork = gesture_init();

    

    buss = gesture(nb2, myNeuralNetwork);

    switch(buss)
    
        case 0
            
            LineFirst(nb);
            DoColor(nb);
           
        case 1
            WallFirst(nb);
            DoColor(nb);
        otherwise
            WallFirst(nb);
            DoColor(nb);
    end

end


function WallBungusFollow(nb, targDist)
    baseSpeed = 8;
    disp("inside wall follow/n");

    kp = 0.047; % Proportional gain
    ki = 0.00004; % Integral gains
    kd = 0.0026; 

    prevError = 0;
    prevTime = 0;
    integral = 0;
    derivative = 0;
    targetDistance = targDist;
    vals = zeros(1, 6);
    mOffScale = 1.112;


    while (~AnyBlack(vals))  % Adjust me if you want to stop your line following 
                     % earlier or let it run longer.
        
        disp("Entered wallfolow loop")
          % take a reading                
        vals = nb.reflectanceRead();
    
        % change from a struct to a list for convenience
        vals = [vals.one, vals.two, vals.three, vals.four, vals.five, vals.six];
    
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

        if(m1Speed > 200)
            m1Speed = 200;
        end
    
    
        nb.setMotor(1, m1Speed);
        nb.setMotor(2, m2Speed);
    
        prevError = error;
    end

    nb.setMotor(1, 0);
    nb.setMotor(2, 0);

    return 
end

% Returns true if vals array detects all black
function [allBlack] = AllBlack(vals, mBaseSpeed)
    whiteThresh = 500;

    if (mBaseSpeed == 9)
        if (vals(1) > whiteThresh || vals(6) > whiteThresh)
            allBlack = 1;
            disp("detected edge black");
        else
            allBlack = 0;
        end

    
    else
        if (vals(1) > whiteThresh && ...
                    vals(2) > whiteThresh && ...
                    vals(3) > whiteThresh && ...
                    vals(4) > whiteThresh && ...
                    vals(5) > whiteThresh && ...
                    vals(6) > whiteThresh)
            allBlack = 1;
            disp("detected all black");

        else
            allBlack = 0;
        end

    
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
        anyBlack = 1;

    else
        anyBlack = 0;
    end

end


% Helper

function Right90(nb)
    % Turn 90 deg right
       tic
       nb.setMotor(1, 10);
       nb.setMotor(2, -10);
       while(toc < 1.255)
        nb.setMotor(1, -9);
        nb.setMotor(2, 9);
       end
       nb.setMotor(1, 0);
        nb.setMotor(2, 0);
end

function Left90(nb)
    % Turn 90 deg right
       tic
       nb.setMotor(1, 10);
       nb.setMotor(2, -10);
       while(toc < 1.255)
        nb.setMotor(1, 9);
        nb.setMotor(2, -9);
       end
       nb.setMotor(1, 0);
        nb.setMotor(2, 0);
end


function Right45(nb)
    % Turn 90 deg right
       tic
       nb.setMotor(1, 10);
       nb.setMotor(2, -10);
       while(toc < 0.6)
        nb.setMotor(1, -9);
        nb.setMotor(2, 9);
       end
       nb.setMotor(1, 0);
        nb.setMotor(2, 0);
end

function Left45(nb)
    % Turn 90 deg right
       tic
       nb.setMotor(1, -10);
       nb.setMotor(2, 10);
       while(toc < 0.6)
        nb.setMotor(1, 9);
        nb.setMotor(2, -9);
       end
       nb.setMotor(1, 0);
        nb.setMotor(2, 0);
end


function [readDistance] = RightIterationCount(nb)

    countSinceLast = 0;
    %nb.setMotor(1, -10);
    %nb.setMotor(2,10);

    while(1)
        readVal = nb.ultrasonicRead1();
        nb.setMotor(1, -9);
        nb.setMotor(2, 9);
        if(readVal > 820)
            countSinceLast = 0;
        
        else
            countSinceLast = countSinceLast + 1;
        end
    

        if(countSinceLast > 18)
           nb.setMotor(1, 0);
           nb.setMotor(2, 0);
           break;
        end
            
    end

    readDistance = nb.ultrasonicRead1();

end


function RightInfrared(nb)
    disp("⚪ Starting Right Infrared Turn...");

    blackThresh = 450;   % threshold for detecting black
    whiteThresh = 250;   % threshold for detecting white
    mOffScale = 1.112;

    % 1️⃣ TURN UNTIL RIGHT SENSOR SEES BLACK
    disp("-- Phase 1: Seeking right sensor black line");
    while true
        vals = nb.reflectanceRead();
        v = [vals.one, vals.two, vals.three, vals.four, vals.five, vals.six];

        % Turn right slowly
        nb.setMotor(1, -9);
        nb.setMotor(2, 9);

        if v(6) > blackThresh
            break;  % right sensor sees black
        end
    end

    % 2️⃣ CONTINUE TURN UNTIL RIGHT SENSOR SEES WHITE
    while true
        vals = nb.reflectanceRead();
        v = [vals.one, vals.two, vals.three, vals.four, vals.five, vals.six];

        nb.setMotor(1, -9);
        nb.setMotor(2, 9);

        if v(6) < whiteThresh
            break;  % right sensor sees white
        end
    end

    % 3️⃣ STOP MOTORS
    nb.setMotor(1, 0);
    nb.setMotor(2, 0);

    disp("✔ RightInfrared complete — right sensor black → white detected!");
end


function LeftInfrared(nb)
    disp("⚪ Starting Right Infrared Turn...");

    blackThresh = 450;   % threshold for detecting black
    whiteThresh = 250;   % threshold for detecting white
    mOffScale = 1.112;

    % 1️⃣ TURN UNTIL RIGHT SENSOR SEES BLACK
    disp("-- Phase 1: Seeking right sensor black line");
    while true
        vals = nb.reflectanceRead();
        v = [vals.one, vals.two, vals.three, vals.four, vals.five, vals.six];

        % Turn right slowly
        nb.setMotor(1, 9);
        nb.setMotor(2, -9);

        if v(6) > blackThresh
            break;  % right sensor sees black
        end
    end

    % 2️⃣ CONTINUE TURN UNTIL RIGHT SENSOR SEES WHITE
    while true
        vals = nb.reflectanceRead();
        v = [vals.one, vals.two, vals.three, vals.four, vals.five, vals.six];

        nb.setMotor(1, -9);
        nb.setMotor(2, 9);

        if v(6) < whiteThresh
            break;  % right sensor sees white
        end
    end

    % 3️⃣ STOP MOTORS
    nb.setMotor(1, 0);
    nb.setMotor(2, 0);

    disp("✔ RightInfrared complete — right sensor black → white detected!");
end



% function RightInfrared(nb)
% 
%     disp("⚪ Starting Right Infrared Turn...");
% 
%     blackThresh = 450;   % tune as needed
%     whiteThresh = 250;   % tune as needed
% 
%     % 1️⃣ TURN UNTIL MIDDLE IS BLACK
%     disp("-- Phase 1: Seeking center black line");
%     while true
%         vals = nb.reflectanceRead();
%         v = [vals.one, vals.two, vals.three, vals.four, vals.five, vals.six];
% 
%         % Turn right
%         nb.setMotor(1, -9);
%         nb.setMotor(2, 9);
% 
%         % Check if center is black
%         if (v(3) > blackThresh)
%             break;
%         end
%     end
% 
%     pause(0.15);   % stabilizer pause
% 
%     % 2️⃣ KEEP TURNING UNTIL SIDE SENSORS SEE WHITE
%     disp("-- Phase 2: Aligning with white boundary");
%     while true
%         vals = nb.reflectanceRead();
%         v = [vals.one, vals.two, vals.three, vals.four, vals.five, vals.six];
% 
%         nb.setMotor(1, -9);
%         nb.setMotor(2, 9);
% 
%         % If either extreme sensor sees white → alignment achieved
%         if (v(1) < whiteThresh || v(6) < whiteThresh || v(2) < whiteThresh || v(5) < whiteThresh)
%             break;
%         end
%     end
% 
%     % 3️⃣ STOP + CENTER
%     nb.setMotor(1, 0);
%     nb.setMotor(2, 0);
% 
%     disp("✔ RightInfrared complete — robot aligned!");
% 
% end

function CreepSearchLine(nb)
    % Creep forward until line is detected under middle sensors
    % and white under outer sensors
    %
    % nb       - nanobot object
    % Duty     - base motor speed
    % Sensors  - 1: leftmost, 2: left-mid, 3: mid-left, 4: mid-right, 5: right-mid, 6: rightmost

    whiteThresh = 250;  % threshold for detecting white
    blackThresh = 450;  % threshold for detecting black (adjust if needed)


    % Initialize vals to prevent index errors
    vals = zeros(1,6);

    % Loop until condition is met
    while true
        % Read sensors
        r = nb.reflectanceRead();
        vals = [r.one, r.two, r.three, r.four, r.five, r.six];

        % Check condition: outer sensors white, middle sensors black
        if vals(1) < whiteThresh && vals(6) < whiteThresh
            break;  % condition met, stop creeping
        end

        % Move forward at duty 8
        nb.setMotor(1, 10);
        nb.setMotor(2, 8);

        pause(0.02);  % small pause for sensor stability
    end

    % Stop motors once condition is met
    nb.setMotor(1, 0);
    nb.setMotor(2, 0);
end


function LineFirst (nb)
    LineFollowBungus(nb, 9, 1);
    
    Left45(nb);
    LeftInfrared(nb);
    % CreepSearchLine(nb);
    LineFollowBungus(nb, 13, 1);


    
    % follow then 180
    LineFollowBungus(nb, 13, 1);
    pause(1);
    RightInfrared(nb);
    
    % get to middle
    LineFollowBungus(nb, 13, 0);
    LineFollowBungus(nb, 13, 1);
    
    % get to wall
    GetToWallBungus(nb);
    pause(1);
    
    RightIterationCount(nb);
    pause(1);
    disp("after right turn");
    dist = nb.ultrasonicRead1();
    WallBungusFollow(nb, dist);
    pause(1);
    
    RightInfrared(nb);
    pause(1);
    LineFollowBungus(nb, 9, 1);
    
    pause(1);
    
    CreepSearchLine(nb);
    LineFollowBungus(nb, 13, 1);
    Right45(nb);
    RightInfrared(nb);
    LineFollowBungus(nb,11,0);

    Right45(nb);
    RightInfrared(nb);
    LineFollowBungus(nb,9,1);



end



function WallFirst (nb)
    LineFollowBungus(nb, 9, 1);
    
    Right45(nb);
    RightInfrared(nb);
    % CreepSearchLine(nb);
    LineFollowBungus(nb, 13, 1);

    % get to wall
    GetToWallBungus(nb);
    pause(1);
    
    RightIterationCount(nb);
    pause(1);
    disp("after right turn");
    dist = nb.ultrasonicRead1();
    WallBungusFollow(nb, dist);
    pause(1);
    
    RightInfrared(nb);
    pause(1);
    LineFollowBungus(nb, 9, 1);
    
    pause(1);
    
    CreepSearchLine(nb);
    LineFollowBungus(nb, 13, 0);


    
    % follow then 180
    LineFollowBungus(nb, 13, 1);
    pause(1);
    RightInfrared(nb);
    
    % get to middle
    LineFollowBungus(nb, 13, 1);
    
    
    Left45(nb);
    LeftInfrared(nb);
    % CreepSearchLine(nb);
    LineFollowBungus(nb, 9, 1);

    %DETECT COLOr

end


function [isBlue] = DetectColor(nb)

    %Take a single RGB color sensor reading
    values = nb.colorRead();
    
    %The sensor values are saved as fields in a structure:
    red = values.red;
    green = values.green;
    blue = values.blue;
    
    if(red > blue)
        isBlue = 0;
    else
        isBlue = 1;
        
    end
end


function myNeuralNetwork = gesture_init() 

    clear; clc; close all; %initialization

    filename = "2025122_125414_TrainingSet_2Digits12Trials.mat";
    data = importdata(filename);
    
    digitCount = height(data); %number of digits is the number of rows (height)
    trialCount = width(data)-1; %number of trials is the number of columns (width)
    TrainingFeatures = zeros(3,150,1,digitCount*trialCount); 
    labels = zeros(1,digitCount*trialCount); 
    
    k=1; %simple counter
    for a = 1:digitCount %iterate through digits
        for b = 1:trialCount %iterate through trials
            % For a CNN, the input is no longer features we define, but the 
            % data itself.  The CNN determines the features.
            TrainingFeatures(:,:,:,k) = data{a,b+1}; %put data into image stack
            labels(k) = data{a,1}; %put each label into label stack
            k = k + 1; %increment
        end
    end
    labels = categorical(labels); %convert labels into categorical
    
    selection = ones(1,digitCount*trialCount); %allocate logical array
                                                 %initialize all to 1 at first
    selectionIndices = []; %initialization
    for b = 1:digitCount %pick 1/4 of the data for testing
        selectionIndices = [selectionIndices,  ...
            round(linspace(1,trialCount,round(trialCount/4))) + ...
            (trialCount*(b-1))];
    end
    selection(selectionIndices) = 0; %set logical to zero to indicate testing 
                                     %data
    
    xTrain = TrainingFeatures(:,:,:,logical(selection)); %get subset (3/4) of features to train on
    yTrain = labels(logical(selection)); %get subset (3/4) of labels to train on

    xTest = TrainingFeatures(:,:,:,~logical(selection)); % get subset (1/4) of features to test on
    yTest = labels(~logical(selection)); %get subset (1/4) of labels to test on
    
    
    [inputsize1,inputsize2,~] = size(TrainingFeatures); %input size is defined by features
    numClasses = length(unique(labels)); %output size (classes) is defined by number of unique labels

    learnRate = 0.01; % how quickly network makes changes and learns
    maxEpoch = 20; % how long the network learns (how many times all the data 
                   % is passed through the CNN)
    
    layers = [
    imageInputLayer([inputsize1,inputsize2,1])
    convolution2dLayer([2,10],5)  % Reduced filters from 20 to 5
    reluLayer
    fullyConnectedLayer(numClasses)
    softmaxLayer
    classificationLayer
];
    
    
options = trainingOptions('sgdm','InitialLearnRate', learnRate, ...
    'MaxEpochs', maxEpoch,'Shuffle','every-epoch', ...
    'Plots','none', 'ValidationData',{xTest,yTest});
 
    [myNeuralNetwork, info] = trainNetwork(xTrain,yTrain,layers,options);

end
 
function dir = gesture(nb, myNeuralNetwork)
   
    
    nb.ledWrite(0); % turn off the LED
    
    numreads = 150; % about 2 seconds (on serial); adjust as needed, but we 
                    % will be using a value of 150 for Labs 4 and 5
    pause(.5);
    
    clc; % clear the command line
    countdown("Beginning in", 3);
    disp("Make A Gesture!");
    nb.ledWrite(1);  % Turn on the LED to signify the start of recording data
    
    % Gesture is performed during the segement below
    for i = 1:numreads
        val = nb.accelRead();
        vals(1,i) = val.x;
        vals(2,i) = val.y;
        vals(3,i) = val.z;
    end
    
    nb.ledWrite(0); % Turn the LED off to signify end of recording data
    
    rtdata = [vals(1,:);vals(2,:);vals(3,:)];
    
    % put accelerometer data into NN input form
    xTestLive = zeros(3,150,1,1);
    xTestLive(:,:,1,1) = rtdata;
    
    % Prediction based on NN
    dir = double(classify(myNeuralNetwork,xTestLive));
end




function [detection] = BlueDetected(nb)
    colVals = nb.colorRead();

    threshold = 100;

    if(colVals.blue > threshold)
        detection = 1;
    
    else
        detection = 0;
    end

end

function [detection] = RedDetected(nb)
    colVals = nb.colorRead();

    threshold = 120;

    if(colVals.red > threshold)
        detection = 1;
    
    else
        detection = 0;
    end

end
function CreepForward(nb)

    mOffScale = 1.112;

    tic
    while(toc < .5)
        nb.setMotor(1, 9);
        nb.setMotor(2, 9*mOffScale);
    
    end
end


nb.setMotor(1, 0);
    nb.setMotor(2, 0);


% Call functions

% 1 is WallBungusfirst
% 2 is linefirst


%%
dist = nb.ultrasonicRead1();

WallBungusFollow(nb, dist);

%%
main(nb);
%%
LineFollowBungus(nb, 13, 0);
LineFollowBungus(nb, 13, 1);

GetToWallBungus(nb);
pause(1);

RightIterationCount(nb);
pause(1);
disp("after right turn");
dist = nb.ultrasonicRead1();
WallBungusFollow(nb, dist);
pause(1);

RightInfrared(nb);
pause(1);
LineFollowBungus(nb, 9, 1);

pause(1);

CreepSearchLine(nb);
LineFollowBungus(nb, 13, 0);
LineFollowBungus(nb, 13, 1);

%%
GetToWallBungus(nb);
disp("after get to wall");
dist = RightIterationCount(nb);
WallBungusFollow(nb, dist);


%%

while(1)
    nb.colorRead();
end
%%
LineFollowBungus(nb, 9, 1);

Left45(nb);
LeftInfrared(nb);
% CreepSearchLine(nb);
LineFollowBungus(nb, 13, 1);


%%
Right30(nb);
    RightInfrared(nb);
    LineFollowBungus(nb,9,1);
%%

while(1)
    nb.colorRead()
    
end


%%
clc
clear all
nb = nanobot('/dev/cu.usbmodem1101', 115200, 'serial');

nb.setMotor(1, 0);
nb.setMotor(2, 0);

mOffScale = 1.112;

nb.initUltrasonic2('D2', 'D3');


%%
% while(1)
%     nb.ultrasonicRead1()
% end
% WallBungus(nb)
% Right45(nb)
% LineFollowBungus(nb)

RightInfrared(nb);
CreepSearchLine(nb);
LineFollowBungus(nb, 9);


%%


nb.setMotor(1, 0);
nb.setMotor(2, 0);

%%
main();

%%


%%

function DoColor(nb)
    isBlue = DetectColor(nb);
        
    mOffScale = 1.112;
    disp('somethign');
    
    if(isBlue)
        CreepForward(nb);
        Right45(nb);
        while(~BlueDetected(nb))
            nb.setMotor(1, 9);
            nb.setMotor(2, 9*mOffScale)
        end
        nb.setMotor(1,0);
        nb.setMotor(2,0);
    
    else
        CreepForward(nb);
    
        Left45(nb);
        while(~RedDetected(nb))
            nb.setMotor(1, 9);
            nb.setMotor(2, 9*mOffScale)
        end
        nb.setMotor(1,0);
        nb.setMotor(2,0);
    end

end

%%



