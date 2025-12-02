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
    dir = classify(myNeuralNetwork,xTestLive);
end


%%

nb = nanobot('/dev/cu.usbmodem101', 115200, 'serial');

myNeuralNetwork = gesture_init();

gesture(nb, myNeuralNetwork)

