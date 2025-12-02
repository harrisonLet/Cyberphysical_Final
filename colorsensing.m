clc
clear all

% Create an instance of the nanobot class
nb = nanobot('/dev/cu.usbmodem1101', 115200, 'serial');



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



function Right30(nb)
    % Turn 90 deg right
       tic
       nb.setMotor(1, 10);
       nb.setMotor(2, -10);
       while(toc < 0.4)
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
       while(toc < 0.4)
        nb.setMotor(1, 9);
        nb.setMotor(2, -9);
       end
       nb.setMotor(1, 0);
        nb.setMotor(2, 0);
end


function main(nb)
    mOffScale = 1.112;

    % Initialize the reflectance array.
    nb.initReflectance();
    nb.initColor();
    
    
    minReflectance = [131.3, 101.5, 88.2, 83.6, 95.3, 94.1];
    % vals from advanced board minReflectance = [139.6, 103.7, 93.5,93.5,103.7,93.5];
    maxReflectance = [1426.1, 987.7, 817.6, 757.7, 793.2, 581];
    
    
    kp = 0;
    ki = 0;
    kd = 0;
    
    val = nb.encoderRead(1);    
    isBlue = DetectColor(nb)
    
    if(isBlue)
        Right45(nb);
    else
        Left45(nb);
    end
    tic
    while(toc < 2)
        nb.setMotor(1, 9);
        nb.setMotor(2, 9*mOffScale);
    end
    while()
    
        encoderVal = nb.encoderRead(1);

    
    end

    


end

%%
main(nb);