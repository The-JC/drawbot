function [] = lowerPen(lowerPen, brickObj)
%LOWERPEN Summary of this function goes here
%   Detailed explanation goes here

power = 20;

m =  brickObj.motorA;
m.speedRegulation = true;
m.limitMode = 'Tacho';
m.brakeMode = 'Brake';
m.limitValue = 100;
m.resetTachoCount;

if(lowerPen)
    m.power = power;
else
    m.power = -power;
end

m.start;
m.waitFor;

end

