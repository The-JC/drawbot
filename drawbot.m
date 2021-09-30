h = EV3;
h.connect('usb');



try
    m = motor(h);
    
    m.lowerPen(0);
    
    m.calibrate();
    pause(1);
    
%     p = [[-5, 22, 0]; [-5, 26, 1]; [5, 26, 1]; [5, 22, 1]; [-5,22,1]];
%     
%     p_sin = [];
%     i = 1;
%     for x = 0:2/50:4
%         p_sin(i,1) = sin(x*pi);
%         p_sin(i,2) = x;
%         p_sin(i,3) = 1;
%         i = i+1;
%     end
%     p_sin(1,3) = 0;

      p_sin = importsvg('bird.svg')
    
      p_sin = m.fitPath(p_sin);
%       m.gotoPoint([0,22], 1, 30);
      m.followPath(p_sin, 70);
    
%     m.gotoPoint([-5, 22], 1, [NaN, NaN], 30);
%     
%     m.lowerPen(1);
%     pause(1);
%     
%     m.gotoPoint([-5, 26], 1, [NaN, NaN], 30);
%     pause(1);
%     m.gotoPoint([5, 26], 1, [NaN, NaN], 30);
%     pause(1);
%     m.gotoPoint([5, 22], 1, [NaN, NaN], 30);
%     pause(1);
%     m.gotoPoint([-5, 22], 1, [NaN, NaN], 30);
    h.disconnect();
    
    
catch e
    h.disconnect();
    rethrow(e);
end