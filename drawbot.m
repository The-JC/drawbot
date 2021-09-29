h = EV3;
h.connect('usb');



try
    m = motor(h);
    m.calibrate();
    pause(1);
    
    m.gotoPoint([0, 22], 1, [NaN, NaN], 50);
    
    pause(1);
    
    m.gotoPoint([0, 18], 1, [NaN, NaN], 50);
    pause(1);
    m.gotoPoint([10, 18], 1, [NaN, NaN], 50);
    pause(1);
    m.gotoPoint([10, 22], 1, [NaN, NaN], 50);
    pause(1);
    m.gotoPoint([0, 22], 1, [NaN, NaN], 50);
    h.disconnect();
    
    
catch e
    h.disconnect();
    rethrow(e);
end