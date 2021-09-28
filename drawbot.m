h = EV3;
h.connect('usb');



% try
    m = motor(h);
    m.calibrate();
    
    
% catch
%     h.disconnect();
% end