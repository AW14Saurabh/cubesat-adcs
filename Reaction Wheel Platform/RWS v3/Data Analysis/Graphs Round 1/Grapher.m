
function Grapher()
    clear;
    [v,T,vT]=xlsread('2PointZ.xlsx');
    %v: Double
    %T and vT : cell
    %use v containing numbers 
    tSec = v(:,1)/1000;
    tRange = find(tSec);
    tRange = find(tSec>5.7 & tSec<30);
    t = (tSec(tRange)-tSec(tRange(1)));
    wx = v(tRange,7);
    wy = v(tRange,8);
    wz = v(tRange,9);
    ena = v(tRange,5)/1000*20-0.05;
    
    q0 = v(tRange,11);
    q1 = v(tRange,12);
    q2 = v(tRange,13);
    q3 = v(tRange,14);
    q = cell2mat({q0 q1 q2 q3});
    eul = quat2eul(q);
    roll = eul(:,3);
    pitch = eul(:,2);
    yaw = eul(:,1);
    
    wh1 = v(tRange,16);
    wh2 = v(tRange,17);
    wh3 = v(tRange,18);
    wh4 = v(tRange,19);
    wh1 = 2*pi/6*(wh1-1.0806)/0.3832;
    wh2 = 2*pi/6*(wh2-1.0806)/0.3832;
    wh3 = 2*pi/6*(wh3-1.0806)/0.3832;
    wh4 = 2*pi/6*(wh4-1.0806)/0.3832;
    
    current = -v(tRange,21)/267.31+2.0577;
    voltage = v(tRange,22)*0; %to calibrate
    
    
    
    
    
    
    
    
    
    
    figure(1);
    titleText = 'Air Bearing, Detumble CCW around Z+';
    
    subplot(2,2,1);
    hold off;
    plot(t,wx,'r');
    hold on;
    plot(t,wy,'m');
    plot(t,wz,'color', [0.8 0.8 0]);
    ena = v(tRange,5)/1000*20-0.05;
    plot(t,ena,'b');
    title(titleText);
    xlabel('Time (s)');
    ylabel('Satellite \omega (rad/s)');
    legend('X-axis','Y-axis','Z-axis','Control');
    
    subplot(2,2,2);
    hold off;
    plot(t,roll,'r');
    hold on;
    plot(t,pitch,'m');
    plot(t,yaw,'color', [0.8 0.8 0]);
    ena = v(tRange,5)/1000*200-0.5;
    plot(t,ena,'b');
    title(titleText);
    xlabel('Time (s)');
    ylabel('Satellite Heading (rad)');
    legend('X-axis','Y-axis','Z-axis','Control');

    subplot(2,2,3);
    hold off;
    plot(t,wh1,'r');
    hold on;
    plot(t,wh2,'g');
    plot(t,wh3,'color', [0.8 0.8 0]);
    plot(t,wh4,'c');
    ena = v(tRange,5)*50;
    plot(t,ena,'b');
    title(titleText);
    xlabel('Time (s)');
    ylabel('Wheel Speed (rad/s)');
    legend('Wheel 1','Wheel 2','Wheel 3', 'Wheel 4','Control');
    
    subplot(2,2,4);
    hold off;
    plot(t,current,'color', [0.8 0.8 0]);
    hold on;
    plot(t,voltage,'r');
    ena = v(tRange,5)/1000*50;
    plot(t,ena,'b');
    title(titleText);
    xlabel('Time (s)');
    ylabel('Arduino Units');
    legend('Current','Voltage','Control');
    
end

