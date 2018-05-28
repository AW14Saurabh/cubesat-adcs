
function Grapher()
    clear;
    [v,T,vT]=xlsread('0Control.xlsx');
    %v: Double
    %T and vT : cell
    %use v containing numbers 
    tSec = v(:,1)/1000;
    tRange = find(tSec);
    tRange = find(tSec>53 & tSec<253);
    t = (tSec(tRange)-tSec(tRange(1)));
    wx = v(tRange,7);
    wy = v(tRange,8);
    wz = v(tRange,9);
    targAng = v(tRange,3)
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
    
    adjAng = find(targAng>pi);
    targAng(adjAng) = targAng(adjAng)-(2*pi);

    
    wh1 = v(tRange,16);
    wh2 = v(tRange,17);
    wh3 = v(tRange,18);
    wh4 = v(tRange,19);
    wh1 = 2*pi/6*(wh1-1.0806)/0.3832;
    wh2 = 2*pi/6*(wh2-1.0806)/0.3832;
    wh3 = 2*pi/6*(wh3-1.0806)/0.3832;
    wh4 = 2*pi/6*(wh4-1.0806)/0.3832;
    

    current = -current/267.31+2.0577;
    voltage = voltage*0.019+0.0328;
    power = current.*voltage;
    
    
    
    
    
    
    figure(1);
    titleText = 'Air Bearing Control Test';
    
    subplot(2,2,1);
    hold off;
    plot(t,wx,'r');
    hold on;
    plot(t,wy,'m');
    plot(t,wz,'color', [0.8 0.8 0]);
    %ena = v(tRange,5)/1000*20-0.05;
    %plot(t,ena,'b');
    title('Satellite Angular Velocity');
    xlabel('Time (s)');
    ylabel('\omega (rad/s)');
    legend('X-axis','Y-axis','Z-axis','Control');
    
    subplot(2,2,3);
    hold off;
    plot(t,roll,'r');
    hold on;
    plot(t,pitch,'m');
    plot(t,yaw,'color', [0.8 0.8 0]);
    ena = v(tRange,5)/1000*200-0.5;
    plot(t,ena,'b');
    title('Satellite Heading');
    xlabel('Time (s)');
    ylabel('Heading (rad)');
    legend('X-axis','Y-axis','Z-axis','Control');

    subplot(2,2,2);
    hold off;
    plot(t,wh1,'r');
    hold on;
    plot(t,wh2,'g');
    plot(t,wh3,'color', [0.8 0.8 0]);
    plot(t,wh4,'c');
    ena = v(tRange,5)*50;
    plot(t,ena,'b');
    %ta = (targAng/pi+1)*200;
    %plot(t,ta,'c');
    title('Motor Speeds');
    xlabel('Time (s)');
    ylabel('\omega (rad/s)');
    legend('Wheel 1','Wheel 2','Wheel 3', 'Wheel 4','Control');
    
    subplot(2,2,4);
    hold off;
    plot(t,current,'color', [0.8 0.8 0]);
    hold on;
    plot(t,voltage,'r');
    ena = v(tRange,5)/1000*50;
    plot(t,ena,'b');
    %ta = (targAng/pi+1)*0.5;
    %plot(t,ta,'c');
    title('Total Motor Power Draw');
    xlabel('Time (s)');
    ylabel('Power (W)');
    legend('Power','Control');
end

