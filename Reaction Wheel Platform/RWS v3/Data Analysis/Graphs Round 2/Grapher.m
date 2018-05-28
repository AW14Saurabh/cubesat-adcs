
function Grapher()
    clear;
    %[v,T,vT]=xlsread('airDet.xlsx');
    %[v,T,vT]=xlsread('airPoint.xlsx');
    %[v,T,vT]=xlsread('airPointNonZ.xlsx');
    %[v,T,vT]=xlsread('stringDetY.xlsx');
    %[v,T,vT]=xlsread('pwrSpdZero.xlsx');
    %[v,T,vT]=xlsread('pwrSpdMin.xlsx');
    %[v,T,vT]=xlsread('pwrSpd700.xlsx');
    %[v,T,vT]=xlsread('pwrSpdFull.xlsx');
    [v,T,vT]=xlsread('pwrSpdOIO.xlsx');
    %v: Double
    %T and vT : cell
    %use v containing numbers 
    tSec = v(:,1)/1000;
    
    
    %midPoint = 409.241; %z+
    %midPoint = 443.470; %z-
    
    %midPoint = 517.488; %y+
    %midPoint = 636.876; %x+
    %midPoint = 775.463; %x+y+
    
    %midPoint = 548.886; %y-
    %midPoint = 703.839; %x-
    %midPoint = 809.103; %x-y-

    tRange = find(tSec);
    %tRange = find(tSec>=midPoint-2 & tSec<midPoint+8); %det
    %tRange = find(tSec>10 & tSec<110);%pointing 63.218); %point z
    %tRange = find(tSec>=3.813 & tSec<=60.344); %point x
    %tRange = find(tSec>60.344 & tSec<=119.645); %point y
    %tRange = find(tSec>119.645 & tSec<=176.198); %point xy
    
    t = tSec(tRange)-tSec(tRange(1));
    targAng = v(tRange,2);
    mode = v(tRange,3);
    ena = v(tRange,4);
    wx = v(tRange,5);
    wy = v(tRange,6);
    wz = v(tRange,7);
    q0 = v(tRange,8);
    q1 = v(tRange,9);
    q2 = v(tRange,10);
    q3 = v(tRange,11);
    wh1 = v(tRange,12);
    wh2 = v(tRange,13);
    wh3 = v(tRange,14);
    wh4 = v(tRange,15);
    current = v(tRange,16);
    voltage = v(tRange,17);
    
    
    adjAng = find(targAng>pi);
    targAng(adjAng) = targAng(adjAng)-(2*pi);
    
    q = cell2mat({q0 q1 q2 q3});
    eul = quat2eul(q);
    roll = eul(:,3);
    pitch = eul(:,2);
    yaw = eul(:,1);
    
    wh1 = 2*pi/6*(wh1-1.0806)/0.3832;
    wh2 = 2*pi/6*(wh2-1.0806)/0.3832;
    wh3 = 2*pi/6*(wh3-1.0806)/0.3832;
    wh4 = 2*pi/6*(wh4-1.0806)/0.3832;

    %current = -current/267.31+2.0577;
    current = -current*0.0032+1.7271;
    voltage = voltage*0.019+0.0328;
    power = current.*voltage;
    
    
    
    
    
    figure(1);
    %{
    subplot(2,2,1);
    %subplot(1,3,1);
    
    hold off;
    plot(t,wx,'r');
    hold on;
    plot(t,wy,'m');
    plot(t,wz,'color', [0.8 0.8 0]);
    e = ena;  %+ve: *0.05-0.1; %-ve: *0.02-0.12
    plot(t,e,'b');
    %ta = (targAng/pi)*0.2-0.1;
    %plot(t,ta,'c');
    title('Satellite Angular Velocity');
    xlabel('Time (s)');
    ylabel('\omega (rad/s)');
    legend('X-axis','Y-axis','Z-axis');%,'Control');
    
    subplot(2,2,3);
    
    hold off;
    plot(t,roll,'r');
    hold on;
    plot(t,pitch,'m');
    plot(t,yaw,'color', [0.8 0.8 0]);
    e = ena*0.5-2;
    plot(t,e,'b');
    plot(t,targAng,'c');
    title('Satellite Heading');
    xlabel('Time (s)');
    ylabel('Heading (rad)');
    legend('X-axis','Y-axis','Z-axis','Control','Target Angle');
    %}
    %subplot(2,2,2);
    %subplot(1,3,2);
    subplot(1,2,1);
    hold off;
    plot(t,wh1,'r');
    hold on;
    plot(t,wh2,'g');
    plot(t,wh3,'color', [0.8 0.8 0]);
    plot(t,wh4,'color',	[0.5,0,0.5]);
    e = ena*100;
    plot(t,e,'b');
    %ta = (targAng/pi+1)*200;
    %plot(t,ta,'c');
    title('Motor Speeds');
    xlabel('Time (s)');
    ylabel('\omega (rad/s)');
    legend('Wheel 1','Wheel 2','Wheel 3', 'Wheel 4');%,'Control');
    
    %subplot(2,2,4);
    subplot(1,2,2);
    
    %subplot(1,3,3);
    hold off;
    plot(t,power,'r');
    %plot(t,current,'r');
    hold on;
    %plot(t,voltage,'color', [0.8 0.8 0]);
    e = ena*0.25;
    plot(t,e,'b');
    %ta = (targAng/pi+1)*0.5;
    %plot(t,ta,'c');
    title('Total Motor Power Draw');
    xlabel('Time (s)');
    ylabel('Power (W)');
    legend('Power');%,'Control');
    
end

