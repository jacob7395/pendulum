clc
clear all

Target_IP   = '192.168.168.3';
Local_IP    = '192.168.168.3';
obj1 = instrfind('Type', 'udp', 'RemoteHost', Target_IP, 'RemotePort', 55056, 'Tag', '');

% Create the udp object if it does not exist
% otherwise use the object that was found.
if isempty(obj1)
    obj1 = udp(Target_IP, 3333);
else
    fclose(obj1);
    obj1 = obj1(1)
end

% Configure instrument object
% these our our ip and port
% port must be > 1024
set(obj1, 'LocalHost', Local_IP);
set(obj1, 'LocalPort', 63232);
set(obj1, 'LocalPortMode', 'manual');

% Connect to instrument object
fopen(obj1);

%make figure name and make figure windwom
figurename =['Real Time PI'];
fig = figure('name', figurename,'NumberTitle','off');

n = 2;

ax1 = subplot(2,2,1); % top subplot

grid on, axis([0,30,-1500,1500])
set(gca,'fontsize',12,'fontweight','bold') % Fontsize
title('Cart Position','fontsize',12,'fontweight','bold')
ylabel('','fontsize',12,'fontweight','bold')
xlabel('Time Seconds','fontsize',12,'fontweight','bold')
hold on

ax2 = subplot(2,2,2); % bottom subplot

grid on, axis([0,30,-1.0,1.0])
set   (gca,'fontsize',12,'fontweight','bold') % Fontsize
title ('Cart Velocity','fontsize',12,'fontweight','bold')
ylabel('Velocity(Ms)','fontsize',12,'fontweight','bold')
xlabel('Time Seconds','fontsize',12,'fontweight','bold')
hold on

ax3 = subplot(2,2,3); % bottom subplot

grid on, axis([0,30,-180,180])
set   (gca,'fontsize',12,'fontweight','bold') % Fontsize
title ('Pendulum Angle','fontsize',12,'fontweight','bold')
ylabel('Degrees','fontsize',12,'fontweight','bold')
xlabel('Time Seconds','fontsize',12,'fontweight','bold')
hold on

ax4 = subplot(2,2,4); % bottom right subplot

grid on, axis([0,30,-180,180])
set   (gca,'fontsize',12,'fontweight','bold') % Fontsize
title ('PID','fontsize',12,'fontweight','bold')
ylabel('Degrees','fontsize',12,'fontweight','bold')
xlabel('Time Seconds','fontsize',12,'fontweight','bold')
hold on

data = [0,0,0,0,0];

fopen( 'run.txt', 'wt' );

while exist('run.txt', 'file') == 2
    % wait for udp packet
    % default time = 10 sec but can be changed
    s = fscanf(obj1);
    %check a packet has been resived
    if length(s) > 1 && size(str2num(s),2) == 5
        data_buffer = str2num(s);
        if     data(size(data,1),1) < data_buffer(1,1)
            data = [data;data_buffer];
                %plot scroling graph
                while size(data,1)-300 > n
                    axis(ax1,[data(n,1),1+data(n+300,1),-1500,1500])
                    axis(ax3,[data(n,1),1+data(n+300,1),-180,180])
                    axis(ax2,[data(n,1),1+data(n+300,1),-1.0,1.0])

                    Plot_One    = plot(ax1,data(n:n+300,1),  data(n:n+300,2), 'r');
                    Plot_Two    = plot(ax2,data(n:n+300,1),  data(n:n+300,3), 'g');
                    Plot_Three  = plot(ax3,data(n:n+300,1),  data(n:n+300,4), 'b');

                    n = n + 10;
                    pause(0.00001)

                    delete(Plot_One)
                    delete(Plot_Two)
                    delete(Plot_Three)
                end
                %plot non scroling graph
                if size(data,1)-300 < n
                    Data_Size = size(data,1);

                    axis(ax1,[data(n,1), data(n,1) + 20,-1500,1500])
                    axis(ax3,[data(n,1), data(n,1) + 20,-180 ,180])
                    axis(ax2,[data(n,1), data(n,1) + 20,-1.0 ,1.0])

                    Plot_One    = plot(ax1,data(2:Data_Size,1),  data(2:Data_Size,2), 'r');
                    Plot_Two    = plot(ax2,data(2:Data_Size,1),  data(2:Data_Size,3), 'g');
                    Plot_Three  = plot(ax3,data(2:Data_Size,1),  data(2:Data_Size,4), 'b');

                    pause(0.00001)

                    delete(Plot_One)
                    delete(Plot_Two)
                    delete(Plot_Three)
                end
        else
            n = 1;
            data = [0,0,0,0,0];
            data = [data;str2num(s)];
        end
    else
        disp('Bad Coms')
    end
end

% Disconnect from instrument object, obj1.
fclose(obj1);

% Clean up all objects.
delete(obj1);