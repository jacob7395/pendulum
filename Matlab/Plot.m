clc
clear all

x = 1;
filename =['/Volumes/Home Directory/Desktop/Data/Run-' num2str(x) '.csv'];

while exist(filename, 'file') == 0
end

while exist(filename, 'file') == 2
   x = x + 1;
   filename =['Run-' num2str(x) '.csv'];
end

filename =['Run-' num2str(x-1) '.csv'];
data = csvread(filename);

T = zeros(size(data,1),2);

for m = 1:size(data,1)
   T(m, 1) = 0.01 * m;
end

fig = figure;

while 1

    n = 1;
    
    ax1 = subplot(2,1,1); % top subplot

    grid on, axis([0,30,-1500,1500])
    set(gca,'fontsize',12,'fontweight','bold') % Fontsize
    title('Cart Position','fontsize',12,'fontweight','bold')
    ylabel('','fontsize',12,'fontweight','bold')
    xlabel('Time Seconds','fontsize',12,'fontweight','bold')
    hold on

    ax2 = subplot(2,1,2); % bottom subplot

    grid on, axis([0,30,-180,180])
    set   (gca,'fontsize',12,'fontweight','bold') % Fontsize
    title ('Pendulum Angle','fontsize',12,'fontweight','bold')
    ylabel('Degrees','fontsize',12,'fontweight','bold')
    xlabel('Time Seconds','fontsize',12,'fontweight','bold')
    hold on
  
    check_name =['Run-' num2str(x) '.csv'];
    while exist(check_name, 'file') == 0 

        data = csvread(filename);
        
        if size(T,1) <= size(data,1)
            T = zeros(size(data,1),2);

            for m = 1:size(data,1)
                T(m, 1) = 0.01 * m;
            end
        end

        while size(data,1)-2900 > n && exist(check_name, 'file') == 0 
            
            axis(ax1,[0+(n*0.01),30+(n*0.01),-1500,1500])
            axis(ax2,[0+(n*0.01),30+(n*0.01),-180,180])

            Plot_One = plot(ax1,T(n:n+2900),  data(n:n+2900,1), 'r');
            Plot_Two = plot(ax2,T(n:n+2900),  data(n:n+2900,3), 'b');

            n = n + 1;
            pause(0.00001)

            delete(Plot_One)
            delete(Plot_Two)
        end
    end
    %now a new file in avalible make file name then incoroment count
    filename =['Run-' num2str(x) '.csv'];
    x = x + 1;
    
    clf(fig);
end

