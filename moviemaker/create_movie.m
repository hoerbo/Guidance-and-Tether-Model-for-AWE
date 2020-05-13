%% Movie Maker Script
%
%This script creates video files based on simulation data provided by
%simulink_model.slx. 

close all
clc
addpath('scripts');

%Create time vector for 30 fps
tint = linspace(0,max(Y_out.Time),30*max(Y_out.Time));

%% Tether States
Y = Y_out.Data;
Tetherforce = Y(:,1:3)';

%Calculate norm of Tetherforce
for i=1:length(Tetherforce) 
    Tetherforce(4,i) = norm(Tetherforce(1:3,i));
end

L0 = Y(:,end)';

Y = Y(:,4:end-1)';
[r,~] = size(Y);
pm=r/6;

%Interpolate Tether Position
Y_interp = zeros(r,length(tint));
for i=2:pm
    for z=1:3
    Ytemp = griddedInterpolant(Y_out.Time,Y(z+3*(i-1),:)', 'pchip');
    Y_interp(z+3*(i-1),:) = Ytemp(tint);
    end
end

%Interpolate Tether Force
tetherforcetemp = griddedInterpolant(Y_out.Time,Tetherforce(4,:)', 'pchip');
Tetherforce = tetherforcetemp(tint);

%Interpolate Tether Initial Length
L0temp = griddedInterpolant(Y_out.Time,L0', 'pchip');
L0 = L0temp(tint);


%% Aircraft Position
Pos = Position_out.Data;
Pos = Pos';
Pos_interp = zeros(3,length(tint));
for z=1:3
    Postemp = griddedInterpolant(Position_out.Time,Pos(z,:)', 'pchip');
    Pos_interp(z,:) = Postemp(tint);
end


%% Create Video
v = VideoWriter('3DPlot.avi');
open(v);


for i=1:length(tint)
    %Lemniscate
    s = linspace(0,2*pi,100);
    x_w = Gamma(s,guidance);
    x_o = T1(pi)*T3(pi+guidance.windangle)*x_w*guidance.heightcmd;
    plot3(x_o(2,:),x_o(1,:),-x_o(3,:),'k--');
    hold on

    %Formatting
    xlabel('East [m]','interpreter','latex');
    ylabel('North [m]','interpreter','latex');
    zlabel('Height [m]','interpreter','latex');
    xlim([-200,200]);
    ylim([-450,0]);
    zlim([0,450]);
    view(20,15)
    title('Ground-Generation System','interpreter','latex')
    box on

    %Aircraft (red dot) and Tether Position
    plot3([0 Y_interp(5,i)], [0 Y_interp(4,i)],[0 -Y_interp(6,i)],'Color','k');
    for z=1:pm-1
        if z==pm-1
            plot3([Y_interp(5+3*(z-1),i) Pos_interp(2,i)],[Y_interp(4+3*(z-1),i) Pos_interp(1,i)] ,[-Y_interp(6+3*(z-1),i) -Pos_interp(3,i)],'-o','Color','k','MarkerFaceColor','red','MarkerIndices',2);
        else
            plot3([Y_interp(5+3*(z-1),i) Y_interp(8+3*(z-1),i)],[Y_interp(4+3*(z-1),i) Y_interp(7+3*(z-1),i)] ,[-Y_interp(6+3*(z-1),i) -Y_interp(9+3*(z-1),i)],'-o','Color','k','MarkerFaceColor','black');
        end
    end
    
    %Following Line
    plot3(Pos_interp(2,1:i),Pos_interp(1,1:i),-Pos_interp(3,1:i),'red','LineWidth',1);

    %Tether Strain
    L_tot = 0;
    for z=1:pm
        if z==pm
            p1 = Y_interp(1+3*(z-1):3+3*(z-1),i);
            p2 = Pos_interp(:,i);
            L = norm(p2-p1);
            L_tot = L_tot + L;
        else
        p1 = Y_interp(1+3*(z-1):3+3*(z-1),i);
        p2 = Y_interp(4+3*(z-1):6+3*(z-1),i);
        L = norm(p2-p1);
        L_tot = L_tot + L;
        end
    end
    L0_tot = L0(i)*pm;
    Tether_strain = (L_tot-L0_tot)/L0_tot*100;
    Tether_strainstr = sprintf('$\\epsilon = %.1f$',Tether_strain);
    Tether_strainstr = strcat(Tether_strainstr,' \%');
    text(90,0,260,Tether_strainstr,'Interpreter','latex');
    
    %Tetherforce
    Tetherforcestr = sprintf('$F = %.1f$ kN',Tetherforce(i)/1000);
    text(85,0,220,Tetherforcestr,'interpreter','latex')
 
    %Header
    text(85,0,300,'Tether States:','interpreter','latex')
        
    %Create Video Frame
    frame = getframe(gcf);
    writeVideo(v,frame);
    hold off
end

close(v);
