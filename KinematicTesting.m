

%% Coordinates

x1 = [0 0.05 0.1 0.15 0.2 0.1936 0.1732 0.1323 0 -0.1323 -0.1732 -0.1936 -0.2 -0.15 -0.1 -0.05];
y1 = [-0.4 -0.4 -0.4 -0.4 -0.4 -0.35 -0.3 -0.25 -0.2 -0.25 -0.3 -0.35 -0.4 -0.4 -0.4 -0.4];
x2 = [0 -0.1323 -0.1732 -0.1936 -0.2 -0.15 -0.1 -0.05 0 0.05 0.1 0.15 0.2 0.1936 0.1732 0.1323];
y2 = [-0.2 -0.25 -0.3 -0.35 -0.4 -0.4 -0.4 -0.4 -0.4 -0.4 -0.4 -0.4 -0.4 -0.35 -0.3 -0.25];

%% Links Lengths
l1 = 0.27 ;
l2 = 0.27 ;

%% Inverse Kinematic Equations

for j = 1:5
    for i = 1:16
        if (x1(i)^2 + y1(i)^2 <= 0.8)
            q2 = acos((x1(i)^2 + y1(i)^2 - l1^2 - l2^2)/(2*l1*l2));
            q1 = -( atan(y1(i)/x1(i)) + atan((l2*sin(q2))/(l1 + l2*cos(q2))) );
        
            if x1(i) < 0
            q1 = q1 + pi;
            end
        else
            return
        end

        if (x2(i)^2 + y2(i)^2 <= 0.8)
            q2_2 = acos((x2(i)^2 + y2(i)^2 - 0.1458)/(0.1458));
            q1_2 = -( atan(y2(i)/x2(i)) + atan((0.27*sin(q2_2))/(0.27 + 0.27*cos(q2_2))) );
        
            if x2(i) < 0
            q1_2 = q1_2 + pi;
            end
        else
            return
        end
        
        
        %% Trigonometric abbreviations
        c1 = cos(q1);
        c2 = cos(q2);
        c12 = cos(q1+q2);
        
        s1= sin(q1);
        s2 = sin(q2);
        s12 = sin(q1+q2);
        
        c1_2 = cos(q1_2);
        c2_2 = cos(q2_2);
        c12_2 = cos(q1_2+q2_2);

        s1_2 = sin(q1_2);
        s2_2 = sin(q2_2);
        s12_2 = sin(q1_2+q2_2);
        
        
        %% Tip position
        % These equations are derived from the Forward Kinematic model of the 2DOF
        % robot
        FLxt = l1*c1+l2*c12 ;
        FLyt = -l1*s1-l2*s12 ;
        FLpt = [ FLxt FLyt ] ;

        RLxt = l1*c1_2+l2*c12_2;
        RLyt = -l1*s1_2-l2*s12_2;
        RLpt = [ RLxt RLyt ] ;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %% Plot the legs
        figure (1) 
        % set(2,'position',[116 190 560 420])
        
        FLx1 = 0 ;
        FLy1 = 0 ;
        FLx2 = l1*c1 ;
        FLy2 = -l1*s1 ;
        
        RLx1 = 0 ;
        RLy1 = 0 ;
        RLx2 = l1*c1_2;
        RLy2 = -l1*s1_2;
        
        
        
        FLxx = [ FLx1; FLx2; FLpt(1,1) ] ;
        FLyy = [ FLy1; FLy2; FLpt(1,2) ] ;
        RLxx = [ RLx1+0.8 ; RLx2+0.8 ; RLpt(1,1)+0.8 ] ;
        RLyy = [ RLy1; RLy2; RLpt(1,2) ] ;

        FRxx = [ RLx1; RLx2; RLpt(1,1) ] ;
        FRyy = [ RLy1; RLy2; RLpt(1,2) ] ;
        RRxx = [ FLx1+0.8 ; FLx2+0.8 ; FLpt(1,1)+0.8 ] ;
        RRyy = [ FLy1; FLy2; FLpt(1,2) ] ;
        
        plot(FRxx,FRyy,'o-','color', [.7 .7 .7],'Linewidth',2);
        hold on
        plot(RRxx,RRyy,'o-','color', [.7 .7 .7],'Linewidth',2);
        plot(FLxx,FLyy,'ko-','Linewidth',2);
        plot(RLxx,RLyy,'ko-','Linewidth',2);
        plot(x1,y1,'.');
        plot(x1+0.8,y1,'.');
        plot([0 0.8],[0 0],'ko-','Linewidth',2)
        axis equal
            
        xlabel('x (m)') ; ylabel('y (m)') ;
        % text(FLpt(1,1),FLpt(1,2),'x') ; text(FLpt(1,1) + 0.002,FLpt(1,2) + 0.002,'ptStart') ;
        % text(FLpt(4,1),FLpt(4,2),'x') ; text(FLpt(4,1) + 0.002,FLpt(4,2) + 0.002,'ptEnd') ;
        % text(RLpt(1,1),RLpt(1,2),'x') ; text(RLpt(1,1) + 0.002,RLpt(1,2) + 0.002,'ptStart') ;
        % text(RLpt(4,1),RLpt(4,2),'x') ; text(RLpt(4,1) + 0.002,RLpt(4,2) + 0.002,'ptEnd') ;
        axis([-0.5 1.3 -0.6 0.2])
        pause(0.05)
        if FLyy(3) <= RLyy(3)
            yline(FLyy(3),'green');
        else 
            yline(RLyy(3),'green');
        end
        hold off
        pause(0.03)
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
end
