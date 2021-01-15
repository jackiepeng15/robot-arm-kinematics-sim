% Jackie Peng
% MAE 4710 Final Project
% Robot Arm

% User selects a starting end effector (E) position, then selects
% destination E position(s). The end effector will go to the selected
% destination E position(s) in the order in which they were selected.
% Special features include smooth animated path, efficiency through
% minimized unnecessary arm movement to save time and power.

clear all

% defining arm lengths (user can change if desired)
l1= 1; l2= 1; l3= 1;

% packing geometry defs (don't touch)
p.l1= l1; p.l2= l2; p.l3= l3;

% time vector
n= 101;
tspan= linspace(0,pi,n);

% initial E coordinates (starting location of E at t=0)
% draw circle to specify maximum range of arm
drawCircle(p);
title('Select starting E position within the circle.')
    
% user clicks input for desired starting E coordinates
[xE0, yE0]= ginput(1);
   
% final unchangeable values of xE0 and yE0 (selected from the very
% beginning)
p.xE00= xE0;
p.yE00= yE0;

% temporary xE0 and yE0 (will change throughout execution)
p.xE0= xE0;
p.yE0= yE0;

% initially E is at E0; used for solving for initial joint angles
p.xE= xE0;
p.yE= yE0;

% used for animation of end effector E path
% initialize Epath matrix with initial E location
% columns are x, y position of end effector; time varies with rows
Epath=[xE0, yE0];

% solve for initial Q vector given initial E vector
thetavector0= solveangles(p);

% desired final E coordinates (user input)
    % draw circle to specify maximum range of arm
    drawCircle(p);
    hold on
    % plot starting E location
    plot(p.xE00,p.yE00,'r.','MarkerSize',45)
    text(p.xE00,p.yE00, 'start')
    title({'Select destination E position(s) within the circle.',...
            'Hit return when finished.'})
    
    % user clicks input for desired ending E coordinates
    % xEvector and yEvector are column vectors with the user's clicked in
    % inputs
    [xEvector, yEvector]= ginput;
    
    % loop over this for each user input coordinate pair for E
    for j=1:length(xEvector)
        p.xE= xEvector(j);
        p.yE= yEvector(j);

        % find final Q space corresponding to final E coordinates
        thetavector= solveangles(p);

        % redefine Q space angles if necessary such that |angle to travel|<pi always
        % (for more efficient arm movement in animation)
        for i=1:3
            while abs(thetavector0(i))>pi
                if thetavector0(i)>0
                    thetavector0(i)= thetavector0(i)-2*pi;
                else
                    thetavector0(i)= thetavector0(i)+2*pi;
                end
            end
            while abs(thetavector(i)-thetavector0(i))>pi
                if thetavector(i)>0
                    thetavector(i)= thetavector(i)-2*pi;
                else
                    thetavector(i)= thetavector(i)+2*pi;
                end
            end
        end

        % find matrix that represents locations of points on arm during animation
        % thetastemp(3,n) is a matrix where rows are the theta values over
        % tspan (columns) for each joint (1, 2, or 3)
        % incorporates smooth movement of arm
        thetastemp= zeros(3,n);
        for i=1:n
            thetasdiff1= thetavector(1)-thetavector0(1);
            thetasdiff2= thetavector(2)-thetavector0(2);
            thetasdiff3= thetavector(3)-thetavector0(3);
            thetastemp(1,i)= thetasdiff1/2*(1-cos(tspan(i)))+thetavector0(1);
            thetastemp(2,i)= thetasdiff2/2*(1-cos(tspan(i)))+thetavector0(2);
            thetastemp(3,i)= thetasdiff3/2*(1-cos(tspan(i)))+thetavector0(3);
        end

        % animation!
            % draw initial state of arm
            Epath= drawArm(thetavector0,p,Epath,xEvector,yEvector);
            % animate arm movement
            for i=1:n
                Epath= drawArm(thetastemp(:,i),p,Epath,xEvector,yEvector);
            end 
            % change initial angles and end points
            thetavector0= thetavector;  % initial Q vector is the previous ending location
            p.xE0= p.xE;
            p.yE0= p.yE;
    end

%% FUNCTIONS

% Solves for angles of each joint on the arm given the end effector
% coordinates (which are stored in p already as p.xE and p.yE).
% thetavector= [theta1; theta2; theta3]
function thetavector= solveangles(p)
    % gives function handle in proper form to put into fsolve
    mykinematicssum= @(z) kinematicssum(z,p);

    % set up fsolve
    initialguess= 6.3*rand(3,1);
    options = optimoptions('fsolve', 'FunctionTolerance', 1e-30,...
      'OptimalityTolerance',1e-8, 'MaxFunctionEvaluations', 10000,...
      'MaxIterations', 10000,'Disp','off'); % display off or iter

    [thetavector,fval,exitflag]= fsolve(mykinematicssum,initialguess,options);

    if exitflag< 1 
          disp('FSOLVE is not happy. We want fval to be close to zeros.'); 
          fval, %This should be zero if FSOLVE did its job correctly.
    end

end


% Gives the sum of the kinematic equations defining the location of E
% given thetas (joint angles values).
% We want to make this sum 0 later when doing calculations to find thetas
% given xE and yE.
% z= [theta1; theta2; theta3]
function ksum= kinematicssum(z,p)
    theta1= z(1); theta2= z(2); theta3= z(3);
    l1= p.l1; l2= p.l2; l3= p.l3; xE= p.xE; yE= p.yE;
    ksum= [l1*cos(theta1)+l2*cos(theta1+theta2)+l3*cos(theta1+theta2+theta3)-xE;...
           l1*sin(theta1)+l2*sin(theta1+theta2)+l3*sin(theta1+theta2+theta3)-yE];
end


% Draws arm given theta values in thetavector.
% Also draws end effector's path so far as defined in Epath.
% Updates Epath, the matrix containing E's path history so far.
% thetavector= [theta1; theta2; theta3]
function Epath= drawArm(thetavector,p,Epath,xEvector,yEvector)
    % plot destination E positions
    for i=1:length(xEvector)
        plot(xEvector(i),yEvector(i),'y.','MarkerSize',45)
        text(xEvector(i),yEvector(i),num2str(i))
        hold on
    end
    
    % starting E position
    plot(p.xE00,p.yE00,'r.','MarkerSize',45)
    text(p.xE00,p.yE00, 'start')
      
    % coordinates for arm parts
    l1=p.l1; l2=p.l2; l3=p.l3;
    theta1= thetavector(1); theta2= thetavector(2); theta3= thetavector(3);
    x_arm1= l1*cos(theta1);
    y_arm1= l1*sin(theta1);
    x_arm2= x_arm1+l2*cos(theta1+theta2);
    y_arm2= y_arm1+l2*sin(theta1+theta2);
    x_arm3= x_arm2+l3*cos(theta1+theta2+theta3);
    y_arm3= y_arm2+l3*sin(theta1+theta2+theta3);    
    Epath= [Epath; x_arm3, y_arm3];

    % drawing arm
    plot(Epath(:,1),Epath(:,2),'r')     % E path so far
    plot([0 x_arm1],[0 y_arm1],'k','LineWidth',5)     % arm segment 1
    plot([x_arm1 x_arm2],[y_arm1 y_arm2],'k','LineWidth',5) % arm segment 2
    plot([x_arm2 x_arm3],[y_arm2 y_arm3],'k','LineWidth',5) % arm segment 3
    plot(x_arm1,y_arm1,'g.','MarkerSize',35)    % joint 1
    plot(x_arm2,y_arm2,'g.','MarkerSize',35)    % joint 2
    plot(x_arm3,y_arm3,'c.','MarkerSize',35)    % end effector
       
    hold off
    axis equal
    xlim([-3 3])
    ylim([-3 3])
    title('Robot arm trajectory')
    pause(.01)
end


% Draws circle for allowable range of arm.
function drawCircle(p)
    th = 0:pi/50:2*pi;
    r= p.l1+p.l2+p.l3;
    xunit = r * cos(th);
    yunit = r * sin(th);
    plot(xunit, yunit);
    axis equal
    xlim([-3 3])
    ylim([-3 3])
end