clear all;
close all;
question=2;  %  1 for part 1 / 2 for part 2
robot = mdl_ur10e();


% initial values
q0 = [-0.140 -1.556 -1.359 1.425 -1.053 -1.732];
T = robot.fkine(q0);
Ts = 0.002; % time step
goe=double(T);  % forward kinematics

poc=transpose([0.4 0 0.2]);
roc=[1 0 0; 0 1 0; 0 0 1];
goc=[roc poc;0 0 0 1];  %camera

%ball has different position and rotation for every moment
%initially
yb_initial=[0;  0.9351 ;-0.3543];
wsp=Wspace();
[pcb,vcb,wcb]=wsp.sim_ball(0);
rcb=[[1; 0; 0] yb_initial cross([1; 0; 0] ,yb_initial)];
gcb=[rcb pcb; 0 0 0 1];

pBE_desired = [0; 0; 0.45];   %following ball  
rbe_desired = roty(180); 
gbe_desired=[rbe_desired pBE_desired; 0 0 0 1];

k=1; % count control circles
theta=atan2(yb_initial(3),yb_initial(2));    %initial angle for ball

v=zeros(6,1);  %for velocity

for i=Ts:Ts:5

    if question==2  %for part 2 to get the ball
    if pBE_desired(3)>0.06
        pBE_desired(3)=pBE_desired(3)-0.0005;
        gbe_desired=[rbe_desired pBE_desired; 0 0 0 1];
    end
    end
    
    if det(gcb)~=0   %between ball and robot edge
    gbe=inv(gcb)*inv(goc)*goe;
    end

    position(k,:)=gbe(1:3,4);    
    angles=acos((trace(gbe(1:3,1:3))-1)/2);  %orientation angle
    if angles~=0  %orientation axis
    kk=(1/(2*sin(angles)))*[gbe(3,2)-gbe(2,3); gbe(1,3)-gbe(3,1); gbe(2,1)-gbe(1,2)];
    end
    angle(k,1)=angles;
    angle(k,2:4)=kk;
    pos(k,:)=q0;   %for plots


    %what i want    
    goe_desired=goc*gcb*gbe_desired;
   

    %inverse jacobean because we need to get the velocity for every link 
    %but we know v and w for the edge
    if det(robot.jacob0(q0,'eul'))~=0
    J1 = inv(robot.jacob0(q0,'eul'));
    end


    %orientation error
    r=goe(1:3,1:3)*inv(goe_desired(1:3,1:3));
    th=acos((trace(r)-1)/2);  %orientation error angle
    if th~=0  %orientation error axis
    kapa=(1/(2*sin(th)))*[r(3,2)-r(2,3); r(1,3)-r(3,1); r(2,1)-r(1,2)];
    end

    %controller for velocity   
    u=[vcb;wcb;]-[16*(-goe_desired(1:3,4)+goe(1:3,4));([36*th*kapa(1); 0.001*th*kapa(2); 0.001*th*kapa(3)])];
    vel=J1*u;   %velocity input for every link
   


     a=(vel-v)/Ts;  %acceleration
     %filter for acceleration
       for j=1:6
           if a(j)>=0
               a(j)=min(a(j),250);
           else
               a(j)=max(a(j),-250);
           end
           vel(j)=a(j)*Ts+v(j);
       end   %velocity with filtered acceleration


        %filters for velocity
      vmax=deg2rad([120; 120 ;180 ;180; 180 ;180 ]);
        for j=1:6
            if vel(j)>=0
                 vel(j)=min(vel(j),vmax(j));
             elseif vel(j)<0
                  vel(j)=max(vel(j),-vmax(j));
            end
        end
    v=vel;


    velocity(k,:)=v;
    acceleration(k,:)=a;   %for plots
    

    q0=q0+transpose(v)*Ts;    %new link positions
    T=robot.fkine(q0);
    goe=double(T);  

    %ball keeps moving 
    [pcb,vcb,wcb]=wsp.sim_ball(Ts);
 theta=theta+wcb(1)*Ts;
  rcb=rotx(rad2deg(theta));
gcb=[rcb pcb; 0 0 0 1];
    k=k+1;
end

f=wsp.visualize(robot,transpose(pos),[90,0]);


    figure %pbe plots
    plot(position,'LineWidth',3)
   legend("p in axis x","p in axis y","p in axis z")
    xline(0,"HandleVisibility","off")
    yline(0,"HandleVisibility","off")
    title("Position BE")



    figure  %rbe plots
    plot(angle,'LineWidth',3)
    legend("angle","x axis","y axis","z axis")
    xline(0,"HandleVisibility","off")
    yline(0,"HandleVisibility","off")
    title("Rotation BE")



   figure  %link positions
    plot(pos,'LineWidth',3)
   legend("pos of link 1","pos of link 2","pos of link 3","pos of link 4","pos of link 5","pos of link 6")
    xline(0,"HandleVisibility","off")
    yline(0,"HandleVisibility","off")
    title("Position of links")



   figure  %link velocities
    plot(velocity,'LineWidth',3)
   legend("velocity of link 1","velocity of link 2","velocity of link 3","velocity of link 4","velocity of link 5","velocity of link 6")
    xline(0,"HandleVisibility","off")
    yline(0,"HandleVisibility","off")
    title("Velocity of links")



   figure  %link acceleration
    plot(acceleration,'LineWidth',3)
    legend("Acceleration of link 1","Acceleration  of link 2","Acceleration  of link 3","Acceleration  of link 4","Acceleration  of link 5","Acceleration  of link 6")
    xline(0,"HandleVisibility","off")
    yline(0,"HandleVisibility","off")
    title("Acceleration of links")
 