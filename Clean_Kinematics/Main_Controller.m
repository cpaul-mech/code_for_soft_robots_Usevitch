clc
clear
close all
%This File contains a clean and hopefully more modular implementation for
%octahedron CV robots to enable things like distributed control


%% Add subfolder file paths and Useful Conversions

addpath(genpath(pwd))
in2m=2.54/100;

%% Define the Geometry of the Robot

%Geometry of the Nodes
d_space=2.5*in2m;  %The spacing between the two rollers
d_offset=10*in2m;   %The distance between the kinematic joint and tube
d_offset_normal=0*in2m; %The Offset to Move Normal to the plane of the tube
d_offset_base=2*in2m;   %Offset of how big the box is. 
L_tube=134*in2m;
[ x_all, Edges_Tube, Edges_Con, T2T_Qual, B_T, N_subsections, Inds_All ] = Initialize_Octahedron(d_space, d_offset, d_offset_normal, L_tube  );
L2th=inv(B_T'*B_T)*B_T'; %The psuedo inverse
Edges_All=[Edges_Tube; Edges_Con]
Passive=[1,3]; %Note that this is implicit in how I defined the tube routing with Initialize_Octahedron

n_all=size(x_all,1);
N_Edges_True=size(Edges_Tube,1);
%% %Show a Diagram of the Robot with things labeled

figure
subplot 121  
Plot_Edges(Edges_All, x_all,'o-')  %All of the Required Edges
axis equal
hold on

for i=1:size(Edges_All,1)
    Node1=x_all(Edges_All(i,1),:);
    Node2=x_all(Edges_All(i,2),:);
    Center=(Node1+Node2)/2
    text(Center(1),Center(2),Center(3),num2str(i))
end
title('Edges')

%
subplot 122
Plot_Edges(Edges_All, x_all,'o-')  %All of the Required Edges
hold on
Number_Nodes( x_all )
axis equal
title('Nodes')

%% Define Contact with the Environment
Support=[1 2 5]; %The Support Polygon Base
% Support=[1, 27, 18]; %For some reason part of this doesn't work, which seems wrong.
C=zeros(6,3*n_all);  %6 Constraints needed in this case. 
C(1,Support(1))=1; %Fix Everything on the First Point
C(2,Support(1)+n_all)=1;
C(3,Support(1)+2*n_all)=1;
C(4,Support(2)+2*n_all)=1;
C(5,Support(2)+n_all)=1;
C(6,Support(3)+2*n_all)=1;
rank(C)

C_hist(:,:,1)=C;

%% Assign Mass Properties

m_meausured=2.83;    %Mass of an active roller module
m_passive=1.6;       %Mass of a passive roller

%Determine the Center of Mass
m_node=ones(1,24)*m_meausured/2; %Assume all other nodes have arbitrary mass
m_kin=zeros(1,6); %Assume the joints have no mass

m_tot=[m_kin,m_node];

for it=1:length(Passive)
   Neighbors=find(Edges_Con(:,1)==Passive(it));
   for j=1:length(Neighbors)    %Hardcoding the Fact that Each of these nodes has four neighbros
       m_tot(Edges_Con(Neighbors(j),2))=m_passive/2;       
   end
end

M=(m_tot/sum(m_tot));
CoM=M*x_all;
Z=zeros(1,n_all);

M_Mat=[M, Z, Z; 
       Z, M, Z;
       Z, Z, M];

%%  Specify the Desired Positions along the tube

% y_tot=reshape(x_all,3*n_all,1)';
% L_init=Get_Lengths_E(Edges_All, reshape(y_tot,n_all,3));
% L_1Tri=sum(L_init(1:3));
% N=L_1Tri/3;
% S=.3*N;
% L=(L_1Tri-S)/2;
% N_Rollers=size(B_T,2);
% L_Nom=[L_1Tri/3,2*L_1Tri/3]'; %The center Positions
% Target_Nominal=repmat(L_Nom,4,1);
% Target_Tall=[N; 2*N; S; S+L; L; L+L; L; S+L];
% Node_Speed=.08;
% 
% L_s=.9*(L_1Tri/2)
% S_s=(L_1Tri-L_s)/2
% Target_Short=[N; 2*N; L_s; L_s+S_s; S_s; S_s+S_s ;S_s; S_s+L_s]
% 
% S_3=.5*N;
% L_3=(L_1Tri-S_3)/2;
% 
% L_4=1.2*N;
% S_4=(L_1Tri-L_4)/2
% % Target_Point=[N; 2*N; L_3; L_3+S_3; L_3; L_3+S_3 ;S_4; S_4+L_4]
% Target_Point=[N; 2*N; L_3; L_3+S_3; L_3; L_3+S_3 ; N; 2*N]
% %My hand coded version
% All_Moves=[Target_Tall,Target_Short,Target_Point,Target_Nominal];
% time_move=[3.4            8.6           14.5       7.7    5];
% 
% 
% 
% %Commands_Data
% Sent_Commands=[0  0  0  0 0  0   0  0;
%                0 -7 -7 12 0  6   7 -14;
%                0 4  4 -9  0 -5  -4 9;
%                0 -6 4 -9  0  5   1 4;
%                0  0 0  0  0  0   0 0];
% % time_move=ones(size(Sent_Commands,1),1)*2;          
%            
% Command_sort=[5, 1, 8, 2, 6, 4, 7, 3];
% %Sort into the actual commands
% All_Moves=[];
% Command_Mult=1.6;
% for i=1:size(Sent_Commands,1)
% Command=Target_Nominal+Sent_Commands(i,Command_sort)'*in2m*Command_Mult;
% All_Moves=[All_Moves, Command];
% end

%% Specify the Desired Motions of the Nodes and Work Backwards
A_com{1}=M_Mat(1:3,:);
% D=.15;  %A Value of .2 seems to approach instability .15 works well with
% only the com constrained
D=.15;
b_com{1}=[D 0 -D]'; %Where to move the center of mass
b_com{2}=[0 0 2*D]';
b_com{3}=[-2*D 0 0]';
b_com{4}=[0 0 -2*D]';
b_com{5}=[D 0 D]';
b_com{6}=[0 0 0]';
time_move=ones(length(b_com),1)*1; %How long for each move in the integrator

x0_com=M_Mat*reshape(x_all,n_all*3,1);
target=x0_com;
for i=1:length(b_com)
    target(:,i+1)=target(:,i)+b_com{i};
    speed(i)=norm(b_com{i})/time_move(i);
end
target=target(:,2:end); %Remove the First Row from the target
%Have it just work on a timer? Or put an actual reached point event?
%Probably makes sense to have an event in the integrator for once rollers
%start turning off. 

%Also use the ground constraint matrix
C_Lock=zeros(9,3*n_all);
for i=1:length(Support)
        C_Lock(3*(i-1)+1,Support(i))=1;
        C_Lock(3*(i-1)+2,Support(i)+n_all)=1;
        C_Lock(3*(i-1)+3,Support(i)+2*n_all)=1;
end

Lock_Node=zeros(3,3*n_all);
Lock_Index=6;
Lock_Node(1,Lock_Index)=1;
Lock_Node(2,Lock_Index+n_all)=1;
Lock_Node(3,Lock_Index+2*n_all)=1;

for i=1:length(b_com)
% A{i}=[ A_com{1}([1,3],:); C_Lock; Lock_Node(:,:)];  %Note in this case I always use the same A_com
% b{i}=[ b_com{i}([1,3],:); zeros(9,1); zeros(3,1)];

% A{i}=[  C_Lock; Lock_Node(:,:)];  %Note in this case I always use the same A_com
% b{i}=[  zeros(9,1); zeros(3,1)];

A{i}=[ A_com{1}; C_Lock];  %Note in this case I always use the same A_com
b{i}=[ b_com{i}; zeros(9,1)];
end
% ([1,3],:)
%With 6 constraints, should be fully constrained.

% plot3(Target(1),Target(2),Target(3), 'o')

% Generate the Tube Constraints Matrix

% Loop_Con=zeros(Num_Circuits, N_Edge_Tube);
Loop_Con=[];
Num_Circuits=4;
for i=1:Num_Circuits
    Elements=3;
    Loop_Con=blkdiag(Loop_Con, ones(1,Elements));
end

%% Perform the Dynamic Simulations

y_tot=reshape(x_all,3*n_all,1)';
Ind_Support_hist=[0, Support];  %Makeing a time marked thing for these to change.
count_sup=1;

t_tot=0;

% N_Moves=size(All_Moves,2);
N_Moves=length(A);
i_support=1;

for iter=1:N_Moves

    
%     Dynamics=@(t,x)Controled_Motion( t, x, Edges_All, Edges_Tube, Loop_Con, L2th, T2T_Qual, A{iter}, b{iter});
%     Dynamics=@(t,x)Controled_Motion_Waypoints( t, x, Edges_All, Edges_Tube, Loop_Con, L2th, T2T_Qual, A{iter}, b{iter}, target(:,iter), speed(iter) );
    Dynamics=@(t,x) Controller_Roller_Combo(     t, x, Edges_All, Edges_Tube, Loop_Con, L2th, T2T_Qual, A{iter}, b{iter}, target(:,iter), speed(iter),  B_T, C);

%     From the desired node motions, back out the desired roller motions
    
    %     x_start=reshape(x_initial,3*n,1);
    x_start=y_tot(end,:)';
%     [t,y]=ode45(Dynamics,[0:.01:1], x_initial);   
    tic
    %Maintain fixed time step time? 
    time=[0:.005:time_move(iter)*2]'; %Now I need a longer time, should reach before the end
%     [t,y]=ode45(Dynamics,time, x_staFrt);
%     [t,y]=Euler_Integration_No_Events( Dynamics,time,x_start);
    x_mat=reshape(x_start,n_all,3);
%     Roll_Event= @(t,x) ~inpolygon(M_Mat(1,:)*x,M_Mat(2,:)*x,x_mat(Ind_Support,1),x_mat(Ind_Support,2));
    
    %Determine if the structure has reached the target
%     Target_Event= @(t,x) norm(); 
    Roll_Event=@(t,x) 0;
    
    if iter==N_Moves
        Waypoint_Events=@(t,x) 0;
    else
        Waypoint_Events=@(t,x) (norm(target(:,iter)-M_Mat*x)<1e-3)
    end
    %Need a seperate event function
    [t,y, Event_Flag]=Euler_Integration(Dynamics, time, x_start, Waypoint_Events);
%     t=t';
    toc
    t_tot=[t_tot; t+max(t_tot)]; 
    y_tot=[y_tot; y];  %This is double counting starting and ending configurations
    time_step(iter)=max(t)
    %If the center of mass leaves, roll over and define new contact
    %condition
    
    %Rolling Code for if the time comes.
%     if  Event_Flag
%         [x_next, Ind_Support]=Rotate_Robot(y(end,:)', Ind_Support, M_Mat, Adj);
%         count_sup=count_sup+1;
%         y_tot=[y_tot; x_next'];
%         t_tot=[t_tot; t_tot(end)];
%         Ind_Support_hist(count_sup,:)=([t_tot(end), Ind_Support]);
%         C=zeros(6,3*n_all);  %6 Constraints needed in this case. 
%         C(1,Ind_Support(1))=1; %Fix Everything on the First Point
%         C(2,Ind_Support(1)+n_all)=1;
%         C(3,Ind_Support(1)+2*n_all)=1;
%         C(4,Ind_Support(2)+2*n_all)=1;
%         C(5,Ind_Support(2)+n_all)=1;
%         C(6,Ind_Support(3)+2*n_all)=1;
%         i_support=i_support+1;
%         C_hist(:,:,i_support)=C;
%         
%         %After Rolling, Return to the normal configuration
%         Edges_Tip=Get_Lengths_E(Edges_Test,reshape(x_next,n_all,3));
%         All_Moves(:,iter+1)=L_init(1:N_L)-Edges_Tip(1:N_L);
%     end
    
end

OneifRoller=Event_Flag

%% Define the Loading

g=9.81;  %Gravity
Fz=m_tot*g; %Z force
F_tot=zeros(3*n_all,1); 
F_tot(2*n_all+1:end)=Fz; %The total force to apply

%% Compute the Loads Given the Kinematics

in2m=.0254;
psi2kpa=6.89476;
    
% Beam Geometry Parameters
r=2.54/2*in2m;
L=132/3*in2m;

% Material Parameters
% E=3*10^9; %The Modulus of the material, Pa
t=.01*in2m; %The thickness of the material
% t=.01*in2m; %This is the measured thickness of the red thick fabric 
% G=  4.1e9/1000;

%Material Parameters for LDPE
E=227e6;
poissons=.51;
G=E/(2*(1+poissons));
P= 5*psi2kpa*1000;
I=pi*r^3*t;
% F_cr_Euler=E*I*pi^2/L^2
% F_cr_Fichter=(E*I*pi^2/L^2*(P+G*pi*r*t))/(E*I*pi^2./L^2+P+G*pi*r*t)
Comp_Euler=@(r,L)  E*pi*r.^3*t*pi^2./L.^2;
Comp_Fichter=@(r,L,P)  (E*pi*r.^3*t*pi^2./L.^2.*(P+G*pi.*r*t))./(E*pi*r.^3*t*pi^2./L.^2+P+G*pi*r*t)

%Note that this is being computed with a different C matrix each time...
i_support=1;

Ind_Support_hist_Aug=[Ind_Support_hist; NaN, NaN, NaN, NaN];
N_Angle_Con=size(T2T_Qual,1);
for i=1:length(t_tot)
    if t_tot(i)==Ind_Support_hist_Aug(i_support+1,1) 
        i_support=i_support+1;           
    end
    C=C_hist(:,:,i_support);
    %Compute the Lengths at a Configuration
    L_hist_tube(:,i)=Get_Lengths_E(Edges_Tube,reshape(y_tot(i,:)',n_all,3));
    %Compute the Loading in the Given Configuration
    Loads_All(:,i)=Get_Force_Double_Roller_K(y_tot(i,:)', Edges_All, C, T2T_Qual, F_tot);  %Compute the Resulting Forces and Moments
    L_Load(:,i)=Loads_All(1:size(Edges_All,1),i);
    Torques_Bisection(:,i)=Loads_All(N_Edges_True+1:N_Edges_True+N_Angle_Con,i);
    Torques_OutofPlane(:,i)=Loads_All(N_Edges_True+N_Angle_Con+1:N_Edges_True+2*N_Angle_Con+N_Angle_Con,i);
    
    %I could examine the Fichter Buckling Load if needed
%     Buckle_Load(:,i)=Comp_Fichter(r, L_hist_tube(:,i), P);
    %Given the Lengths, Compute the Buckling Loads
%     Ratio(:,i)=(abs(L_Load(:,i))./Buckle_Load(:,i));
end


%% Compute the Center of Mass Path
x_com=M_Mat*y_tot';

figure
Plot_Edges(Edges_All, x_all,'o-g')  %All of the Required Edges
hold on
plot3(x_com(1,:),x_com(2,:),x_com(3,:),'r')

x_end=reshape(y_tot(end,:),n_all,3)
Plot_Edges(Edges_All, x_end,'o-b')
for i=1:6  %Also Show the Motion of the kinematic nodes
    plot3(y_tot(:,i),y_tot(:,i+n_all),y_tot(:,i+2*n_all),'color',[.1 .1 .1])
end

plot3(target(1,:),target(2,:),target(3,:),'o')
axis equal

%Save a Path and Use it to Compar Later
% save('J_Ldot','y_tot')
% save('Params_All_Form')

%% Play Back the Results
oldcmap = colormap;
colormap( flipud(oldcmap) )
colormap
%% Plot the robot motions and loading
MOVIE_COMBINED_LOAD=1


if MOVIE_COMBINED_LOAD
    clear h
    clear F
    Max_Load=max(max(L_Load));
    Min_Load=min(min(L_Load));
    figure
    caxis([Min_Load Max_Load]);
    count=1;
    xmin=-3;
    xmax=1.6;
    ymin=-.25;
    ymax=2.5;
    zmin=-.1;
    zmax=2;
    Ground=[xmin ymin; xmax ymin; xmax ymax; xmin ymax];
    
    d_up=3.25*in2m;  %Distance from the center of the node
    d_down=1.15*in2m;
    d_length=8/2*in2m;
    d_width=5/2*in2m;

t_movie_real=sum(time_move);
Frame_Rate=29.97;
N_frames=t_movie_real*Frame_Rate;
Inds_Good=round(linspace(0,length(t_tot),N_frames));
Inds_Good(1)=1;
%     caxis=[0 1];
% for i=Inds_Good
for i=1:10:length(t_tot)
        clf
        colormap(flipud(oldcmap))
        Indices_Tube=L_Load(:,i); %./Max_Load;
%         Indices_Tube=Ratio(:,i);
%         Indices_Con=ones(N_Edges,1);
        Indices_Con=ones(size(Edges_All,1)-12,1);
        Indices=[Indices_Tube; Indices_Con ];
%         Plot_Robot(Edges_Test,reshape(y_tot(i,:)',n_all,3),Indices, .05);
%         Plot_Robot_Edges(Edges_Test,reshape(y_tot(i,:)',n_all,3),Indices, .05);

        Plot_Robot_Edges(Edges_All(1:12,:),reshape(y_tot(i,:)',n_all,3),Indices_Tube, .05);
%         Plot_Robot_Edges_Color_Spec( Edges_Test(1:12,:), reshape(y_tot(i,:)',n_all,3), .05, [1 0 0] )
        %Plot the Constraint Connections
        [Edge_Con, x_conPlot]=Plot_Connections( y_tot(i,:)', T2T_Qual );
        Plot_Robot_Edges(Edge_Con,x_conPlot,ones(size(T2T_Qual,1),1),.025);
        %Plot the Node/Node Connection Joints with Spheres? 
        Plot_Node_Boxes(y_tot(i,:)', T2T_Qual, d_up, d_down, d_length, d_width );
        
         %Plot the Ground
%         trisurf(Tri_Ground,Ground(:,1), Ground(:,2),zmin*ones(length( Ground(:,2)),1),[.9 .9 .9])
        patch(Ground(:,1), Ground(:,2),zmin*ones(length( Ground(:,2)),1),[.9 .9 .9]);
        
        %Plot the Path of the CoM Along the Ground and Through the Air
        plot3(x_com(1,1:i),x_com(1,1:i),x_com(1,1:i),'k');
        
        
        C_bar=colorbar;
%         C_bar=colorbar('Direction','reverse')
        caxis([Min_Load Max_Load]);
%         caxis([-Max_Load Max_Load]);
        ylabel(C_bar, 'Force (N)')
        
        ax = gca;
       
        axis equal
        axis([xmin xmax ymin ymax zmin zmax]);
        
%         ax.Projection='perspective'
% %         ax.CameraPosition=[-.25, -5, 1.0];
%         ax.CameraPosition=[-0, -3, 1.0];
%         ax.CameraTarget=[0 0 .4;]
        set(gca, 'YTick',[])
        set(gca, 'XTick',[])
        set(gca, 'ZTick',[])
        set(gca,'Visible','off')
%         box off
%         view([-1,7])
%         view([0 17])
        view([158 25])
%         view([35 90])
        drawnow
        F(count)=getframe(gcf);
        count=count+1;
        hold off
        clear caxis
        
%      pause()
    end
end
xlabel('x')


%%
SAVEMOVIE=0;
if SAVEMOVIE==1
    v = VideoWriter('Chicken_Head_Fast');
    v.Quality = 100;    % Default 75, but I think this only applies to compressed videos
    v.FrameRate = 29.97;
    open(v)
    writeVideo(v,F)
    close(v)
end

