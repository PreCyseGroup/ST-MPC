clear all
close all
clc

%Simulation Parameters
video_recording=false;
rng(1); %seed of the noise/disturbance
Nsets=25;
N_steps_simulation=30;
x0=[-0.15;4.9]; %initial state of the system
fontsize=30;
%

if video_recording
    %saving a video recording using as file name the current date and time
    datetime=datestr(now);
    datetime=strrep(datetime,':','_'); %Replace colon with underscore
    datetime=strrep(datetime,'-','_');%Replace minus sign with underscore
    datetime=strrep(datetime,' ','_');%Replace space with underscore
    %file_video=strcat('video_',datetime,'.avi');
    file_video=strcat('videoRecording_',datetime);
    nFrames = 1;
    vidObj = VideoWriter(file_video,'MPEG-4');
    vidObj.Quality = 100; %quality of the video <=100
    vidObj.FrameRate =1.5;
    open(vidObj);
end

%open a figure
h1=figure(1);
set(h1, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]); % Enlarge figure to full screen.
set(h1,'Renderer','zbuffer'); %setting the plotting rendering for the video recording

%Mass spring damper parameters
K=24;
M=25;
beta=8;

%continuos-time model of the system
A = [0 1; -K/M -beta/M];
B = [0 ;1/M];
C = eye(2);
D = [0; 0];
sys = ss(A,B,C,D);

%discretized-model of the system
Ts = 0.1;
sys_d=c2d(sys,Ts,'zoh');
Ad =  [1,0.0975;0 0.95];
Bd =  [0.0246;0.4877];
model = LTISystem('A',Ad,'B',Bd,'Ts',Ts);

%constraints on states
model.x.min = [-2;-5];
model.x.max = [2;5];
X = Polyhedron('lb',model.x.min,'ub',model.x.max);

%constraint on input 
model.u.min = [-1.1];
model.u.max = [1.2];
mu = model.u.min;
mx = model.u.max;
U = Polyhedron('lb',model.u.min,'ub',model.u.max);

%bounded disturbance 'W' acting on both states 
a1 = -0.03;
b1 = 0.03;
a2 = -0.05;
b2 = 0.05;
w.min = [a1; a2];
w.max = [b1; b2];
W = Polyhedron('lb', w.min, 'ub', w.max);

%plotting the state constraints
plot([model.x.min(1) model.x.max(1) model.x.max(1) model.x.min(1) model.x.min(1)],[model.x.min(2) model.x.min(2) model.x.max(2) model.x.max(2) model.x.min(2)],'LineWidth',3)
hold on
title('OFFLINE PHASE started...')
fprintf('*** OFFLINE PHASE started ***\n\n')

% LQR design 
fprintf('LQR controller design...')
Q_LQ=[100 0;0 1]; %state weight matrices Q>=0 for LQ design
R_LQ=eye(model.nu); %input weight matrices R>0 for LQ design
model.x.penalty = QuadFunction(Q_LQ);
model.u.penalty = QuadFunction(R_LQ);
K = model.LQRGain; % MPT3 function to obtain the LQR feedback
fprintf('COMPLETED!\n\n')

fprintf('Smallest RPI computation....')
%compute the smallest RPI region using  Rakovic et al, TAC 05 (on the closed-loop model)
Acl = Ad+(Bd*K);% Closed-loop dynamical model (disturbance to be added to this model for RPI computation)
alpha= 0.001; % alpha = 0.01 ;%% alpha should be in [0 1)
T0=computeRPI(Acl,alpha,W); %algorithm developed by Rakovic et al, TAC 05.
%check if the RPI is valid for the given state and input constraints
if X.contains(T0)==0 
    error('Smallest RPI not contained in the state constraints')
end
if U.contains(K*T0)==0
    error('Control effort to keep the system inside the RPI not admissible')
end
                                                     
fprintf('COMPLETED!\n\n')

%plotting the computed RPI region
plot(T0,'Alpha',0.2,'color','g','wired','true','LineWidth',1.5);
set(gca,'FontSize',fontsize)
if video_recording
    immagine=getframe(h1);
    writeVideo(vidObj,immagine);
end

fprintf('Family of ROSC sets computation....\n')
%ROSC sets computation
Pn = T0; % compute reachable sets starting from T0
for i =1:Nsets
    fprintf('   ROSC set %i of %i\n',i,Nsets)
    %ROSC Set formula implemented in the next three lines of code: (T0-W)\oplus(-BdU)*Ad. 
    R=(T0-W);
    R=R.plus(-Bd*U);
    R=R*Ad;
    R = R.intersect(X); % state constraint imposition
    hold on
    plot(R,'Alpha',0,'Color','green','wired','true','LineWidth',1.5) %plotting the ROSC set
    pause(0.1) % pause only for visualization pourposes
     if video_recording
        immagine=getframe(h1);
        writeVideo(vidObj,immagine);
    end
    Pn = [Pn,R]; %accumulate the set family
    if R==T0 %stop if the set does not grow
        break
    else
        T0 = R;
    end
end

if i<Nsets
    title('*** OFFLINE PHASE stopped due saturation! Press a button *** ') %changing the title to the opened figure
    fprintf('ROSC family computation STOPPED due saturation!\n\n')
else
    title('*** OFFLINE PHASE completed! Press a button ***') %changing the title to the opened figure
    fprintf('ROSC family computation COMPLETED!\n\n')
end
fprintf('Press a button to continue with the online phase....\n\n')
pause
title('ONLINE PHASE started...') %changing the title to the opened figure
fprintf('*** ONLINE PHASE started ***\n')

% Variables initialization for online phase
x_evolution=zeros(size(Ad,1),N_steps_simulation);
index=zeros(1,N_steps_simulation);
u_evolution=zeros(size(Bd,2),N_steps_simulation);
d_evolution=zeros(size(Ad,1),N_steps_simulation);

%online forloop
for k=1:N_steps_simulation
    fprintf('  step %i of %i\n',k,N_steps_simulation)
    if k==1
        x_curr=x0; % x0 is the initial state
    else
        x_curr=x_evolution(:,k-1); % update the current state of the system
    end
    plot(x_curr(1),x_curr(2),'.','MarkerSize',40)
   
  
    %CONTROL LOGIC
    
    %finding set-membership index
    for i=1:size(Pn,1)
        if (Pn(i,1).contains(x_curr))
            index(:,k)=i;
            break
        end
    end

    % computing control command
    if index(:,k)==1
        %terminal control law
        command = K*x_curr;
    else
        text(x_curr(1)+0.05,x_curr(2)+0.01,sprintf('k=%i',k),'FontSize',fontsize); %plot text next to the current state
        %constrained one-step evolution optimization problem
        command = commandcalculation(x_curr,Pn(index(:,k)-1,1),W,Ad,Bd,U);    
    end
    
    %random noise in the considered bound
    a=(b1-a1)*rand(1,1)+a1; 
    b=(b2-a2)*rand(1,1)+a2;
    d_evolution(:,k)=[a;b];

    %state evolution
    x_evolution(:,k)= Ad*x_curr+Bd*(command)+d_evolution(:,k) ;
    u_evolution(:,k) = command;
    
    if video_recording
        immagine=getframe(h1);
        writeVideo(vidObj,immagine);
    end

    pause(0.1) %simulation paused only for visualization pourposes
   
end
title('*** ONLINE PHASE completed ***') %changing the title to the opened figure
fprintf('ONLINE part completed! Showing now other plots...\n')

%PLOTTING
duration = size(u_evolution,2)*Ts-Ts;
Time_vec=[0:Ts:duration];

%Set membership index
h2=figure;
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]); % Enlarge figure to full screen.
plot(Time_vec,index-1,'-o','LineWidth',3,'MarkerSize',20); % -1 to show that the terminal set index is 0
title('Set Membership Index "i"')
xlabel('TIME [sec]')
set(gca,'FontSize',fontsize)


%Plot control input and constraints
h3=figure;
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]); % Enlarge figure to full screen.
plot(Time_vec,u_evolution,'color','blue','LineWidth',3);
hold on
minimum_u=model.u.min*ones(1,size(u_evolution,2));
plot(Time_vec,minimum_u,'--','color','red','LineWidth',3);
maximum_u=model.u.max*ones(1,size(u_evolution,2));
plot(Time_vec,maximum_u,'--','color','red','LineWidth',3);
legend('u','min','max')
title('input signal')
xlabel('TIME [sec]')
set(gca,'FontSize',fontsize)

%closing video recording
if video_recording
    close(vidObj);
end

fprintf('SIMULATION ended correctly!\n')
