%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Name                  : plant_estimation_and_controller_design.m
% Author                : Patrick Gsoels
% Contact               : patrick.gsoels@gmx.at
% Creation Date         : 12.10.2020
% Version               : 1.1
% Description           : This script is conceived for creating an appropriate
%                         controller for a given plant transfer function.
% Language              : MATLAB R2018a
% Category              : script
% To Do                 : -
% -----------------------------------------------------------------------------
% 1:01: initial version [PG]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
close all;
clc;

%% file and path declaration

%controller_type = 'P';
%controller_type = 'PI';
controller_type = 'PID';

sim_data_file = 'measured_frequency_response_of_the_plant.txt';
modified_sim_data_file  = 'measured_frequency_response_of_the_plant_modified.txt';

[script_dir,~,~] = fileparts(mfilename('fullpath'));
sim_data_dir = fullfile(script_dir,'');

%% variable declaration

t_start = 0;
t_stop = 5;
time_step = 1e-5;
disturbance_amplitude = 0.5;
disturbance_frequency = 10;
disturbance_phase = 0;
disturbance_offset = 0.5;
step_time = 1;
step_gain = 1;

% transfer function estimation
num = 1;
den = 2;

%% modifying initial bode-diagram txt file

fh_input_file = fopen(fullfile(sim_data_dir,sim_data_file), 'r');
fh_output_file = fopen(fullfile(sim_data_dir,modified_sim_data_file), 'w');

%running through file content
while ~feof(fh_input_file)
   fline = fgetl(fh_input_file); %file row
   fline = strrep(fline,',',' ');
   fprintf(fh_output_file, '%s\r\n', fline);  
end  

%% gathering bode-diagram data

data = dlmread(fullfile(sim_data_dir,modified_sim_data_file), '', 1, 0);
 
f = data(:,1);
gain = data(:,2);
phase = data(:,3);

%% estimating the plant transfer function

fs = f(2)-f(1); % sampling frequency of the bode-diagram (equidistant-spaced frequency -> linear)
Ts = 1/fs; % sampling time

response = gain.*exp(1i*phase*pi/180); % total frequency response vector
w=2*pi*f; % angular frequency
sys=frd(response,w); % convertes the frequency response vector into a frequency-response data model
gfr=idfrd(sys); % constructs an idfrd object that stores the frequency response
warning ('off','all');
Options = tfestOptions;
Options.Focus = 'prediction';
tf_estimation=tfest(gfr,den,num,Options) % estimation of the transfer function
[numerator,denominator] = tfdata(tf_estimation,'v'); % gathering numerator and denominator coefficients of tf
%[msg,warnID] = lastwarn
warning ('on','all');
bode_option = bodeoptions;
bode_option.PhaseWrapping = 'on';
figure(1);
bodeplot(tf_estimation,w,bode_option); % create bode plot
%step(tf_estimation)

%% open-loop method from Ziegler-Nichols (analysing the step response of the plant)

[h, tsim] = step(tf_estimation, linspace(0,0.05,1e5)); % gathering step response values
dhdt = diff(h)./diff(tsim); % discrete time derivative of h
[~, idx] = max(dhdt); % maximum value of the time derivative for finding the inflection point of h

k = dhdt(idx); % slope within the inflection point
d = h(idx) - k*tsim(idx); % offset of the linear tangent equation
tangent = k*tsim+d; % linear tangent equation 

index_Tv = find( tangent <= 0, 1, 'last'); % vector index for equivalent dead time
Tv = tsim(index_Tv); % equivalent dead time
Ks = h(end); % system gain
index_Tg = find( tangent <= Ks, 1, 'last'); % vector index for build-up time
Tg = tsim(index_Tg) - Tv; % build-up time

figure(2);
hold on;
grid on, box on;
plot(tsim,h,'b',tsim(idx),h(idx),'ro',tsim(index_Tv),tangent(index_Tv),'ro');
plot(tsim, tangent, 'g');
plot(tsim, Ks*ones(1,length(h)), 'k');
plot(Tg+Tv, Ks, 'ro');
ylim([-0.2 1])
hold off;

%% controller type

if( strcmp(controller_type,'P') )
  model = 'sim_P_controller';
  KP = Tg/(Ks*Tv);
  TN = inf;
  TV = 0;
elseif( strcmp(controller_type,'PI') )
  model = 'sim_PI_controller';
  KP = 0.9*Tg/(Ks*Tv);
  TN = (3+1/3)*Tv;
  TV = 0;  
elseif( strcmp(controller_type,'PID') )
  model = 'sim_PID_controller';
  KP = 1.2*Tg/(Ks*Tv);
  TN = 2*Tv;
  TV = 0.5*Tv;
else
  disp('[ERROR]: Either choose P, PI or PID as controller-type!');
  return;
end

KI = KP/TN;
KD = KP*TV;

disp(['KP = ',num2str(KP)])
disp(['TN = ',num2str(TN)])
disp(['TV = ',num2str(TV)])
disp(['KI = ',num2str(KI)])
disp(['KD = ',num2str(KD)])

%% loading simulink file and creating the workspace with variables

% initialize workspace file
if( ~exist('workspace_simulink_params.mat', 'file') )
  fclose(fopen('workspace_simulink_params.mat','w'));
end

% loading simulink file
open_system(model,'loadonly'); 
  
% creating simulink workspace
hws = get_param(model, 'ModelWorkspace');
hws.DataSource = 'MAT-File';
hws.FileName = 'workspace_simulink_params';
hws.assignin('numerator', numerator);
hws.assignin('denominator', denominator);
hws.assignin('step_time', step_time);
hws.saveToSource;
hws.reload;
  
% open simulation model
open_system(model,'loadonly');

% get opject info
sim_setting = Simulink.SimulationInput(model);

% setting parameter via blockParameter assignment
if( strcmp(controller_type,'P') )
  sim_setting = sim_setting.setBlockParameter([model,'/KP_gain'],'Gain',num2str(KP));
elseif( strcmp(controller_type,'PI') )
  sim_setting = sim_setting.setBlockParameter([model,'/KP_gain'],'Gain',num2str(KP));
  sim_setting = sim_setting.setBlockParameter([model,'/KI_gain'],'Gain',num2str(KI));
elseif( strcmp(controller_type,'PID') )
  sim_setting = sim_setting.setBlockParameter([model,'/KP_gain'],'Gain',num2str(KP));
  sim_setting = sim_setting.setBlockParameter([model,'/KI_gain'],'Gain',num2str(KI));
  sim_setting = sim_setting.setBlockParameter([model,'/KD_gain'],'Gain',num2str(KD));
else
  disp('[ERROR]: Either choose P, PI or PID as controller-type!');
  return;
end

% setting simulation specific parameters
sim_setting = sim_setting.setModelParameter('SolverType', 'Fixed-step',...
                                            'Solver', 'ode4',...
                                            'StartTime', num2str(t_start),...
                                            'StopTime', num2str(t_stop),...
                                            'FixedStep', num2str(time_step));                         
                                                    
% load previously generated simulink-workspace
hws = get_param(bdroot, 'ModelWorkspace');
hws.DataSource = 'MAT-File';
hws.FileName = 'workspace_simulink_params';
hws.reload;

% start simulink simulation
simulation = sim(sim_setting);

% get simulation data
t_s = simulation.get('t_sim');  
disturbance_s = simulation.get('disturbance_sim');
y_s = simulation.get('y_sim');
r_s = simulation.get('r_sim');
e_s = simulation.get('e_sim');

% close simulation model
close_system(model);

figure(3);
hold on;
grid on, box on;
plot(t_s,r_s,'b');
plot(t_s, y_s, 'k');
plot(t_s, e_s, 'g');
plot(t_s, disturbance_s, 'r');
ylim([-1.5*max(r_s) 1.5*max(r_s)])
hold off;

