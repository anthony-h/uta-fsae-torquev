%% Car-Suspension Variables
% They Can Change Any Moment
W= 326.58 ; % Mass of Car with Driver in Kgs
Wdf=55; % Front Weight Distribution
Wuf=48; % front Unsprung Mass at Front Unsprung Mass CG
Wur=50; % Rear Unsprung Mass at Rear Sprung Mas CG
Ws=228.61; % Sprung Mass of Car in Kg
Zwf=260.35/1000;% Front Unsprung Mass CG in mm- Input in MM-Getting Converted into M for all below values
Zwr=260.35/1000; % Rear Unsprung Mass CG in mm- Input in MM
as=760/1000; % Distance of Sprung CG from Front Axle in mm
h2=228.611/1000; % Parpendicular Distance Between Sprung Mass CG and Roll Axis
h= 292.1/1000; %CG Height in MM
tf=1244.6/1000; % Front Tread in MM
tr=1193.8/1000; % Rear Tread in MM
L=1676.4/1000;  % Wheel base in MM
b=778.9164/1000; % X-Distance of CG from front Wheel Center
c=L-b; % X-Distance of CG from rear Wheel Center
g=9.81; % Aceeleration Due to Gravity
h1= 207.518/1000; % Difference between Roll Axis heiht and CG Height Assumed
hf=81.026/1000; % Front Roll Center Height
hr=86.106/1000; % Rear Roll Center Height;
Kf=16016; % Front Roll Rate - From Excel Sheet- Will Write Program for it in Future
Kr=19253.07;% Rear Roll Rate- N-m/rad-From Excel Sheet- Will Write Program for it in Future
Kff=Kf-(((L-as)*Ws*g*h2)/L); % Kf' formula from RCVD Page 681
Krr=(Kr-((as*Ws*g*h2)/(L))); % Kr' Formula from RCVD Pages 681
DA=0;
ha=100/1000;
%% Input THis data is always availaible
a_lg=-1.0;% Longitudinal Accelration input from Outside
a_lat=1.1; %Lateral Acceleration input from accelerometer Left Turn Positive and Right Trun is Negative
tp=70; % Throttle Position input
bp=0; % Regenrative Brake Position
sfL=9000; % Front Left Motor RPM Not Wheel RPM
sfR=9000; % Front Right Motor RPM Not a Wheel RPM
srL=9000; % Rear Left Motor RPM.z`
srR=9000; % Rear Right Motor RPM.
theta_degrees= 0; %steerig angle input
%% Constants Input
Mode=1; % Mode 1 Control Diffrential  Mode 2 Insane Mode
steeringwheel_play= 0.4 ; % Steering wheel play in steering 
rack_ratio=3.31;
steeringangle_Play=(steeringwheel_play/rack_ratio)*(pi/180);
% Motor Constant Input
S_peakr=21715; % from Curve Fit rear Motor_Torque speed Curve Fit_LAR
T_peakr=44.21;    % From Curve Fit Re Motor_Torque speed Curve Fit_LAR
T_maxr_100=30;  % From Motor Simulation Data
% Front Motors
S_peakf=17674.91; % from Curve Fit Front Motor_Torque speed Curve Fit_LAR
T_peakf=25.01;    % From Curve Fit Front Motor_Torque speed Curve Fit_LAR
T_maxf_100=15;  % From Motor Simulation Data

%% Mode Selection
% In this car we can distribute the Torque propotional to weight on wheels
% proptinal to the cars weight we are calling this mode as a Mode=1, Control mode,
% In this mode car acts as if its having three differential 1) Diffrential between front left and Right tires
% 2) Second differential between rear left and right tires 3) Thirdd
% differential between Front and Rear two Tires
% If we select mode 2-Mode Insane the car acts as a Two differential , which we might
% need during the acceleration or braking to accelrate car faster in this
% mode . In this mode car acts as if it has two differefntial 1)
% Differential between front left and right tires 2) Differential between
% left and right rear tires.
% We will use mode 1 for Truning 
% Depending on Test Results we can use the Mode 2 for acceleration and
% braking 
%% If We turned the Steering Wheel and There is no Output from Lateral Accerometer
% Here we are managing if Driver want to steer and we are not reciving
% Lateral acceleration input from acclerometer then average the Velocity
% of vehicle and then Calculate Raduis of turn at CG and a_lat= V^2/R
%REMEMBER THIS IS ONLY FOR INITIAL FEW MILISECONDS TO HELP DRIVER TO TAKE TURN
%Side Note -We can Reduce this loop during program Optimization if we dont
%want to Calculate or record values of Velocity
gear_ratio=1/13;
wheel_speed_RF = sfR* gear_ratio; 
wheel_speed_LF = sfL * gear_ratio;
wheel_speed_RR = srR * gear_ratio;
wheel_speed_LR = srL * gear_ratio;
% Calculaton of Velcoity at each Wheel
Dw= 514.604/1000; %Diameter of Wheel
VfR=pi*Dw*(wheel_speed_RF/60); % Linear velocity of front right wheel in m/s
VfL=pi*Dw*(wheel_speed_LF/60); %Linear velocity of front left wheel in m/s
VrL=pi*Dw*(wheel_speed_RR/60); % Linear Velocity of rear left wheel in m/s
VrR=pi*Dw*(wheel_speed_LR/60); % Linear Velocity of reaar right wheel in m/s
% Average Velcotiy of Vehicle
V=(VfR+VfL+VrL+VrR)/4;
%% Solving coasting Problem 
if tp==0 && bp==0 
    bp= 2;
end
if tp>0
    itp=tp;
else
    itp=bp;
end
%% Estimation of Radius of Turn of Center
% Calcualltion of Steering angle at Wheels
rack_ratio= 3.31;
rCG=0;
delta_radians=(theta_degrees/rack_ratio)*(pi/180);
if delta_radians < -steeringangle_Play && a_lat==0 % Left hand Turn lateral accelration input ==0
    rLR= L/tan(abs(delta_radians)); %dist from LR wheel to turn center in m.
    rRR= tf+rLR;               %dist from RR wheel to turn center in m.
    rRF= sqrt((rLR^2)+(L^2));  %dist from RF wheel to turn center in m.
    rLF= sqrt((rRR^2)+(L^2));  %dist from LF wheel to turn center in m.
    rCG=sqrt(((rLR+tf*0.5)^2)+(c^2)); % Radius of Turn 
end
if delta_radians >= steeringangle_Play && a_lat==0 % Right hand Turn lateral accelration input ==0
    rRR= L/tan(abs(delta_radians)); %dist from RR wheel to turn center in m.
    rLR= tf+rRR;               %dist from LR wheel to turn center in m.
    rRF= sqrt((rRR^2)+(L^2));  %dist from RF wheel to turn center in m.
    rLF= sqrt((rLR^2)+(L^2));  %dist from LF wheel to turn center in m.
    rCG=-sqrt(((rRR+tf*0.5)^2)+(c^2));
end 
%% Calculation of Lateral Acceleation Input -Redudant loop
if rCG~=0 && a_lat==0
a_lat=(V^2)/(rCG*g); % Averaged Lateral G for car
end
%% Calculation of Load Transfer due to Longitudinal acceleration
wfr=(W*c-DA*ha-W*a_lg*h)/(2*L); % Weight Transfer equation for Front Individual tires during longitudinal acceleration
wrr=(W*b+DA*ha+W*a_lg*h)/(2*L); % Weight Transfer equation for Rear Individual  tires during longitudinal acceleration
%% Calculation of Fy at Front and Rear -steady State Cornering-Yaw Control 
%Fyf=2*wfr*a_lat;%2*wfr*a_lat*g; % Phase 2 code dont use it in TV1
%Fyr=2*wrr*a_lat;%2*wrr*a_lat*g; % Phase 2 code dont use it in TV1
%% Calculation of Rolbnm,.l angle Phi-Redudant-Yaw Control
%Phi= (Ws*h1*a_lat*g)/(Kf+Kr-Ws*g*h1);
%% Calculation of Load Transfer Due to Lateral Acceleration-Gilespie Book
%DFzf=(Fyf*hf+Kf*Phi)/tf; % Calculating load transfer for Front Wheel-NEED TO MODIFY THIS ACCORDING TO RCVD
%DFzr=(Fyr*hr+Kr*Phi)/tr; % Calculating load transfer for Rear Wheel-NEED TO MODIFY THIS ACCORDING TO RCVD
%% Calculation of Load Transfer Due to Lateral Acceleration-RCVD-Milleken Formulas
DFzf=a_lat*(((Ws*g/tf)*((((h2*Kff)/(Kf+Kr-Ws*g*h2))+(((L-as)/L)*hf))))+((Wuf*g*Zwf)/(tf)));%RCVD PAge 682 Formula Force in N
DFzr=a_lat*(((Ws*g/tr)*((((h2*Krr)/(Kf+Kr-Ws*g*h2))+(((as)/L)*hr))))+((Wuf*g*Zwr)/(tr))); %RCVD Page 682 Fromula Force in N
DFzf=DFzf/g;
DFzr=DFzr/g;
%% Calculation of Individual Wheel Load Considering Left Turn

if a_lat>0 % For Left Trun Scenario
wfrL=wfr-DFzf;
wfrR=wfr+DFzf;
wrrL=wrr-DFzr;
wrrR=wrr+DFzr;
end

if a_lat<0  % For Right Turn Scenario
    wfrL=wfr+DFzf;
    wfrR=wfr-DFzf;
    wrrL=wrr+DFzr;
    wrrR=wrr-DFzr;
end
if  a_lat==0 % Change
    wfrL=wfr+DFzf;
    wfrR=wfr-DFzf;
    wrrL=wrr+DFzr;
    wrrR=wrr-DFzr;
end
w=wfrL+wfrR+wrrL+wrrR; % Its Just a chek no need to include in Program
%% Calculation of Torque Availaible at each Motor
if tp>0
% For Rear Motors
%S_peakr=21715; % from Curve Fit rear Motor_Torque speed Curve Fit_LAR
%T_peakr=44.21;    % From Curve Fit Re Motor_Torque speed Curve Fit_LAR
%T_maxr_100=30;  % From Motor Simulation Data

tp=(tp/100); % Converting Throttle Position from Percentage to absolute
S_maxr= tp*S_peakr;
T_maxr=tp*T_peakr;
if srL<S_maxr
TrL=-(T_peakr/S_peakr)*srL+(T_maxr);
    if TrL>T_maxr_100 % If Calacualted Torque of Rear Left Motor is greater than Max Output Torque 
        TrL=T_maxr_100;% if yes then calculated Torque is T_maxf_100
    end
else
    TrL=0;
end
if srR<S_maxr
    %T_max=tp*T_peakr;%Problem
    TrR=-(T_peakr/S_peakr)*srR+(T_maxr);
    if TrR>T_maxr_100 % if front right calculated torque greater than Maximum capacity
        TrR=T_maxr_100; % if yes then calculated Torque is T_maxf_100
    end
else
    TrR=0;
end
 %% Calculation of Torque for Front Motor
%S_peakf=17674.91; % from Curve Fit Front Motor_Torque speed Curve Fit_LAR
%T_peakf=25.01;    % From Curve Fit Front Motor_Torque speed Curve Fit_LAR
%T_maxf_100=15;  % From Motor Simulation Data
S_maxf= tp*S_peakf; % Calculating S-axis intercept for given Throttle Position- Redudant
T_maxf=tp*T_peakf; % Calculating T axis intercept for given Throttle Position
if sfL<S_maxf
TfL=-(T_peakf/S_peakf)*sfL+(T_maxf); % Calculating Torque at given Speed
    if TfL>T_maxf_100 % if front left calculated  torque greater than Maximum capacity
        TfL=T_maxf_100; % if yes then calculated Torque is T_maxf_100
    end
else
    TfL=0;
end
if sfR<S_maxf
    TfR=-(T_peakf/S_peakf)*sfR+(T_maxf);
    if TfR>T_maxf_100 % if front right calculated torque greater than Maximum capacity
        TfR=T_maxf_100;% if yes then calculated Torque is T_maxf_100
    end
else
    TfR=0;
end
end
%% Calculation of Mototr Torques During Regenrative Breaking
if bp>0
    % For Rear Motors
    %S_peakr=21715; % from Curve Fit rear Motor_Torque speed Curve Fit_LAR
    %T_peakr=-44.21;    % From Curve Fit Re Motor_Torque speed Curve Fit_LAR
    %T_maxr_100=-30;  % From Motor Simulation Data
    bp=(bp/100); % Converting Throttle Position from Percentage to absolute
    S_maxr=bp*S_peakr;% Redudant 
    T_maxr=bp*-T_peakr;
    if srL<S_maxr
    TrL=-(-T_peakr/S_peakr)*srL+(T_maxr);
    if TrL<-T_maxr_100 % If Calacualted Torque of Rear Left Motor is greater than Max Output Torque 
        TrL=-T_maxr_100;% if yes then calculated Torque is T_maxf_100
    end
    else
        TrL=0;
    end
    %T_max=bp*T_peakr; % PROBLEM
    if srR<S_maxr
    TrR=-(-T_peakr/S_peakr)*srR+(T_maxr);
    if TrR<-T_maxr_100 % if front right calculated torque greater than Maximum capacity
        TrR=-T_maxr_100; % if yes then calculated Torque is T_maxf_100
    end
    else
        TrR=0;
    end
    %% Calculation of Torque for Front Motor
    %S_peakf=17674.91; % from Curve Fit Front Motor_Torque speed Curve Fit_LAR
    %T_peakf=-25.01;    % From Curve Fit Front Motor_Torque speed Curve Fit_LAR
    %T_maxf_100=-15;  % From Motor Simulation Data
    S_maxf= bp*S_peakf; % Calculating S-axis intercept for given Throttle Position
    T_maxf=bp*-T_peakf; % Calculating T axis intercept for given Throttle Position
    if sfL< S_maxf 
        TfL=-(-T_peakf/S_peakf)*sfL+(T_maxf); %%%%%% Calculating Torque at given Speed %Vhange calculaton defined it once only
    if TfL<-T_maxf_100 % if front left calculated  torque greater than Maximum capacity
        TfL=-T_maxf_100; % if yes then calculated Torque is T_maxf_100
    end
    else 
        TfL=0;
    end
    if sfR<S_maxf
    TfR=-(-T_peakf/S_peakf)*sfR+(T_maxf);
    if TfR<-T_maxf_100 % if front right calculated torque greater than Maximum capacity
        TfR=-T_maxf_100;% if yes then calculated Torque is T_maxf_100
    end
    else
        TfR=0;
    end
end
   
%% Distribution of Torque According to Weight Distribution
if Mode==1 % Mode Control
t=TfL+TfR+TrL+TrR; % Total absolute torue Avarege
TFL=(wfrL/w)*t; % Output Front Left Torque based on weight Ratio
TFR=(wfrR/w)*t; % Output-Front Right Torque based in Weight Ratio
TRL=(wrrL/w)*t; % Output-Rear Left Torque Based on weight Ratio
TRR=(wrrR/w)*t; % Output
end
if Mode==2 % Mode Insane
    TF=TfL+TfR; % Total absolute Torque Front
    TR=TrL+TrR; % Total absolute torue rear
    t=TF+TR;% For checking
    TFL=(wfrL/(2*wfr))*TF; % Output Front Left Torque based on weight Ratio
    TFR=(wfrR/(2*wfr))*TF; % Output-Front Right Torque based in Weight Ratio
    TRL=(wrrL/(2*wrr))*TR; % Output-Rear Left Torque Based on weight Ratio
    TRR=(wrrR/(2*wrr))*TR; % Output
end

%if tp>0
% If calcualted Torque is Above Max Possible Output Capacity of Motor
if TFL>T_maxf_100 % If Calculated required torque is greater than 100 percent Motor Capacity
   TFL=T_maxf_100; % If yes then will provide maximum Torque Availaible at Motor
end
% For Particukar Throttle Position if Calculated Torque is below zero- Negative
% Torque condition
if TFL < 0
    if TFL< -T_maxf_100 % If required Reverse Torque is Less than 100 % Reverese Capacity. Apply maximum revrse torque.
        TFL=-T_maxf_100; % Applying Maximum availaible Reverse Torque
    end
end
% If calcualted Torque is Above Max Possible Output Capacity of fornt Right Motor
if TFR>T_maxf_100 % If Calculated required torque is greater than Front Motor 100 percent Capacity
  TFR=T_maxf_100; % We will provide MAximum Torque Avaialaible
end
% For Particukar Throttle Position if Calculated Torque is 0-Negative
% Torque
% percent capacity but above motor y intercept
if TFR < 0
    if TFR< -T_maxf_100 % If required Reverse Torque is Less than 100 % Reverese Capacity. Apply maximum revrse torque.
        TFR=-T_maxf_100; % Applying Maximum availaible Reverse Torque
    end
end
    
% If calcualted Torque is Above Max Possible Output Capacity of Rear Left Motor
if TRL>T_maxr_100 % If Calculated required torque is greater than Rear Motor 100 percent Capacity
    TRL=T_maxr_100; % If yes then TRL will provide maximum Torque Availaible at that position
end
% For Particukar Throttle Position if Calculated Torque is 0
% percent capacity but above motor y intercept
if TRL < 0
    if TRL< -T_maxr_100 % If required Reverse Torque is Less than 100 % Reverese Capacity. Apply maximum revrse torque.
        TRL=-T_maxr_100; % Applying Maximum availaible Reverse Torque
    end 
end

% If calcualted Torque is Above Max Possible Output Capacity of Rear Right Motor
if TRR>T_maxr_100 % If Calculated required torque is greater than Rear Motor 100 percent Capacity
    TRR=T_maxr_100; % If yes then TRR will provide maximum Torque Availaible at that position
end
       
% For Particukar Throttle Position if Calculated Torque is below 0
% percent capacity but above motor y intercept
if TRL < 0
    if TRL< -T_maxr_100 % If Calculated required torque is greater than Rear Motor 100 percent Capacity
        TRL=-T_maxr_100; % If yes then TRR will provide maximum Torque Availaible at that position
    end
end
%end

%% Reverse Torque Exceeds Maximum Capacity of Motor

%if bp > 0
 % If calcualted Torque is Above Max Possible Output Capacity of Motor
%if TFL<T_maxf_100 % if calculated torque is more than maximum reverse capacity/numerically less
   %TFL=T_maxf_100; % If yes then will provide maximum reverse Torque Availaible at Motor
%end
% For Particukar Throttle Position if Calculated Torque is above zero-
% Positive
% Torque condition
%if TFL > 0
   % if TFL> -T_maxf_100 % If required Reverse Torque is more than 100  Capacity.
    %    TFL=-T_maxf_100; % Applying Maximum availaible Torque
    %end
%end
% Isf calcualted Torque is Above Max Possible Output Capacity of fornt Right Motor
%if TFR<T_maxf_100  % if calculated torque is more than maximum reverse capacity/numerically less
 % TFR=T_maxf_100; % If yes then will provide maximum reverse Torque Availaible at Motor
%end
% For Particukar Throttle Position if Calculated Torque is 0-Negative
% Torque
% percent capacity but above motor y intercept
%if TFR>0
    %if TFR> -T_maxf_100 % If required Reverse Torque is more than 100  Capacity.
       % TFR=-T_maxf_100; % Applying Maximum availaible Torque
    %end
%end
    
% If calcualted Torque is Above Max Possible Output Capacity of Rear Left Motor
%if TRL<T_maxr_100 % if calculated torque is more than maximum reverse capacity/numerically less
    %TRL=T_maxr_100;% If yes then will provide maximum reverse Torque Availaible at Motor
%end
% For Particukar Throttle Position if Calculated Torque is 0
% percent capacity but above motor y intercept
%if TRL > 0
   % if TRL> -T_maxr_100 % If required Reverse Torque is more than 100  Capacity.
      %  TRL=-T_maxr_100; % Applying Maximum availaible Torque
   %end 
%end

% If calcualted Torque is Above Max Possible Output Capacity of Rear Right Motor
%if TRR<T_maxr_100% if calculated torque is more than maximum reverse capacity/numerically less
   % TRR=T_maxr_100; % If yes then will provide maximum reverse Torque Availaible at Motor
%end
       
% For Particukar Throttle Position if Calculated Torque is below 0
% percent capacity but above motor y intercept
%if TRL > 0
    %if TRL> -T_maxr_100 % If required Reverse Torque is more than 100  Capacity.
      % TRL=-T_maxr_100;% Applying Maximum availaible Torque
    %end
%end
%end
%% Calculating Torque Ratio
t_fL_ratio=(TFL-TfL)/(TfL);
t_fR_ratio=(TFR-TfR)/(TfR);
t_rL_ratio=(TRL-TrL)/(TrL);
t_rR_ratio=(TRR-TrR)/(TrR);
if TFL==0;
    t_fL_ratio=0;
end
if TFR==0
   t_fR_ratio=0;
end
if TRL==0
    t_rL_ratio=0;
end
if TRR==0
    t_rR_ratio=0;
end
%% Output of Program- Percentage increase or decrease in Torque
lambda_TV_FL=(1+t_fL_ratio)*itp; % New TP/bp
lambda_TV_FR=(1+t_fR_ratio)*itp; % New Tp/bP
lambda_TV_RL=(1+t_rL_ratio)*itp; % New TP/BP
lambda_TV_RR=(1+t_rR_ratio)*itp; % New TP/BP
