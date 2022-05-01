clf;clear;clc

%Vehicle Model Initializing
VehicleModel = differentialDriveKinematics('VehicleInputs',"VehicleSpeedHeadingRate" );
initialState = [0 0 0];

f = figure(1);
f.Position = [350 270 1100 500];

% Reference Inputs
CenterRefInput = 101;
SizeRefInput = 30;

% Top View Initials
VehicleSize = [5 2.5];
VehicleCenter = initialState(1:2);  % Initial Vehicle Position
VehicleAngle =  initialState(3);    % Initial Vehicle Angle
tcurrent = 0;                       % Initial Time
BallAngle = randi([-60 60])*pi/180; % Ball Is Positioned To Be in The View of The Camera
BallCenterx = randi([10 25]);       % Distance is randomly assigned
BallCentery = BallCenterx*tan(BallAngle); 
BallCenter = [BallCenterx BallCentery];

%Extreme cases

% Q = -60, d = 50 (d = BallCenterx/cos(Q))
% % % BallAngle = -60*pi/180;
% % % BallCenterx = 25;
% % % BallCentery = BallCenterx*tan(BallAngle); 
% % % BallCenter = [BallCenterx BallCentery];

% % % % Q = 60, d = 50
% % % BallAngle = 60*pi/180;
% % % BallCenterx = 25;
% % % BallCentery = BallCenterx*tan(BallAngle); 
% % % BallCenter = [BallCenterx BallCentery];

% % % % Q = -60, d = 10
% % % BallAngle = -60*pi/180;
% % % BallCenterx = 5;
% % % BallCentery = BallCenterx*tan(BallAngle); 
% % % BallCenter = [BallCenterx BallCentery];

% % % % % Q = 60, d = 10
% % % BallAngle = 60*pi/180;
% % % BallCenterx = 5;
% % % BallCentery = BallCenterx*tan(BallAngle); 
% % % BallCenter = [BallCenterx BallCentery];


Q = VehicleAngle-BallAngle;  %Angle Error
d = sqrt((VehicleCenter(1)-BallCenter(1))^2+(VehicleCenter(2)-BallCenter(2))^2); %Distance 
plotViews(VehicleSize,0,[0 0],Q,d,BallCenter)


stepsize = 0.0125;

%Check if The Ball is Initialized inside the ROI
if Q>(-45*pi/180)&&Q<(45*pi/180)
    isInsideROI = 1;
else 
    isInsideROI = 0;
end


%Until The Ball Get Inside The ROI
while ~isInsideROI

tcurrent=tcurrent+stepsize;
tspan = (tcurrent-stepsize):stepsize:tcurrent;   

BallCenterPos = [(Q+60*pi/180)*200/(120*pi/180)+1,(d-10)*(81-60)/40+60]
BallSize = (55-5)/(10-50)*(d-10)+55

BallCenterPosError = CenterRefInput-BallCenterPos(1); %[75 -75]
BallSizeError = SizeRefInput - BallSize; % [25 -25]
    
V_idle  = 0;
w_left  =  pi/2;
w_right = -pi/2;

% Rules To Place The Ball Inside The ROI
if BallCenterPosError<101&&BallCenterPosError>74 %101-126 %Leftside of the Camera
    V = V_idle;
    w = w_left;
elseif BallCenterPosError<-74&&BallCenterPosError>-101  %175-201 %Rightside of The Camera
    V = V_idle;
    w = w_right;
else
    isInsideROI = 1;
    V =0; w = 0;
end
    
inputs =[V,w]; % [Vehicle Speed Vehicle, Angular Velocity],[V w],[m/s r/s]

%Kinematic Calculations
[t,y] = ode45(@(t,y)derivative(VehicleModel,y,inputs),tspan,initialState);

VehicleCenter = [y(end,1) y(end,2)];  %New Vehicle Center
VehicleAngle = y(end,3);              %New Vehicle Angle
BallAngle = atan2 ( BallCenter(2)-VehicleCenter(2), BallCenter(1)-VehicleCenter(1));  %New Ball Angle  

Q = VehicleAngle-BallAngle;                  %New Angle Error
d = sqrt((VehicleCenter(1)-BallCenter(1))^2+(VehicleCenter(2)-BallCenter(2))^2); %New Distance 

plotViews(VehicleSize,VehicleAngle,VehicleCenter,Q,d,BallCenter)

initialState = [VehicleCenter VehicleAngle];  %New Initial State

end

BallCenterPos = [ (Q+60*pi/180)*200/(120*pi/180)+1,(d-10)*(81-60)/40+60];
BallSize = (55-5)/(10-50)*(d-10)+55;

%Initializing some variables to store the data to plot output response at the end of the algorithm
tvalues = cell(1,1);
BallCenterC_values = cell(1,1);
BallSizeValues = cell(1,1);
ctr = 0;



% Fuzzy Control Algorithm
while round(BallCenterPos(1),4)~=101 ||  round(BallSize,4)~=30
    
ctr =ctr+1;

tcurrent=tcurrent+stepsize;
tvalues{ctr} = tcurrent; %Store the time data

BallCenterPos = [ (Q+60*pi/180)*200/(120*pi/180)+1,(d-10)*(81-60)/40+60]
BallSize      = (55-5)/(10-50)*(d-10)+55

BallSizeValues{ctr} = BallSize;             %Store the size data
BallCenterC_values{ctr} = BallCenterPos(1); %Store the position data

BallCenterPosError = CenterRefInput-BallCenterPos(1); %[75 -75]
BallSizeError = SizeRefInput - BallSize; % [25 -25]

[V,w] = fuzzyController(BallCenterPosError,BallSizeError); %Call the fuzzy controller function

tspan = (tcurrent-stepsize):stepsize:tcurrent;

inputs =[V,w]; % [Vehicle Speed Vehicle Angular Velocity],[V w],[m/s r/s]
%Kinematic Calculations
[t,y] = ode45(@(t,y)derivative(VehicleModel,y,inputs),tspan,initialState);
    
VehicleCenter = [y(end,1) y(end,2)];  %New Vehicle Center
VehicleAngle = y(end,3);              %New Vehicle Angle
BallAngle = atan2 ( BallCenter(2)-VehicleCenter(2), BallCenter(1)-VehicleCenter(1));  %New Ball Angle  

%Inputs to The Fuzzy Controller
Q = VehicleAngle-BallAngle;  %New Angle Error
d = sqrt((VehicleCenter(1)-BallCenter(1))^2+(VehicleCenter(2)-BallCenter(2))^2); %New Distance 

plotViews(VehicleSize,VehicleAngle,VehicleCenter,Q,d,BallCenter)

    
initialState = [VehicleCenter VehicleAngle];  %New Initial State
end

fprintf("\n finished in %f seconds \n",tcurrent) %Print the final time



%Plot the output responses
tvaluesa = zeros(1,size(tvalues,2));
BallCenterC_valuesa =zeros(1,size(tvalues,2));
BallSizeValuesa = zeros(1,size(tvalues,2));
for i = 1 :size(tvalues,2)
    tvaluesa(i) =tvalues{i};
    BallCenterC_valuesa(i) = BallCenterC_values{i};
    BallSizeValuesa(i) = BallSizeValues{i};
end
figure(2)
plot(tvaluesa,BallCenterC_valuesa,'LineWidth',2); hold on
title('System Response')
xlabel('time(s)')
ylabel('Ball Center Pos(pixel)')

figure(3)
plot(tvaluesa,BallSizeValuesa,'LineWidth',2); hold on
title('System Response')
xlabel('time(s)')
ylabel('Ball Size(pixel)')






function plotViews(VehicleSize,VehicleAngle,VehicleCenter,Q,d,BallCenter)

figure(1)


% Top View    
subplot(1,2,1);
axis([-50 50 -50 50])
axis square
rectangle('Position',[-50 -50 100 100],'FaceColor',[224 122 95]/255); hold on %Ground
viscircles(BallCenter,1,'Color',[128, 237, 153]/255); hold on                 %Ball
title('Top View')

VehicleCornersRelativeToCenter = flip(rot90([-VehicleSize(1)/2  +VehicleSize(2)/2 0;  
                                             +VehicleSize(1)/2  +VehicleSize(2)/2 0;
                                             +VehicleSize(1)/2  -VehicleSize(2)/2 0;
                                             -VehicleSize(1)/2  -VehicleSize(2)/2 0]));
    for j=1:4
        VehicleCornersRelativeToCenter(:,j) = RotZ(VehicleCornersRelativeToCenter(:,j),VehicleAngle);
    end
    
VehicleCorners = flip(rot90([VehicleCenter 0])) + VehicleCornersRelativeToCenter;    
Vehicle = patch(VehicleCorners(1,:),VehicleCorners(2,:),[61 64 91]/255);     %Vehicle

%Camera View
subplot(1,2,2);
axis([0 201 0 201])
axis square
rectangle('Position',[0 101 201  100],'FaceColor',[0.3010 0.7450 0.9330]);hold on %Sky
text(0,195,'Sky','FontSize',7);hold on
rectangle('Position',[0   0 201 101],'FaceColor',[224 122 95]/255);hold on        %Ground
text(0,5,'Ground','FontSize',7);hold on
rectangle('Position',[25 0 151 201],'EdgeColor','b','LineWidth',2); hold on       %ROI
text(25,195,'ROI','FontSize',7);hold on
title('Camera View')

if Q<2*pi && Q>300*pi/360
    Q2 = Q -2*pi;
else
    Q2 = Q;
end
BallCenterC = [ (Q2+60*pi/180)*201/(120*pi/180),(d-5)*(81-60)/45 + 60 ];
BallRadiusC = (55-5)/(5-50)*(d-5)+55; 

rectangle('Position',[BallCenterC(1)-BallRadiusC BallCenterC(2)-BallRadiusC BallRadiusC*2 BallRadiusC*2],...
          'Curvature',[1 1],'FaceColor',[128, 237, 153]/255);  hold on               %Ball
plot(BallCenterC(1),BallCenterC(2),'Marker','o','MarkerSize',1,'Color','b'); hold on %Ball Center
pause(0.001)
end





function p = RotZ(p_old,Q) %Rotation Matrix to Rotate The Vehicle

RotMatrix = [cos(Q) -sin(Q) 0;sin(Q) cos(Q) 0;0 0 1];

p = RotMatrix*p_old;

end
