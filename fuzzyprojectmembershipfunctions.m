

%Fuzzy Membersip Functions

% BallCenterPosError = CenterRefInput-BallCenterPos(1); %[101 -100]
% BallSizeError = SizeRefInput - BallSize; % [25 -25]

BallCenterPosError = -101:0.1:101;
% 5 BallCenterPosError Memberships Functions
BigLeft    =  trimf(BallCenterPosError,[ 50  75  75]);  % BigLeft    Membership Functions 
SmallLeft  =  trimf(BallCenterPosError,[  0  50  75]);  % SmallLeft  Membership Functions
BigRight   =  trimf(BallCenterPosError,[-75 -75 -50]);  % BigRight   Membership Functions
SmallRight =  trimf(BallCenterPosError,[-75 -50   0]);  % SmallRight Membership Functions
ZeroD      =  trimf(BallCenterPosError,[ -10  0  10]);  % Front      Membership Functions


plot(BallCenterPosError,BigLeft,'LineWidth',2);     hold on
plot(BallCenterPosError,SmallLeft,'LineWidth',2);   hold on
plot(BallCenterPosError,BigRight,'LineWidth',2);    hold on
plot(BallCenterPosError,SmallRight,'LineWidth',2);  hold on
plot(BallCenterPosError,ZeroD,'LineWidth',2);       hold on
xlabel(' BallCenterPosError')
legend('BigLeft ','SmallLeft','BigRight','SmallRight','ZeroD')
title('BallCenterPosError Membership Functions')
axis([-75 75 0 1])
xticks(-75:5:75);



BallSizeError = -25:0.1:25;
% 5 BallSizeError Membership Functions
BigPositive   = trimf(BallSizeError,[ 15  25  25]);   % BigPositive  Membership
Positive      = trimf(BallSizeError,[  0  15  25]);   % Positive     Membership 
Zero          = trimf(BallSizeError,[ -5   0   5]);   % Zero         Membership 
Negative      = trimf(BallSizeError,[-25 -15   0]);   % Negative     Membership 
BigNegative   = trimf(BallSizeError,[-25 -25 -15]);   % BigNegative  Membership 


figure(2)
plot(BallSizeError,BigPositive,'LineWidth',2);    hold on
plot(BallSizeError,Positive,'LineWidth',2);       hold on
plot(BallSizeError,Zero,'LineWidth',2);           hold on
plot(BallSizeError,Negative,'LineWidth',2);       hold on
plot(BallSizeError,BigNegative,'LineWidth',2);    hold on
xlabel(' BallSizeError')
legend('BigPositive','Positive','Zero','Negative','BigNegative')
title('BallSizeError Membership Functions')



V = -100:0.1:100;
% 5 V Output Memberships Functions
ForwardFast  =  trimf(V,[  50  100  100]);     % ForwardFast    Membership Functions 
Forward      =  trimf(V,[   0   50  100]);     % Forward        Membership Functions
BackwardFast =  trimf(V,[-100 -100  -50]);     % BackwardFast   Membership Functions
Backward     =  trimf(V,[-100  -50    0]);     % Backward       Membership Functions
Stop         =  trimf(V,[ -10    0   10]);     % Stop           Membership Functions


figure(3)
plot(V,ForwardFast,'LineWidth',2);     hold on
plot(V,Forward,'LineWidth',2);         hold on
plot(V,BackwardFast,'LineWidth',2);    hold on
plot(V,Backward,'LineWidth',2);        hold on
plot(V,Stop,'LineWidth',2);            hold on
xlabel('output variable V(m/s)')
legend('ForwardFast ','Forward','BackwardFast','Backward','Stop')
title('V Membership Functions')



w = -20:0.1:20;
% 5 w Output Memberships Functions
LeftFast  =  trimf(w,[ 10  20  20]);     % LeftFast    Membership Functions 
Left      =  trimf(w,[  0  10  20]);     % Left        Membership Functions
RightFast =  trimf(w,[-20 -20 -10]);     % RightFast   Membership Functions
Right     =  trimf(w,[-20 -10   0]);     % Right       Membership Functions
StopR     =  trimf(w,[-10   0  10]);     % StopR       Membership Functions


figure(4)
plot(w,LeftFast,'LineWidth',2);     hold on
plot(w,Left,'LineWidth',2);         hold on
plot(w,RightFast,'LineWidth',2);    hold on
plot(w,Right,'LineWidth',2);        hold on
plot(w,StopR,'LineWidth',2);        hold on
xlabel('output variable w(r/s)')
legend('LeftFast ','Left','RightFast','Right','StopR')
title('w Membership Functions')


