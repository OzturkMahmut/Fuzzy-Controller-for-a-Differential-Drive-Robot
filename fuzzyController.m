function [V_value,w_value] = fuzzyController(BallCenterPosError,BallSizeError)

%Rule Base

% BallCenterPosError = CenterRefInput-BallCenterPos(1); %[75 -75]
% BallSizeError = SizeRefInput - BallSize; % [25 -25]

%5 BallCenterPosError Memberships
BigLeft    =  trimf(BallCenterPosError,[ 50  75  75]); % BigLeft    Membership  
SmallLeft  =  trimf(BallCenterPosError,[  0  50  75]); % SmallLeft  Membership 
BigRight   =  trimf(BallCenterPosError,[-75 -75 -50]); % BigRight   Membership 
SmallRight =  trimf(BallCenterPosError,[-75 -50   0]); % SmallRight Membership 
ZeroD      =  trimf(BallCenterPosError,[-10   0  10]); % Front      Membership 


%5 BallSizeError Memberships
BigPositive  = trimf(BallSizeError,[ 15  25  25]);   % BigPositive  Membership
Positive     = trimf(BallSizeError,[  0  15  25]);   % Positive     Membership 
Zero         = trimf(BallSizeError,[ -5   0   5]);   % Zero         Membership 
Negative     = trimf(BallSizeError,[-25 -15   0]);   % Negative     Membership 
BigNegative  = trimf(BallSizeError,[-25 -25 -15]);   % BigNegative  Membership 


%%Inference

membershipValuesV = [0 0 0 0 0]; %Initialize Fuzzy Set for V
membershipValuesW = [0 0 0 0 0]; %Initialize Fuzzy Set for w

%%Rule Base For Linear Velocity
if  BigPositive>0 
    V = 'ForwardFast';
    membershipValuesV(1) = BigPositive;
end
if  Positive>0 
    V = 'Forward';
    membershipValuesV(2) = Positive;
end
if  Zero>0 
    V = 'Stop';
    membershipValuesV(3) = Zero;
end
if  Negative>0 
    V = 'Backward';
    membershipValuesV(4) = Negative;
end
if  BigNegative>0 
    V = 'BackwardFast';
    membershipValuesV(5) = BigNegative;
end



%%Rule Base For Angular Velocity
if  BigLeft>0 
    w = 'LeftFast';
    membershipValuesW(1)=BigLeft;
end
if  SmallLeft>0 
    w = 'Left';
    membershipValuesW(2)=SmallLeft;
end
if  BigRight>0 
    w = 'RightFast';
    membershipValuesW(3)=BigRight;
end
if  SmallRight>0 
    w = 'Right';
    membershipValuesW(4)=SmallRight;
end
if  ZeroD>0 
    w = 'StopR';
    membershipValuesW(5)=ZeroD;
end

%Defuzzification
[V_value,w_value] = defuzz(membershipValuesV,membershipValuesW);

end



% Defuzzification Function
function [V_value,w_value] = defuzz(membershipValuesV,membershipValuesW)



%V
ForwardFastV  = membershipValuesV(1);
ForwardV      = membershipValuesV(2);
StopV         = membershipValuesV(3);
BackwardV     = membershipValuesV(4);
BackwardFastV = membershipValuesV(5);


numV = 0;
denV = 0;
for i=-101:0.5:101
    V_inc = i;
    
    ForwardFastDF  =  trimf(V_inc,[  50  100  100]);
    if ForwardFastDF>ForwardFastV; ForwardFastDF=ForwardFastV;end
    ForwardDF      =  trimf(V_inc,[   0   50  100]);   
    if ForwardDF>ForwardV; ForwardDF=ForwardV;end
    BackwardFastDF =  trimf(V_inc,[-100 -100  -50]);    
    if BackwardFastDF>BackwardFastV; BackwardFastDF=BackwardFastV;end
    BackwardDF     =  trimf(V_inc,[-100  -50    0]);
    if BackwardDF>BackwardV; BackwardDF=BackwardV;end
    StopDF        =  trimf(V_inc,[ -10    0   10]); 
    if StopDF>StopV; StopDF=StopV;end
    
    y_inc = max([ForwardFastDF ForwardDF BackwardFastDF BackwardDF StopDF]);
    
    numV = numV + y_inc*V_inc;
    denV = denV + y_inc;
end

V_value = numV/denV;


%w
LeftFastV   = membershipValuesW(1);
LeftV       = membershipValuesW(2);
RightFastV  = membershipValuesW(3);
RightV      = membershipValuesW(4);
StopRV      = membershipValuesW(5);

numW = 0;
denW = 0;

for i = -20:0.1:20
    
      w_inc = i;
          
      LeftFastDF  =  trimf(w_inc,[ 10  20  20]); 
      if LeftFastDF>LeftFastV; LeftFastDF=LeftFastV;end
      LeftDF      =  trimf(w_inc,[  0  10  20]);
      if LeftDF>LeftV; LeftDF=LeftV;end
      RightFastDF =  trimf(w_inc,[-20 -20 -10]);
      if RightFastDF>RightFastV; RightFastDF=RightFastV;end
      RightDF     =  trimf(w_inc,[-20 -10   0]);
      if RightDF>RightV; RightDF=RightV;end
      StopRDF     =  trimf(w_inc,[ -10  0   10]);
      if StopRDF>StopRV; StopRDF=StopRV;end
           
      y_inc = max([LeftFastDF LeftDF RightFastDF RightDF StopRDF]);
    
      numW = numW + y_inc*w_inc;
      denW = denW + y_inc;
    
end

w_value = numW/denW;


end