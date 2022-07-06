function [move,mem] = robostrategy_mine(env,mem)
%%Dor Agababa Id208133116 & David Sumhayev ID209959220
%Strategy for robot tournament game, following opponent
%
%Environment Struct
% field:
% info,  STRUCT{team, fuel, myPos, oppPos}
% basic, STRUCT{walls, rRbt, rMF, lmax}
% mines, STRUCT{nMine, mPos, mineScr, mineExist}
% fuels, STRUCT{nFuel, fPos, fuelScr, fuelExist}
%
%Memory Struct
% field:
% path, STRUCT{start, dest, pathpt, nPt, proc, lv}
%  fuelsPositions=env.fuels.fPos;

RealTimeFuelPos=env.fuels.fPos(env.fuels.fExist==1,:);%%checks the available fuels
RealTimeBombPos=env.mines.mPos(env.mines.mExist==1,:);%%checks the available bombs
[BombsXY,~,BombsXY2]=GetNearBomb(RealTimeBombPos,env.info.myPos);%%return the 2 nearest available fuels

if((env.fuels.fExist==1)==0)%%if there is no fuel dont move
move=[0,0];
 return;
end

[FuXY]=GetNearFuel(RealTimeFuelPos,env.info.myPos);%% return the near fuel 
if(Getdistace(env.info.myPos(1)-env.info.opPos(1),env.info.myPos(2)-env.info.opPos(2))<1.5 && env.info.fuel<env.info.fuel_op)%%if there is enemy near to me and i have less fuel 
    NewPos=enemyDodge(env.info.myPos,env.info.opPos,env,FuXY);%%calculate to where to go ,run or go to fuel
move=[NewPos(1),NewPos(2)];
return;
end

 if(BombOnTheWay(BombsXY,env.info.myPos,FuXY)==1 )%% if there is bomb on the way move away
     NewPos=dodge(env.info.myPos,BombsXY,env,FuXY);
    move=[NewPos(1),NewPos(2)];
 end
if( BombOnTheWay(BombsXY2,env.info.myPos,FuXY)==1)%% if there is bomb on the way move away
     NewPos=dodge(env.info.myPos,BombsXY2,env,FuXY);
    move=[NewPos(1),NewPos(2)];
end
if(BombOnTheWay(BombsXY,env.info.myPos,FuXY)==0 && BombOnTheWay(BombsXY2,env.info.myPos,FuXY)==0)%%if there is not any bombs so go directly to the fuel
move=[FuXY(1)-env.info.myPos(1),FuXY(2)-env.info.myPos(2)];
end



end

function [movee]=dodge (MyPos,BombPos,env,fuelDest)
VectorX=[MyPos(1),BombPos(1)];
VectorY=[MyPos(2),BombPos(2)];
%%    1.4*env.basic.lmax
p=polyfit(VectorX,VectorY,1);%%calculate the linear equation from me to the bomb
m=-1/p(1);%%take the opposite 90 degree
x0=MyPos(1);y0=MyPos(2);
X1dodge=sqrt(((0.19)^2)/(1+m^2))+x0;%% a calulation of x that may should be the right one
Y1dodge=m*(X1dodge-x0)+y0;%%put the x in the polyfit we made

X2dodge=-1*sqrt(((0.19)^2)/(1+m^2))+x0;%% a calulation of x that may should be the right one
Y2dodge=m*(X2dodge-x0)+y0;%%put the x in the polyfit we made
if(X1dodge>10||X1dodge< 0) 
    X1dodge=round(abs(X1dodge));
end
if(X2dodge>10||X2dodge< 0) %% take care of limits
    X2dodge=round(abs(X2dodge));
end
if(Y1dodge>10||Y1dodge< 0) %% take care of limits
    Y1dodge=round(abs(Y1dodge));
end
if(Y2dodge>10||Y2dodge< 0) %% take care of limits
    Y2dodge=round(abs(Y2dodge));
end
RealTimeBombPos=env.mines.mPos(env.mines.mExist==1,:);%%checks the available bombs
[BombsXY,~,BombsXY2]=GetNearBomb(RealTimeBombPos,[X2dodge,Y2dodge]);%%check for each bomb the 2 near bombs
[BombsXY3,~,BombsXY4]=GetNearBomb(RealTimeBombPos,[X1dodge,Y1dodge]);%%check for each bomb the 2 near bombs
if(Getdistace(fuelDest(1)-X1dodge,fuelDest(2)-Y1dodge)>Getdistace(fuelDest(1)-X2dodge,fuelDest(2)-Y2dodge))%%check wich one is the nearest to the destination 
    if(Getdistace(X2dodge-BombsXY(1),Y2dodge-BombsXY(2))> 0.5 && Getdistace(X2dodge-BombsXY2(1),Y2dodge-BombsXY2(2))> 0.5 && ~BombOnTheWay(BombsXY,[X2dodge,Y2dodge],fuelDest) && ~BombOnTheWay(BombsXY2,[X2dodge,Y2dodge],fuelDest))
        movee=[X2dodge-MyPos(1),Y2dodge-MyPos(2)];%% take the nearest and the one with no any near bomb and no bomb on the way to dest
    else
        if(Getdistace(X1dodge-BombsXY3(1),Y1dodge-BombsXY3(2))> 0.5 && Getdistace(X1dodge-BombsXY4(1),Y1dodge-BombsXY4(2))> 0.5 &&~BombOnTheWay(BombsXY3,[X1dodge,Y1dodge],fuelDest) && ~BombOnTheWay(BombsXY4,[X1dodge,Y1dodge],fuelDest))
            movee=[X1dodge-MyPos(1),Y1dodge-MyPos(2)];%%take the last nearest but with no bombs on the way
        else
            movee=[X2dodge-MyPos(1),Y2dodge-MyPos(2)];%% take nearest even if he got bombs
        end
    end
else
if(Getdistace(X1dodge-BombsXY3(1),Y1dodge-BombsXY3(2))> 0.5 && Getdistace(X1dodge-BombsXY4(1),Y1dodge-BombsXY4(2))> 0.5 && ~BombOnTheWay(BombsXY3,[X1dodge,Y1dodge],fuelDest) && ~BombOnTheWay(BombsXY4,[X1dodge,Y1dodge],fuelDest))
movee=[X1dodge-MyPos(1),Y1dodge-MyPos(2)];%% take the nearest and the one with no any near bomb and no bomb on the way to dest
else
    if(Getdistace(X2dodge-BombsXY(1),Y2dodge-BombsXY(2))> 0.5 && Getdistace(X2dodge-BombsXY2(1),Y2dodge-BombsXY2(2))> 0.5 && ~BombOnTheWay(BombsXY,[X2dodge,Y2dodge],fuelDest) && ~BombOnTheWay(BombsXY2,[X2dodge,Y2dodge],fuelDest))
            movee=[X2dodge-MyPos(1),Y2dodge-MyPos(2)];%%take the last nearest but with no bombs on the way
    else
           movee=[X1dodge-MyPos(1),Y1dodge-MyPos(2)];%% take nearest even if he got bombs
    end
end
end
end

%%return if the nearest bomb is on the way to the near fuel
function BombOnTheway = BombOnTheWay(NearBomb,mypos,fuelsDest)
VectorX=[mypos(1),fuelsDest(1)];
VectorY=[mypos(2),fuelsDest(2)];

p=polyfit(VectorX,VectorY,1);%%calculate the linear equation from me to the fuel
%%take assume that radius is 0.5
XonLineBom=((NearBomb(2)-p(2))/p(1));%%find the x that should be equal to the x bomb
if(abs(XonLineBom-NearBomb(1))<0.65&&Getdistace(mypos(1)-NearBomb(1),mypos(2)-NearBomb(2))<0.7)%%check if the x of the bomb is near to the point on the linear way we calculate on p
BombOnTheway=1;
return;
end

if(((NearBomb(2)<= p(1)*NearBomb(1)+p(2)+0.65)&& (NearBomb(2)>= p(1)*NearBomb(1)+p(2)-0.65)))%%check if the y of the bomb is between 0.6>y>-0.6 of the y of the linear way to the fuel
  
   if(Getdistace(mypos(1)-NearBomb(1),mypos(2)-NearBomb(2))<Getdistace(mypos(1)-fuelsDest(1),mypos(2)-fuelsDest(2)))%% check if the bomb on the way and not behinde
       if(Getdistace(NearBomb(1)-fuelsDest(1),NearBomb(2)-fuelsDest(2))<Getdistace(mypos(1)-fuelsDest(1),mypos(2)-fuelsDest(2)))%% check if the bomb on the way and not behinde
    BombOnTheway=1;
    return;
       end
    end
else
    BombOnTheway=0;
    return;
end
BombOnTheway=0;
end

%%return the two nearest bombs
function [BombPos,Value,BombPos2]= GetNearBomb(BombsPositions,mypos)
VectorDeltaX=BombsPositions(:,1)-mypos(1);%% get vector x of the delta x between me and bombs
VectorDeltaY=BombsPositions(:,2)-mypos(2);%% get vector y of the delta y between me and bombs
VectorDistance = Getdistace(VectorDeltaX,VectorDeltaY);%% create vector of distance between me and the bombs
Value=  min(min(VectorDistance));%% give me the value of the minimal one (distance)
Index=find(VectorDistance==Value); 
BombPos=BombsPositions(Index,:);%% return the position of the bombs from the original array
VectorDistance(Index)=1000;%% put temp value that the min one will not disturb me
Value2=  min(min(VectorDistance));%%give me the second value of the minimal one (distance)
Index2=find(VectorDistance==Value2); %% return the position of the bombs from the original array
BombPos2=BombsPositions(Index2,:);
VectorDistance(Index)=Value;%% return the min value originaly
end
%%give me the nearest fuel
function [FuelPos]= GetNearFuel(fuelsPositions,mypos)
VectorDeltaX=fuelsPositions(:,1)-mypos(1);%% get vector x of the delta x between me and fuels
VectorDeltaY=fuelsPositions(:,2)-mypos(2);%% get vector y of the delta y between me and fuels
VectorDistance = Getdistace(VectorDeltaX,VectorDeltaY);%% create vector of distance between me and the fuels
Value=  min(min(VectorDistance));%% give me the value of the minimal one (distance)
Index=find(VectorDistance==Value); 
FuelPos=fuelsPositions(Index,:);%% return the position of the bombs from the original array
end
%%simply return the distance between two points
function distance = Getdistace(x,y)
distance = sqrt(x.^2 + y.^2);
end


function [movee]=enemyDodge (MyPos,EnemyPos,env,fuelDest)
VectorX=[MyPos(1),EnemyPos(1)];%%vector x
VectorY=[MyPos(2),EnemyPos(2)];

p=polyfit(VectorX,VectorY,1);
m=p(1);%% give me the m for the linear way from me to the enemy
%%checks when should i need to go positive or negative x
x0=MyPos(1);y0=MyPos(2);
X1dodge=env.basic.lmax+x0;
Y1dodge=m*(X1dodge)+p(2);

X2dodge=x0-env.basic.lmax;
Y2dodge=m*(X2dodge)+p(2);
if(X1dodge>10||X1dodge< 0) 
    X1dodge=round(abs(X1dodge));
end
if(X2dodge>10||X2dodge< 0) %% take care of limits
    X2dodge=round(abs(X2dodge));
end
if(Y1dodge>10||Y1dodge< 0) %% take care of limits
    Y1dodge=round(abs(Y1dodge));
end
if(Y2dodge>10||Y2dodge< 0) %% take care of limits
    Y2dodge=round(abs(Y2dodge));
end
    

if(Getdistace(fuelDest(1)-MyPos(1),fuelDest(2)-MyPos(2))<3 && Getdistace(EnemyPos(1)-fuelDest(1),EnemyPos(2)-fuelDest(2))>Getdistace(MyPos(1)-fuelDest(1),MyPos(2)-fuelDest(2)))%%if you got an a fuel near to you more than your enemy near to it , take it
        movee=[fuelDest(1)-MyPos(1),fuelDest(2)-MyPos(2)];
        return;
end
if(Getdistace(X1dodge-EnemyPos(1),Y1dodge-EnemyPos(2))>Getdistace(X2dodge-EnemyPos(1),Y2dodge-EnemyPos(2)))%%run to the way that your enemy will be more far away 
    movee=[X1dodge-MyPos(1),Y1dodge-MyPos(2)];       
else
    movee=[X2dodge-MyPos(1),Y2dodge-MyPos(2)];
end
end


