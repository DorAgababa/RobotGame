function [move,mem] = robostrategy_nomove(~,mem)
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
fuelsPositionss=env.fuels.fPos;
move = [0 0];
[Xfu,Yfu]=GetNearFuel(fuelsPositionss,env.info.myPos);
display(Xfu,Yfu);
end

function [Xfuel ,Yfuel]= GetNearFuel(fuelsPositions,mypos)
VectorDeltaX=fuelsPositions(:,1)-mypos(1);
VectorDeltaY=fuelsPositions(:,2)-mypos(2);
VectorDistance = Getdistace(VectorDeltaX,VectorDeltaY);
FuelPos =  min(min(VectorDistance));
Xfuel=FuelPos(1);
Yfuel=FuelPos(2);
end

function distance = Getdistace(x,y)
distance = sqrt(x.^2 + y.^2);
end
