$title Resource Recharging Station Location Routing Problem

option lp = CBC,mip=cbc;

Sets i node index /1*5/

     h(i) RRSNode /2,3/
     v vehicle index /1*2/
     r resource interval index /1*6/
     minR(r) minimum resource level /1/
     t time interval index /1*6/
     alias (i,j,k)
     alias (t,t1,t2)
     alias (r,r1,r2);

*RRS capacity
parameter cap(i) capacity of every resource recharging station
/
1 0
2 2
3 2
4 0
5 0
/;


*RRS construction cost
parameter ConstructionCost(i) construction cost of every RRS|||    =100 means this is not a RRS
/
1 100
2 10
3 10
4 100
5 100
/;

*link cost
table TravelCost(i,j)  travel cost of physical links
        1        2        3        4        5
1       99       1        1        99       99
2       99       99       99       2        99
3       99       99       99       1        99
4       99       99       99       99       1
5       99       99       99       99       99
;

*resource cost
table ResourceChange(i,j)  travel cost of physical links
        1        2        3        4        5
1       99       -1       -1       99       99
2       99       99       99       4        99
3       99       99       99       2        99
4       99       99       99       99       -3
5       99       99       99       99       99

parameter Demand(i,j,t1,t2) demand link in space time network
/
4.5.5.6 1
4.5.4.5 1
/;

parameter ArcCost(i,j,t1,t2,r1,r2);
ArcCost(i,j,t1,t2,r1,r2) = 1000;

*resource space time network construction
*travel links
loop(i,
  loop(j$(TravelCost(i,j)<99),
    loop(t,
      loop(r,
        ArcCost(i,j,t,t+TravelCost(i,j), r, r + ResourceChange(i,j)) = TravelCost(i,j)
          )
        )
      )
    );

*waiting links
loop(i,
  loop(j$(ord(i)=ord(j)),
    loop(t,
      loop(r,
        ArcCost(i,i,t,t+1,r,r)= 0
      )
   )
 )
);

*destination links
ArcCost(i,i,t,t,r,'1')=0;

*depot construction budget
parameter Budget;
Budget = 21;


*vehicles parameter
parameter VehicleO(v)
/
1 1
/;

parameter VehicleD(v)
/
1 5
/;

parameter VehicleStartTime(v)
/
1 1
/;

parameter VehicleArriveTime(v)
/
1 6
/;

parameter VehicleInitialFuel(v)
/
1 3
/;

parameter VehicleTankCapacity(v)
/
1 5
/;

parameter origin(v,i,t,r) origin nodes and departure time and initial resource level
/1.1.1.2 1
2.1.1.3 1
/;

parameter destination(v,i,t,r);
*destination nodes and arrival time and final resource level
destination('1','5','6','1')=1;
destination('2','5','6','1')=1;

parameter intermediate(v,i,t,r);
intermediate(v,i,t,r)=(1-origin(v,i,t,r))*(1-destination(v,i,t,r));

binary variables
         x(v,i,j,t1,t2,r1,r2) vehicle routing variable: =1 if vehicle v use space-time link i j t1 t2 r1 r2
         w(k) depot available variable: =1 if depot k is available
;

variable z sum of total transportation j;

equation obj objective function
*        VehicleFlowBalance_Departing(v,i,t,r) vehicle flow balance constraints on super origin depot
         VehicleFlowBalance_Departing_Unic(v,i,t,r) vehicle flow balance constraints on super origin depot

*        VehicleFlowBalance_Arriving(v,i,t,r)  vehicle flow balance constraints on super destination depot
         VehicleFlowBalance_Arriving_Unic(v,j,t,r) vehicle flow balance constraints on super destination depot

         VehicleFlowBalance_InterMediateNode(v,i,t,r)  vehicle flow balance constations on other depot

         RRSCapacityConstraint(k) the usage of rrs can not exceed the rrs capacity

         DemandSatisfyingConstraints(i,j,t1,t2) all demand must be satisfied

         StationConstructionBudgetConstraint
;

*objective function
obj.. z =e= sum(v,sum((i,j,t1,t2,r1,r2),ArcCost(i,j,t1,t2,r1,r2)*x(v,i,j,t1,t2,r1,r2)))
;

*Constraints
*group 1, Flow Balance constraints
VehicleFlowBalance_Departing_Unic(v,i,t1,r1)$(origin(v,i,t1,r1))..
sum((j,t2,r2),x(v,i,j,t1,t2,r1,r2))=e=1;

*VehicleFlowBalance_Arriving(v,i,t1,r1)$(ord(i)= VehicleD(v) and ord(t1)=VehicleArriveTime(v))..
VehicleFlowBalance_Arriving_Unic(v,j,t2,r2)$(destination(v,j,t2,r2))..
sum((i,t1,r1), x(v,i,j,t1,t2,r1,r2))=e= 1;

VehicleFlowBalance_InterMediateNode(v,i,t1,r1)$(intermediate(v,i,t1,r1))..
sum((j,t2,r2),x(v,i,j,t1,t2,r1,r2))
-
sum((j,t2,r2),x(v,j,i,t2,t1,r2,r1))
=e= 0;

*group 2, rrs capacity constraints
RRSCapacityConstraint(h(k))..
sum(v,
  sum(t1,
    sum(r1,
      sum((j,t2,r2)$(ord(t1)<>ord(t2)),
         x(v,k,j,t1,t2,r1,r2)
      )
    )
  )
)
=l= w(k)*cap(k);


*group 3, demand satisfying constraints
DemandSatisfyingConstraints(i,j,t1,t2)$(ord(t1)<>ord(t2) and ord(i)<>ord(j))..
sum(v,
 sum((r1,r2),
  x(v,i,j,t1,t2,r1,r2)
 )
)=g= Demand(i,j,t1,t2);


*group 4, rrs construnction budget constraint
StationConstructionBudgetConstraint..
sum(k, ConstructionCost(k)*w(k)) =l= Budget;


Model RRSLRP /all/;

solve RRSLRP using mip minimizing z;

display x.l;











