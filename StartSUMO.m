clear all
close all

format compact
clc

javaaddpath('traci4matlab.jar')
projectPath = 'C:\Queue_Estimation\config.sumocfg';
output = 'C:\Queue_Estimation\results.xml';

try
system(['sumo-gui' ' -c ' projectPath ' --remote-port 8813'    '  --netstate-dump ' output ' --step-length    0.1'   ' --start &' ]);
catch err
end

%initialization
[traciVersion,sumoVersion] = traci.init()

traci.gui.setSchema('View #0', 'real world')

D = [];
Sp = [];
i = 1;
while i <= 120*10 + 1
    traci.simulationStep();
    %pause(0.001)

    vehicles= traci.vehicle.getIDList();
    for ii=1:length(vehicles)
        
        
        traci.vehicle.setSpeedMode(cell2mat(vehicles(ii)),0);
        %traci.vehicle.setSpeed(cell2mat(vehicles(ii)),20);
        
        %Get the leader of the vehicle along the edge
        leader = traci.vehicle.getLeader(cell2mat(vehicles(ii))); 
        D(ii,i) = traci.vehicle.getDistance(cell2mat(vehicles(ii)));
        Sp(ii,i) = traci.vehicle.getSpeed(cell2mat(vehicles(ii)));
        
        if((~isempty(leader)) )
           %Get the position of the leader
           l = traci.vehicle.getPosition(leader);
           
           %Get the distance from the leader
           dist = traci.vehicle.getDrivingDistance2D(cell2mat(vehicles(ii)),l(1), l(2));

           %traci.vehicle.getSpeed(cell2mat(vehicles(ii)));
           
           if(dist <= 10) %stop
               speed = traci.vehicle.getSpeed(leader);
           elseif((dist >= 50) && (traci.vehicle.getSpeed(cell2mat(vehicles(ii))) == 0)) %car starts
               speed = 10;
           else
               speed = 10;
           end
           
        else
           speed = 10;
        end

        %Get the next TLS
        nextTLS = traci.vehicle.getNextTLS(cell2mat(vehicles(ii)));
        
        if(~isempty(nextTLS))
            
            %Get the distance from next TLS
            distanceToNextTLS = nextTLS{1,1}{1,3};
            %Get the state of next TLS
            stateOfNextTLS = nextTLS{1,1}{1,4};

            if ((distanceToNextTLS <=4) && (stateOfNextTLS == 'r'))
                traci.vehicle.setSpeed(cell2mat(vehicles(ii)),0);
            elseif((distanceToNextTLS <=10) && (distanceToNextTLS >4))
                traci.vehicle.setSpeed(cell2mat(vehicles(ii)),speed/2);
            elseif((distanceToNextTLS <=4) && (stateOfNextTLS == 'g'))
                traci.vehicle.setSpeed(cell2mat(vehicles(ii)),speed);
            else
                traci.vehicle.setSpeed(cell2mat(vehicles(ii)),speed);

            end
        end
     
    end
    
    i=i+1;

end
save('distances.mat', 'D');
save('speeds.mat', 'Sp');

traci.close();
system('Taskkill /IM sumo-gui.exe /F');