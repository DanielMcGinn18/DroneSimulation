%% System Mass Validation
% Run Simulation TorqueLimit.slx
gravity = 9.8;
smallSphereMass = .1; largeSphereMass = 1.6; rotorMass = .02; % in kg
quadrotorMass = largeSphereMass+4*smallSphereMass+4*rotorMass;
torqueMax = .12; % in N*m
propRotVel = max(Prop1rotVel.Data); % Rotor Angular velocity when max torque is applied for some time
maxLift = ((2.08)*(9.80665)*propRotVel*abs(propRotVel)/4/(200^2));
theoreticalAcc = (4*maxLift/quadrotorMass)-gravity;
maxAccLine(1:length(droneZacc.Data)) = theoreticalAcc;
figure(1); plot(droneZacc.Time,droneZacc.Data,droneZacc.Time,maxAccLine);
legend('Acceleration from Simulation','Theoretical Acceleration Limit', ...
    'Position',[0.7 0.78 0.1 0.05]);
xlabel('Time (s)'); ylabel('Acceleration (m/s^2)')
title('Verification of System Mass');