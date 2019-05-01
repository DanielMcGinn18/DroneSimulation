figure(1);
plot(rotorTorque1.Time,rotorTorque1.Data,rotorTorque2.Time,rotorTorque2.Data,rotorTorque3.Time,rotorTorque3.Data,rotorTorque4.Time,rotorTorque4.Data);
title('Rotor Torque');xlabel('Time (s)');ylabel('Torque (Nm)');
legend('Rotor 1','Rotor 2','Rotor 3','Rotor 4');

figure(2);
plot(rotorTorque1.Time,rotorTorque1.Data,rotorTorque2.Time,abs(rotorTorque2.Data),rotorTorque3.Time,rotorTorque3.Data,rotorTorque4.Time,abs(rotorTorque4.Data));
title('Absolute Value of Rotor Torque');xlabel('Time (s)');ylabel('Torque (Nm)');
legend('Rotor 1','Rotor 2','Rotor 3','Rotor 4'); ylim([.079 .121]);

figure(3);
plot3(droneX.Data,droneY.Data,droneZ.Data);
hold on; plot3(0,0,0,'o','Color','g'); 
plot3(5,0,0,'o','Color','r'); plot3(10,0,2,'o','Color','r');
plot3(35,0,0,'o','Color','r');plot3(35,10,5,'o','Color','r'); hold off;
title('Done Path Through Obstacle Course');
legend('Drone Position','Starting Point','Hoop Positions');
xlabel('Position in X');ylabel('Position in Y');zlabel('Position in Z');