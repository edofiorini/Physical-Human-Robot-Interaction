%% Plot the Velocity

figure(1);
plot(out.velocity.Time',out.velocity.Data(:,1)', 'LineWidth',1.8);
hold on
plot(out.velocity.Time',out.velocity.Data(:,2)', 'LineWidth',1.8);
title('position');xlabel('Time');
legend( 'Master', 'Slave')