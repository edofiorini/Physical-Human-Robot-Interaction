%% Plot the pose

figure(1);

subplot(3,1,1)
plot(out.position.Time',out.position.Data(:,1)', 'LineWidth',1.8);
hold on
plot(out.position.Time',out.position.Data(:,2)', 'LineWidth',1.8);
hold on
plot(out.position.Time',out.position.Data(:,3)', 'LineWidth',1.8);
title('position');xlabel('Time');
legend('Reference', 'Master', 'Slave')
subplot(3,1,2)
plot(out.velocity.Time',out.velocity.Data(:,1)', 'LineWidth',1.8);
hold on
plot(out.velocity.Time',out.velocity.Data(:,2)', 'LineWidth',1.8);
title('Velocity');xlabel('Time');
legend( 'Master', 'Slave')
subplot(3,1,3)
plot(out.force.Time',out.force.Data(:,1)', 'LineWidth',1.8);
hold on
plot(out.force.Time',out.force.Data(:,2)', 'LineWidth',1.8);
title('Force');xlabel('Time');
legend( 'Master', 'Slave')