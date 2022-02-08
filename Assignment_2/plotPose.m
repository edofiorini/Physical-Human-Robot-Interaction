%% Plot the pose

figure(1);

plot(out.position.Time',out.position.Data(:,1)', 'LineWidth',1.8);
hold on
plot(out.position.Time',out.position.Data(:,2)', 'LineWidth',1.8);
hold on
plot(out.position.Time',out.position.Data(:,3)', 'LineWidth',1.8);
title('position');xlabel('Time');
legend('Reference', 'Master', 'Slave')