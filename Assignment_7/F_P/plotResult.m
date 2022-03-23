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
legend( 'Slave', 'Master')

%%
figure(2)
subplot(2,2,1)
plot(out.tank1.Time', out.tank1.Data(:,1)',  'LineWidth',1.8)
title('Tank master')
subplot(2,2,2)
plot(out.tank2.Time', out.tank2.Data(:,1)',  'LineWidth',1.8)
title('Tank slave')
subplot(2,2,3)
plot(out.TL_m.Time', out.TL_m.Data(:,1)',  'LineWidth',1.8)
hold on
plot(out.PL_m.Time', out.PL_m.Data(:,1)',  'LineWidth',1.8)
title('TL vs PL master')
legend('TL', 'PL')
subplot(2,2,4)
plot(out.TL_s.Time', out.TL_s.Data(:,1)',  'LineWidth',1.8)
hold on
plot(out.PL_s.Time', out.PL_s.Data(:,1)',  'LineWidth',1.8)
legend('TL', 'PL')
title('TL vs PL slave')