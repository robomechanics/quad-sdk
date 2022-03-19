success_rate = [100, 32, 0, 0, 0;
    100, 100, 96, 72, 56;
    98, 98, 94, 74, 44];
step_height = [20, 25, 30, 35, 40];

figure
hold on
plot(step_height, success_rate(1, :),'DisplayName', 'Vanilla Leg')
plot(step_height, success_rate(2, :),'DisplayName', 'Proprioception Leg')
plot(step_height, success_rate(3, :),'DisplayName', 'Proprioception Leg and Tail')
hold off
legend('Location', 'best')
xlabel('Step Height (cm)')
ylabel('Success Rate (%)')
xticks(step_height)
xticklabels(string(step_height))