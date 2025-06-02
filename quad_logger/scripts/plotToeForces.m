function plotToeForces(state, toeForces)

figure
labels = {'x (N)', 'y (N)', 'z (N)'};
for ii = 1:3
    subplot(3,1,ii)
    plot(state.time, toeForces(:,ii + (0:3:9)))
    ylabel(labels(ii));
end
xlabel('Time (s)')
subplot(3,1,1)
title('Naively estimated foot forces')

figure
hold on
for ii = 1:3
    plot(state.time, sum(toeForces(:,ii + (0:3:9)),2))
end
xlabel('Time (s)')
title('Sum of naively estimated foot forces')