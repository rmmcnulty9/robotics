%% Script for doing some basic plotting for FIR filters

load test_data/straight3m_1.txt;
load o.txt;

plot(straight3m_1(:,1),'b.-')
hold on
plot(o(:,1),'g.-')
xlabel('Steps')
ylabel('NS x')
legend('Raw','Filtered')

figure
plot(straight3m_1(:,2),'b.-')
hold on
plot(o(:,2),'g.-')
xlabel('Steps')
ylabel('NS y')
legend('Raw','Filtered')

figure
plot(straight3m_1(:,3),'b.-')
hold on
plot(o(:,3),'g.-')
xlabel('Steps')
ylabel('NS Theta')
legend('Raw','Filtered')

figure
plot(straight3m_1(:,4),'b.-')
hold on
plot(o(:,4),'g.-')
xlabel('Steps')
ylabel('WE Delta Left')
legend('Raw','Filtered')

figure
plot(straight3m_1(:,5),'b.-')
hold on
plot(o(:,5),'g.-')
xlabel('Steps')
ylabel('WE Delta Right')
legend('Raw','Filtered')

figure
plot(straight3m_1(:,6),'b.-')
hold on
plot(o(:,6),'g.-')
xlabel('Steps')
ylabel('WE Delta Rear')
legend('Raw','Filtered')

%% 'Press any key to close all Figures'
pause
close all
%% Maybe plot the WE totals also?
