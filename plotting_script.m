%% Script for doing some basic plotting for FIR filters

 %load test_data/straight3m_1.txt;
 %raw = straight3m_1;

%load test_data/turning_left.txt;
%raw = turning_left;

load test_data/wall_to_wall.txt
raw = wall_to_wall;

load o.txt;
len_o = length(o(:,1));

plot(raw(:,1),'b.-')
hold on
plot(o(5:len_o,1),'g.-')robot->
xlabel('Steps')
ylabel('NS x')
legend('Raw','Filtered')

figure
plot(raw(:,2),'b.-')
hold on
plot(o(5:len_o,2),'g.-')
xlabel('Steps')
ylabel('NS y')
legend('Raw','Filtered')

figure
plot(raw(:,3),'b.-')
hold on
plot(o(5:len_o,3),'g.-')
xlabel('Steps')
ylabel('NS Theta')
legend('Raw','Filtered')

figure
plot(raw(:,4),'b.-')
hold on
plot(o(5:len_o,4),'g.-')
xlabel('Steps')
ylabel('WE Delta Left')
legend('Raw','Filtered')

figure
plot(raw(:,5),'b.-')
hold on
plot(o(5:len_o,5),'g.-')
xlabel('Steps')
ylabel('WE Delta Right')
legend('Raw','Filtered')

figure
plot(raw(:,6),'b.-')
hold on
plot(o(:,6),'g.-')
xlabel('Steps')
ylabel('WE Delta Rear')
legend('Raw','Filtered')


%% Maybe plot the WE totals also?
