function [N_SIM, T_SIM, T, data] = read_log(name)
%% Read log file and create simulation variables


data = readtable(name,"ReadVariableNames",1,"NumHeaderLines",0);

% store acceleration into an array
data.acc = [data.acc_0,data.acc_1,data.acc_2];

N_SIM = length(data.time);
T_SIM = 1: N_SIM;
T = 0.016;

end