%% This script applies a random policy on a constant game
clc;
close all; 
clear all;

%% Get the constant game
totalRounds = 1000;
% game = gameGaussian(2,totalRounds);
% game = gameConstant();
% game = gameAdversarial();
% Latencies
% load('data/univLatencies.mat');
% game = gameLookupTable(univ_latencies,1);
% totalRounds = size(univ_latencies,2);
% Robot
load('data/plannerPerformance.mat');
game = gameLookupTable(planner_performance,0);
totalRounds = size(planner_performance,2);

%% Get a set of policies to try out
policies = {policyConstant(), policyRandom(), policyGWM(), policyEXP3(), policyUCB()};
policy_names = {'policyConstant', 'policyRandom', 'policyGWM', 'policyEXP3', 'policyUCB'};

%% Run the policies on the game
figure;
hold on;
r = zeros(length(policies),totalRounds);
a = zeros(length(policies),totalRounds);
c = zeros(totalRounds,1);
for k = 1:length(policies)
    policy = policies{k};
    game.resetGame();
    [reward, action, regret] = game.play(policy);
    fprintf('Policy: %s Reward: %.2f\n', class(policy), sum(reward));
    r(k,:) = regret;
    a(k,:) = action;
    if k==5
        c = policy.cb;
    end
end
plot(r')
legend(policy_names);
title('Regret Bound');
figure;
histA = histc(a',1:max(max(a)));
bar(histA);
% axis([.5 2.5 0 1100]);
legend(policy_names,'Location','NorthWest');
title('Actions');
% axis([-100 1100 -1 3]);
figure;
semilogx(c')
title('Upper Confidence')
legend('1','2')
xlabel('round (log scale)')

