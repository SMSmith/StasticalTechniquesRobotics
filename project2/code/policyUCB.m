classdef policyUCB < Policy
    %POLICYUCB This is a concrete class implementing UCB.

        
    properties
        % Member variables
        nbActions
        S
        C
        lastAction
        t
        cb
    end
    
    methods
        function init(self, nbActions)
            % Initialize
            self.nbActions = nbActions;
            self.S = zeros(self.nbActions,1);
            self.C = zeros(self.nbActions,1);
            self.lastAction = 1;
            self.t = 0;
        end
        
        function action = decision(self)
            % Choose action
            if self.t < self.nbActions
                action = self.t+1;
            else
                self.cb = [self.cb self.S./self.C+sqrt(log(self.t)/2/self.C)'];
                [~,action] = max(self.S./self.C+sqrt(log(self.t)/2/self.C)');
            end
            self.lastAction = action;
            self.t = self.t+1;
        end
        
        function getReward(self, reward)
            % Update ucb
            self.S(self.lastAction) = self.S(self.lastAction) + reward;
            self.C(self.lastAction) = self.C(self.lastAction)+ 1;
        end        
    end

end
