classdef gameGaussian < Game
    %GAMEGAUSSIAN This is a concrete class defining a game where rewards a
    %   are drawn from a gaussian distribution.
    
    methods
        
        function self = gameGaussian(nbActions, totalRounds) 
            % Input
            %   nbActions - number of actions
            %   totalRounds - number of rounds of the game
            
            self.nbActions = nbActions;
            self.totalRounds = totalRounds;
            mu = rand();
            sigma = rand();
            s = -1*ones(nbActions,totalRounds);
            for i=1:nbActions*totalRounds
                s(i) = normrnd(mu,sigma);
                while s(i) < 0 || s(i) > 1
                    s(i) = normrnd(mu,sigma);
                end
            end
            self.tabR = s;
            self.N = 0;
        end
        
    end    
end

