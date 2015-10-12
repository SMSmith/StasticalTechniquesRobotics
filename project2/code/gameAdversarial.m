classdef gameAdversarial<Game
    %GAMEADVERSARIAL This is a concrete class defining a game where rewards
    %   are adversarially chosen.

    methods
        
        function self = gameAdversarial()
            self.nbActions = 2;
            self.totalRounds = 1000;
            
            self.tabR = ones(2,1000);
            for i=1:3:self.totalRounds
                self.tabR(i) = 2;
            end                
            
            self.N = 0;
        end
        
    end    
end

