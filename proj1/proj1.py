# -*- coding: utf-8 -*-
import random
import datetime
import sys
import os

graphStuff=True
try:
	import matplotlib.pyplot as plt
except:
	graphStuff=False

def main():
	# How many exerts are we using (the first 3 are the ones specified in the assignment)
	numberOfExperts = 7

	# How many environments are we using (the first 3 specified in the assignment)
	natures = 3

	# Set the reduction factor
	beta = 0.5

	# Observations
	start = datetime.datetime(2015,9,28,0,0)
	now = datetime.datetime.now()
	elapsed = (now-start).total_seconds()
	numLines = sum(1 for line in open(os.path.basename(__file__)))
	windows = sys.platform=='win32'

	observations = (elapsed, numLines, windows)

	# Run T trials
	T = [i for i in range(100)]
	R,Rr,L,Lr = runTrials(numberOfExperts,natures,beta,T,observations)

	if graphStuff:
		# Plot Regret for WMA
		plt.plot(T,R[0],'b.',T,R[1],'g.',T,R[2],'r.')
		plt.legend(['Stochastic Environment','Deterministic Environment','Adversarial Environment'],loc=2)
		plt.title('Weigted Majority Regret - Eta=0.5')
		plt.ylabel('Regret')
		plt.xlabel('Trial')
		plt.show()

		# Plot Loss for WMA
		plt.plot(T,L[0],'b.',T,L[1],'g.',T,L[2],'r.')
		plt.legend(['Stochastic Environment','Deterministic Environment','Adversarial Environment'],loc=2)
		plt.title('Weigted Majority Loss - Eta=0.5')
		plt.ylabel('Loss')
		plt.xlabel('Trial')
		plt.show()

		# Plot Regret for RWMA
		plt.plot(T,Rr[0],'b.',T,Rr[1],'g.',T,Rr[2],'r.')
		plt.legend(['Stochastic Environment','Deterministic Environment','Adversarial Environment'],loc=2)
		plt.title('Randomized Weighted Majority Regret - Eta=0.5')
		plt.ylabel('Regret')
		plt.xlabel('Trial')
		plt.show()

		# Plot Loss for RWMA
		plt.plot(T,Lr[0],'b.',T,Lr[1],'g.',T,Lr[2],'r.')
		plt.legend(['Stochastic Environment','Deterministic Environment','Adversarial Environment'],loc=2)
		plt.title('Randomized Weigted Majority Loss - Eta=0.5')
		plt.ylabel('Loss')
		plt.xlabel('Trial')
		plt.show()

		# Run for smaller beta
		beta = 0.25
		R,Rr,L,Lr = runTrials(numberOfExperts,natures,beta,T,observations)

		# Regret RWMA
		plt.plot(T,Rr[0],'b.',T,Rr[1],'g.',T,Rr[2],'r.')
		plt.legend(['Stochastic Environment','Deterministic Environment','Adversarial Environment'],loc=2)
		plt.title('Randomized Weighted Majority Regret - Eta=0.25')
		plt.ylabel('Regret')
		plt.xlabel('Trial')
		plt.show()

		# Loss RWMA
		plt.plot(T,Lr[0],'b.',T,Lr[1],'g.',T,Lr[2],'r.')
		plt.legend(['Stochastic Environment','Deterministic Environment','Adversarial Environment'],loc=2)
		plt.title('Randomized Weigted Majority Loss - Eta=0.25')
		plt.ylabel('Loss')
		plt.xlabel('Trial')
		plt.show()

		# And even smaller beta
		beta = 0.1
		R,Rr,L,Lr = runTrials(numberOfExperts,natures,beta,T,observations)

		# Regret RWMA
		plt.plot(T,Rr[0],'b.',T,Rr[1],'g.',T,Rr[2],'r.')
		plt.legend(['Stochastic Environment','Deterministic Environment','Adversarial Environment'],loc=2)
		plt.title('Randomized Weighted Majority Regret - Eta=0.1')
		plt.ylabel('Regret')
		plt.xlabel('Trial')
		plt.show()

		# Loss WMA
		plt.plot(T,Lr[0],'b.',T,Lr[1],'g.',T,Lr[2],'r.')
		plt.legend(['Stochastic Environment','Deterministic Environment','Adversarial Environment'],loc=2)
		plt.title('Randomized Weigted Majority Loss - Eta=0.5')
		plt.xlabel('Trial')
		plt.ylabel('Loss')
		plt.show()

def runTrials(numberOfExperts,natures,beta,T,o):
	# Scores of the WMA algorithm for each nature
	WMAS = [0 for i in range(natures)]
	RWMAS = [0 for i in range(natures)]

	# Set up the weights (initialize to 1) for all of the experts and environments
	W = [[1.0 for j in range(numberOfExperts)] for i in range(natures)]
	Wr = [[1.0 for j in range(numberOfExperts)] for i in range(natures)]

	# Loss for each expert in each environment
	S = [[0 for j in range(numberOfExperts)] for i in range(natures)]
	Sr = [[0 for j in range (numberOfExperts)] for i in range(natures)]

	# Set the number of rounds and cycle through
	SList = []
	SrList = []
	R = [[] for i in range(natures)]
	Rr = [[] for i in range(natures)]
	L = [[] for i in range(natures)]
	Lr = [[] for i in range(natures)]
	for i in range(len(T)):
		print 'Round ',i
		W,S,WMAS = weightedMajority(W,beta,i,natures,S,WMAS,o)
		Wr,Sr,RWMAS = randomizedWeightedMajority(Wr,beta,i,natures,Sr,RWMAS,o)
		SList.append(S)	

		# Regrets
		for j in range(len(WMAS)):
			R[j].append(WMAS[j]-min(S[j]))
			Rr[j].append(RWMAS[j]-min(Sr[j]))
		# R[i].append([WMAS[j]-min(S[j]) for j in range(len(WMAS))])
		# Rr.append([RWMAS[j]-min(Sr[j]) for j in range(len(RWMAS))])
			print R[j][-1], Rr[j][-1]
			L[j].append(WMAS[j])
			Lr[j].append(RWMAS[j])


		# Learner Loss
		print S, Sr

	return R,Rr,L,Lr


# WMA
def weightedMajority(W,beta,r,n,S,WMAS,o):
	# The predictinos from the experts
	expertGuesses = [experts(i+1,r,o) for i in range(len(W[0]))]

	# For each environment, run the algorithm
	for i in range(n):
		# Get the environment's answer
		A = adversary(i+1,W[i],r,o)
		print "Environment "+str(i+1)+":", A
		expertWeights = [W[i][j]*expertGuesses[j]/sum(W[i]) for j in range(len(expertGuesses))]
		guess = int(round(sum(expertWeights)))
		print "WMA Guess: ", guess

		for e in range(len(expertGuesses)):
			if expertGuesses[e] != A:
				W[i][e]*=beta
				S[i][e]+=1

		if guess!=A:
			WMAS[i]+=1

		s = sum(W[i])
		for e in range(len(W[i])):
			W[i][e]/=s

	return W,S,WMAS

# One of the environments needs to be adversarial
def weightedMinority(W,r,a,o):
	e = [experts(i+1,r,o) for i in range(len(W))]
	we = [e[i]*W[i] for i in range(len(e))]
	wes = sum(we)/sum(W)
	return 1-int(round(wes))

# RWMA
def randomizedWeightedMajority(W,beta,r,n,S,RWMAS,o):
	# The predictions from the experts
	expertGuesses = [experts(i+1,r,o) for i in range(len(W[0]))]

	# For each environment, run the algorithm
	for i in range(n):
		A = adversary(i+1,W[i],r,o)
		print "Environment "+str(i+1)+':', A
		guess = weightedChoice(W[i],expertGuesses)
		print "RWMA Guess: ", guess

		for e in range(len(expertGuesses)):
			if expertGuesses[e] != A:
				W[i][e]*=beta
				S[i][e]+=1

		if guess!=A:
			RWMAS[i]+=1

		s = sum(W[i])
		for e in range(len(W[i])):
			W[i][e]/= s

	return W, S, RWMAS

# To get the adversary's answer
def adversary(advType, W, r, o):
	# Observation information
	elapsed = o[0]
	numLines = o[1]
	windows = o[2]
	# Returns either 1 or 0 - randomly
	if advType == 1:
		return random.randint(0,1)
	# lots of experts, lots of confusion
	elif advType == 2:
		if elapsed > 200000:
			return experts(r%7+1,r,o)
		elif numLines> 300:
			return experts((r+1)%7+1,r,o)
		elif windows:
			return experts((r+2)%7+1,r,o)
		else:
			return experts((r+3)%7+1,r,o)
	# Return the minimal weighted majority?
	elif advType == 3:
		# a weighted minority algorithm is pretty adversarial
		return weightedMinority(W,r,advType,o)
	else:
		raise NameError("Adversarial type specified does not exist")

# To get the expert's answer
def experts(exp, r, o):
	# Observation information
	elapsed = o[0]
	numLines = o[1]
	windows = o[2]

	### The Experts ###
	# The pessimist
	if exp == 1:
		return 1
	# The die hard
	elif exp == 2:
		return 0
	# The politician
	elif exp == 3:
		return (r+1)%2
	# The long game
	elif exp == 4:
		if r>50 and windows:
			return weightedChoice([.25,.75],[0,1])
		elif windows:
			return weightedChoice([.75,.25],[0,1])
		else:
			return random.randint(0,1)
	# The time keeper
	elif exp == 5:
		if int(elapsed)%2==0:
			return 1
		else:
			return 0
	# The child prodigy
	elif exp == 6:
		if numLines*1000 > elapsed: 
			return 1
		else:
			return 0
	# The Visionary
	elif exp == 7:
		if elapsed/numLines > 800 and windows:
			return 1
		else:
			return 0
	else:
		raise NameError("Expert Number is incorrect")

def weightedChoice(W,E):
   total = sum(W)
   r = random.uniform(0, total)
   upto = 0
   for e, w in zip(E,W):
      if upto + w > r:
         return e
      upto += w
   assert False, "Bad behavior"

if __name__ == "__main__":
	main()