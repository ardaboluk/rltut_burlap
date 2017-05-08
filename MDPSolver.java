package source;

import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.domain.singleagent.graphdefined.GraphDefinedDomain;
import burlap.oomdp.auxiliary.DomainGenerator;
import burlap.oomdp.auxiliary.common.NullTermination;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.statehashing.DiscretizingHashableStateFactory;

public class MDPSolver {
	
	private int numStates;
	private int numActions;
	private double[][][] probabilitiesOfTransitions;
	private double[][][] rewards;
	
	DomainGenerator dg;
	Domain domain;
	State initState;
	RewardFunction rf;
	TerminalFunction tf;
	DiscretizingHashableStateFactory hashFactory;
	
	public MDPSolver(int numStates, int numActions, double[][][] probabilitiesOfTransitions, double[][][] rewards){
		
		this.numStates = numStates;
		this.numActions = numActions;
		this.probabilitiesOfTransitions = probabilitiesOfTransitions;
		this.rewards = rewards;
		
		this.dg = new GraphDefinedDomain(this.numStates);
		this.setTransitions();
		this.domain = this.dg.generateDomain();
		this.initState = GraphDefinedDomain.getState(this.domain, 0);
		this.rf = new MyRewards(this.rewards);
		this.tf = new NullTermination();
		this.hashFactory = new DiscretizingHashableStateFactory(0.5);
		
	}
	
	/*
	 *	Checks if probabilities for a given state-action pair add up to 1
	 * 
	 * */
	private boolean checkProbabilities(){
		
		for(int i = 0; i < this.probabilitiesOfTransitions.length; i++){
			
			for(int j = 0; j < this.probabilitiesOfTransitions[i].length; j++){
				
				double sum = 0;
				
				for(int k = 0; k < this.probabilitiesOfTransitions[i][j].length; k++){
					
					sum += this.probabilitiesOfTransitions[i][j][k];
				}
				
				if(sum != 1){
					
					return false;
				}
			}
		}			
		
		return true;				
	}
	
	private void setTransitions(){
		
		// check if the probabilities add up to 1
		if(checkProbabilities() == true){
			System.out.println("Probabilities for given state-action pairs add up to 1...");
		}else{
			System.out.println("Probabilities for given state-action pairs doesn't add up to 1 !");
			System.exit(-1);
		}
		
		for(int i = 0; i < this.probabilitiesOfTransitions.length; i++){
			
			for(int j = 0; j < this.probabilitiesOfTransitions[i].length; j++){
				
				for(int k = 0; k < this.probabilitiesOfTransitions[i][j].length; k++){
					
					double currentProbability = this.probabilitiesOfTransitions[i][j][k];
					
					if(currentProbability != 0){
						((GraphDefinedDomain)this.dg).setTransition(i, j, k, currentProbability);
					}
				}
			}
		}
		
	}
	
	private ValueIteration computeValue(double gamma){
		
		double maxDelta = 0.0001;
		int maxIterations = 1000;
		
		ValueIteration vi = new ValueIteration(this.domain, this.rf, this.tf, gamma, this.hashFactory, maxDelta, maxIterations);
		
		vi.planFromState(this.initState);
		
		return vi;				
	}
	
	
	
	public double valueOfState(int state, double gamma){
		
		return computeValue(gamma).value(GraphDefinedDomain.getState(this.domain, state));
	}
	
	class MyRewards implements RewardFunction{
		
		double[][][] rewards;
		
		public MyRewards(double[][][] rewards){
			
			this.rewards = rewards;
		}

		@Override
		public double reward(State s, GroundedAction a, State sprime) {
			
			int id_s = GraphDefinedDomain.getNodeId(s);
			int id_sprime = GraphDefinedDomain.getNodeId(sprime);
			int id_a = Integer.parseInt(a.toString().replaceAll(GraphDefinedDomain.BASEACTIONNAME, ""));
			
			return rewards[id_s][id_a][id_sprime];						
		}
		
	}

}
