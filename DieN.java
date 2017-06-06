package source;

import java.util.List;

import burlap.behavior.policy.Policy;
import burlap.behavior.singleagent.EpisodeAnalysis;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.domain.singleagent.graphdefined.GraphDefinedDomain;
import burlap.oomdp.auxiliary.DomainGenerator;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.statehashing.DiscretizingHashableStateFactory;

public class DieN {

	private int numStates;
	private int numActions;
	private double gamma;
	private boolean [] isBadSide;

	DomainGenerator dg;
	Domain domain;
	State initState;
	RewardFunction rf;
	TerminalFunction tf;
	DiscretizingHashableStateFactory hashFactory;

	public DieN(int numSides, boolean [] isBadSide){

		this.numStates = numSides + 1;
		this.numActions = 2;	// 0 for roll and 1 for quit
		this.isBadSide = isBadSide;
		
		this.gamma = .5;

		this.dg = new GraphDefinedDomain(this.numStates);
		this.setTransitions();
		this.domain = this.dg.generateDomain();
		this.rf = new MyRewards(isBadSide);
		this.tf = new MyTerminals(isBadSide);
		this.hashFactory = new DiscretizingHashableStateFactory(0.5);

	}

	private void setTransitions(){

		for(int i = 0; i < numStates - 1; i++){

			if(isBadSide[i] == false){

				for(int k = 0; k < numStates - 1; k++){

					((GraphDefinedDomain)this.dg).setTransition(i, 0, k, 1.0 / (numStates - 1));
				}
			}else{

				((GraphDefinedDomain)this.dg).setTransition(i, 0, numStates - 1, 1);
			}

			((GraphDefinedDomain)this.dg).setTransition(i, 1, numStates - 1, 1);
		}

		((GraphDefinedDomain)this.dg).setTransition(numStates - 1, 0, numStates - 1, 1);
		((GraphDefinedDomain)this.dg).setTransition(numStates - 1, 1, numStates - 1, 1);

	}
	
	public double expectedValue(){
		
		double maxDelta = 0.001;
		int maxIterations = 1000;
		
		double totalReward = 0;
		
		ValueIteration vi = new ValueIteration(this.domain, this.rf, this.tf, this.gamma, this.hashFactory, maxDelta, maxIterations);
		
		// plan from each state in order and compute average total reward
		for(int i = 0; i < numStates - 1; i++){
			
			double curTotalReward = 0;
			
			Policy curPolicy = vi.planFromState(GraphDefinedDomain.getState(this.domain, i));
			EpisodeAnalysis ea = curPolicy.evaluateBehavior(GraphDefinedDomain.getState(this.domain, i), this.rf, this.tf);
			
			List<Double> rewardList = ea.rewardSequence;
			for(int j = 0; j < rewardList.size(); j++){
				
				curTotalReward += rewardList.get(j);
			}
			
			System.out.println("Total reward from state " + i + " is: " + curTotalReward);
			totalReward += curTotalReward;
		}		
		
		return totalReward / (this.numStates - 1);
	}

	class MyRewards implements RewardFunction{

		boolean [] isBadSide;

		public MyRewards(boolean [] isBadSide){

			this.isBadSide = isBadSide;
		}

		@Override
		public double reward(State s, GroundedAction a, State sprime) {

			int id_s = GraphDefinedDomain.getNodeId(s);
			int id_sprime = GraphDefinedDomain.getNodeId(sprime);
			int id_a = Integer.parseInt(a.toString().replaceAll(GraphDefinedDomain.BASEACTIONNAME, ""));

			if(id_s == numStates - 1 || id_sprime == numStates - 1){
				
				return 0;
			}
			
			if(this.isBadSide[id_s] == false){

				if(this.isBadSide[id_sprime] == true){

					return 0;
				}else if(id_sprime < numStates - 1){

					return id_sprime + 1; 
				}
			}

			return 0;
		}

	}
	
	class MyTerminals implements TerminalFunction{
		
		private boolean [] isBadSide;
		
		public MyTerminals(boolean [] isBadSide){
			
			this.isBadSide = isBadSide;
		}

		@Override
		public boolean isTerminal(State s) {
			
			int s_id = GraphDefinedDomain.getNodeId(s);
			
			/*if(s_id == this.isBadSide.length){
				return true;
			}*/
			
			if(s_id == this.isBadSide.length || this.isBadSide[s_id] == true){
				
				return true;
			}
			
			return false;
		}
		
	}
	
	public static void main(String[] args){

		boolean [] badSides = {false, false, false, true, false, false};
		DieN d = new DieN(6, badSides);
		//boolean [] badSides = {true,false,false,true,false,true,false,true,true,true,false,true,false,true,true,true,false,true,false,true};
		//DieN d = new DieN(20, badSides);

		System.out.println(d.expectedValue());
	}
}
