package source;

import burlap.behavior.singleagent.learning.actorcritic.ActorCritic;
import burlap.behavior.singleagent.learning.actorcritic.actor.BoltzmannActor;
import burlap.behavior.singleagent.learning.actorcritic.critics.TDLambda;
import burlap.behavior.valuefunction.ValueFunctionInitialization;
import burlap.domain.singleagent.graphdefined.GraphDefinedDomain;
import burlap.oomdp.auxiliary.DomainGenerator;
import burlap.oomdp.core.AbstractGroundedAction;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.singleagent.environment.Environment;
import burlap.oomdp.singleagent.environment.SimulatedEnvironment;
import burlap.oomdp.statehashing.DiscretizingHashableStateFactory;

public class BestLambda{
	
	private double probToState1;
	private double [] valueEstimates;
	private double [] rewards;
	
	DomainGenerator dg;
	Domain domain;
	State initState;
	RewardFunction rf;
	TerminalFunction tf;
	DiscretizingHashableStateFactory hashFactory;
	
	public BestLambda(double probToState1, double [] valueEstimates, double [] rewards){
		
		this.probToState1  = probToState1;
		this.valueEstimates = valueEstimates;
		this.rewards = rewards;
		
		this.dg = new GraphDefinedDomain(7);
		this.setTransitions();
		this.domain = this.dg.generateDomain();
		this.initState = GraphDefinedDomain.getState(this.domain, 0);
		this.rf = new MyRewards(this.rewards);
		this.tf = new MyTerminals();
		this.hashFactory = new DiscretizingHashableStateFactory(0.5);
	}
	
	private double calcTDValue(double lambda){
		
		double gamma = 1.0;
		double learningRate = 1.0;
		
		Environment env = new SimulatedEnvironment(this.domain, this.rf, this.tf, this.initState);
		
		TDLambda tdl = new TDLambda(this.rf, this.tf, gamma, this.hashFactory, learningRate, new MyVInit(this.valueEstimates, this.rewards, this.probToState1), lambda);
		
		BoltzmannActor actor = new BoltzmannActor(this.domain, this.hashFactory, learningRate);
		
		ActorCritic ac = new ActorCritic(this.domain, gamma, actor, tdl);
		
		ac.runLearningEpisode(env);
		
		return tdl.value(this.initState);
	}
	
	public double findBestLambda(){
		
		// find the value of TD(1)
		double lambdaOneTDValue = this.calcTDValue(1.0);
		System.out.println("TD Value of TD(1) is " + lambdaOneTDValue);
		
		double minDist = 10000000;
		double minLamb = 10000000;
		
		/*for(double curLamb = 0; curLamb < 1; curLamb += 0.0001){
			
			double curDist = Math.abs(this.calcTDValue(curLamb) - lambdaOneTDValue);
			if(curDist < minDist){
				
				minDist = curDist;
				minLamb = curLamb;
			}
		}*/
		
		for(double curLamb = 0; curLamb < 1; curLamb += 0.0001){
			
			if(this.calcTDValue(curLamb) == lambdaOneTDValue){
				
				minLamb = curLamb; 
			}
		}
		
		//System.out.println("Min dist: " + minDist);
		System.out.println("Min Lambda: " + minLamb);
		
		return minLamb;
	}
	
	private void setTransitions(){
		
		((GraphDefinedDomain)this.dg).setTransition(0, 0, 1, this.probToState1);
		((GraphDefinedDomain)this.dg).setTransition(0, 0, 2, 1-this.probToState1);
		((GraphDefinedDomain)this.dg).setTransition(1, 0, 3, 1);
		((GraphDefinedDomain)this.dg).setTransition(2, 0, 3, 1);
		((GraphDefinedDomain)this.dg).setTransition(3, 0, 4, 1);
		((GraphDefinedDomain)this.dg).setTransition(4, 0, 5, 1);
		((GraphDefinedDomain)this.dg).setTransition(5, 0, 6, 1);
		
	}
	
	class MyRewards implements RewardFunction{
		
		private double [] rewards;
		
		public MyRewards(double [] rewards){
			
			this.rewards = rewards;					
		}

		@Override
		public double reward(State s, GroundedAction a, State sprime) {
			
			int id_s = GraphDefinedDomain.getNodeId(s);
			int id_sprime = GraphDefinedDomain.getNodeId(sprime);
			
			double reward = 0;
		
			if(id_s == 0 && id_sprime == 1){				
				reward = this.rewards[0];
			}else if(id_s == 0 && id_sprime == 2){
				reward = this.rewards[1];
			}else if(id_s == 1){
				reward = this.rewards[2];
			}else if(id_s == 2){
				reward = this.rewards[3];
			}else if(id_s == 3){
				reward = this.rewards[4];
			}else if(id_s == 4){
				reward = this.rewards[5];
			}else if(id_s == 5){
				reward = this.rewards[6];
			}
			
			return reward;
		}		
	}
	
	class MyTerminals implements TerminalFunction{

		@Override
		public boolean isTerminal(State s) {
			
			int id_s = GraphDefinedDomain.getNodeId(s);
			
			if(id_s == 6){
				
				return true;
			}
			
			return false;
		}
	}
	
	class MyVInit implements ValueFunctionInitialization{
		
		private double [] valueEstimates;
		private double [] rewards;
		double probToState1;
		
		public MyVInit(double [] valueEstimates, double [] rewards, double probToState1){
			
			this.valueEstimates = valueEstimates;
			this.rewards = rewards;
			this.probToState1 = probToState1;
		}

		@Override
		public double value(State s) {
			
			int id_s = GraphDefinedDomain.getNodeId(s);
			
			return this.valueEstimates[id_s];
		}

		@Override
		public double qValue(State s, AbstractGroundedAction a) {
			
			// WE ASSUMED gamma = 1
			// Changing initial values of q-values doesn't seem to be effect anything !!
			
			double qvalue = 0;
			
			int id_s = GraphDefinedDomain.getNodeId(s);
			
			if(id_s == 0){
				
				qvalue = this.probToState1 * (this.rewards[0] + this.valueEstimates[1]) + (1-this.probToState1) * (this.rewards[1] + this.valueEstimates[2]);
			}else if(id_s == 1 || id_s == 2){
				
				qvalue = this.rewards[id_s + 1] + this.valueEstimates[3];
			}else{
				
				qvalue = this.rewards[id_s + 1] + this.valueEstimates[id_s + 1];
			}
			
			return qvalue;
		}
	}
	
	public static void main(String [] args){
		
		double probToState1 = 0.5;
		double [] valueEstimates = {0.0,3.0,8.0,2.0,1.0,2.0,0.0};
		double [] rewards = {0.0,0.0,0.0,4.0,1.0,1.0,1.0};
		
		/*double probToState1 = 0.89;
		double [] valueEstimates = {0.0,3.8,1.9,1.5,4.9,0.0,0.0};
		double [] rewards = {-1.9,7.1,1.3,-3.0,2.4,1.7,-0.7};*/
		
		BestLambda bl = new BestLambda(probToState1, valueEstimates, rewards);
		//System.out.println(bl.findBestLambda());
		bl.findBestLambda();
	}
}
