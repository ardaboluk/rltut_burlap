package source;


import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.domain.singleagent.graphdefined.GraphDefinedDomain;
import burlap.domain.singleagent.graphdefined.GraphTF;
import burlap.oomdp.auxiliary.DomainGenerator;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.statehashing.DiscretizingHashableStateFactory;

public class SecondMDP {
	
	DomainGenerator dg;
	Domain domain;
	State initState;
	RewardFunction rf;
	TerminalFunction tf;
	DiscretizingHashableStateFactory hashFactory;

	public SecondMDP(double p1, double p2){
		
		int numStates = 6;
		this.dg = new GraphDefinedDomain(numStates);
		
		((GraphDefinedDomain)this.dg).setTransition(0, 0, 0, p1);	// action a p1
		((GraphDefinedDomain)this.dg).setTransition(0, 0, 1, 1-p1);	// action a 1-p1
		((GraphDefinedDomain)this.dg).setTransition(0, 1, 2, 1);	// action b
		((GraphDefinedDomain)this.dg).setTransition(1, 0, 3, 1-p2);	// action c 1-p2
		((GraphDefinedDomain)this.dg).setTransition(1, 0, 5, p2);	// action c p2
		((GraphDefinedDomain)this.dg).setTransition(1, 1, 4, 1);	// action d
		((GraphDefinedDomain)this.dg).setTransition(2, 0, 1, 1);
		((GraphDefinedDomain)this.dg).setTransition(3, 0, 1, 1);
		((GraphDefinedDomain)this.dg).setTransition(4, 0, 5, 1);
		
		this.domain = this.dg.generateDomain();
		this.initState = GraphDefinedDomain.getState(this.domain, 0);
		this.rf = new MyRewards();
		this.tf = new GraphTF(5);
		this.hashFactory = new DiscretizingHashableStateFactory(0.5);
	}
	
	public static class MyRewards implements RewardFunction{

		@Override
		public double reward(State s, GroundedAction a, State sprime) {
			
			int id_s = GraphDefinedDomain.getNodeId(s);
			int id_sprime = GraphDefinedDomain.getNodeId(sprime);
			
			if(id_s == 0){
				if(id_sprime == 0){
					return -1;
				}else if(id_sprime == 1){
					return 3;
				}else if(id_sprime == 2){
					return 1;
				}								
			}else if(id_s == 1){
				if(id_sprime == 3){
					return 1;
				}else if(id_sprime == 5){
					return 0;
				}else if(id_sprime == 4){
					return 2;
				}
			}else if(id_s == 2){
				if(id_sprime == 1){
					return 0;
				}				
			}else if(id_s == 3){
				if(id_sprime == 1){
					return 0;
				}				
			}else if(id_s == 4){
				if(id_sprime == 5){
					return 0;
				}
			}
			
			return 0;
		}
		
		
	}
	
	private ValueIteration computeValue(double gamma){
		
		double maxDelta = 0.0001;
		int maxIterations = 1000;
		
		ValueIteration vi = new ValueIteration(this.domain, this.rf, this.tf, gamma, this.hashFactory, maxDelta, maxIterations);
		
		vi.planFromState(this.initState);
		
		return vi;
	}
	
	public String bestActions(double gamma){
		
		String result = "";
		
		ValueIteration vi = this.computeValue(gamma);
		
		double qs0a = vi.getQs(GraphDefinedDomain.getState(this.domain, 0)).get(0).q;
		double qs0b = vi.getQs(GraphDefinedDomain.getState(this.domain, 0)).get(1).q;
		double qs1c = vi.getQs(GraphDefinedDomain.getState(this.domain, 1)).get(0).q;
		double qs1d = vi.getQs(GraphDefinedDomain.getState(this.domain, 1)).get(1).q;
		
		if(qs0a >= qs0b){
			result += "a";
		}else{
			result +="b";
		}
		
		result += ",";
		
		if(qs1c >= qs1d){
			result += "c";
		}else{
			result +="d";
		}
		
		return result;
	}
	
	public static void main(String[] args) {
		
		double p1 = .5, p2 = .5;
		SecondMDP smdp = new SecondMDP(p1, p2);
		
		double gamma = .5;
		System.out.println("Best actions: " + smdp.bestActions(gamma));

	}

}
