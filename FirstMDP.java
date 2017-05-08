package source;

import java.util.List;

import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.oomdp.statehashing.DiscretizingHashableStateFactory;
import burlap.domain.singleagent.graphdefined.GraphDefinedDomain;
import burlap.oomdp.auxiliary.DomainGenerator;
import burlap.oomdp.auxiliary.common.NullTermination;
import burlap.oomdp.core.states.State;
import burlap.oomdp.core.*;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;

public class FirstMDP {
	
	DomainGenerator dg;
	Domain domain;
	State initState;
	RewardFunction rf;
	TerminalFunction tf;
	DiscretizingHashableStateFactory hashFactory;
	
	
	public FirstMDP(double p1, double p2, double p3, double p4){
		
		int numStates = 6;
		this.dg = new GraphDefinedDomain(numStates);
		
		((GraphDefinedDomain)this.dg).setTransition(0, 0, 1, 1.);
		((GraphDefinedDomain)this.dg).setTransition(0, 1, 2, 1.);
		((GraphDefinedDomain)this.dg).setTransition(0, 2, 3, 1.);		
		((GraphDefinedDomain)this.dg).setTransition(1, 0, 1, 1.);
		((GraphDefinedDomain)this.dg).setTransition(2, 0, 4, 1.);
		((GraphDefinedDomain)this.dg).setTransition(3, 0, 5, 1.);
		((GraphDefinedDomain)this.dg).setTransition(4, 0, 2, 1.);
		((GraphDefinedDomain)this.dg).setTransition(5, 0, 5, 1.);
		
		this.domain = this.dg.generateDomain();		
		this.initState = GraphDefinedDomain.getState(this.domain, 0);
		this.rf = new FourParamRF(p1, p2, p3, p4);
		this.tf = new NullTermination();
		this.hashFactory = new DiscretizingHashableStateFactory(0.5);
	}
	
	public static class FourParamRF implements RewardFunction{
		
		double p1, p2, p3, p4;
		
		public FourParamRF(double p1, double p2, double p3, double p4){
			
			this.p1 = p1;
			this.p2 = p2;
			this.p3 = p3;
			this.p4 = p4;
		}

		@Override
		public double reward(State s, GroundedAction a, State sprime) {
			
			int id_s = GraphDefinedDomain.getNodeId(s);
			int id_sprime = GraphDefinedDomain.getNodeId(sprime);
			
			if(id_s == 0 || id_s == 3){
				return 0;
			}else if(id_s == 1){
				if(id_sprime == 1){
					return this.p1;
				}
			}else if(id_s == 2){
				if(id_sprime == 4){
					return this.p2;
				}
			}else if(id_s == 4){
				if(id_sprime == 2){
					return this.p3;
				}
			}else if(id_s == 5){
				if(id_sprime == 5){
					return this.p4;
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
	
	public String bestFirstAction(double gamma){
		
		ValueIteration vi = this.computeValue(gamma);
		
		List<State> viStates = vi.getAllStates();
		
		double q1 = vi.getQs(GraphDefinedDomain.getState(this.domain, 1)).get(0).q;
		double q2 = vi.getQs(GraphDefinedDomain.getState(this.domain, 2)).get(0).q;
		double q3 = vi.getQs(GraphDefinedDomain.getState(this.domain, 3)).get(0).q;
		
		if(q1 >= q2 && q1 >= q3){
			return "action a"; 
		}else if(q2 >= q1 && q2 >= q3){
			return "action b";
		}else if(q3 >= q1 && q3 >= q1){
			return "action c";
		}
		
		return "";
	}
	
	public static void main(String[] args){
		
		double p1 = 5., p2 = 6., p3 = 3., p4 = 7.;
		double gamma = 0.6;
		
		FirstMDP fmdp = new FirstMDP(p1, p2, p3, p4);
		System.out.println("Best initial action: " + fmdp.bestFirstAction(gamma));
		
	}

}
