package source;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.oomdp.auxiliary.DomainGenerator;
import burlap.oomdp.core.Attribute;
import burlap.oomdp.core.Attribute.AttributeType;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.ObjectClass;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.core.TransitionProbability;
import burlap.oomdp.core.objects.MutableObjectInstance;
import burlap.oomdp.core.objects.ObjectInstance;
import burlap.oomdp.core.states.MutableState;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.FullActionModel;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.singleagent.SADomain;
import burlap.oomdp.singleagent.common.SimpleAction;
import burlap.oomdp.statehashing.SimpleHashableStateFactory;

public class DieN implements DomainGenerator{
	
	// current money the player has accumulated
	// in this setting, states are based on how much money the player has and the state of the game
	public static final String ATTMONEY = "money";
	// game state: true if ended
	public static final String ATTSTATE = "state";
	
	public static final String CLASSAGENT = "agent";
	
	public static final String ACTIONROLL = "roll";
	public static final String ACTIONQUIT = "quit";
	
	boolean [] badSides;
	int numSides;
	int numBadSides;
	
	public DieN(int numSides, boolean [] isBadSide){

		this.badSides = isBadSide;
		this.numSides = isBadSide.length;
		
		int badCount = 0;
		for(int i = 0; i < this.numSides; i++){
			
			if(isBadSide[i] == true){
				badCount++;
			}
		}
		
		this.numBadSides = badCount;
	}
	
	protected class DieRoll extends SimpleAction implements FullActionModel{
		
		public DieRoll(String actionName, Domain domain){
			
			super(actionName, domain);
		}
		
		@Override
		public List<TransitionProbability> getTransitions(State s, GroundedAction groundedAction) {
			
			ObjectInstance agent = s.getFirstObjectOfClass(CLASSAGENT);
			int curMoney = agent.getIntValForAttribute(ATTMONEY);
			
			List<TransitionProbability> tps = new ArrayList<TransitionProbability>(); 
				
			// first add the probability of ending up in a bad state
			State ns = s.copy();
			ObjectInstance nagent = ns.getFirstObjectOfClass(CLASSAGENT);
			nagent.setValue(ATTMONEY, 0);
			nagent.setValue(ATTSTATE, true);
			tps.add(new TransitionProbability(ns, ((double)numBadSides) / numSides));

			// determine the transition probabilities of ending up in "good" states
			for(int i = 1; i <= numSides; i++){

				State ks = s.copy();
				ObjectInstance kagent = ks.getFirstObjectOfClass(CLASSAGENT);

				if(badSides[i - 1] == false){

					kagent.setValue(ATTMONEY, curMoney + i);
					tps.add(new TransitionProbability(ks, 1.0/numSides));						
				}					

			}
			
			return tps;
		}

		@Override
		protected State performActionHelper(State s, GroundedAction groundedAction) {
			
			// get the agent and it's current money
			ObjectInstance agent = s.getFirstObjectOfClass(CLASSAGENT);
			int curMoney = agent.getIntValForAttribute(ATTMONEY);
			
			// roll the dice
			Random random = new Random();
			int side = random.nextInt(numSides) + 1;

			// update the money according to the side and also update the state
			if(badSides[side - 1] == true){

				agent.setValue(ATTMONEY, 0);
				agent.setValue(ATTSTATE, true);					

			}else{

				agent.setValue(ATTMONEY, curMoney + side);
			}
			
			return s;
		}
		
	}
	
	// we could merge both actions into one class and distinguish them by their names
	protected class DieQuit extends SimpleAction implements FullActionModel{
		
		public DieQuit(String actionName, Domain domain){
			
			super(actionName, domain);
		}
		
		@Override
		public List<TransitionProbability> getTransitions(State s, GroundedAction groundedAction) {
			
			List<TransitionProbability> tps = new ArrayList<TransitionProbability>();
			State ns = s.copy();
			ObjectInstance nagent = ns.getFirstObjectOfClass(CLASSAGENT);
			nagent.setValue(ATTSTATE, true);
			tps.add(new TransitionProbability(ns, 1));
			
			return tps;
		}

		@Override
		protected State performActionHelper(State s, GroundedAction groundedAction) {
			
			ObjectInstance agent = s.getFirstObjectOfClass(CLASSAGENT);
			agent.setValue(ATTSTATE, true);
			return s;
		}		
	}
	
	public static class DieRF implements RewardFunction{

		@Override
		public double reward(State s, GroundedAction a, State sprime) {
			
			ObjectInstance agent = sprime.getFirstObjectOfClass(CLASSAGENT);
			if(agent.getBooleanValForAttribute(ATTSTATE) == true){
				
				return agent.getIntValForAttribute(ATTMONEY);
			}
			
			return 0;
		}		
	}
	
	public static class DieTF implements TerminalFunction{

		@Override
		public boolean isTerminal(State s) {
			
			ObjectInstance agent = s.getFirstObjectOfClass(CLASSAGENT);						
			return agent.getBooleanValForAttribute(ATTSTATE) == true || agent.getIntValForAttribute(ATTMONEY) > 300;
		}		
	}
	
	@Override
	public Domain generateDomain() {
		
		SADomain domain = new SADomain();
		
		Attribute attmoney = new Attribute(domain, ATTMONEY, AttributeType.INT);
		Attribute attstate = new Attribute(domain, ATTSTATE, AttributeType.BOOLEAN);
		
		attmoney.setLims(0, 1000);
		
		ObjectClass agentClass = new ObjectClass(domain, CLASSAGENT);
		agentClass.addAttribute(attmoney);
		agentClass.addAttribute(attstate);
		
		new DieRoll(ACTIONROLL, domain);
		new DieQuit(ACTIONQUIT, domain);
		
		return domain;
	}
	
	public static State getExampleState(Domain domain, int money){
		
		State s = new MutableState();
		ObjectInstance agent = new MutableObjectInstance(domain.getObjectClass(CLASSAGENT), "agent0");
		agent.setValue(ATTMONEY, money);
		agent.setValue(ATTSTATE, false);
		s.addObject(agent);
		return s;
	}
	
	public String actionToTake(int score){
	
		Domain domain = this.generateDomain();
		State initialState = DieN.getExampleState(domain, score); 
		ValueIteration vi = new ValueIteration(domain, new DieRF(), new DieTF(), 1, new SimpleHashableStateFactory(), 0.001, 1000);
		vi.toggleReachabiltiyTerminalStatePruning(true);
		vi.planFromState(initialState);
		
		if(vi.value(initialState) > score){
			
			return "roll";
		}else{
			
			return "quit";
		}
	}
	
	public double expectedValue(){
		
		Domain domain = this.generateDomain();
		State initialState = DieN.getExampleState(domain, 0); 
		ValueIteration vi = new ValueIteration(domain, new DieRF(), new DieTF(), 1, new SimpleHashableStateFactory(), 0.001, 1000);
		vi.toggleReachabiltiyTerminalStatePruning(true);
		vi.planFromState(initialState);
		
		return vi.value(initialState);		
	}	
	
	public static void main(String[] args){
		
		boolean [] badSides = {false,false,false,true,false,false};
		
		DieN dien = new DieN(6, badSides);
		System.out.println(dien.expectedValue());
		System.out.println(dien.actionToTake(8));
	}
}
