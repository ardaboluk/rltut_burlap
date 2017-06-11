package source;

import burlap.domain.singleagent.gridworld.GridWorldDomain;
import burlap.oomdp.auxiliary.stateconditiontest.StateConditionTest;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.singleagent.environment.Environment;
import burlap.oomdp.statehashing.HashableStateFactory;

public class BasicBehavior {
	
	GridWorldDomain gwdg;
	Domain domain;
	RewardFunction rf;
	TerminalFunction tf;
	StateConditionTest goalCondition;
	State initialState;
	HashableStateFactory hashingFactory;
	Environment env;
	
	public BasicBehavior(){
		
		gwdg = new GridWorldDomain(11, 11);
		gwdg.setMapToFourRooms();
		domain = gwdg.generateDomain();
	}

}
