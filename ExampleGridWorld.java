package source;

import java.util.ArrayList;
import java.util.List;

import burlap.oomdp.auxiliary.DomainGenerator;
import burlap.oomdp.core.Attribute.AttributeType;
import burlap.oomdp.core.Attribute;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.ObjectClass;
import burlap.oomdp.core.PropositionalFunction;
import burlap.oomdp.core.TransitionProbability;
import burlap.oomdp.core.objects.ImmutableObjectInstance;
import burlap.oomdp.core.objects.MutableObjectInstance;
import burlap.oomdp.core.objects.ObjectInstance;
import burlap.oomdp.core.states.MutableState;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.FullActionModel;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.SADomain;
import burlap.oomdp.singleagent.common.SimpleAction;
import burlap.oomdp.singleagent.explorer.TerminalExplorer;

public class ExampleGridWorld implements DomainGenerator{
	
	public static final String ATTX = "x";
	public static final String ATTY = "y";
	
	public static final String CLASSAGENT = "agent";
	public static final String CLASSLOCATION = "location";
	
	public static final String ACTIONNORTH = "north";
	public static final String ACTIONSOUTH = "south";
	public static final String ACTIONEAST = "east";
	public static final String ACTIONWEST = "west";
	
	public static final String PFAT = "at";
	
	//ordered so first dimension is x
	protected int [][] map = new int[][]{
				{0,0,0,0,0,1,0,0,0,0,0},
				{0,0,0,0,0,0,0,0,0,0,0},
				{0,0,0,0,0,1,0,0,0,0,0},
				{0,0,0,0,0,1,0,0,0,0,0},
				{0,0,0,0,0,1,0,0,0,0,0},
				{1,0,1,1,1,1,1,1,0,1,1},
				{0,0,0,0,1,0,0,0,0,0,0},
				{0,0,0,0,1,0,0,0,0,0,0},
				{0,0,0,0,0,0,0,0,0,0,0},
				{0,0,0,0,1,0,0,0,0,0,0},
				{0,0,0,0,1,0,0,0,0,0,0},
		};
		
	protected class Movement extends SimpleAction implements FullActionModel{
		
		protected double [] directionProbs = new double[4];
		
		public Movement(String actionName, Domain domain, int direction){
			super(actionName, domain);
			
			for(int i = 0; i < 4; i++){
				if(i == direction){
					directionProbs[i] = .8;
				}else{
					directionProbs[i] = .2 / 3.;
				}
			}
		}
		
		protected int [] moveResult(int curX, int curY, int direction){
			
			// first get change in x and y from direction using 0: north; 1: south; 2: east; 3: west
			int xdelta = 0;
			int ydelta = 0;
			if(direction == 0){
				ydelta = 1;
			}else if(direction == 1){
				ydelta = -1;
			}else if(direction == 2){
				xdelta = 1;
			}else if(direction == 3){
				xdelta = -1;
			}
			
			int nx = curX + xdelta;
			int ny = curY + ydelta;
			
			int width = ExampleGridWorld.this.map.length;
			int height = ExampleGridWorld.this.map[0].length;
			
			// make sure new position is valid (not a wall or out of bounds)
			if(nx < 0 || nx >= width || ny < 0 || ny >= height ||
					ExampleGridWorld.this.map[nx][ny] == 1){
				
				nx = curX;
				ny = curY;
			}
			
			return new int[]{nx, ny};
		}
		
		@Override
		protected State performActionHelper(State s, GroundedAction groundedAction){
			
			// get agent and current position
			ObjectInstance agent = s.getFirstObjectOfClass(CLASSAGENT);
			int curX = agent.getIntValForAttribute(ATTX);
			int curY = agent.getIntValForAttribute(ATTY);
			
			// sample direction with random roll
			double r = Math.random();
			double sumProb = 0.;
			int dir = 0;			
			for(int i = 0; i < this.directionProbs.length; i++){
				
				sumProb += this.directionProbs[i];
				if(r < sumProb){
					dir = i;
					break;		// found the direction
				}
			}
			
			// get the resulting position
			int [] newPos = this.moveResult(curX, curY, dir);
			
			// set the new position
			agent.setValue(ATTX, newPos[0]);
			agent.setValue(ATTY, newPos[1]);
			
			// return the state we've just modified
			return s;
		}
		
		@Override
		public List<TransitionProbability> getTransitions(State s, GroundedAction groundedAction){
			
			ObjectInstance agent = s.getFirstObjectOfClass(CLASSAGENT);
			int curX = agent.getIntValForAttribute(ATTX);
			int curY = agent.getIntValForAttribute(ATTY);
			
			List<TransitionProbability> tps = new ArrayList<TransitionProbability>(4);			
			TransitionProbability noChangeTransition = null;			
			for(int i = 0; i < this.directionProbs.length; i++){
				int [] newPos = this.moveResult(curX, curY, i);
					if(newPos[0] != curX || newPos[1] != curY){
						// new possible outcome
						State ns = s.copy();
						ObjectInstance nagent = ns.getFirstObjectOfClass(CLASSAGENT);
						nagent.setValue(ATTX, newPos[0]);
						nagent.setValue(ATTY, newPos[1]);
						
						// create transition probability object and add to our list of outcomes
						tps.add(new TransitionProbability(ns, this.directionProbs[i]));
						
					}else{
						// this direction didn't lead to anywhere new
						// if there are existing possible directions
						// that wouldn't lead anywhere, aggregate with them
						if(noChangeTransition != null){
							noChangeTransition.p += this.directionProbs[i];
						}else{
							// otherwise create this new state and transition
							noChangeTransition = new TransitionProbability(s.copy(), this.directionProbs[i]);
							tps.add(noChangeTransition);
						}
					}
				}		
			
			return tps;
		}
	}
	
	protected class AtLocation extends PropositionalFunction{
		
		public AtLocation(Domain domain){
			super(PFAT, domain, new String []{CLASSAGENT, CLASSLOCATION});
		}
		
		@Override
		public boolean isTrue(State s, String[] params){
			
			ObjectInstance agent = s.getObject(params[0]);
			ObjectInstance location = s.getObject(params[1]);
			
			int ax = agent.getIntValForAttribute(ATTX);
			int ay = agent.getIntValForAttribute(ATTY);
			
			int lx = location.getIntValForAttribute(ATTX);
			int ly = location.getIntValForAttribute(ATTY);
			
			return ax == lx && ay == ly;
		}
	}
	
	@Override
	public Domain generateDomain(){
		
		SADomain domain = new SADomain();
		
		Attribute xatt = new Attribute(domain, ATTX, AttributeType.INT);
		xatt.setLims(0, 10);
		
		Attribute yatt = new Attribute(domain, ATTY, AttributeType.INT);
		yatt.setLims(0,10);
		
		ObjectClass agentClass = new ObjectClass(domain, CLASSAGENT);
		agentClass.addAttribute(xatt);
		agentClass.addAttribute(yatt);
		
		ObjectClass locationClass = new ObjectClass(domain, CLASSLOCATION);
		locationClass.addAttribute(xatt);
		locationClass.addAttribute(yatt);
		
		new Movement(ACTIONNORTH, domain, 0);
		new Movement(ACTIONSOUTH, domain, 1);
		new Movement(ACTIONEAST, domain, 2);
		new Movement(ACTIONWEST, domain, 3);
		
		new AtLocation(domain);
		
		return domain;
	}
	
	public static State getExampleState(Domain domain){
		
		State s = new MutableState();
		ObjectInstance agent = new MutableObjectInstance(domain.getObjectClass(CLASSAGENT), "agent0");
		agent.setValue(ATTX, 0);
		agent.setValue(ATTY, 0);
		
		ObjectInstance location = new MutableObjectInstance(domain.getObjectClass(CLASSLOCATION), "location0");		
		location.setValue(ATTX, 10);
		location.setValue(ATTY, 10);
		
		s.addObject(agent);
		s.addObject(location);
		
		return s;
	}
	
	public static void main(String args[]){
		
		ExampleGridWorld gen = new ExampleGridWorld();
		Domain domain = gen.generateDomain();
		
		State initialState = ExampleGridWorld.getExampleState(domain);
		
		TerminalExplorer exp = new TerminalExplorer(domain, initialState);
		exp.explore();
	}
	

}
