package source;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import burlap.behavior.policy.EpsilonGreedy;
import burlap.behavior.policy.Policy;
import burlap.behavior.singleagent.EpisodeAnalysis;
import burlap.behavior.singleagent.auxiliary.EpisodeSequenceVisualizer;
import burlap.behavior.singleagent.learning.LearningAgent;
import burlap.behavior.valuefunction.QFunction;
import burlap.behavior.valuefunction.QValue;
import burlap.behavior.valuefunction.ValueFunctionInitialization;
import burlap.domain.singleagent.gridworld.GridWorldDomain;
import burlap.domain.singleagent.gridworld.GridWorldTerminalFunction;
import burlap.domain.singleagent.gridworld.GridWorldVisualizer;
import burlap.oomdp.core.AbstractGroundedAction;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.singleagent.common.UniformCostRF;
import burlap.oomdp.singleagent.environment.Environment;
import burlap.oomdp.singleagent.environment.EnvironmentOutcome;
import burlap.oomdp.singleagent.environment.SimulatedEnvironment;
import burlap.oomdp.statehashing.HashableState;
import burlap.oomdp.statehashing.HashableStateFactory;
import burlap.oomdp.statehashing.SimpleHashableStateFactory;
import burlap.oomdp.visualizer.Visualizer;
import weka.classifiers.bayes.net.search.global.SimulatedAnnealing;

public class QLTutorial extends burlap.behavior.singleagent.MDPSolver implements LearningAgent, QFunction{
	
	protected Map<HashableState, List<QValue>> qValues;
	protected ValueFunctionInitialization qinit;
	protected double learningRate;
	protected Policy learningPolicy;
	
	public QLTutorial(Domain domain, double gamma, HashableStateFactory hashingFactory, 
			ValueFunctionInitialization qinit, double learningRate, double epsilon) {
		
		this.solverInit(domain, null, null, gamma, hashingFactory);
		this.qinit = qinit;
		this.learningRate = learningRate;
		this.qValues = new HashMap<>();
		this.learningPolicy = new EpsilonGreedy(this, epsilon);
	}

	@Override
	public double value(State s) {
		
		return QFunctionHelper.getOptimalValue(this, s);
	}

	@Override
	public List<QValue> getQs(State s) {
		// first, get hashed state
		HashableState sh = this.hashingFactory.hashState(s);
		
		// check if we already have stored values
		List<QValue> qs = this.qValues.get(sh);
		
		// create and add initialized Q-values if we don't have them stored for this state
		if(qs == null){
			
			List<GroundedAction> actions = this.getAllGroundedActions(s);
			qs = new ArrayList<QValue>(actions.size());
			// create q-value for each action
			for(GroundedAction ga : actions){
				// add q with initialized value
				qs.add(new QValue(s, ga, this.qinit.qValue(s, ga)));
			}
			// store this for later
			this.qValues.put(sh, qs);
		}
		
		return qs;
	}

	@Override
	public QValue getQ(State s, AbstractGroundedAction a) {
		// first get all q-values
		List<QValue> qs = this.getQs(s);
		
		// translate action parameters to source for q-values if needed
		//a = ((GroundedAction)a).translateParameters(s, qs.get(0).s);
		
		// iterate through stored q-values to find a match for the input action
		for(QValue q : qs){
			if(q.a.equals(a)){
				return q;
			}
		}
		
		throw new RuntimeException("Could not find matching Q-value.");
	}

	@Override
	public EpisodeAnalysis runLearningEpisode(Environment env) {

		return this.runLearningEpisode(env, -1);
	}

	@Override
	public EpisodeAnalysis runLearningEpisode(Environment env, int maxSteps) {

		// initialize our episode analysis object with the initial state of the environment
		EpisodeAnalysis ea = new EpisodeAnalysis(env.getCurrentObservation());
		
		// behave until a terminal state or max steps is reached
		State curState = env.getCurrentObservation();
		int steps = 0;
		while(!env.isInTerminalState() && (steps < maxSteps || maxSteps == -1)){
			
			// select an action
			GroundedAction a = (GroundedAction)this.learningPolicy.getAction(curState);
			
			// take the action and observe outcome
			EnvironmentOutcome eo = a.executeIn(env);
			
			// record result
			ea.recordTransitionTo(a, eo.o, eo.r);
			
			// get the max q-value of the resulting state if it's not terminal, 0 otherwise
			double maxQ = eo.terminated ? 0. : this.value(eo.op);
			
			// update the old q-value
			QValue oldQ = this.getQ(curState, a);
			oldQ.q = oldQ.q + this.learningRate * (eo.r + this.gamma * maxQ - oldQ.q);
			
			// move on to the next state
			curState = eo.op;
			steps++;
		}
		
		return ea;		
	}

	@Override
	public void resetSolver() {
		
		this.qValues.clear();		
	}
	
	public static void main(String [] args){
		
		GridWorldDomain gwd = new GridWorldDomain(11,11);
		gwd.setMapToFourRooms();
		
		// only go in intended direction 80% of the time
		gwd.setProbSucceedTransitionDynamics(0.8);
		
		Domain domain = gwd.generateDomain();
		
		// get the initial state with agent in 0,0
		State s = GridWorldDomain.getOneAgentNoLocationState(domain);
		GridWorldDomain.setAgent(s, 0, 0);
		
		// all transitions return -1
		RewardFunction rf = new UniformCostRF();
		
		// terminate in the top right corner
		TerminalFunction tf = new GridWorldTerminalFunction(10, 10);
		
		// create environment 
		SimulatedEnvironment env = new SimulatedEnvironment(domain, rf, tf, s);
		
		// create QLearning
		QLTutorial agent = new QLTutorial(domain, 0.99, new SimpleHashableStateFactory(), 
				new ValueFunctionInitialization.ConstantValueFunctionInitialization(), 0.1, 0.1);
		
		// run Q-learning and store results in a list
		List<EpisodeAnalysis> episodes = new ArrayList<EpisodeAnalysis>(1000);
		for(int i = 0; i < 1000; i++){
			
			episodes.add(agent.runLearningEpisode(env));
			env.resetEnvironment();
		}
		
		Visualizer v = GridWorldVisualizer.getVisualizer(gwd.getMap());
		new EpisodeSequenceVisualizer(v, domain, episodes);		
	}

}
