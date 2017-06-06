package source;

import source.MDPSolver;

public class DieN {
	
	private int numStates;
	private int numActions;
	private double gamma;
	private boolean [] isBadSide;
	
	private double [][][] rewards;
	private double [][][] transitions;
	
	private MDPSolver mdpSolver;
	
	public DieN(int numSides, boolean [] isBadSide){
		
		this.numStates = numSides + 1;	// one more state for quit action
		this.numActions = 2;	// 0: roll, 1: quit
		this.gamma = .5;
		this.isBadSide = isBadSide;
		
		this.rewards = new double [numStates][2][numStates];
		this.transitions = new double [numStates][2][numStates];
		
		setRewards();
		setTransitions();
		
		mdpSolver = new MDPSolver(numStates, numActions, transitions, rewards);
	}
	
	public double expectedValue(){

		//return mdpSolver.valueOfState(1, this.gamma);
		
		double totalValue = 0;
		for(int i = 0; i < this.numStates - 1; i++){
			
			totalValue += mdpSolver.valueOfState(i, i, this.gamma);
		}
		
		return totalValue / (this.numStates - 1);
	}
	
	public String actionToTake(int score){
		
		return "roll";
	}
	
	private void setRewards(){
		
		// rewards are set for roll action, rewards for quit action are 0
		
		for(int i = 0; i < this.numStates - 1; i++){
			
			if(this.isBadSide[i] == false){
				
				for(int k = 0; k < this.numStates - 1; k++){

					if(this.isBadSide[k] == true){
						this.rewards[i][0][k] = 0;
					}
					else{
						this.rewards[i][0][k] = k + 1;
					}
				}
			}
		}
	}
	
	private void setTransitions(){
		
		for(int i = 0; i < numStates - 1; i++){
			
			if(isBadSide[i] == false){
				
				for(int k = 0; k < numStates - 1; k++){
						
					this.transitions[i][0][k] = 1.0 / (numStates - 1);				
				}
			}else{
				
				this.transitions[i][0][numStates - 1] = 1;
			}
			
			this.transitions[i][1][numStates - 1] = 1;
		}
		
		this.transitions[numStates - 1][0][numStates - 1] = 1;
		this.transitions[numStates - 1][1][numStates - 1] = 1;
	}
	
	public static void main(String[] args){
		
		//boolean [] badSides = {false, false, false, true, false, false};
		//DieN d = new DieN(6, badSides);
		boolean [] badSides = {true,false,false,true,false,true,false,true,true,true,false,true,false,true,true,true,false,true,false,true};
		DieN d = new DieN(20, badSides);
		
		System.out.println(d.expectedValue());
	}
}
