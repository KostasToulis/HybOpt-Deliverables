namespace CHRVRP;

public class Configurations
{
    /**************************************************************
     *                       RUN PARAMETERS                       *
     **************************************************************/
    // the position of the starting file from the sequence of files
    public int StartingPoint { get; set; }
    // the position after the last to be included file from the sequence of files
    public int EndingPoint { get; set; }
    // how many different runs the program will do for each instance-file
    public int NumberOfRuns { get; set; }
    // main KPI to calculate and check
    public int MainKpi  { get; set; }
    
    /**************************************************************
     *                  SOLUTION INIT PARAMETERS                  *
     **************************************************************/
    // how large the restricted candidate list is
    public int RclSize { get; set; }
    // controls the radius of neighbors to be included for each node
    public float NearestNeighborsP { get; set; }
    
    /**************************************************************
     *                  LOCAL SEARCH PARAMETERS                   *
     **************************************************************/
    // how many times the local search will run where the solution hasn't improved
    public int MaxNonImproving { get; set; }
    // if local search uses the math model to pertubate the solutions
    public bool UseMath { get; set; }
    // when to use math model to pertubate the solution
    public double MathModelThresh { get; set; }
    // how much to penalize solutions violating capacity restrictions
    public int Penalty { get; set; }
    
    /**************************************************************
     *                   MATH MODEL PARAMETERS                    *
     **************************************************************/
    // how many nodes to add when using the math model
    public int MaxAdditionsRemovalsPerRoute { get; set; }
    // how many nodes to remove when using the math model
    public double MinSPChange { get; set; }
    
    /**************************************************************
     *                    PROMISES PARAMETERS                     *
     **************************************************************/
    // controls after how many non improving local search iterations the promises mechanism resets
    public float ResetThresholdMultiplier { get; set; }
}