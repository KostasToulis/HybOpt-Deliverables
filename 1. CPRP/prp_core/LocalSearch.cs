
//Custom definitions
//#define MY_CHECKS
#define TESTING


using System;
using System.Collections.Generic;
using System.Linq;
using System.Diagnostics;
using System.Net.Mail;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices.ComTypes;
using System.Xml;
using Microsoft.VisualBasic.CompilerServices;
using System.IO;

namespace PRP
{
    public struct LocalSearchParameters
    {
        public int nonImprovIterLim;
        public int nonImprovDiveLim;
        public int timeLimit;
        public double MIPBigGapLimit;
        public int whenEnterMultiplier; //Default 3
        public int whenExitMultiplier; //Default 2
        public int maxRepairMoves;
        public int minRepairMoves;
        public string algorithm;
    }

    public class LocalSearch
    {
        static PRP model;

        public static double[,] distMatrix;
        //public static List<Node> nodes;
        //public static List<Node> customers;
        public static int horizonDays;
        public static Solution solution;
        public static double capacity;
        public static bool CODE_CHECK = false;
        public static double minimumLocalSearchObjective = double.MaxValue;
        public static Solution bestLocalSearchSolution;
        public static double constantFirstPeriodInvCost;
        public static List<int> feasibleOperators;
        public static List<dynamic> feasibleMoves;
        public static double[,,] promisesArcsPerDay;
        public static double[,] promisesCustDay;
        public static StreamWriter testOutput;
        public static Stopwatch stopwatch;
        public static Stopwatch stopwatch_extra;
        public static Stopwatch stopwatchTermCond;
        public static Stopwatch stopwatchRepair;
        public static LocalSearchParameters parameters;

        public static double[,] reductionArcTable;
        public static bool[,] boolReductionArcTable;
        public static double[,] reductionVisitTable;
        public static bool[,] boolReductionVisitTable;

        //Heatmap
        public static int[] customerHeatMap;
        public static int[,] customerVisitPerDayHeatMap;

        //TESTING OBJECTIVE WEIGHTS
        public static double inventoryCostWeight = 1.0;
        
        public static int dummy = 0;


        //Temp Arrays to avoid reinitializionts on every iteration
        private static int[] arcsDaysCreated = new int[10 * 3 + 1];
        private static int[] arcsDaysDeleted = new int[10 * 3 + 1];
        
        public static double GenerateInitialSolution(PRP irpModel)
        {
            return 1.0;
        }

        internal static Solution ApplyLocalSearch(PRP irp, Solution input_sol, int i)
        {
            stopwatch = new Stopwatch();
            stopwatch_extra = new Stopwatch();
            stopwatchTermCond = new Stopwatch();
            stopwatchRepair = new Stopwatch();
            stopwatchTermCond.Start();

            testOutput = new StreamWriter("TestOutput_" + irp.instanceName);

            input_sol.restartElapsedMs = 0;
            input_sol.totalTimeRepairing = 0;
            input_sol.feasibleSpaceIters = 0;
            input_sol.infeasibleSpaceIters = 0;
            input_sol.totalRepairs = 0;

        //Random r = new Random(i);
#if TESTING
            Random r = new Random(i);
#else
            Random r = new Random((int) DateTime.Now.Ticks);
#endif
            
            feasibleOperators = new List<int>();
            feasibleMoves = new List<dynamic>();
            CreatePointersToModel(irp, input_sol);
            CreateEmptyCustomerLists();
            CalculateObjectiveComponenents();
            InitializeCustomerLists();
            InitializeRouteLists();

            //Heatmaps init
            customerHeatMap = new int[solution.nodes.Count];
            customerVisitPerDayHeatMap = new int[solution.nodes.Count, solution.model.horizonDays];
            ResetCustomerHeatmap();

            constantFirstPeriodInvCost = CalculateConstantZeroPeriodInvCost();
            
            solution.TestEverythingFromScratch();
            minimumLocalSearchObjective = double.MaxValue;

            CreateRoutingInventoryPromisesTable();
            
            promisesArcsPerDay = new double[horizonDays, solution.nodes.Count, solution.nodes.Count];
            promisesCustDay = new double[horizonDays, solution.nodes.Count];


            bestLocalSearchSolution = new Solution(solution, 0); //Init bestLocalSearchSolution

            //Main Logic
            AllOperators(r);


            //Report heatmap
            //ReportCustomerHeatmap();
            //ReportCustomerVisitPerDayHeatmap();

            //throw new Exception("testing heatmaps");
            testOutput.Close();

            return bestLocalSearchSolution;
        }

        private static void CreateRoutingInventoryPromisesTable()
        {
            promisesArcsPerDay = new double[horizonDays, solution.nodes.Count, solution.nodes.Count];
            promisesCustDay = new double[horizonDays, solution.nodes.Count];
        }

        
        private static void AllOperators(Random r)
        {
            int iterationOfBest = 0;
            int reinit = 0;
            int iterCounter = 0;
            int nonImprovingIters = 0;
            int nonImprovingDiveCounter = 0;
            double whenEnterMultiplier = parameters.whenEnterMultiplier; //3.0;
            double whenExitMultiplier = parameters.whenEnterMultiplier; //2.0;
            int prodScheduleReconstructionLim = 5;
            int prodScheduleReconstructionCount = 0;
            int nonImprovingItersforLP = 0;
            int improvingStreak = 0;
            double oldObjective = Double.MaxValue;
            INFEASIBILITY_STATUS oldStatus = INFEASIBILITY_STATUS.FEASIBLE;
            bool intraBan = false;
            bool enableArcReduction = false;
            bool enableVisitReduction = true;
            bool use_inventory_operators;
            bool use_routing_operators = true;
            bool force_feasible_ls_exploration = false;
            int feasibleSpaceIters = 0;
            int infeasibleSpaceIters = 0;
            int totalRepairs = 0;
            double totalTimeRepairing = 0;
            double restartElapsedMs = 0;;

            
            InitializeStopwatch();
            ResetRoutingInventoryPromisesTable();
            InfeasibleSolutionManager infSolManager = new InfeasibleSolutionManager();
            List<Solution> prevSols = new List<Solution>();
            
            //Randomize customer saw status
            for (int i = 0; i < solution.customers.Count; i++)
            {
                Node cust = solution.customers[i];

                if (model.input.customers_zero_inv_cost) //model.input.type == PRP_DATASET_VERSION.BOUDIA_FMT || (model.input.type == PRP_DATASET_VERSION.ARCHETTI_FMT && ))
                {
                    if (r.Next() % 100 > 50)
                        cust.sawmode = true;
                    else
                        cust.sawmode = false;
                } else
                {
                    if (cust.unitHoldingCost < solution.depot.unitHoldingCost)
                        cust.sawmode = true;
                    else
                        cust.sawmode = false;
                }
            }

            //int whenEnter = (int)Math.Ceiling(whenEnterMultiplier * model.input.customerNum) + r.Next(1, model.input.customerNum);
            //int whenExit = whenEnter + (int)Math.Ceiling(whenExitMultiplier * model.input.customerNum) + r.Next(1, model.input.customerNum);

            // Disable the Infeasibility MIP 
            int whenEnter = 10000000;
            int whenExit = 10000000;

            int MIPinFeasibleLim = 10;
            int currentMIPinFeasible = 0;
            int dirtyFix = 0;
            int infeasible_passes = 0;


            //Initialize neigborhood modes
            if (solution.status == INFEASIBILITY_STATUS.FEASIBLE)
                use_inventory_operators = false;
            else
            {
                use_inventory_operators = true;
                force_feasible_ls_exploration = true;
                whenEnter = whenExit + (int)Math.Ceiling(whenEnterMultiplier * model.input.customerNum) + r.Next(1, model.input.customerNum); ; // it is already in infeasible space
            }

            //while (nonImprovingDiveCounter < parameters.nonImprovDiveLim && stopwatchTermCond.ElapsedMilliseconds < parameters.timeLimit)
            while (nonImprovingDiveCounter < parameters.nonImprovDiveLim && stopwatchTermCond.ElapsedMilliseconds < parameters.timeLimit && nonImprovingIters < parameters.nonImprovIterLim)
            {
                int testFeas = 0;
                int testMIP = 0;
                int testLP = 0;
                dynamic bestMove;
                MOVES operatorType = MOVES.NONE;

                GlobalUtils.writeToConsole("Iter {2}: whenEnter {0}  -  whenExit {1}", whenEnter, whenExit, iterCounter);

                //if (iterCounter == 602)
                //    Console.WriteLine("Break");
                 
                //Reset Infeasibility Coefficient in the case of feasible search
                float old_coeff = solution.infeasibilityCoeff;
                if (force_feasible_ls_exploration)
                    solution.setVehicleCapacityCoeff(1.0f);


                bool randomize = false;
                if (model.input.type == PRP_DATASET_VERSION.BOUDIA_FMT && model.input.customerNum == 200)
                    randomize = true;
                bestMove = ExploreAllOperators(ref operatorType, r, use_inventory_operators, use_routing_operators, intraBan, enableArcReduction, enableVisitReduction, randomize);
                
                //if (reinit == 100 || bestMove is null) //best results so far (Archetti, Boudia 50
                if (reinit == solution.customers.Count() || bestMove is null) //Testing
                {
                    ResetRoutingInventoryPromisesTable();
                    reinit = 0;
                }
                
                reinit++;

                oldStatus = solution.status;

                ApplyBestMove(bestMove, operatorType);

                //Restore Infeasibility Coefficient in the case of feasible only search
                if (force_feasible_ls_exploration)
                    solution.setVehicleCapacityCoeff(old_coeff);
                
                //Recalculate solution status
                solution.status = solution.TestEverythingFromScratch(); //Update solution status

                if (solution.status != INFEASIBILITY_STATUS.FEASIBLE)
                {
                    solution.violationCost = solution.calcVehicleLoadViolations();
                    //Insert solution to the infeasible solution manager
                    infSolManager.saveSolution(solution);
                    testFeas = 0;
                }
                else
                {
                    testFeas = 1;
                    solution.violationCost = 0;
                }
                    

                if (!Solution.checkSolutionStatus(solution.status))
                {
                    Console.WriteLine("Non Expected Solution Status {0}", solution.status.ToString());
                    throw new Exception("KARAMALAKIES TOUMPANA");
                }
                    

                //Fix vehicles
                solution.fixVehicles();
                //Check for vehicle feasibility
                bool vehicle_feasibility = solution.checkVehicleFeasibility();
                bool solution_improved = false;
                bool solution_local_improved = false;
                //int infeasible_passes = 0;
                double gap_from_best = double.MaxValue;

                if (solution.totalObjective < oldObjective)
                {
                    solution_local_improved = true;
                    improvingStreak--;
                }
                else
                {
                    improvingStreak++;
                }
                    
                oldObjective = solution.totalObjective;
                
                //Calculate gap from best
                if (bestLocalSearchSolution != null)
                    gap_from_best = 100 * (-bestLocalSearchSolution.totalObjective + solution.totalObjective) / bestLocalSearchSolution.totalObjective;


                //Update MIP Time limits
                double effMIPMaxGap = 0.02;
                if (nonImprovingDiveCounter > parameters.nonImprovDiveLim / 2)
                {
                    effMIPMaxGap = parameters.MIPBigGapLimit;
                    //Console.WriteLine("Changing the effMIPMaxGap to {0}", effMIPMaxGap);
                }

                // if a solution has the exact same value as the objective of the optimal solve lp and tsp 
                /*
                if (minimumLocalSearchObjective == solution.totalObjective)
                {
                    //Try to further improve the solution
                    bool lpstatus = LP.runSimultaneousDeliveryProductionReoptimizationLP(ref solution);
                    double tsp_obj_improv = TSP_GRB_Solver.SolveTSP(solution);
                    //lpstatus = MIP.runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemoval(ref solution);

                    if (CloneAndStoreBestLocalSearchSolution(iterCounter, ref iterationOfBest))
                    {
                        ResetRoutingInventoryPromisesTable();
                        reinit = 0;
                        nonImprovingIters = 0;
                        nonImprovingItersforLP = 0;
                        improvingStreak = 0;
                        use_inventory_operators = false;
                        //intraBan = false;
                        use_routing_operators = true;
                        continue;
                    }

                    Console.WriteLine("Iteration {0} (FEASIBLE) Operator: ++{8,-24} Solution Total Obj: {1} | Best Obj: {2} | Routing Obj: {3} | Inv Obj: {4} | Prod Setup Obj: {5} | Prod Unit Obj: {6} ", iterCounter,
                            solution.totalObjective, bestLocalSearchSolution.totalObjective, solution.routingCost, solution.holdingCost, solution.setupProductionCost, solution.totalUnitProductionCost, improvingStreak, operatorType);

                }*/


                // check if back and forth from feasible to infeasible whenever i am in infeasible space
                /*if (!force_feasible_ls_exploration)
                {
                    /*
                    if (oldStatus != INFEASIBILITY_STATUS.FEASIBLE && solution.status == INFEASIBILITY_STATUS.FEASIBLE)
                    {
                        //whenEnter = iterCounter + (int)Math.Ceiling(whenEnterMultiplier * model.input.customerNum) + r.Next(1, model.input.customerNum);
                        whenExit = whenEnter + (int)Math.Ceiling(whenExitMultiplier * model.input.customerNum) + r.Next(1, model.input.customerNum);
                    }
                    else if (oldStatus == INFEASIBILITY_STATUS.FEASIBLE && solution.status != INFEASIBILITY_STATUS.FEASIBLE)
                    {
                        whenEnter = whenExit + (int)Math.Ceiling(whenEnterMultiplier * model.input.customerNum) + r.Next(1, model.input.customerNum);
                        //whenExit = whenEnter + (int)Math.Ceiling(whenExitMultiplier * model.input.customerNum) + r.Next(1, model.input.customerNum);
                    }
                    
                    if (solution.status == INFEASIBILITY_STATUS.FEASIBLE && iterCounter == whenExit)
                    {                     
                        //whenEnter = iterCounter + (int)Math.Ceiling(whenEnterMultiplier * model.input.customerNum) + r.Next(1, model.input.customerNum);
                        whenExit = whenEnter + (int)Math.Ceiling(whenExitMultiplier * model.input.customerNum) + r.Next(1, model.input.customerNum);
                    }
                    
                }*/



                //Do not save any solutions that violate the actual available vehicle number
                if (vehicle_feasibility && solution.status == INFEASIBILITY_STATUS.FEASIBLE)
                {
                    if (CloneAndStoreBestLocalSearchSolution(iterCounter, ref iterationOfBest))
                    {
                        solution.resetVehicleCapacityCoeff();
                        ResetRoutingInventoryPromisesTable();
                        reinit = 0;
                        nonImprovingIters = 0;
                        nonImprovingDiveCounter = 0;
                        improvingStreak = 0;
                        solution_improved = true;
                        use_inventory_operators = false;
                        //intraBan = false;
                        use_routing_operators = true;

                        //Update WhenEnter/WhenExit Counters only when in feasible space?
                        if (force_feasible_ls_exploration || (!force_feasible_ls_exploration && whenEnter == iterCounter))
                        {
                            whenEnter = iterCounter + (int)Math.Ceiling(whenEnterMultiplier * model.input.customerNum) + r.Next(1, model.input.customerNum);
                            whenExit = whenEnter + (int)Math.Ceiling(whenExitMultiplier * model.input.customerNum) +
                                r.Next(1, model.input.customerNum);
                        } else if (!force_feasible_ls_exploration && whenExit == iterCounter) // if we find a new best just before entering infeasible space
                        {
                            whenExit += 1; //postopone for next iteration
                            if (whenExit == whenEnter)
                            {
                                whenEnter = iterCounter + (int)Math.Ceiling(whenEnterMultiplier * model.input.customerNum) + r.Next(1, model.input.customerNum);
                                whenExit = whenEnter + (int)Math.Ceiling(whenExitMultiplier * model.input.customerNum) +
                                    r.Next(1, model.input.customerNum);
                            }
                        }
                    }
                    
                }

                if (iterCounter > whenEnter || iterCounter > whenExit)
                {
                    //throw new Exception("Gamieste gia counter gamw thn poytanas sas mpourdela");
                    Console.WriteLine("Gamieste gia counter gamw thn poytanas sas mpourdela");
                    whenEnter = iterCounter + (int)Math.Ceiling(whenEnterMultiplier * model.input.customerNum) + r.Next(1, model.input.customerNum);
                    whenExit = whenEnter + (int)Math.Ceiling(whenExitMultiplier * model.input.customerNum) +
                        r.Next(1, model.input.customerNum);
                }

                if (solution.infeasibilityCoeff == 1.0f)
                {
                    feasibleSpaceIters += 1;
                }
                else
                {
                    infeasibleSpaceIters += 1;
                }
                    

                if (!solution_improved)
                {
                    //Extra Logic
                
                    //Run Repair LP
                    bool lpstatus = true;
                    if (nonImprovingIters % 10000 == 0 && nonImprovingIters >= 10000)
                    {

                    }

                    //Enable infeasible space
                    //GlobalUtils.writeToConsole("whenEnter {0} - whenExit {1}", whenEnter, whenExit);
                    if (iterCounter == whenEnter && solution.status == INFEASIBILITY_STATUS.FEASIBLE)
                    {
                        //whenEnter = 400 + r.Next(1, 400); //500 Lef 1000 Greg
                        whenEnter = whenExit + (int)Math.Ceiling(whenEnterMultiplier * model.input.customerNum) +
                            r.Next(1, model.input.customerNum);

                        //GlobalUtils.writeToConsole("whenEnter {0} ", whenEnter);

                        //intraBan = true;

                        Console.WriteLine("ENABLING INFEASIBLE SPACE SEARCH");
                        //Enable infeasibility search from the best solution with a small prob. 
                        //if (r.Next() > 70)
                        //solution = new Solution(bestLocalSearchSolution, 0.0);
                        //solution.setVehicleCapacityCoeff(1.05f); //Best for Boudia 50

                        //Fixed 
                        //solution.setVehicleCapacityCoeff(1.02f);
                        //infSolManager.report();
                        //infSolManager.saveToFile();
                        infSolManager.clearSolutions();

                        //Fully random
                        int val = r.Next() % 100;
                        if (val > 50)
                            solution.setVehicleCapacityCoeff(1.02f); //2
                        else if (val > 20)
                            solution.setVehicleCapacityCoeff(1.04f); //5
                        else
                            solution.setVehicleCapacityCoeff(1.06f); //8

                        /*
                        if (r.Next() > 90)
                            solution.setVehicleCapacityCoeff(1.08f);
                        else if (r.Next() > 70)
                            solution.setVehicleCapacityCoeff(1.05f);
                        else
                            solution.setVehicleCapacityCoeff(1.02f);
                        */

                        //use_inventory_operators = true; //Best
                        use_inventory_operators = true;
                        use_routing_operators = true;
                        force_feasible_ls_exploration = false;

                        //Since we enter the infeasible space, we probably failed to improve the solution before by any means. Increase the dive counter here
                        nonImprovingDiveCounter++;
                    }
                    else if (iterCounter == whenExit && !force_feasible_ls_exploration) //solution.status != INFEASIBILITY_STATUS.FEASIBLE)
                    //else if (nonImprovingIters % whenExit == 1 && solution.status != INFEASIBILITY_STATUS.FEASIBLE && nonImprovingIters > whenExit)
                    {
                        currentMIPinFeasible = 0;
                       
                        //whenExit = iterCounter + whenEnter + 300 + r.Next(1, 150); //200
                        whenExit = whenEnter + (int) Math.Ceiling(whenExitMultiplier * model.input.customerNum) +
                            r.Next(1, model.input.customerNum);
                        // GlobalUtils.writeToConsole("whenExit {0} ", whenExit);

                        //lpstatus = LP.runSimultaneousDeliveryProductionReoptimizationLPVehCapInfeas(solution);
                        //lpstatus = MIP.runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertion(solution);
                        //lpstatus = MIP.runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemoval(ref solution);
                        //lpstatus = MIP.runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemovalSameDayRoutes(ref solution, true, true);

                        //Select Solutions probabilistically

                        // 15 30 bestObjRouting
                        int val = r.Next() % 100;
                        if (val < 15 && !(infSolManager.bestObjRouting is null))
                            solution = infSolManager.bestObjRouting;
                        else if (val < 30) { }
                        else if (!(infSolManager.bestObjTotal is null))
                            solution = infSolManager.bestObjTotal;
                        else { }



                        /*
                        if (val > 85)
                            solution = infSolManager.bestObjRouting;
                        else if (val > 70) { } 
                        else
                            solution = infSolManager.bestObjTotal;
                            */

                        //The default option is to keep the current solution
                        int allowedMoves = parameters.maxRepairMoves;
                        if ( Math.Abs(bestLocalSearchSolution.totalObjective - solution.totalObjective) 
                             < 0.01 * bestLocalSearchSolution.totalObjective) //percentage?
                        {
                            allowedMoves = parameters.minRepairMoves;
                        }
                        //stopwatch_extra.Reset();
                        //stopwatch_extra.Start();
                        stopwatchRepair.Start();
                        //Solution back_sol = new Solution(solution, 0.0);
                        //Solution back_sol2 = new Solution(solution, 0.0);

                        lpstatus = MIP.runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemovalSameDayRoutesOriginal(ref solution, allowedMoves, allowedMoves, effMIPMaxGap, dirtyFix);
                        //lpstatus = MIP.runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemovalSameDayRoutesImprovedFormulation(ref solution, allowedMoves, allowedMoves, effMIPMaxGap);
                        stopwatchRepair.Stop();
                        totalRepairs += 1;
                        testMIP = 1;
                        //stopwatch_extra.Stop();
                        //GlobalUtils.writeToConsole("Elappsed milis {0}", stopwatch_extra.ElapsedMilliseconds);
                        //solution = new Solution(bestPool, 0.0);
                        if (solution.status == INFEASIBILITY_STATUS.FEASIBLE)
                        {
                            infeasible_passes = 0;
                            dirtyFix = 0;
                        }
                        else
                        {
                            infeasible_passes += 1;
                            whenEnter = whenExit + (int)Math.Ceiling(whenEnterMultiplier * model.input.customerNum) + r.Next(1, model.input.customerNum);
                        }

                        //Try to decrease the infeasibility coefficient every 3 infeasible runs
                        if (infeasible_passes == 1)
                        {
                            dirtyFix = 25;
                            //lpstatus = MIP.runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemovalSameDayRoutesOriginal(ref solution, allowedMoves, allowedMoves, effMIPMaxGap, dirtyFix);
                            //dirtyFix = 0;

                            //force_feasible_ls_exploration = true;
                            //use_inventory_operators = true;
                            //use_routing_operators = true;
                            //solution.findMaxVehicleCapacityCoeff();
                            //solution.reportVehicleLoadViolations();
                            //solution.SaveToFile("check_what_is_wrong");
                            Console.WriteLine("Infeasibility Coefficient Decreased to : {0}", solution.infeasibilityCoeff);
                        } else if (solution.status == INFEASIBILITY_STATUS.FEASIBLE)
                        {
                            Console.WriteLine("RETURNING BACK TO FEASIBLE SPACE");
                            // reset the promises when the solution is repaired and we pass from infeasible to feasible space 
                            ResetRoutingInventoryPromisesTable();
                            reinit = 0;

                            solution.resetVehicleCapacityCoeff(); //Reset coeffs
                            if (CloneAndStoreBestLocalSearchSolution(iterCounter, ref iterationOfBest))
                            {
                                ResetRoutingInventoryPromisesTable(); // now is reduntant unless changed
                                reinit = 0; // now is reduntant unless changed
                                nonImprovingIters = 0;
                                nonImprovingItersforLP = 0;
                                improvingStreak = 0;
                                use_inventory_operators = false;
                                //intraBan = false;
                                use_routing_operators = true;

                                //Update WhenEnter/WhenExit Counters
                                whenEnter = iterCounter + (int)Math.Ceiling(whenEnterMultiplier * model.input.customerNum) 
                                    + r.Next(1, model.input.customerNum);
                                whenExit = whenEnter + (int)Math.Ceiling(whenExitMultiplier * model.input.customerNum) +
                                    r.Next(1, model.input.customerNum);

                                continue;
                            } 
                            GlobalUtils.writeToConsole("Iteration {0} (FEAS - REPAIRED) Operator: {8,-24} Solution Total Obj: {1} | Best Obj: {2} | Routing Obj: {3} | Inv Obj: {4} | Prod Setup Obj: {5} | Prod Unit Obj: {6} ", iterCounter,
                                solution.totalObjective, bestLocalSearchSolution.totalObjective, solution.routingCost, solution.holdingCost, solution.setupProductionCost, solution.totalUnitProductionCost, improvingStreak, operatorType);
                        }
                        else
                        {
                            GlobalUtils.writeToConsole("Iteration {0} (INFEASIBLE) Operator: {9,-24} Solution Total Obj: {1} | Best Obj: {2} | Routing Obj: {3} | Inv Obj: {4} | Prod Setup Obj: {5} | Prod Unit Obj: {6} | Load Violations: {7} ", iterCounter,
                                solution.totalObjective, !(bestLocalSearchSolution is null) ? bestLocalSearchSolution.totalObjective : Double.MaxValue, solution.routingCost, solution.holdingCost, solution.setupProductionCost, solution.totalUnitProductionCost,
                                solution.violationCost, improvingStreak, operatorType);
                        }
                        
                    }
                    else if (iterCounter % 200 == 0 && solution.status == INFEASIBILITY_STATUS.FEASIBLE && iterCounter > 0)
                    {
                        //Try to further improve the solution
                        lpstatus = LP.runSimultaneousDeliveryProductionReoptimizationLP(ref solution);
                        testLP = 1;
                        double tsp_obj_improv = TSP_GRB_Solver.SolveTSP(solution);
                        //lpstatus = MIP.runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemoval(ref solution);

                        if (CloneAndStoreBestLocalSearchSolution(iterCounter, ref iterationOfBest))
                        {
                            ResetRoutingInventoryPromisesTable();
                            reinit = 0;
                            nonImprovingIters = 0;
                            nonImprovingDiveCounter = 0;
                            improvingStreak = 0;
                            use_inventory_operators = false;
                            //intraBan = false;
                            use_routing_operators = true;

                            //Update WhenEnter/WhenExit Counters only when in feasible space?
                            if (force_feasible_ls_exploration || (!force_feasible_ls_exploration && whenEnter == iterCounter))
                            {
                                whenEnter = iterCounter + (int)Math.Ceiling(whenEnterMultiplier * model.input.customerNum) + r.Next(1, model.input.customerNum);
                                whenExit = whenEnter + (int)Math.Ceiling(whenExitMultiplier * model.input.customerNum) +
                                    r.Next(1, model.input.customerNum);
                            }
                            else if (!force_feasible_ls_exploration && whenExit == iterCounter) // if we find a new best just before entering infeasible space
                            {
                                whenExit += 1; //postpone for next iteration
                                if (whenExit == whenEnter)
                                {
                                    whenEnter = iterCounter + (int)Math.Ceiling(whenEnterMultiplier * model.input.customerNum) + r.Next(1, model.input.customerNum);
                                    whenExit = whenEnter + (int)Math.Ceiling(whenExitMultiplier * model.input.customerNum) +
                                        r.Next(1, model.input.customerNum);
                                }
                            }

                            continue;
                        }
                        
                        GlobalUtils.writeToConsole("Iteration {0} (FEASIBLE) Operator: {8,-24} Solution Total Obj: {1} | Best Obj: {2} | Routing Obj: {3} | Inv Obj: {4} | Prod Setup Obj: {5} | Prod Unit Obj: {6} ", iterCounter,
                                solution.totalObjective, bestLocalSearchSolution.totalObjective, solution.routingCost, solution.holdingCost, solution.setupProductionCost, solution.totalUnitProductionCost, improvingStreak, operatorType);

                    }
                    /*
                    else if (nonImprovingIters % 1000 == 1 && solution.status == INFEASIBILITY_STATUS.FEASIBLE && nonImprovingIters > 1000 && currentMIPinFeasible < MIPinFeasibleLim)
                    {
                        //Try to further improve the solution
                        //stopwatch_extra.Start();
                        //lpstatus = MIP.runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemovalSameDayRoutesOriginal(ref solution, 1, 1);
                        //stopwatch_extra.Stop();
                        //GlobalUtils.writeToConsole("Elappsed milis {0}", stopwatch_extra.ElapsedMilliseconds);
                        currentMIPinFeasible += 1;

                        
                        //int allowedMoves = 4;
                        //if (Math.Abs(bestLocalSearchSolution.totalObjective - solution.totalObjective) < 1500) //percentage?
                        //{
                        //    allowedMoves = 2;
                        //}
                        //stopwatchRepair.Start();
                        //lpstatus = MIP.runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemovalSameDayRoutesOriginal(ref solution, allowedMoves, allowedMoves);
                        //stopwatchRepair.Stop();
                        


                        if (CloneAndStoreBestLocalSearchSolution(iterCounter, ref iterationOfBest))
                        {
                            ResetRoutingInventoryPromisesTable();
                            reinit = 0;
                            nonImprovingIters = 0;
                            nonImprovingDiveCounter = 0;
                            improvingStreak = 0;
                            use_inventory_operators = false;
                            //intraBan = false;
                            use_routing_operators = true;


                            //Update WhenEnter/WhenExit Counters only when in feasible space?
                            if (force_feasible_ls_exploration || (!force_feasible_ls_exploration && whenEnter == iterCounter))
                            {
                                whenEnter = iterCounter + (int)Math.Ceiling(whenEnterMultiplier * model.input.customerNum) + r.Next(1, model.input.customerNum);
                                whenExit = whenEnter + (int)Math.Ceiling(whenExitMultiplier * model.input.customerNum) +
                                    r.Next(1, model.input.customerNum);
                            }
                            else if (!force_feasible_ls_exploration && whenExit == iterCounter) // if we find a new best just before entering infeasible space
                            {
                                whenExit += 1; //postopone for next iteration
                                if (whenExit == whenEnter)
                                {
                                    whenEnter = iterCounter + (int)Math.Ceiling(whenEnterMultiplier * model.input.customerNum) + r.Next(1, model.input.customerNum);
                                    whenExit = whenEnter + (int)Math.Ceiling(whenExitMultiplier * model.input.customerNum) +
                                        r.Next(1, model.input.customerNum);
                                }
                            }

                            continue;
                        }

                        GlobalUtils.writeToConsole("Iteration {0} (FEASIBLE) Operator: {8,-24} Solution Total Obj: {1} | Best Obj: {2} | Routing Obj: {3} | Inv Obj: {4} | Prod Setup Obj: {5} | Prod Unit Obj: {6} ", iterCounter,
                                solution.totalObjective, bestLocalSearchSolution.totalObjective, solution.routingCost, solution.holdingCost, solution.setupProductionCost, solution.totalUnitProductionCost, improvingStreak, operatorType);
                    }
                    */

                    else if (solution.status != INFEASIBILITY_STATUS.FEASIBLE)
                    {
                        GlobalUtils.writeToConsole("Iteration {0} (INFEASIBLE) Operator: {9,-24} Solution Total Obj: {1} | Best Obj: {2} | Routing Obj: {3} | Inv Obj: {4} | Prod Setup Obj: {5} | Prod Unit Obj: {6} | Load Violations: {7} ", iterCounter,
                            solution.totalObjective, !(bestLocalSearchSolution is null) ? bestLocalSearchSolution.totalObjective : Double.MaxValue, solution.routingCost, solution.holdingCost, solution.setupProductionCost, solution.totalUnitProductionCost,
                            solution.violationCost, improvingStreak, operatorType);    
                    }
                    else
                    {
                        GlobalUtils.writeToConsole("Iteration {0} (FEASIBLE) Operator: {8,-24} Solution Total Obj: {1} | Best Obj: {2} | Routing Obj: {3} | Inv Obj: {4} | Prod Setup Obj: {5} | Prod Unit Obj: {6} ", iterCounter,
                            solution.totalObjective, bestLocalSearchSolution.totalObjective, solution.routingCost, solution.holdingCost, solution.setupProductionCost, solution.totalUnitProductionCost, improvingStreak, operatorType);
                    }
                }

                // CUSTOMER FLIPPING
                if (nonImprovingIters > 1000 && nonImprovingIters % 1000 == 1)
                {
                    //Randomize customer saw status
                    GlobalUtils.writeToConsole("CUSTOMER FLIPPING");
                    for (int i = 0; i < solution.customers.Count; i++)
                    {
                        Node cust = solution.customers[i];
                        if (model.input.type == PRP_DATASET_VERSION.BOUDIA_FMT)
                        {
                            if (r.Next() % 100 > 50) //BOUDIA 
                                cust.sawmode = !cust.sawmode;
                        } else
                        {
                            if (r.Next() % 100 > 95) //ARCHETTI + ADUL
                                cust.sawmode = !cust.sawmode;
                        }
                    }
                }

                //WRite to testOutput
                testOutput.Write("{0} {1} {2} {3} {4}\n",
                                iterCounter, solution.totalObjective, solution.status == INFEASIBILITY_STATUS.FEASIBLE ? 1 : 0, testMIP, testLP);


                nonImprovingIters++;
                iterCounter++;
            }
            
            restartElapsedMs = stopwatch.ElapsedMilliseconds;
            totalTimeRepairing = stopwatchRepair.ElapsedMilliseconds;

            bestLocalSearchSolution.feasibleSpaceIters = feasibleSpaceIters;
            bestLocalSearchSolution.infeasibleSpaceIters = infeasibleSpaceIters;
            bestLocalSearchSolution.totalRepairs = totalRepairs;
            bestLocalSearchSolution.totalTimeRepairing = totalTimeRepairing;
            bestLocalSearchSolution.restartElapsedMs = restartElapsedMs;
    }

        private static void InitializeStopwatch()
        {
            stopwatch.Reset();
            stopwatch.Start();
        }

        private static dynamic ExploreAllOperators(ref MOVES operatorType, Random r, bool use_inv, bool use_rout, bool intraBan, bool enableArcReduction, bool enableVisitReduction, bool randomize)
        {
            //INVENTORY OPERATORS
            DayRelocationMove drel;
            DayDeletionMove ddel;
            DayInsertionMove dins;
            
            //ROUTING OPERATORS
            RelocationMove rel;
            SwapMove swap;
            _2OptMove top;

            //MIX OPERATORS
            RelocationWithInventoryMove relwithInv = new RelocationWithInventoryMove();
            relwithInv.totalObjectiveChange = Double.MaxValue; //Neglect move for testing

            //Init everything to empty
            drel = new DayRelocationMove();
            drel.totalObjectiveChange = Double.MaxValue; //Neglect move for testing

            ddel = new DayDeletionMove();
            ddel.totalObjectiveChange = Double.MaxValue; //Neglect move for testing

            dins = new DayInsertionMove();
            dins.totalObjectiveChange = Double.MaxValue; //Neglect move for testing


            rel = new RelocationMove();
            rel.totalObjectiveChange = Double.MaxValue;

            swap = new SwapMove();
            swap.totalObjectiveChange = Double.MaxValue;

            top = new _2OptMove();
            top.totalObjectiveChange = Double.MaxValue;


            //INVENTORY OPERATORS
            if (use_inv)
            {
                if (randomize)
                {
                    int move_id = r.Next() % 3;

                    switch (move_id)
                    {
                        case 0:
                            drel = FindBestDeliveryDayRelocation(enableVisitReduction);
                            break;
                        case 1:
                            ddel = FindBestDeliveryDayDeletion(enableVisitReduction);
                            break;
                        case 2:
                            dins = FindBestDeliveryDayInsertion(enableVisitReduction);
                            break;

                    }
                }
                else
                {
                    drel = FindBestDeliveryDayRelocation(enableVisitReduction);
                    ddel = FindBestDeliveryDayDeletion(enableVisitReduction);
                    dins = FindBestDeliveryDayInsertion(enableVisitReduction);
                }
            }
            
            if (use_rout)
            {

                if (randomize)
                {
                    int move_id = r.Next() % 3;

                    switch (move_id)
                    {
                        case 0:
                            rel = FindBestCustomerRelocation(intraBan, enableArcReduction);
                            break;
                        case 1:
                            swap = FindBestCustomerSwap(intraBan, enableArcReduction);
                            break;
                        case 2:
                            top = FindBestTwoOptMove(enableArcReduction);
                            break;

                    }
                }
                else
                {
                    //Relocates a 1 within the routes of a period, from one route to another
                    rel = FindBestCustomerRelocation(intraBan, enableArcReduction);
                    //Swaps two customers located on the same or different routes of the same period
                    swap = FindBestCustomerSwap(intraBan, enableArcReduction);
                    //Finds a 2Opt move on a route of a period
                    top = FindBestTwoOptMove(enableArcReduction);
                }

            }

            if (NoFeasibleMove(drel, ddel, dins, rel, swap, top, relwithInv))
            {
                operatorType = MOVES.NONE;
                return drel;
            }

            GenerateFeasibleMoveList(feasibleMoves, drel, ddel, dins, rel, swap, top, relwithInv);
            dynamic move;
            
            if (AllOpsCostDeteriorating(feasibleMoves))
                move = RandomlySelectAmongTheFeasibleOperators(r, drel, ddel, dins, rel, swap, top, relwithInv);
            else
                move = OverallBestMove(feasibleMoves);

            if (move is null)
                return null;
            operatorType = move.operatorType;
            return move;
        }

        private static dynamic OverallBestMove(List<dynamic> feasibleMoves)
        {
            double bc = double.MaxValue;
            dynamic overallBestMove = null;
            foreach (var mv in feasibleMoves)
            {
                if (mv.totalObjectiveChange < bc)
                {
                    bc = mv.totalObjectiveChange;
                    overallBestMove = mv;
                }
            }
            return overallBestMove;
        }

        private static bool AllOpsCostDeteriorating(List<dynamic> feasibleMoves)
        {
            foreach (var mv in feasibleMoves)
            {
                if (mv.totalObjectiveChange < -0.00001)
                {
                    return false;
                }
            }
            return true;
        }

        private static void GenerateFeasibleMoveList(List<dynamic> feasibleMoves, DayRelocationMove drel, DayDeletionMove ddel, DayInsertionMove dins, 
            RelocationMove rel, SwapMove swap, _2OptMove top, RelocationWithInventoryMove relwithInv)
        {
            feasibleMoves.Clear();
            if (drel.totalObjectiveChange < double.MaxValue)
            {
                feasibleMoves.Add(drel);
            }
            if (ddel.totalObjectiveChange < double.MaxValue)
            {
                feasibleMoves.Add(ddel);
            }
            if (dins.totalObjectiveChange < double.MaxValue)
            {
                feasibleMoves.Add(dins);
            }
            if (rel.totalObjectiveChange < double.MaxValue)
            {
                feasibleMoves.Add(rel);
            }
            if (swap.totalObjectiveChange < double.MaxValue)
            {
                feasibleMoves.Add(swap);
            }
            if (top.totalObjectiveChange < double.MaxValue)
            {
                feasibleMoves.Add(top);
            }
            if (relwithInv.totalObjectiveChange < double.MaxValue)
            {
                feasibleMoves.Add(relwithInv);
            }
        }

        private static dynamic RandomlySelectAmongTheFeasibleOperators(Random r, DayRelocationMove drel, DayDeletionMove ddel, DayInsertionMove dins, 
            RelocationMove rel, SwapMove swap, _2OptMove top, RelocationWithInventoryMove relwithInv)
        {
            feasibleMoves.Clear();
            if (drel.totalObjectiveChange < double.MaxValue)
            {
                feasibleMoves.Add(drel);
            }

            if (dins.totalObjectiveChange < double.MaxValue)
            {
                feasibleMoves.Add(dins);
            }
            
            if (ddel.totalObjectiveChange < double.MaxValue)
            {
                feasibleMoves.Add(ddel);
            }
            
            
            if (rel.totalObjectiveChange < double.MaxValue)
            {
                feasibleMoves.Add(rel);
            }
            if (swap.totalObjectiveChange < double.MaxValue)
            {
                feasibleMoves.Add(swap);
            }
            if (top.totalObjectiveChange < double.MaxValue)
            {
                feasibleMoves.Add(top);
            }
            if (relwithInv.totalObjectiveChange < double.MaxValue)
            {
                feasibleMoves.Add(relwithInv);
            }
            
            
            if (feasibleMoves.Count == 0)
            {
                //GlobalUtils.writeToConsole((solution.ToString()));
                return null;
            }
                
            return feasibleMoves[r.Next(feasibleMoves.Count)];
        }
        

        private static bool AllOpsCostDeteriorating(DayRelocationMove drel, DayDeletionMove ddel, DayInsertionMove dins, RelocationMove rel, SwapMove swap, _2OptMove top)
        {
            double precision = -0.0001;
            if (drel.totalObjectiveChange >= precision && ddel.totalObjectiveChange >= precision && dins.totalObjectiveChange >= precision &&
                rel.totalObjectiveChange >= precision && swap.totalObjectiveChange >= precision && top.totalObjectiveChange >= precision)
            {
                return true;
            }
            return false;
        }

        //This is ugly, maintain all moves in a move list, use the same base class for all of them and clean this shit
        private static bool NoFeasibleMove(DayRelocationMove drel, DayDeletionMove ddel, DayInsertionMove dins, RelocationMove rel, SwapMove swap, _2OptMove top, RelocationWithInventoryMove relwithInv)
        {
            if (drel.totalObjectiveChange == double.MaxValue && ddel.totalObjectiveChange == double.MaxValue && dins.totalObjectiveChange == double.MaxValue &&
                rel.totalObjectiveChange == double.MaxValue && swap.totalObjectiveChange == double.MaxValue && top.totalObjectiveChange == double.MaxValue && relwithInv.totalObjectiveChange == double.MaxValue)
            {
                return true;
            }
            return false;
        }

        private static void ResetCustomerHeatmap()
        {
            for (int i = 0; i < solution.nodes.Count; i++)
            {
                customerHeatMap[i] = 0;
                for (int j = 0; j < solution.model.horizonDays; j++)
                    customerVisitPerDayHeatMap[i, j] = 0;
            }
        }

        private static void ReportCustomerHeatmap()
        {
            //Get sum of applied moves
            int total_moves = customerHeatMap.Sum();

            for (int i = 1; i < solution.nodes.Count; i++)
            {
                Console.Write("Customer {0} : Move percent: {1:F5}", i, (double)customerHeatMap[i] * 100.0 / total_moves);
                if (customerHeatMap[i] == 0)
                    Console.Write(" - NOT MOVED - ");
                Console.Write("\n");
            }
                            
        }

        private static void ReportCustomerVisitPerDayHeatmap()
        {
            StreamWriter SW = new StreamWriter("heatmap.txt");
            
            for (int i = 1; i < solution.nodes.Count; i++)
            {
                //Get sum of applied moves
                int total_moves = 0;
                for (int j = 0; j < solution.model.horizonDays; j++)
                    total_moves += customerVisitPerDayHeatMap[i, j];

                //Reports
                SW.Write("Customer {0} ", i);
                for (int j = 0; j < solution.model.horizonDays; j++)
                    SW.Write(" {1:F3}", j, (double) customerVisitPerDayHeatMap[i, j] * 100.0 / total_moves);
                SW.Write("\n");
            }

            SW.Close();
        }


        private static void ResetRoutingInventoryPromisesTable()
        {
            for (int i = 0; i < horizonDays; i++)
            {
                for (int k = 0; k < solution.nodes.Count; k++)
                {
                    for (int l = 0; l < solution.nodes.Count; l++)
                    {
                        promisesArcsPerDay[i, k, l] = double.MaxValue;
                    }
                }
            }

            for (int i = 0; i < horizonDays; i++)
            {
                for (int k = 0; k < solution.nodes.Count; k++)
                {
                    promisesCustDay[i, k] = double.MaxValue;
                }
            }
        }

        private static dynamic ExploreAllRoutingOperators(ref int operatorType, Random r, bool enableArcReduction, bool enableVisitReduction)
        {
            RelocationMove rel = FindBestCustomerRelocation(false, enableArcReduction);
            SwapMove swap = FindBestCustomerSwap(false, enableArcReduction);
            _2OptMove top = FindBestTwoOptMove(enableArcReduction);

            if (rel.totalObjectiveChange == double.MaxValue && swap.totalObjectiveChange == double.MaxValue && top.totalObjectiveChange == double.MaxValue)
            {
                return rel;
            }

            if (AllRoutingOpsCostDeteriorating(rel, swap, top))
            {
                dynamic move = null;
                RandomlySelectAmongTheFeasibleRoutingOperators(r, rel, swap, top, ref move, ref operatorType);
                return move;
            }

            if (rel.totalObjectiveChange < swap.totalObjectiveChange && rel.totalObjectiveChange < top.totalObjectiveChange)
            {
                operatorType = 3;
                return rel;
            }
            else if (swap.totalObjectiveChange < top.totalObjectiveChange)
            {
                operatorType = 4;
                return swap;
            }
            else if (top.totalObjectiveChange < double.MaxValue)
            {
                operatorType = 5;
                return top;
            }

            operatorType = -1;
            return top;
        }

        private static void RandomlySelectAmongTheFeasibleRoutingOperators(Random r, RelocationMove rel, SwapMove swap, _2OptMove top, ref dynamic move, ref int opType)
        {
            feasibleOperators.Clear();
            if (rel.totalObjectiveChange < double.MaxValue)
            {
                feasibleOperators.Add(3);
            }
            if (swap.totalObjectiveChange < double.MaxValue)
            {
                feasibleOperators.Add(4);
            }
            if (top.totalObjectiveChange < double.MaxValue)
            {
                feasibleOperators.Add(5);
            }

            opType = feasibleOperators[r.Next(feasibleOperators.Count)];

            if (opType == 3)
            {
                move = rel;
            }
            else if (opType == 4)
            {
                move = swap;
            }
            else if (opType == 5)
            {
                move = top;
            }
        }

        private static bool AllRoutingOpsCostDeteriorating(RelocationMove rel, SwapMove swap, _2OptMove top)
        {
            double precision = -0.0001;
            if (rel.totalObjectiveChange >= precision && swap.totalObjectiveChange >= precision && top.totalObjectiveChange >= precision)
            {
                return true;
            }
            return false;
        }

        private static dynamic ExploreAllInventoryOperators(ref int operatorType, Random r, bool enableArcReduction, bool enableVisitReduction)
        {
            DayRelocationMove rel = FindBestDeliveryDayRelocation(enableVisitReduction);
            DayDeletionMove del = FindBestDeliveryDayDeletion(enableVisitReduction);
            DayInsertionMove ins = FindBestDeliveryDayInsertion(enableVisitReduction);

            if (rel.totalObjectiveChange == double.MaxValue && del.totalObjectiveChange == double.MaxValue && ins.totalObjectiveChange == double.MaxValue)
            {
                return rel;
            }

            if (AllInvOpsCostDeteriorating(rel, del, ins))
            {
                dynamic move = null;
                RandomlySelectAmongTheFeasibleInventoryOperators(r, rel, del, ins, ref move, ref operatorType);
                return move;
            }

            //if (rel.totalObjectiveChange >= 0 && del.totalObjectiveChange >= 0 && ins.totalObjectiveChange >= 0)
            //{
            //    int oper = r.Next(3);
            //    if (oper == 0)
            //    {
            //        operatorType = 0;
            //        return rel;
            //    }
            //    else if (oper == 1)
            //    {
            //        operatorType = 1;
            //        return del;
            //    }
            //    else
            //    {
            //        operatorType = 2;
            //        return ins;
            //    }
            //}

            if (rel.totalObjectiveChange < del.totalObjectiveChange && rel.totalObjectiveChange < ins.totalObjectiveChange)
            {
                operatorType = 0;
                return rel;
            }
            else if (del.totalObjectiveChange < ins.totalObjectiveChange)
            {
                operatorType = 1;
                return del;
            }
            else if (ins.totalObjectiveChange < double.MaxValue)
            {
                operatorType = 2;
                return ins;
            }

            operatorType = -1;
            return ins;
        }

        private static bool AllInvOpsCostDeteriorating(DayRelocationMove rel, DayDeletionMove del, DayInsertionMove ins)
        {
            double precision = -0.0001;
            if (rel.totalObjectiveChange >= precision && del.totalObjectiveChange >= precision && ins.totalObjectiveChange >= precision)
            {
                return true;
            }
            return false;
        }

        private static void RandomlySelectAmongTheFeasibleInventoryOperators(Random r, DayRelocationMove rel, DayDeletionMove del, DayInsertionMove ins, ref dynamic move, ref int opType)
        {
            feasibleOperators.Clear();
            if (rel.totalObjectiveChange < double.MaxValue)
            {
                feasibleOperators.Add(0);
            }
            if (del.totalObjectiveChange < double.MaxValue)
            {
                feasibleOperators.Add(1);
            }
            if (ins.totalObjectiveChange < double.MaxValue)
            {
                feasibleOperators.Add(2);
            }

            opType = feasibleOperators[r.Next(feasibleOperators.Count)];

            if (opType == 0)
            {
                move = rel;
            }
            else if (opType == 1)
            {
                move = del;
            }
            else
            {
                move = ins;
            }
        }

        private static void BasicRandomOperatorEachIteration(Random r, bool enableArcReduction, bool enableVisitReduction)
        {
            int iterationOfBest = 0;
            for (int i = 0; i < 1000000; i++)
            {
                MOVES operatorType = SelectOperatorType(r);
                var bestMove = FindBestMoveForOperatorType(operatorType, enableArcReduction, enableVisitReduction);
                ApplyBestMove(bestMove, operatorType);
                //GlobalUtils.writeToConsole(i + " " + solution.totalObjective + " " + bestLocalSearchSolution.totalObjective);
                solution.TestEverythingFromScratch();
                CloneAndStoreBestLocalSearchSolution(i, ref iterationOfBest);
            }
        }

        private static double CalculateConstantZeroPeriodInvCost()
        {
            double tc = 0;
            tc += (solution.depot.startingInventory) * solution.depot.unitHoldingCost;

            for (int i = 0; i<solution.customers.Count; i++)
            {
                Node n = solution.customers[i];
                tc += (n.unitHoldingCost * n.startingInventory);
            }
            return tc;
        }

        private static void InitializeRouteLists()
        {
            for (int p = 0; p < solution.periods.Count; p++)
            {
                Period per = solution.periods[p];

                for (int i = 0; i < per.periodRoutes.Count; i++)
                {
                    Route rt = per.periodRoutes[i];
                    rt.SetLoadAndCostLists(p, solution.model);
                }
            }
        }

        private static bool CloneAndStoreBestLocalSearchSolution(int iter, ref int iterBestFound)
        {
            if (solution.totalObjective < minimumLocalSearchObjective)
            {
                //Try to further improve the solution
                bool lpstatus = LP.runSimultaneousDeliveryProductionReoptimizationLP(ref solution);
                //bool lpstatus = LP.runSimultaneousDeliveryProductionReoptimizationLPwithCustRemoval(solution);
                
                if (!lpstatus)
                    Console.WriteLine("DELIVERY PRODUCTION LP Failed.");

                double tsp_obj_improv = TSP_GRB_Solver.SolveTSP(solution);

                if (tsp_obj_improv > 0.0)
                    Console.WriteLine("Routes are suboptimal. Optimizing...");
                else if (tsp_obj_improv < 0.0)
                    Console.WriteLine("TSP returned with increased cost");

                Console.WriteLine("Updated Solution objective after extra optimizations : {0}", solution.totalObjective);


                if (solution.totalObjective > minimumLocalSearchObjective)
                    Console.WriteLine("MALAKIES");

                iterBestFound = iter;
                minimumLocalSearchObjective = solution.totalObjective;
                solution.ellapsedMs = stopwatch.ElapsedMilliseconds;
                double totalSeconds = solution.ellapsedMs / 1000;
                bestLocalSearchSolution = new Solution(solution ,0); //Use my operator
                UpdateCustomerVisitPerDayHeatMap(solution);
                
                //solution.depot.ReportInventory();
                //bestLocalSearchSolution = SolutionManager.CloneSolution(solution, model, constantFirstPeriodInvCost);
                Console.WriteLine("NEW BEST | Iteration {0} Best Obj:{1}", iter,
                    bestLocalSearchSolution.totalObjective);
                return true;
            }
            
            return false;
        }

        private static void UpdateCustomerVisitPerDayHeatMap(Solution sol)
        {
            for (int i = 1; i < sol.nodes.Count; i++)
            {
                Node cust = sol.nodes[i];
                for (int j = 0; j < sol.model.horizonDays; j++)
                {
                    if (cust.visitSchedule[j])
                        customerVisitPerDayHeatMap[i, j]++;
                }
            }
        }

        private static void CalculateRoutingObjective()
        {
            solution.routingCost = 0;
            double totalRoutingCost = 0;
            for (int p = 0; p < solution.periods.Count; p++)
            {
                Period per = solution.periods[p];

                for (int r = 0; r < per.periodRoutes.Count; r++)
                {
                    Route rt = per.periodRoutes[r];
                    solution.routingCost += rt.totalRoutingCost;
                }
            }
        }

        private static void CalculateInventoryObjective()
        {
            
        }
        

        private static void CalculateObjectiveComponenents()
        {
            solution.routingCost = Solution.EvaluateRoutingObjectivefromScratch(solution);
            solution.holdingCost = Solution.EvaluateInventoryObjectivefromScratch(solution);
            solution.totalUnitProductionCost = Solution.EvaluateUnitProductionObjectivefromScratch(solution);
            solution.setupProductionCost = Solution.EvaluateSetupProductionObjectivefromScratch(solution);

            solution.totalObjective = solution.routingCost + solution.holdingCost + solution.totalUnitProductionCost +
                                      solution.setupProductionCost;
        }

        
        private static void ApplyBestMove(dynamic bestMove, MOVES operatorType)
        {
            //Return if move is null
            if (bestMove is null)
                return;

            //GlobalUtils.writeToConsole("Applying Operator {0}", operatorType);
            
            switch (operatorType)
            {
                case MOVES.DELIVERYDAYRELOCATION:
                {
                    DayRelocationMove drm = (DayRelocationMove)bestMove;
                    if (drm.totalObjectiveChange < double.MaxValue)
                        ApplyMove(drm);
                    break;
                }
                case MOVES.DELIVERYDAYDELETION:
                {
                    DayDeletionMove ddm = (DayDeletionMove)bestMove;
                    if (ddm.totalObjectiveChange < double.MaxValue)
                        ApplyMove(ddm);
                    break;
                }
                case MOVES.DELIVERYDAYINSERTION:
                {
                    DayInsertionMove dim = (DayInsertionMove)bestMove;
                    if (dim.totalObjectiveChange < double.MaxValue)
                        ApplyMove(dim);
                    break;
                }
                case MOVES.RELOCATE:
                {
                    RelocationMove rm = (RelocationMove)bestMove;
                    if (rm.totalObjectiveChange < double.MaxValue)
                        ApplyMove(rm);
                    break;
                }
                case MOVES.SWAP:
                {
                    SwapMove sm = (SwapMove)bestMove;
                    if (sm.totalObjectiveChange < double.MaxValue)
                        ApplyMove(sm);
                    break;
                }
                case MOVES.TWOOPT:
                {
                    _2OptMove tom = (_2OptMove)bestMove;
                    if (tom.totalObjectiveChange < double.MaxValue)
                        ApplyMove(tom);
                    break;
                }
                case MOVES.RELOCATEWITHINVENTORY:
                {
                    RelocationWithInventoryMove relwithInv = (RelocationWithInventoryMove) bestMove;
                    if (relwithInv.totalObjectiveChange < double.MaxValue)
                        ApplyMove(relwithInv);
                    break;
                }
                case MOVES.NONE:
                    break;
            }
        }

        private static void ApplyMove(_2OptMove tom)
        {
            MakePromises(tom.arcsDeleted);

            Route rt1 = solution.periods[tom.day].periodRoutes[tom.firstRoutePosition];
            Route rt2 = solution.periods[tom.day].periodRoutes[tom.secondRoutePosition];

            if (rt1 == rt2)
            {
                List<Node> reversedSegment = new List<Node>();
                for (int ind = tom.secondNodePosition; ind > tom.firstNodePosition; ind --)
                {
                    reversedSegment.Add(rt1.nodes[ind]);
                }
                for (int ind = 0; ind < reversedSegment.Count; ind++ )
                {
                    rt1.nodes[tom.firstNodePosition + 1 + ind] = reversedSegment[ind];
                }

                rt1.totalRoutingCost += tom.firstRouteObjectiveChange;
            }
            else
            {
                List<Node> rangeRelocatedFromRt2 = rt2.nodes.GetRange(tom.secondNodePosition + 1, rt2.nodes.Count - (tom.secondNodePosition + 1));
                rt2.nodes.RemoveRange(tom.secondNodePosition + 1, rt2.nodes.Count - (tom.secondNodePosition + 1));
                List<Node> rangeRelocatedFromRt1 = rt1.nodes.GetRange(tom.firstNodePosition + 1, rt1.nodes.Count - (tom.firstNodePosition + 1));
                rangeRelocatedFromRt1.ForEach(x => { if (x.uid != 0) x.horizonDeliveryServices[tom.day].route = rt2; });
                rt2.nodes.AddRange(rangeRelocatedFromRt1);

                rt1.nodes.RemoveRange(tom.firstNodePosition + 1, rt1.nodes.Count - (tom.firstNodePosition + 1));
                rt1.nodes.AddRange(rangeRelocatedFromRt2);
                rangeRelocatedFromRt2.ForEach(x => { if (x.uid != 0) x.horizonDeliveryServices[tom.day].route = rt1; });

                int rt1Load = rt1.load;

                rt1.load = rt1.loadTillMe[tom.firstNodePosition] + (rt2.load - rt2.loadTillMe[tom.secondNodePosition]);
                rt2.load = rt2.loadTillMe[tom.secondNodePosition] + (rt1Load - rt1.loadTillMe[tom.firstNodePosition]);

                double costInRelocatedRangeOfRt1 = rt1.costTillEnteringMe.Last() - rt1.costTillEnteringMe[tom.firstNodePosition + 1];
                double costInRelocatedRangeOfRt2 = rt2.costTillEnteringMe.Last() - rt2.costTillEnteringMe[tom.secondNodePosition + 1];

                rt1.totalRoutingCost = rt1.costTillEnteringMe[tom.firstNodePosition] + costInRelocatedRangeOfRt2 + 
                    distMatrix[rt1.nodes[tom.firstNodePosition].uid, rt1.nodes[tom.firstNodePosition + 1].uid];

                rt2.totalRoutingCost = rt2.costTillEnteringMe[tom.secondNodePosition] + costInRelocatedRangeOfRt1 +
                    distMatrix[rt2.nodes[tom.secondNodePosition].uid, rt2.nodes[tom.secondNodePosition + 1].uid];
            }

            solution.routingCost += (tom.firstRouteObjectiveChange + tom.secondRouteObjectiveChange);
            solution.totalObjective += (tom.totalObjectiveChange);

            rt1.SetLoadAndCostLists(tom.day, solution.model);
            rt2.SetLoadAndCostLists(tom.day, solution.model);

            rt1.modified = true;
            rt2.modified = true;

            //GlobalUtils.writeToConsole("Applied 2opt");
        }
        
        private static void ApplyMove(RelocationWithInventoryMove rm)
        {
            Route originRoute = solution.periods[rm.day].periodRoutes[rm.originRoutePosition];
            Route targetRoute = solution.periods[rm.day].periodRoutes[rm.targetRoutePosition];
            Node relocated = originRoute.nodes[rm.originNodePosition];
            
            
            MakePromises(rm.arcsDeleted);
            MakePromisesTotalObjective(relocated, rm.day);
            
            originRoute.nodes.RemoveAt(rm.originNodePosition);

            int insertionPoint = rm.targetNodePosition + 1;

            if (rm.originRoutePosition == rm.targetRoutePosition)
            {
                if (rm.originNodePosition < rm.targetNodePosition)
                {
                    insertionPoint = rm.targetNodePosition;
                }
            }

            targetRoute.nodes.Insert(insertionPoint, relocated);
            
            originRoute.totalRoutingCost += rm.originRouteObjectiveChange;
            targetRoute.totalRoutingCost += rm.targetRouteObjectiveChange;
            
            ArrangeDeliveryScheduleForCustomerRelocationWithInventory(relocated, targetRoute, rm);
            
            solution.routingCost += (rm.originRouteObjectiveChange + rm.targetRouteObjectiveChange);
            solution.holdingCost += (rm.inventoryCustomerObjectiveChange + rm.inventoryDepotObjectiveChange);
            solution.totalObjective += (rm.totalObjectiveChange);

            originRoute.modified = true;
            targetRoute.modified = true;
            
            if (CODE_CHECK)
            {
                CheckInventoryCostForDepotAndCustomer(relocated);
            }
            
            if (!Solution.checkSolutionStatus(solution.TestEverythingFromScratch()))
                Console.WriteLine("MALAKIES TOUMPANA");
            
            //GlobalUtils.writeToConsole("Relocation move applied on day {0}, node {1} from route {2}, pos {3} to route {4}, pos {5}",
            //    rm.day, relocated.uid, rm.originRoutePosition, rm.originNodePosition, rm.targetRoutePosition, rm.targetNodePosition);
        }

        private static void ApplyMove(RelocationMove rm)
        {
            MakePromises(rm.arcsDeleted);

            Route originRoute = solution.periods[rm.day].periodRoutes[rm.originRoutePosition];
            Route targetRoute = solution.periods[rm.day].periodRoutes[rm.targetRoutePosition];

            Node relocated = originRoute.nodes[rm.originNodePosition];
            originRoute.nodes.RemoveAt(rm.originNodePosition);
            originRoute.load = originRoute.load - relocated.deliveredQuantities[rm.day];

            
            int insertionPoint = rm.targetNodePosition + 1;

            if (rm.originRoutePosition == rm.targetRoutePosition)
            {
                if (rm.originNodePosition < rm.targetNodePosition)
                {
                    insertionPoint = rm.targetNodePosition;
                }
            }

            targetRoute.nodes.Insert(insertionPoint, relocated);
            targetRoute.load += relocated.deliveredQuantities[rm.day];

            originRoute.totalRoutingCost += rm.originRouteObjectiveChange;
            targetRoute.totalRoutingCost += rm.targetRouteObjectiveChange;

            solution.routingCost += (rm.originRouteObjectiveChange + rm.targetRouteObjectiveChange);
            solution.totalObjective += (rm.totalObjectiveChange);

            relocated.horizonDeliveryServices[rm.day].route = targetRoute;

            targetRoute.SetLoadAndCostLists(rm.day, solution.model);
            originRoute.SetLoadAndCostLists(rm.day, solution.model);
            
            originRoute.modified = true;
            targetRoute.modified = true;

            if (CODE_CHECK)
            {
                CheckInventoryCostForDepotAndCustomer(relocated);
            }
            
            //GlobalUtils.writeToConsole("Relocation move applied on day {0}, node {1} from route {2}, pos {3} to route {4}, pos {5}",
            //    rm.day, relocated.uid, rm.originRoutePosition, rm.originNodePosition, rm.targetRoutePosition, rm.targetNodePosition);
        }

        private static void MakePromises(int[] arcsDeleted)
        {

            for (int i = 0; i < arcsDeleted[0]; i++)
            {
                int fromId = arcsDeleted[1 + i * 3 + 0];
                int toId = arcsDeleted[1 + i * 3 + 1];
                int day = arcsDeleted[1 + i * 3 + 2];
                promisesArcsPerDay[day, fromId, toId] = solution.totalObjective;
            }
        }

        private static void ApplyMove(SwapMove sm)
        {
            MakePromises(sm.arcsDeleted);

            Route rt1 = solution.periods[sm.day].periodRoutes[sm.firstRoutePosition];
            Route rt2 = solution.periods[sm.day].periodRoutes[sm.secondRoutePosition];

            Node swapped1 = rt1.nodes[sm.firstNodePosition];
            Node swapped2 = rt2.nodes[sm.secondNodePosition];

            rt1.nodes[sm.firstNodePosition] = swapped2;
            rt2.nodes[sm.secondNodePosition] = swapped1;

            rt1.load = rt1.load - swapped1.deliveredQuantities[sm.day] + swapped2.deliveredQuantities[sm.day];
            rt2.load = rt2.load - swapped2.deliveredQuantities[sm.day] + swapped1.deliveredQuantities[sm.day];

            rt1.totalRoutingCost += sm.firstRouteObjectiveChange;
            rt2.totalRoutingCost += sm.secondRouteObjectiveChange;

            solution.routingCost += (sm.firstRouteObjectiveChange + sm.secondRouteObjectiveChange);
            solution.totalObjective += (sm.totalObjectiveChange);

            swapped1.horizonDeliveryServices[sm.day].route = rt2;
            swapped2.horizonDeliveryServices[sm.day].route = rt1;

            rt1.SetLoadAndCostLists(sm.day, solution.model);
            rt2.SetLoadAndCostLists(sm.day, solution.model);

            rt1.modified = true;
            rt2.modified = true;

            if (CODE_CHECK)
            {
                CheckInventoryCostForDepotAndCustomer(swapped1);
                CheckInventoryCostForDepotAndCustomer(swapped2);
            }
            
            //GlobalUtils.writeToConsole("Swap move applied on day {0}, route {1}, pos {2} (Node {3}),  route {4}, pos {5} (Node {6}) ",
            //    sm.day, sm.firstRoutePosition, sm.firstNodePosition, swapped1.uid, sm.secondRoutePosition, sm.secondNodePosition, swapped2.uid);
        }



        private static void ApplyMove(DayInsertionMove dim)
        {
            //Route inserted to
            Route rt_to = solution.periods[dim.insertedDay].periodRoutes[dim.insertedRoutePosition];
            Node insertedCustomer = solution.nodes[dim.insertedCustomer];
            rt_to.nodes.Insert(dim.insertedNodePosition + 1, insertedCustomer);
            rt_to.totalRoutingCost += dim.insertedRouteObjectiveChange;
            rt_to.modified = true;
            
            MakePromisesTotalObjective(insertedCustomer, dim.insertedDay); //STRICT
            ArrangeDeliveryScheduleForDayInsertion(insertedCustomer, rt_to, dim);

            insertedCustomer.totalHoldingCost += dim.inventoryCustomerObjectiveChange;
            solution.depot.totalHoldingCost += dim.inventoryDepotObjectiveChange;

            solution.holdingCost += dim.inventoryCustomerObjectiveChange + dim.inventoryDepotObjectiveChange;
            solution.routingCost += dim.insertedRouteObjectiveChange;
            
            solution.totalObjective += dim.inventoryCustomerObjectiveChange + dim.inventoryDepotObjectiveChange + dim.insertedRouteObjectiveChange;

            UpdateCustomerListsGivenDeliveriesAreValid(insertedCustomer);

            if (CODE_CHECK)
            {
                CheckInventoryCostForDepotAndCustomer(insertedCustomer);
            }

            //Update heatmap
            customerHeatMap[insertedCustomer.uid]++;

            //GlobalUtils.writeToConsole("Inserted visit on day {0} from customer {1} ", dim.insertedDay, insertedCustomer.uid);
        }

        private static void ArrangeDeliveryScheduleForDayInsertion(Node cust, Route rt_to, DayInsertionMove dim)
        {
            //Report status before move application
            //cust.ReportInventory();
            //depot.ReportInventory();
            
            //Recalculate Customer Inventories
            
            //Set new visit schedule
            cust.visitSchedule.CopyTo(cust.auxiliary_visitSchedule, 0);
            cust.auxiliary_visitSchedule[dim.insertedDay] = true;
            
            //Insert new customer delivery
            Node.copyHorizonDeliveries(cust.horizonDeliveryServices, cust.auxiliary_horizonDeliveryServices, horizonDays);
            cust.auxiliary_horizonDeliveryServices[dim.insertedDay].route = rt_to;
            cust.auxiliary_horizonDeliveryServices[dim.insertedDay].quantity = 0;
            
            //Recalculate delivery quantities
            cust.ApplyNewSaw(cust.auxiliary_visitSchedule, solution);
            
            //Repair arrays for prev and next delivery days

            //Recalculate Depot Inventories (Remember that delivered quantities for the depot are negative)

            for (int i = 0; i < horizonDays; i++)
                solution.depot.deliveredQuantities[i] += cust.deliveredQuantities[i] - cust.auxiliary_deliveries[i];
            solution.depot.CalculateInventoryLevels(); //Refresh everything
            
            //Fix affected route loads and capacities
            for (int i = 0; i < horizonDays; i++)
            {
                cust.auxiliary_horizonDeliveryServices[i].quantity = cust.auxiliary_deliveries[i];
                if (cust.auxiliary_horizonDeliveryServices[i].route != null)
                {
                    cust.auxiliary_horizonDeliveryServices[i].route.load +=
                        cust.auxiliary_deliveries[i] - cust.deliveredQuantities[i];
                }
            }
            
            //Save customer temp state
            //Save auxiliary quantities to customer
            cust.saveTempState();
            cust.CalculateInventoryLevels(); //Refresh everything just to make sure
            
            //Recalculate visit days
            cust.CalculateVisitDays(cust.visitSchedule);
            
            //Recalculate LoadTillMe arrays now that everything is cached
            for (int i = 0; i < horizonDays; i++)
            {
                if (cust.horizonDeliveryServices[i].route != null)
                    cust.horizonDeliveryServices[i].route.SetLoadAndCostLists(i, model);
            }
            
            //GlobalUtils.writeToConsole("AFTER");    
            //Report status before move application
            //cust.ReportInventory();
            //depot.ReportInventory();
        }

        private static void ApplyMove(DayRelocationMove drm)
        {

            //Route removed from
            Route rt_from = solution.periods[drm.removedDay].periodRoutes[drm.removedRoutePosition];
            Node cust = rt_from.nodes[drm.removedNodePosition];
            rt_from.nodes.RemoveAt(drm.removedNodePosition);
            rt_from.totalRoutingCost = rt_from.totalRoutingCost + drm.removedRouteObjectiveChange;

            //Route inserted to
            Route rt_to = solution.periods[drm.insertedDay].periodRoutes[drm.insertedRoutePosition];
            rt_to.nodes.Insert(drm.insertedNodePosition + 1, cust);
            rt_to.totalRoutingCost = rt_to.totalRoutingCost + drm.insertedRouteObjectiveChange;

            MakePromisesTotalObjective(cust, drm.removedDay);
            MakePromisesTotalObjective(cust, drm.insertedDay); //STRICT
            
            ArrangeDeliveryScheduleForDayRelocation(cust, rt_from, rt_to, drm);

            cust.totalHoldingCost += drm.inventoryCustomerObjectiveChange;
            solution.depot.totalHoldingCost += drm.inventoryDepotObjectiveChange;

            solution.holdingCost += drm.inventoryCustomerObjectiveChange + drm.inventoryDepotObjectiveChange;
            solution.routingCost += drm.removedRouteObjectiveChange + drm.insertedRouteObjectiveChange;

            solution.totalObjective += drm.inventoryCustomerObjectiveChange + drm.inventoryDepotObjectiveChange +
                drm.removedRouteObjectiveChange + drm.insertedRouteObjectiveChange;

            rt_from.modified = true;
            rt_to.modified = true;

            UpdateCustomerListsGivenDeliveriesAreValid(cust);


            if (CODE_CHECK)
            {
                CheckInventoryCostForDepotAndCustomer(cust);
            }


            //Update heatmap
            customerHeatMap[cust.uid]++;

            //GlobalUtils.writeToConsole("Relocated customer {0} from day {1} to day {2}", cust.uid, drm.removedDay, drm.insertedDay);
            //cust.ReportInventory();
            //depot.ReportInventory();
        }

        private static void MakePromises(Node cust, int dayLeftFrom)
        {
            promisesCustDay[dayLeftFrom, cust.uid] = solution.holdingCost;
        }

        private static void MakePromisesTotalObjective(Node cust, int dayLeftFrom)
        {
            //OLD
            //promisesCustDay[dayLeftFrom, cust.uid] = solution.totalObjective;

            //STRICT VERSION
            //MAKE PROMISE FOR ALL DAYS
            for (int i = 0; i < horizonDays; i++)
                promisesCustDay[i, cust.uid] = solution.totalObjective;
            
        }

        private static void ApplyMove(DayDeletionMove ddm)
        {
            //if (solution.TestEverythingFromScratch() == INFEASIBILITY_STATUS.MISCALC_OBJ_HOLDING_COST)
            //    GlobalUtils.writeToConsole("MALAKIES");

            //Route removed from
            Route rt = solution.periods[ddm.removedDay].periodRoutes[ddm.removedRoutePosition];
            Node cust = rt.nodes[ddm.removedNodePosition];
            rt.nodes.RemoveAt(ddm.removedNodePosition);
            rt.totalRoutingCost = rt.totalRoutingCost + ddm.removedRouteObjectiveChange;

            MakePromisesTotalObjective(cust, ddm.removedDay);

            ArrangeDeliveryScheduleForDayDeletion(cust, ddm);

            cust.totalHoldingCost += ddm.inventoryCustomerObjectiveChange;
            solution.depot.totalHoldingCost += ddm.inventoryDepotObjectiveChange;

            solution.holdingCost += ddm.inventoryCustomerObjectiveChange + ddm.inventoryDepotObjectiveChange;
            solution.routingCost += ddm.removedRouteObjectiveChange;

            solution.totalObjective += ddm.inventoryCustomerObjectiveChange + ddm.inventoryDepotObjectiveChange +
                ddm.removedRouteObjectiveChange;

            UpdateCustomerListsGivenDeliveriesAreValid(cust);

            if (CODE_CHECK)
            {
                CheckInventoryCostForDepotAndCustomer(cust);
            }

            //Update heatmap
            customerHeatMap[cust.uid]++;

            //if (solution.TestEverythingFromScratch() == INFEASIBILITY_STATUS.MISCALC_OBJ_HOLDING_COST)
            //    GlobalUtils.writeToConsole("MALAKIES");

            //GlobalUtils.writeToConsole("Deleted visit on day {0} from customer {1} ", ddm.removedDay, cust.uid);
        }
        
        private static void ArrangeDeliveryScheduleForCustomerRelocationWithInventory(Node cust, Route rt_to, RelocationWithInventoryMove relwithInv)
        {
            //GlobalUtils.writeToConsole("STATUS BEFORE THE MOVE APPLICATION");
            //cust.ReportInventory();
            //solution.depot.ReportInventory();
            
            Route old_route = cust.horizonDeliveryServices[relwithInv.day].route;
            
            //Copy delivery services
            Node.copyHorizonDeliveries(cust.horizonDeliveryServices, cust.auxiliary_horizonDeliveryServices, horizonDays);
            cust.auxiliary_horizonDeliveryServices[relwithInv.day].reset();
            cust.auxiliary_horizonDeliveryServices[relwithInv.day].quantity = 0;
            cust.auxiliary_horizonDeliveryServices[relwithInv.day].route = rt_to;


            if (relwithInv.originRoutePosition != relwithInv.targetRoutePosition)
                cust.ApplyNewSaw(cust.auxiliary_visitSchedule, solution, true, relwithInv.day);
            else
                cust.ApplyNewSaw(cust.auxiliary_visitSchedule, solution);
                
            //Recalculate Depot Inventories (Remember that delivered quantities for the depot are negative)
            for (int i = 0; i < horizonDays; i++)
                solution.depot.deliveredQuantities[i] += cust.deliveredQuantities[i] - cust.auxiliary_deliveries[i];
            solution.depot.CalculateInventoryLevels(); //Refresh everything
            
            //Fix affected route loads and capacities
            for (int i = 0; i < horizonDays; i++)
            {
                cust.auxiliary_horizonDeliveryServices[i].quantity = cust.auxiliary_deliveries[i];
                if (cust.auxiliary_horizonDeliveryServices[i].route != null && i != relwithInv.day)
                {
                    cust.auxiliary_horizonDeliveryServices[i].route.load +=    
                        cust.auxiliary_deliveries[i] - cust.deliveredQuantities[i];
                } else if (i == relwithInv.day)
                {
                    cust.auxiliary_horizonDeliveryServices[i].route.load +=    
                        cust.auxiliary_deliveries[i];
                }
            }
            
            //Replenish load on old route
            old_route.load -= cust.deliveredQuantities[relwithInv.day];
            
            //Save customer temp state
            //Save auxiliary quantities to customer
            cust.saveTempState();
            cust.CalculateInventoryLevels(); //Refresh everything just to make sure
            
            //Recalculate visit days
            cust.CalculateVisitDays(cust.visitSchedule);
            
            //Recalculate LoadTillMe arrays now that everything is cached
            for (int i = 0; i < horizonDays; i++)
            {
                if (cust.horizonDeliveryServices[i].route != null)
                    cust.horizonDeliveryServices[i].route.SetLoadAndCostLists(i, model);
            }
            
            //Fix stuff on the old route
            old_route.SetLoadAndCostLists(relwithInv.day, model);
            
            //GlobalUtils.writeToConsole("AFTER");
            //Report status before move application
            //cust.ReportInventory();
            //solution.depot.ReportInventory();
            
        }

        private static void ArrangeDeliveryScheduleForDayRelocation(Node cust, Route rt_from, Route rt_to,
            DayRelocationMove drm)
        {
            //GlobalUtils.writeToConsole("STATUS BEFORE THE MOVE APPLICATION");
            //cust.ReportInventory();
            //depot.ReportInventory();
            
            //Report status before move application
            //custToBeRelocated.ReportInventory();
            //depot.ReportInventory();

            Route old_route = cust.horizonDeliveryServices[drm.removedDay].route;
            
            //Set new visit schedule
            cust.visitSchedule.CopyTo(cust.auxiliary_visitSchedule, 0);
            cust.auxiliary_visitSchedule[drm.removedDay] = false;
            cust.auxiliary_visitSchedule[drm.insertedDay] = true;
            
            //Copy delivery services
            Node.copyHorizonDeliveries(cust.horizonDeliveryServices, cust.auxiliary_horizonDeliveryServices, horizonDays);
            cust.auxiliary_horizonDeliveryServices[drm.removedDay].reset();
            cust.auxiliary_horizonDeliveryServices[drm.insertedDay].quantity = 0;
            cust.auxiliary_horizonDeliveryServices[drm.insertedDay].route = rt_to;

            //Recalculate delivery quantities
            cust.ApplyNewSaw(cust.auxiliary_visitSchedule, solution);
            
            //Recalculate Depot Inventories (Remember that delivered quantities for the depot are negative)
            for (int i = 0; i < horizonDays; i++)
                solution.depot.deliveredQuantities[i] += cust.deliveredQuantities[i] - cust.auxiliary_deliveries[i];
            solution.depot.CalculateInventoryLevels(); //Refresh everything
            
            //Fix affected route loads and capacities
            for (int i = 0; i < horizonDays; i++)
            {
                cust.auxiliary_horizonDeliveryServices[i].quantity = cust.auxiliary_deliveries[i];
                if (cust.auxiliary_horizonDeliveryServices[i].route != null)
                {
                    cust.auxiliary_horizonDeliveryServices[i].route.load +=    
                        cust.auxiliary_deliveries[i] - cust.deliveredQuantities[i];
                }
            }
            
            //Replenish load on old route
            old_route.load -= cust.deliveredQuantities[drm.removedDay];
            
            //Save customer temp state
            //Save auxiliary quantities to customer
            cust.saveTempState();
            cust.CalculateInventoryLevels(); //Refresh everything just to make sure
            
            //Recalculate visit days
            cust.CalculateVisitDays(cust.visitSchedule);
            
            //Recalculate LoadTillMe arrays now that everything is cached
            for (int i = 0; i < horizonDays; i++)
            {
                if (cust.horizonDeliveryServices[i].route != null)
                    cust.horizonDeliveryServices[i].route.SetLoadAndCostLists(i, model);
            }
            
            //Fix stuff on the old route
            old_route.SetLoadAndCostLists(drm.removedDay, model);
            
            //GlobalUtils.writeToConsole("AFTER");
            //Report status before move application
            //cust.ReportInventory();
            //depot.ReportInventory();
            
        }

        private static void ArrangeDeliveryScheduleForDayDeletion(Node custToBeRelocated, DayDeletionMove drm)
        {
            //Report status before move application
            //custToBeRelocated.ReportInventory();
            //depot.ReportInventory();

            Route old_route = custToBeRelocated.horizonDeliveryServices[drm.removedDay].route;
            
            //Set new visit schedule
            custToBeRelocated.visitSchedule.CopyTo(custToBeRelocated.auxiliary_visitSchedule, 0);
            custToBeRelocated.auxiliary_visitSchedule[drm.removedDay] = false;
            
            //Copy delivery services\
            Node.copyHorizonDeliveries(custToBeRelocated.horizonDeliveryServices, 
                custToBeRelocated.auxiliary_horizonDeliveryServices, horizonDays);
            custToBeRelocated.auxiliary_horizonDeliveryServices[drm.removedDay].route = null;
            custToBeRelocated.auxiliary_horizonDeliveryServices[drm.removedDay].quantity = 0;
            
            //Recalculate delivery quantities
            custToBeRelocated.ApplyNewSaw(custToBeRelocated.auxiliary_visitSchedule, solution);
            
            //Recalculate Depot Inventories (Remember that delivered quantities for the depot are negative)
            for (int i = 0; i < horizonDays; i++)
                solution.depot.deliveredQuantities[i] += custToBeRelocated.deliveredQuantities[i] - custToBeRelocated.auxiliary_deliveries[i];
            solution.depot.CalculateInventoryLevels(); //Refresh everything
            
            //Fix affected route loads and capacities
            for (int i = 0; i < horizonDays; i++)
            {
                custToBeRelocated.auxiliary_horizonDeliveryServices[i].quantity = custToBeRelocated.auxiliary_deliveries[i];
                if (custToBeRelocated.auxiliary_horizonDeliveryServices[i].route != null)
                {
                    custToBeRelocated.auxiliary_horizonDeliveryServices[i].route.load +=
                        custToBeRelocated.auxiliary_deliveries[i] - custToBeRelocated.deliveredQuantities[i];
                }
            }
            
            //Replenish load on old route
            old_route.load -= custToBeRelocated.deliveredQuantities[drm.removedDay];
            
            //Save customer temp state
            //Save auxiliary quantities to customer
            custToBeRelocated.saveTempState();
            custToBeRelocated.CalculateInventoryLevels(); //Refresh everything just to make sure
            
            //Recalculate visit days
            custToBeRelocated.CalculateVisitDays(custToBeRelocated.visitSchedule);
            
            //Recalculate LoadTillMe arrays now that everything is cached
            for (int i = 0; i < horizonDays; i++)
            {
                if (custToBeRelocated.horizonDeliveryServices[i].route != null)
                    custToBeRelocated.horizonDeliveryServices[i].route.SetLoadAndCostLists(i, model);
            }
            
            //Fix stuff on the old route
            old_route.SetLoadAndCostLists(drm.removedDay, model);
            
            //GlobalUtils.writeToConsole("AFTER");
            //Report status before move application
            //cust.ReportInventory();
            //depot.ReportInventory();
            
        }
        
        private static void UpdateCustomerListsGivenDeliveriesAreValid(Node cust)
        {
            ArrangeStockLevel(cust);
            ArrangeDaysFromToDelivery(cust);
            ArrangeMinimumStockTillNextDelivery(cust);
            ArrangeMinimumStockDaysAhead(cust);
        }

        private static void CheckInventoryCostForDepotAndCustomer(Node cust)
        {
            //Starting from horizon Deliveries (which should be OK) of customer calculate everything
            double invCost = 0;
            for (int i = 0; i < cust.endDayInventory.Length; i++)
            {
                invCost += cust.endDayInventory[i] * cust.unitHoldingCost;
            }
            if (!GlobalUtils.IsEqual(invCost, cust.totalHoldingCost))
            {
                Console.WriteLine("Customer holding Cost problem");
            }

            //double depotStock = depot.startingInventory;
            //for (int i = 0; i < solution.periods.Count; i++)
            //{
            //    depotStock += depot.productRate;
            //    depot.endDayInventory[i] = depotStock;
            //}

            int depotStock = solution.depot.startingInventory;

            //check route load consistency with customers and routing costs
            for (int i = 0; i < solution.periods.Count; i++)
            {
                depotStock += solution.depot.productionRates[i];
                Period p = solution.periods[i];
                int totalPeriodProduct = 0;
                for (int k = 0; k < p.periodRoutes.Count; k++)
                {
                    Route rt = p.periodRoutes[k];
                    int rtLoad = 0;
                    for (int nd = 1; nd < rt.nodes.Count - 1; nd++)
                    {
                        Node c = rt.nodes[nd];
                        rtLoad += c.deliveredQuantities[i];
                    }
                    if (!(rtLoad == rt.load))
                    {
                        GlobalUtils.writeToConsole("Customer holding Cost problem");
                    }
                    totalPeriodProduct += rtLoad;
                }
                depotStock = depotStock - totalPeriodProduct;
                solution.depot.endDayInventory[i] = depotStock;
            }

            double invDepotCost = 0;
            for (int i = 0; i < solution.depot.endDayInventory.Length; i++)
            {
                invDepotCost += solution.depot.endDayInventory[i] * solution.depot.unitHoldingCost;
            }
            if (!GlobalUtils.IsEqual(invDepotCost, solution.depot.totalHoldingCost))
            {
                Console.WriteLine("Customer holding Cost problem");
            }
        }

        private static void UpdateHorizonDeliveryServices(Node cust)
        {
            throw new NotImplementedException();
        }

        private static void InitializeCustomerLists()
        {
            //horizonDeliveryServices
            ArrangeHorizonDeliveryServices();

            //endDayInventory
            for (int i = 0; i < solution.customers.Count; i++)
            {
                ArrangeStockLevel(solution.customers[i]);
            }

            //daysTillNextDeliveryOrEnd
            //daysTillPreviousDeliveryOrStart
            for (int nd = 0; nd < solution.customers.Count; nd++)
            {
                ArrangeDaysFromToDelivery(solution.customers[nd]);
            }

            for (int nd = 0; nd < solution.customers.Count; nd++)
            {
                //minimumStockTillNextDelivery_StartInc
                ArrangeMinimumStockTillNextDelivery(solution.customers[nd]);
            }

            for (int nd = 0; nd < solution.customers.Count; nd++)
            {
                //minimumStockTillNextDelivery_StartInc
                ArrangeMinimumStockDaysAhead(solution.customers[nd]);
            }
        }

        private static void ArrangeMinimumStockDaysAhead(Node cust)
        {
            for (int d = 0; d < horizonDays; d++)
            {
                double minStock = double.MaxValue;

                for (int k = 0; k + d < horizonDays; k++)
                {
                    if (cust.endDayInventory[k + d] < minStock)
                    {
                        minStock = cust.endDayInventory[k + d];
                    }

                    cust.minimumStockDaysAhead_StartInc[d, k] = minStock;
                }
            }
        }

        private static void ArrangeMinimumStockTillNextDelivery(Node cust)
        {
            for (int d = 0; d < horizonDays; d++)
            {
                double minimumStock = cust.endDayInventory[d];

                for (int k = d + 1; k < horizonDays; k++)
                {
                    if (cust.horizonDeliveryServices[k].route != null)
                    {
                        break;
                    }

                    if (cust.endDayInventory[k] < minimumStock)
                    {
                        minimumStock = cust.endDayInventory[k];
                    }
                }
                cust.minimumStockTillNextDelivery_StartInc[d] = minimumStock;
            }
        }

        private static void ArrangeDaysFromToDelivery(Node cust)
        {
            for (int p = 0; p < horizonDays; p++)
            {
                int nextIndex = NextDeliveryIndexOrEnd(p, cust);
                if (nextIndex < int.MaxValue)
                {
                    cust.daysTillNextDeliveryOrEnd[p] = nextIndex - p;
                }
                else
                {
                    cust.daysTillNextDeliveryOrEnd[p] = nextIndex;
                }


                int prevIndex = PreviousDeliveryIndexOrStart(p, cust);
                if (prevIndex < int.MaxValue)
                {
                    cust.daysTillPreviousDeliveryOrStart[p] = p - prevIndex;
                }
                else
                {
                    cust.daysTillPreviousDeliveryOrStart[p] = prevIndex;
                }
            }

        }

        private static int NextDeliveryIndexOrEnd(int p, Node cust)
        {
            for (int i = p + 1; i < cust.deliveredQuantities.Length; i++)
            {
                if (cust.deliveredQuantities[i] > 0)
                {
                    return i;
                }
            }
            return int.MaxValue;
        }

        private static int PreviousDeliveryIndexOrStart(int p, Node cust)
        {
            for (int i = p - 1; i >= 0; i--)
            {
                if (cust.deliveredQuantities[i] > 0)
                {
                    return i;
                }
            }
            return int.MaxValue;
        }

        private static void ArrangeHorizonDeliveryServices()
        {
            for (int i = 0; i < horizonDays; i++)
            {
                for (int j = 0; j < solution.periods[i].periodRoutes.Count; j++)
                {
                    Route rt = solution.periods[i].periodRoutes[j];

                    for (int n = 1; n < rt.nodes.Count - 1; n++)
                    {
                        Node nd = rt.nodes[n];
                        nd.horizonDeliveryServices[i].route = rt;
                        nd.horizonDeliveryServices[i].quantity = nd.deliveredQuantities[i];
                    }
                }
            }
        }

        private static void ArrangeStockLevel(Node cust)
        {
            int stock = cust.startingInventory;
            
            for (int i = 0; i < horizonDays; i++)
            {
                cust.startDayInventory[i] = stock;

                stock -= cust.productRate[i];
                stock += cust.deliveredQuantities[i];
                
                cust.endDayInventory[i] = stock;
            }
        }

        private static void CreateEmptyCustomerLists()
        {
            //This method is emptying only the auxiliary lists used for local search
            for (int i = 0; i < solution.customers.Count; i++)
            {
                Node cust = solution.customers[i];

                cust.minimumStockDaysAhead_StartInc = new double[model.horizonDays, model.horizonDays];
                cust.minimumStockTillNextDelivery_StartInc = new double[model.horizonDays];
                cust.daysTillNextDeliveryOrEnd = new int[model.horizonDays];
                cust.daysTillPreviousDeliveryOrStart = new int[model.horizonDays];
                cust.auxiliary_RemainingStock = new double[model.horizonDays];
                cust.auxiliary_deliveries = new int[model.horizonDays];
            }
                
        }

        
        private static void UpdateCustomerLists()
        {
            for (int i = 0; i < solution.customers.Count; i++)
            {
                Node n = solution.customers[i];

                //minimumStockTillNextDelivery_StartInc
                for (int d = 0; d < horizonDays; d++)
                {
                    double minimumStock = n.endDayInventory[d];

                    for (int k = d + 1; k < horizonDays; k++)
                    {
                        if (n.horizonDeliveryServices[k].route != null)
                        {
                            break;
                        }

                        if (n.endDayInventory[k] < minimumStock)
                        {
                            minimumStock = n.endDayInventory[k];
                        }
                    }
                    n.minimumStockTillNextDelivery_StartInc[d] = minimumStock;
                }

                for (int d = 0; d < horizonDays; d++)
                {
                    double minStock = double.MaxValue;

                    for (int k = 0; k + d < horizonDays; k++)
                    {
                        if (n.endDayInventory[k + d] < minStock)
                        {
                            minStock = n.endDayInventory[k + d];
                        }

                        n.minimumStockDaysAhead_StartInc[d, k] = minStock;
                    }
                }
            }
        }

        private static dynamic FindBestMoveForOperatorType(MOVES operatorType, bool enableArcReduction, bool enableVisitReduction)
        {
            switch (operatorType)
            {
                case MOVES.DELIVERYDAYRELOCATION:
                    return FindBestDeliveryDayRelocation(enableVisitReduction);
                case MOVES.DELIVERYDAYDELETION:
                    return FindBestDeliveryDayDeletion(enableVisitReduction);
                case MOVES.DELIVERYDAYINSERTION:
                    return FindBestDeliveryDayInsertion(enableVisitReduction);
                case MOVES.RELOCATE:
                    return FindBestCustomerRelocation(false, enableArcReduction);
                case MOVES.SWAP:
                    return FindBestCustomerSwap(false, enableArcReduction);
                case MOVES.TWOOPT:
                    return FindBestTwoOptMove(enableArcReduction);
                default:
                    return null;
            }
        }

        private static _2OptMove FindBestTwoOptMove(bool enableArcReduction)
        {
            _2OptMove bestMove = new _2OptMove();

            for (int p = 0; p < solution.periods.Count; p++)
            {
                List<Route> routes = solution.periods[p].periodRoutes;

                for (int firstRouteIndex = 0; firstRouteIndex < routes.Count; firstRouteIndex++)
                {
                    Route rt1 = (routes[firstRouteIndex]);

                    int rt1_effective_capacity = 0;
                    //Prevent relocation moves on period 0 on boudia instances in order to prevent capacity violations within the period
                    if (model.input.type == PRP_DATASET_VERSION.BOUDIA_FMT && p == 0)
                        rt1_effective_capacity = rt1.realCapacity;
                    else
                        rt1_effective_capacity = rt1.effectiveCapacity;

                    for (int secondRouteIndex = firstRouteIndex; secondRouteIndex < routes.Count; secondRouteIndex++)
                    {
                        Route rt2 = (routes[secondRouteIndex]);

                        int rt2_effective_capacity = 0;
                        //Prevent relocation moves on period 0 on boudia instances in order to prevent capacity violations within the period
                        if (model.input.type == PRP_DATASET_VERSION.BOUDIA_FMT && p == 0)
                            rt2_effective_capacity = rt2.realCapacity;
                        else
                            rt2_effective_capacity = rt2.effectiveCapacity;

                        int firstNodeEnd = rt1.nodes.Count - 1;
                        if (rt1 == rt2)
                        {
                            firstNodeEnd = rt1.nodes.Count - 3;
                        }

                        for (int firstNodeIndex = 0; firstNodeIndex < firstNodeEnd; firstNodeIndex++)
                        {
                            int startOfSecondNodeIndex = 0;
                            if (rt1 == rt2)
                            {
                                startOfSecondNodeIndex = firstNodeIndex + 2;
                            }

                            double costChangeFirstRoute = 0;
                            double costChangeSecondRoute = 0;

                            for (int secondNodeIndex = startOfSecondNodeIndex; secondNodeIndex < rt2.nodes.Count - 1; secondNodeIndex++)
                            {
                                if (rt1 == rt2)
                                {
                                    Node A = (rt1.nodes[firstNodeIndex]);
                                    Node B = (rt1.nodes[firstNodeIndex + 1]);
                                    Node G = (rt1.nodes[secondNodeIndex]);
                                    Node F = (rt1.nodes[secondNodeIndex + 1]);

                                    if (firstNodeIndex == 0 && secondNodeIndex == rt1.nodes.Count - 2)
                                    {
                                        continue;
                                    }

                                    double costAdded = distMatrix[A.uid, G.uid] + distMatrix[B.uid, F.uid];
                                    double costRemoved = distMatrix[A.uid, B.uid] + distMatrix[G.uid, F.uid];

                                    costChangeFirstRoute = costAdded - costRemoved;

                                    BuilArcList(arcsDaysCreated, A.uid, G.uid, p, B.uid, F.uid, p);
                                    BuilArcList(arcsDaysDeleted, A.uid, B.uid, p, G.uid, F.uid, p);

                                    if (!arcReduction(arcsDaysCreated))
                                        continue;

                                    if (!PromiseKeepingORG(arcsDaysCreated, costChangeFirstRoute))
                                    {
                                        continue;
                                    }

                                    
                                }
                                else
                                {
                                    Node A = (rt1.nodes[firstNodeIndex]);
                                    Node B = (rt1.nodes[firstNodeIndex + 1]);
                                    Node G = (rt2.nodes[secondNodeIndex]);
                                    Node F = (rt2.nodes[secondNodeIndex + 1]);

                                    if (firstNodeIndex == 0 && secondNodeIndex == 0)
                                    {
                                        continue;
                                    }
                                    if (firstNodeIndex == rt1.nodes.Count - 2 && secondNodeIndex == rt2.nodes.Count - 2)
                                    {
                                        continue;
                                    }

                                    if (rt1.loadTillMe[firstNodeIndex] + (rt2.load - rt2.loadTillMe[secondNodeIndex]) > rt1_effective_capacity)
                                    {
                                        continue;
                                    }
                                    if (rt2.loadTillMe[secondNodeIndex] + (rt1.load - rt1.loadTillMe[firstNodeIndex]) > rt2_effective_capacity)
                                    {
                                        continue;
                                    }

                                    costChangeFirstRoute = distMatrix[A.uid, F.uid] - distMatrix[A.uid, B.uid];
                                    costChangeSecondRoute = distMatrix[G.uid, B.uid] - distMatrix[G.uid, F.uid];

                                    BuilArcList(arcsDaysCreated, A.uid, F.uid, p, G.uid, B.uid, p);
                                    BuilArcList(arcsDaysDeleted, A.uid, B.uid, p, G.uid, F.uid, p);

                                    if (!arcReduction(arcsDaysCreated))
                                        continue;

                                    if (!PromiseKeepingORG(arcsDaysCreated, costChangeFirstRoute + costChangeSecondRoute))
                                    {
                                        continue;
                                    }
                                }

                                if (Math.Abs(costChangeFirstRoute + costChangeSecondRoute) < 0.0000001)
                                {
                                    continue;
                                }

                                StoreBest2OptMove(p, firstRouteIndex, secondRouteIndex, firstNodeIndex, secondNodeIndex, costChangeFirstRoute, costChangeSecondRoute, arcsDaysDeleted, bestMove);
                            }
                        }
                    }
                }
            }
            return bestMove;
        }

        private static void StoreBest2OptMove(int p, int firstRouteIndex, int secondRouteIndex, int firstNodeIndex, int secondNodeIndex, double costChangeFirstRoute, 
            double costChangeSecondRoute, int[] arcsDaysDeleted, _2OptMove bestMove)
        {
            double totalObjectiveChange = costChangeFirstRoute + costChangeSecondRoute;

            if (totalObjectiveChange < bestMove.totalObjectiveChange)
            {
                bestMove.totalObjectiveChange = totalObjectiveChange;
                bestMove.day = p;
                bestMove.firstRoutePosition = firstRouteIndex;
                bestMove.firstNodePosition = firstNodeIndex;
                bestMove.secondRoutePosition = secondRouteIndex;
                bestMove.secondNodePosition = secondNodeIndex;
                bestMove.firstRouteObjectiveChange = costChangeFirstRoute;
                bestMove.secondRouteObjectiveChange = costChangeSecondRoute;
                CloneArcs(bestMove.arcsDeleted, arcsDaysDeleted);
            }
        }

        private static void CloneArcs(int[] result, int[] arcsDaysDeleted)
        {
            for (int i = 0; i < arcsDaysDeleted[0] * 3 + 1; i++)
                result[i] = arcsDaysDeleted[i];
        }

        private static SwapMove FindBestCustomerSwap(bool intraBan, bool enableArcReduction)
        {
            SwapMove bestMove = new SwapMove();

            for (int p = 0; p < solution.periods.Count; p++)
            {
                List<Route> routes = solution.periods[p].periodRoutes;
                for (int firstRouteIndex = 0; firstRouteIndex < routes.Count; firstRouteIndex++)
                {
                    Route rt1 = (routes[firstRouteIndex]);

                    int rt1_effective_capacity = 0;
                    //Prevent relocation moves on period 0 on boudia instances in order to prevent capacity violations within the period
                    if (model.input.type == PRP_DATASET_VERSION.BOUDIA_FMT && p == 0)
                        rt1_effective_capacity = rt1.realCapacity;
                    else
                        rt1_effective_capacity = rt1.effectiveCapacity;

                    for (int secondRouteIndex = firstRouteIndex; secondRouteIndex < routes.Count; secondRouteIndex++)
                    {
                        Route rt2 = (routes[secondRouteIndex]);

                        if ((firstRouteIndex == secondRouteIndex) && intraBan)
                            continue;
                        
                        int rt2_effective_capacity = 0;
                        //Prevent relocation moves on period 0 on boudia instances in order to prevent capacity violations within the period
                        if (model.input.type == PRP_DATASET_VERSION.BOUDIA_FMT && p == 0)
                            rt2_effective_capacity = rt2.realCapacity;
                        else
                            rt2_effective_capacity = rt2.effectiveCapacity;

                        for (int firstNodeIndex = 1; firstNodeIndex < rt1.nodes.Count - 1; firstNodeIndex++)
                        {
                            int startOfSecondNodeIndex = 1;
                            if (rt1 == rt2)
                            {
                                startOfSecondNodeIndex = firstNodeIndex + 1;
                            }
                            for (int secondNodeIndex = startOfSecondNodeIndex; secondNodeIndex < rt2.nodes.Count - 1; secondNodeIndex++)
                            {
                                Node a1 = rt1.nodes[firstNodeIndex - 1];
                                Node b1 = rt1.nodes[firstNodeIndex];
                                Node c1 = rt1.nodes[firstNodeIndex + 1];

                                Node a2 = rt2.nodes[secondNodeIndex - 1];
                                Node b2 = rt2.nodes[secondNodeIndex];
                                Node c2 = rt2.nodes[secondNodeIndex + 1];

                                double costChangeFirstRoute = 0;
                                double costChangeSecondRoute = 0;

                                if (rt1 == rt2 && firstNodeIndex == secondNodeIndex - 1)
                                {

                                    double costRemoved = distMatrix[a1.uid, b1.uid] + distMatrix[b1.uid, b2.uid] + distMatrix[b2.uid, c2.uid];
                                    double costAdded = distMatrix[a1.uid, b2.uid] + distMatrix[b2.uid, b1.uid] + distMatrix[b1.uid, c2.uid];

                                    costChangeFirstRoute = costAdded - costRemoved;
                                    costChangeSecondRoute = 0;

                                    BuilArcList(arcsDaysCreated, a1.uid, b2.uid, p, b2.uid, b1.uid, p, b1.uid, c2.uid, p);
                                    BuilArcList(arcsDaysDeleted, a1.uid, b1.uid, p, b1.uid, b2.uid, p, b2.uid, c2.uid, p);


                                    if (!arcReduction(arcsDaysCreated))
                                        continue;

                                    if (!PromiseKeepingORG(arcsDaysCreated, costChangeFirstRoute))
                                    {
                                        continue;
                                    }


                                }
                                else
                                {
                                    double costRemoved1 = distMatrix[a1.uid, b1.uid] + distMatrix[b1.uid, c1.uid];
                                    double costAdded1 = distMatrix[a1.uid, b2.uid] + distMatrix[b2.uid, c1.uid];

                                    double costRemoved2 = distMatrix[a2.uid, b2.uid] + distMatrix[b2.uid, c2.uid];
                                    double costAdded2 = distMatrix[a2.uid, b1.uid] + distMatrix[b1.uid, c2.uid];

                                    costChangeFirstRoute = costAdded1 - costRemoved1;
                                    costChangeSecondRoute = costAdded2 - costRemoved2;

                                    BuilArcList(arcsDaysCreated, a1.uid, b2.uid, p, b2.uid, c1.uid, p, a2.uid, b1.uid, p, b1.uid, c2.uid, p);

                                    BuilArcList(arcsDaysDeleted, a1.uid, b1.uid, p, b1.uid, c1.uid, p, a2.uid, b2.uid, p, b2.uid, c2.uid, p);


                                    if (!arcReduction(arcsDaysCreated))
                                        continue;

                                    if (!PromiseKeepingORG(arcsDaysCreated, costChangeFirstRoute + costChangeSecondRoute))
                                    {
                                        continue;
                                    }
                                }

                                if (firstRouteIndex != secondRouteIndex)
                                {
                                    if (rt1.load - b1.deliveredQuantities[p] + b2.deliveredQuantities[p] > rt1_effective_capacity)
                                    {
                                        continue;
                                    }
                                    if (rt2.load - b2.deliveredQuantities[p] + b1.deliveredQuantities[p] > rt2_effective_capacity)
                                    {
                                        continue;
                                    }
                                }

                                if (Math.Abs(costChangeFirstRoute + costChangeSecondRoute) < 0.0000001)
                                {
                                    continue;
                                }

                                StoreBestSwapMove(p, firstRouteIndex, secondRouteIndex, firstNodeIndex, secondNodeIndex, costChangeFirstRoute, costChangeSecondRoute, arcsDaysDeleted, bestMove);

                            }
                        }
                    }
                }
            }
            return bestMove;
        }

        private static void StoreBestSwapMove(int p, int firstRouteIndex, int secondRouteIndex, int firstNodeIndex, int secondNodeIndex, double costChangeFirstRoute, double costChangeSecondRoute, int[] arcsDaysDeleted, SwapMove bestMove)
        {
            double totalObjectiveChange = costChangeFirstRoute + costChangeSecondRoute;

            if (totalObjectiveChange < bestMove.totalObjectiveChange)
            {
                bestMove.totalObjectiveChange = totalObjectiveChange;
                bestMove.day = p;
                bestMove.firstRoutePosition = firstRouteIndex;
                bestMove.firstNodePosition = firstNodeIndex;
                bestMove.secondRoutePosition = secondRouteIndex;
                bestMove.secondNodePosition = secondNodeIndex;
                bestMove.firstRouteObjectiveChange = costChangeFirstRoute;
                bestMove.secondRouteObjectiveChange = costChangeSecondRoute;
                CloneArcs(bestMove.arcsDeleted, arcsDaysDeleted);
            }
        }

        //This move affects only the routing aspect //Ok!
        private static RelocationMove FindBestCustomerRelocation(bool intraBan, bool enableArcReduction)
        {
            RelocationMove bestMove = new RelocationMove();
            for (int p = 0; p < solution.periods.Count; p++)
            {
                List<Route> routes = solution.periods[p].periodRoutes;
                for (int originRouteIndex = 0; originRouteIndex < routes.Count; originRouteIndex++)
                {
                    Route rt1 = (routes[originRouteIndex]);
                    for (int targetRouteIndex = 0; targetRouteIndex < routes.Count; targetRouteIndex++)
                    {
                        Route rt2 = (routes[targetRouteIndex]);

                        if ((originRouteIndex == targetRouteIndex) && intraBan)
                            continue;

                        int rt2_effective_capacity = 0;
                        //Prevent relocation moves on period 0 on boudia instances in order to prevent capacity violations within the period
                        if (model.input.type == PRP_DATASET_VERSION.BOUDIA_FMT && p == 0)
                            rt2_effective_capacity = rt2.realCapacity;
                        else
                            rt2_effective_capacity = rt2.effectiveCapacity;
                        
                        for (int originNodeIndex = 1; originNodeIndex < rt1.nodes.Count - 1; originNodeIndex++)
                        {
                            for (int targetNodeIndex = 0; targetNodeIndex < rt2.nodes.Count - 1; targetNodeIndex++)
                            {
                                if (originRouteIndex == targetRouteIndex && (targetNodeIndex == originNodeIndex || targetNodeIndex == originNodeIndex - 1))
                                {
                                    continue;
                                }

                                Node a = rt1.nodes[originNodeIndex - 1];
                                Node b = rt1.nodes[originNodeIndex];
                                Node c = rt1.nodes[originNodeIndex + 1];

                                Node insPoint1 = rt2.nodes[targetNodeIndex];
                                Node insPoint2 = rt2.nodes[targetNodeIndex + 1];

                                double costChangeOriginRoute = distMatrix[a.uid, c.uid] - (distMatrix[a.uid, b.uid] + distMatrix[b.uid, c.uid]);
                                double costChangeTargetRoute = distMatrix[insPoint1.uid, b.uid] + distMatrix[b.uid, insPoint2.uid] - distMatrix[insPoint1.uid, insPoint2.uid];

                                if (originRouteIndex != targetRouteIndex)
                                {
                                    if (rt2.load + b.deliveredQuantities[p] > rt2_effective_capacity)
                                    {
                                        continue;
                                    }
                                }

                                BuilArcList(arcsDaysCreated, a.uid, c.uid, p, insPoint1.uid, b.uid, p, b.uid, insPoint2.uid, p);
                                BuilArcList(arcsDaysDeleted, a.uid, b.uid, p, b.uid, c.uid, p, insPoint1.uid, insPoint2.uid, p);

                                double totalObjectiveChange = costChangeOriginRoute + costChangeTargetRoute;
                                double violationCostChange = 0.0;
                                
                                
                                //Calculate change on violations
                                if (solution.periods[p].periodRoutes.Count > model.input.availableVehicles)
                                {
                                    violationCostChange -= 1 * 5000; //5000: is the cost for the extra vehicles
                                }
                                //Load stays on the same period so there won't be a change there

                                if (!arcReduction(arcsDaysCreated))
                                    continue;

                                if (!PromiseKeepingORG(arcsDaysCreated, totalObjectiveChange))
                                    continue;
                                
                                if (Math.Abs(totalObjectiveChange + violationCostChange) < 0.0000001)
                                    continue;
                                
                                StoreBestRelocationMove(p, originRouteIndex, targetRouteIndex, originNodeIndex, targetNodeIndex, 
                                    costChangeOriginRoute, costChangeTargetRoute, violationCostChange, arcsDaysDeleted, bestMove);
                            }
                        }
                    }
                }
            }
            return bestMove;
        }
        
        private static bool FeasibilityCustomerRelocationWithInventory(Node cust, int day, Route remRoute, Route insRoute)
        {
            
            //This operator does not modify the visit schedule.
            cust.visitSchedule.CopyTo(cust.auxiliary_visitSchedule, 0);
            
            //Copy HorizonDeliveryServices and assign the new rout 
            Node.copyHorizonDeliveries(cust.horizonDeliveryServices, cust.auxiliary_horizonDeliveryServices, horizonDays);
            cust.auxiliary_horizonDeliveryServices[day].quantity = 0;
            cust.auxiliary_horizonDeliveryServices[day].route = insRoute;

            bool statusheuristic;
            if (remRoute != insRoute)
                statusheuristic = cust.ApplyNewSaw(cust.auxiliary_visitSchedule, solution, true, day);
            else
                statusheuristic = cust.ApplyNewSaw(cust.auxiliary_visitSchedule, solution);
            
            //Call LP to crosscheck the heuristics
            //bool statusLP = LP.runLP(solution, cust); 

            return statusheuristic;
            
        }
        
        private static RelocationWithInventoryMove FindBestCustomerRelocationWithInventory()
        {
            var arcsDaysCreated = new int[10 * 3 + 1];
            var arcsDaysDeleted = new int[10 * 3 + 1];

            int neglectedVisits = 0;
            
            RelocationWithInventoryMove bestMove = new RelocationWithInventoryMove();
            for (int p = 0; p < solution.periods.Count; p++)
            {
                List<Route> routes = solution.periods[p].periodRoutes;
                for (int originRouteIndex = 0; originRouteIndex < routes.Count; originRouteIndex++)
                {
                    Route rt1 = (routes[originRouteIndex]);
                    for (int targetRouteIndex = 0; targetRouteIndex < routes.Count; targetRouteIndex++)
                    {
                        Route rt2 = (routes[targetRouteIndex]);
                        for (int originNodeIndex = 1; originNodeIndex < rt1.nodes.Count - 1; originNodeIndex++)
                        {
                            for (int targetNodeIndex = 0; targetNodeIndex < rt2.nodes.Count - 1; targetNodeIndex++)
                            {
                                if (originRouteIndex == targetRouteIndex && (targetNodeIndex == originNodeIndex || targetNodeIndex == originNodeIndex - 1))
                                {
                                    continue;
                                }

                                Node a = rt1.nodes[originNodeIndex - 1];
                                Node b = rt1.nodes[originNodeIndex];
                                Node c = rt1.nodes[originNodeIndex + 1];

                                Node insPoint1 = rt2.nodes[targetNodeIndex];
                                Node insPoint2 = rt2.nodes[targetNodeIndex + 1];

                                double costChangeOriginRoute = distMatrix[a.uid, c.uid] - (distMatrix[a.uid, b.uid] + distMatrix[b.uid, c.uid]);
                                double costChangeTargetRoute = distMatrix[insPoint1.uid, b.uid] + distMatrix[b.uid, insPoint2.uid] - distMatrix[insPoint1.uid, insPoint2.uid];

                                double invCostChange_Cust = 0;
                                double invCostChange_Depot = 0;
                                
                                
                                //Calculate inventory Costs
                                //Check Customer/Depot/Route Inventory Feasibility
                                bool customerInventoryFeasibility = false;
                                    
                                if (!FeasibilityCustomerRelocationWithInventory(b, p, rt1, rt2))
                                {
                                    customerInventoryFeasibility = false;
#if MY_CHECKS
                                    //continue
#else
                                    continue;
#endif                                
                                }
                                
                                //Search for neglected visits
                                double routingObjGainFromNeglectedVisits = 0.0;
                                for (int k=0;k<horizonDays;k++)
                                    if (b.visitSchedule[k] && (GlobalUtils.IsEqual(b.auxiliary_deliveries[k], 0.0)))
                                    {
                                        //Find involved routing cost in period
                                        int index = b.horizonDeliveryServices[k].route.nodes.IndexOf(b);
                                        int prev_node = b.horizonDeliveryServices[k].route.nodes[index - 1].uid;
                                        int next_node = b.horizonDeliveryServices[k].route.nodes[index + 1].uid;

                                        routingObjGainFromNeglectedVisits -=
                                            (model.distMatrix[b.uid, prev_node] + model.distMatrix[b.uid, next_node]);

                                        neglectedVisits++;
                                    }
                                        
                                invCostChange_Cust = inventoryCostChange_Customer(b);
                                invCostChange_Depot = inventoryCostChange_Depot((b));
                                
                                
                                BuilArcList(arcsDaysCreated, a.uid, c.uid, p, insPoint1.uid, b.uid, p, b.uid, insPoint2.uid, p);
                                BuilArcList(arcsDaysDeleted, a.uid, b.uid, p, b.uid, c.uid, p, insPoint1.uid, insPoint2.uid, p);

                                double totalObjectiveChange = costChangeOriginRoute + costChangeTargetRoute + invCostChange_Cust + invCostChange_Depot;
                                
                                //Load stays on the same period so there won't be a change there
                                if (!PromiseKeepingORG(arcsDaysCreated, totalObjectiveChange))
                                    continue;
                                
                                //STRICT PROMISES
                                if (!PromiseKeepingTotObjORG(b, p, totalObjectiveChange + routingObjGainFromNeglectedVisits))
                                    continue;
                                
                                
                                if (Math.Abs(totalObjectiveChange + routingObjGainFromNeglectedVisits) < 0.0000001)
                                    continue;
                                
                                StoreBestRelocationWithInventoryMove(p, originRouteIndex, targetRouteIndex, originNodeIndex, targetNodeIndex, costChangeOriginRoute, costChangeTargetRoute,
                                    invCostChange_Cust, invCostChange_Depot, routingObjGainFromNeglectedVisits, arcsDaysDeleted, bestMove);
                            }
                        }
                    }
                }
            }
            
            //if (neglectedVisits > 0)
            //    GlobalUtils.writeToConsole("NEGLECTED VISITS MISSED: " + neglectedVisits);
            
            return bestMove;
        }

        private static bool PromiseKeepingTotObjORG(Node custToBeRelocated, int period, double objectiveChange)
        {
            double precision = 0.0001;
            if (solution.totalObjective + objectiveChange < promisesCustDay[period, custToBeRelocated.uid] - precision)
            {
                return true;
            }
            return false;
        }

        private static bool PromiseKeepingTotObj(Node custToBeRelocated, int period, double objectiveChange)
        {
            double precision = 0.0001;
            if (solution.totalObjective + objectiveChange < promisesCustDay[period, custToBeRelocated.uid] - precision)
            {
                return true;
            }
            return false;
        }

        private static bool PromiseKeepingORG(Node custToBeRelocated, int period, double inventoryCostChange)
        {
            double precision = 0.0001;
            if (solution.holdingCost + inventoryCostChange > promisesCustDay[period, custToBeRelocated.uid] - precision)
            {
                return false;
            }
            return true;
        }

        private static bool PromiseKeeping(Node custToBeRelocated, int period, double inventoryCostChange)
        {
            double precision = 0.0001;
            if (solution.holdingCost + inventoryCostChange < promisesCustDay[period, custToBeRelocated.uid] - precision)
            {
                return true;
            }
            return false;
        }

        private static bool PromiseKeepingORG(int[] arcsDaysCreated, double objChange)
        {
            double precision = 0.0001;
            for (int i = 0; i < arcsDaysCreated[0]; i++)
            {
                var ad = arcsDaysCreated[i];

                int fromId = arcsDaysCreated[1 + 3 * i + 0];
                int toId = arcsDaysCreated[1 + 3 * i + 1];
                int day = arcsDaysCreated[1 + 3 * i + 2];

                if (solution.totalObjective + objChange > promisesArcsPerDay[day, fromId, toId] - precision)
                {
                    return false;
                }
            }
            return true;
        }

        private static bool arcReduction(int[] arcsDaysCreated)
        {
            for (int i = 0; i < arcsDaysCreated[0]; i++)
            {
                var ad = arcsDaysCreated[i];

                int fromId = arcsDaysCreated[1 + 3 * i + 0];
                int toId = arcsDaysCreated[1 + 3 * i + 1];
                //int day = arcsDaysCreated[1 + 3 * i + 2];

                if (boolReductionArcTable[fromId, toId] || boolReductionArcTable[toId, fromId])
                {
                    return false;
                }
            }
            return true;
        }

        private static bool PromiseKeeping(List<ArcDay> arcsDaysCreated, double objChange)
        {
            double precision = 0.0001;
            for (int i = 0; i < arcsDaysCreated.Count; i++)
            {
                var ad = arcsDaysCreated[i];
                if (ad.day == 0 && ad.toId == 0 && ad.fromId == 0)
                    break;

                if (solution.totalObjective + objChange < promisesArcsPerDay[ad.day, ad.fromId, ad.toId] - precision)
                {
                    return true;
                }
            }
            return false;
        }

        private static void BuilArcList(int[] list, params int[] triplets)
        {
            
            int index = 1;
            list[0] = 0;
            for (int i = 0; i < triplets.Length; i++)
            {
                int fromId = triplets[i];
                i++;
                int toId = triplets[i];
                i++;
                int day = triplets[i];

                list[index] = fromId;
                list[index + 1] = toId;
                list[index + 2] = day;
                list[0]++;
                index += 3;
            }
            
        }

        private static void StoreBestRelocationMove(int p, int originRouteIndex, int targetRouteIndex, int originNodeIndex, int targetNodeIndex,
            double costChangeOriginRoute, double costChangeTargetRoute, double violationChange, int[] arcsDaysDeleted, RelocationMove bestMove)
        {
            double totalObjectiveChange = costChangeOriginRoute + costChangeTargetRoute;

            if (totalObjectiveChange +violationChange < bestMove.totalObjectiveChange + bestMove.violationChange)
            {
                bestMove.totalObjectiveChange = totalObjectiveChange;
                bestMove.violationChange = violationChange;
                bestMove.day = p;
                bestMove.originRoutePosition = originRouteIndex;
                bestMove.originNodePosition = originNodeIndex;
                bestMove.targetRoutePosition = targetRouteIndex;
                bestMove.targetNodePosition = targetNodeIndex;
                bestMove.originRouteObjectiveChange = costChangeOriginRoute;
                bestMove.targetRouteObjectiveChange = costChangeTargetRoute;
                CloneArcs(bestMove.arcsDeleted, arcsDaysDeleted);
            }
        }
        
        private static void StoreBestRelocationWithInventoryMove(int p, int originRouteIndex, int targetRouteIndex, int originNodeIndex, int targetNodeIndex,
            double costChangeOriginRoute, double costChangeTargetRoute, double costChangeCustomer, double costChangeDepot, double extraRoutingGain, 
            int[] arcsDaysDeleted, RelocationWithInventoryMove bestMove)        {

            double totalObjectiveChange = costChangeOriginRoute + costChangeTargetRoute + costChangeDepot + costChangeCustomer;

            if (totalObjectiveChange + extraRoutingGain < bestMove.totalObjectiveChange + bestMove.extraRoutingCostChange)
            {
                bestMove.totalObjectiveChange = totalObjectiveChange;
                bestMove.extraRoutingCostChange = extraRoutingGain;
                //bestMove.violationChange = violationChange;
                bestMove.day = p;
                bestMove.originRoutePosition = originRouteIndex;
                bestMove.originNodePosition = originNodeIndex;
                bestMove.targetRoutePosition = targetRouteIndex;
                bestMove.targetNodePosition = targetNodeIndex;
                bestMove.originRouteObjectiveChange = costChangeOriginRoute;
                bestMove.targetRouteObjectiveChange = costChangeTargetRoute;
                bestMove.inventoryCustomerObjectiveChange = costChangeCustomer;
                bestMove.inventoryDepotObjectiveChange = costChangeDepot;
                CloneArcs(bestMove.arcsDeleted, arcsDaysDeleted);
            }
        }

        private static DayInsertionMove FindBestDeliveryDayInsertion(bool enableVisitReduction)
        {
            DayInsertionMove bestMove = new DayInsertionMove();
            
            for (int k = 0; k < solution.customers.Count; k++)
            {
                Node custToBeInserted = solution.customers[k];

                for (int i_to = 0; i_to < solution.periods.Count; i_to++)
                {
                    //Check if already served in this period
                    if (custToBeInserted.horizonDeliveryServices[i_to].route != null)
                        continue;
                    
                    Period perTo = solution.periods[i_to];
                    for (int j_to = 0; j_to < perTo.periodRoutes.Count; j_to++)
                    {
                        
                        Route rtTo = perTo.periodRoutes[j_to];

                        bool statusheuristic = FeasibilityInsertionDayInventory_Customer(custToBeInserted, i_to, rtTo); 
                            
                        //DO NOT ACCEPT MOVES WITH 0 QUANTITY ON INSERTED DAYS
                        //if (custToBeInserted.auxiliary_deliveries[i_to]  == 0)
                        //    continue;

                        //Call LP to crosscheck the heuristics
                        //bool statusLP = LP.runLP(solution, custToBeInserted); 
                    
                        //if (statusLP != statusheuristic)
                        //    GlobalUtils.writeToConsole("misaling with LP on insertion");
                        
                        if (!statusheuristic)
                            continue;
                        
                        //Note that inventoryCostChange_Cust and inventoryCostChange_Depot are equal in terms of quantity and days affected.
                        //The applied holding cost changes as well as the sign 
                        //TODO: Room for optimization via merging the calculation for Customer and the Depot 
                        double invCostChange_Cust = inventoryCostChange_Customer(custToBeInserted);
                        double invCostChange_Depot = inventoryCostChange_Depot(custToBeInserted);

                        if (CODE_CHECK)
                        {
                            double inventoryCostChange_Cust_SECURE = InventoryCostChange_AuxiliaryLists_Customer(custToBeInserted);
                            double inventoryCostChange_Depot_SECURE = InventoryCostChange_AuxiliaryLists_Depot(custToBeInserted);

                            if (!GlobalUtils.IsEqual(inventoryCostChange_Cust_SECURE, invCostChange_Cust))
                            {
                                Console.WriteLine("Inventory Cost Inconsistency from Customer");
                            }
                            if (!GlobalUtils.IsEqual(inventoryCostChange_Depot_SECURE, invCostChange_Depot))
                            {
                                Console.WriteLine("Inventory Cost Inconsistency fro Customer");
                            }
                        }

                        double inventoryCostChange = invCostChange_Cust + invCostChange_Depot;

                        if (CODE_CHECK)
                        {
                            bool routeCapacityFeasibility_SECURE = FeasibilityCapacity_InsertionDay_AuxiliaryLists(custToBeInserted, i_to, rtTo);

                            if (statusheuristic != routeCapacityFeasibility_SECURE)
                            {
                                Console.WriteLine("Capacity Feasibility Inconsistency");
                            }
                        }

                        for (int k_to = 0; k_to < rtTo.nodes.Count - 1; k_to++)
                        {
                            Node prev_to = rtTo.nodes[k_to];
                            Node succ_to = rtTo.nodes[k_to + 1];

                            double routingCostRemoved_DestRoute = distMatrix[prev_to.uid, succ_to.uid];
                            double routingCostAdded_DestRoute = distMatrix[prev_to.uid, custToBeInserted.uid] +
                                distMatrix[custToBeInserted.uid, succ_to.uid];
                            double routingCostChange_DestRoute = routingCostAdded_DestRoute - routingCostRemoved_DestRoute;

                            double totObjChange = routingCostChange_DestRoute + invCostChange_Cust + invCostChange_Depot;
                            double violationCostChange = 0.0;


                            if (perTo.periodRoutes.Count > model.input.availableVehicles)
                            {
                                violationCostChange += 1 * 5000;
                            }
                            
                            
                            if (Math.Abs(totObjChange + violationCostChange) < 0.0000001)
                                continue;
                            
                            if (!PromiseKeepingTotObjORG(custToBeInserted, i_to, totObjChange))
                            {
                                continue;
                            }

#if MY_CHECKS
                            //TESTING: Temporary Store and apply the move to the current solution
                            //Run the test everything function and cofirm the feasibility tests
                            
                            Solution backup_sol = new Solution(solution , 0);
                            Solution old_ref = new Solution(solution , 0);
                            solution = backup_sol;
                            
                            //Apply the temp move to the current sol
                            //Load the correct reference for the modified customer
                            DayInsertionMove tempMove = new DayInsertionMove();
                            StoreInsertionDayMove(custToBeInserted.uid, i_to, j_to, k_to, quantityInserted,
                                routingCostChange_DestRoute, inventoryCostChange_Cust, inventoryCostChange_Depot, tempMove);
                            ApplyMove(tempMove);
                            
                            bool tempFeasible = solution.TestEverythingFromScratch();
                            bool checkFeasible = routeCapacityFeasibility && customerInventoryFeasibility &&
                                                 depotInventoryFeasibility;
                            
                            if (tempFeasible != checkFeasible)
                                GlobalUtils.writeToConsole("Check wtf happened");
                            
                            //Bring back solution
                            solution = old_ref;
                            
                            if (tempFeasible)
                                StoreInsertionDayMove(custToBeInserted.uid, i_to, j_to, k_to, quantityInserted,
                                    routingCostChange_DestRoute, inventoryCostChange_Cust, inventoryCostChange_Depot, bestMove);

#else
                            StoreInsertionDayMove(custToBeInserted.uid, i_to, j_to, k_to,
                                    routingCostChange_DestRoute, invCostChange_Cust, invCostChange_Depot, violationCostChange, bestMove);
#endif
                        }
                    }
                }
            }


            return bestMove;
        }

        private static void StoreInsertionDayMove(int cust_uid, int perTo, int rtTo, int ndTo, double routingCostChange_DestRoute, double inventoryCostChange_Cust,
            double inventoryCostChange_Depot, double violationCostChange, DayInsertionMove bestMove)
        {
            double totalObjectiveChange = routingCostChange_DestRoute + inventoryCostWeight*(inventoryCostChange_Cust + inventoryCostChange_Depot);

            if (totalObjectiveChange + violationCostChange < bestMove.totalObjectiveChange + bestMove.violationCostChange)
            {
                bestMove.insertedCustomer = cust_uid;
                bestMove.insertedDay = perTo;
                bestMove.insertedRoutePosition = rtTo;
                bestMove.insertedNodePosition = ndTo;

                bestMove.insertedRouteObjectiveChange = routingCostChange_DestRoute;

                bestMove.inventoryCustomerObjectiveChange = inventoryCostChange_Cust;
                bestMove.inventoryDepotObjectiveChange = inventoryCostChange_Depot;

                bestMove.totalObjectiveChange = totalObjectiveChange;
                bestMove.violationCostChange = violationCostChange;
            }
        }

        //Looks OK!
        private static bool FeasibilityCapacity_InsertionDay_AuxiliaryLists(Node cust, int insDay, Route targetRoute)
        {
            for (int i = 0; i < horizonDays; i++)
            {
                if (i == insDay)
                {
                    if (targetRoute.load + cust.auxiliary_deliveries[i] > targetRoute.effectiveCapacity)
                    {
                        return false;
                    }
                }
                else
                {
                    if (cust.deliveredQuantities[i] > 0)
                    {
                        Route rt = cust.horizonDeliveryServices[i].route;

                        if (rt.load - cust.horizonDeliveryServices[i].quantity + cust.auxiliary_deliveries[i] > rt.effectiveCapacity)
                        {
                            return false;
                        }
                    }
                }
            }

            return true;

        }

#region InsertionDayMethods

        private static bool InsertionDayMoveConditions(Node cust, int insDay)
        {
            int firstVisitDay = (cust.visitSchedule[0]) ? 0 : cust.nextDeliveryDay[0];
            int lastVisitDay = (cust.visitSchedule[horizonDays - 1]) ? horizonDays - 1 : cust.prevDeliveryDay[horizonDays - 1];
            int prevVisitDay = (cust.prevDeliveryDay[insDay] < 0) ? firstVisitDay : cust.prevDeliveryDay[insDay];
            int nextVisitDay = (cust.nextDeliveryDay[insDay] < 0) ? lastVisitDay : cust.nextDeliveryDay[insDay];
            
            //We do not allow insertions before the first delivery day
            //if (firstVisitDay > insDay)
            //    return false;

            return true;
        }

        private static bool FeasibilityInsertionDayInventory_Customer(Node cust, int insDay, Route insRoute)
        {
            //Find the new quantity to be delivered to the customer on the insDay
            //Find new visit schedule
            cust.visitSchedule.CopyTo(cust.auxiliary_visitSchedule, 0);
            cust.auxiliary_visitSchedule[insDay] = true;
            
            //Copy HorizonDeliveryServices
            Node.copyHorizonDeliveries(cust.horizonDeliveryServices, cust.auxiliary_horizonDeliveryServices, horizonDays);
            cust.auxiliary_horizonDeliveryServices[insDay].quantity = 0;
            cust.auxiliary_horizonDeliveryServices[insDay].route = insRoute;
            
            //solution.SaveToFile("test_lp_sol.txt");


            bool statusheuristic = true;

            statusheuristic = cust.ApplyNewSaw(cust.auxiliary_visitSchedule, solution);
            
            //Call LP to crosscheck the heuristics
            //bool statusLP = LP.runLP(solution, cust); 
            
            if (!statusheuristic)
                return false;
            
            return true;
        }
        
        //Redundant with new Saw
        
        private static double inventoryChangeCustomer(Node cust)
        {
            double change = 0.0;
            for (int i = 0; i < horizonDays; i++)
                change += cust.auxiliary_endDayInventory[i] - cust.endDayInventory[i];
            return change;
        }
        
        private static double inventoryCostChange_Depot(Node cust)
        {
            return -inventoryChangeCustomer(cust) * solution.depot.unitHoldingCost;
        }

        private static double inventoryCostChange_Customer(Node cust)
        {
            return inventoryChangeCustomer(cust) * cust.unitHoldingCost;
        }

#endregion

#region DayDeletionMethods

        private static DayDeletionMove FindBestDeliveryDayDeletion(bool enableVisitReduction)
        {
            DayDeletionMove bestMove = new DayDeletionMove();
            int total_moves_evaluated = 0;
            
            for (int i_from = 0; i_from < solution.periods.Count; i_from++)
            {
                Period perFrom = solution.periods[i_from];

                //Prevent Visit removals from essential visits on day 0 on boudia instances
                if (model.input.type == PRP_DATASET_VERSION.BOUDIA_FMT && i_from == 0)
                    continue;
                
                for (int j_from = 0; j_from < perFrom.periodRoutes.Count; j_from++)
                {
                    Route rtFrom = perFrom.periodRoutes[j_from];

                    for (int k_from = 1; k_from < rtFrom.nodes.Count - 1; k_from++)
                    {
                        Node custToBeRemoved = rtFrom.nodes[k_from];
                        total_moves_evaluated++;
                        
                        //FOR CHECKING
                        //prepareAuxiliaryLists_DayRelocation(custToBeRemoved, i_from, -1);

                        bool customerInventoryFeasibility = true;
                        
                        //Check move feasibility
                        if (!FeasibilityDeletionDayInventory_Customer(custToBeRemoved, i_from))
                        {
                            customerInventoryFeasibility = false;
#if MY_CHECKS
                            //continue;
#else
                            continue;
#endif
                        }

                        //if (custToBeRemoved.deliveredQuantities[i_from] == 0)
                        //    GlobalUtils.writeToConsole("Test");

                        //TODO: I have no idea if the SECURE check covers all the proper checks
                        /*
                        if (CODE_CHECK)
                        {
                            //This method checks for stockouts in the calculated auxiliary inventories
                            bool inventoryFeasibility_SECURE = FeasibilityInventory_AuxiliaryLists(custToBeRemoved);
                            if (inventoryFeasibility != inventoryFeasibility_SECURE)
                            {
                                GlobalUtils.writeToConsole("INVENTORY FEASIBILITY Customer day deletion");
                            }
                        }
                        */

                        //if (custToBeRemoved.uid == 21 && i_from == 13)
                        //    GlobalUtils.writeToConsole("break");

                        double invCostChange_Cust = inventoryCostChange_Customer(custToBeRemoved);
                        //TODOPdevelop the efficient one
                        double invCostChange_Depot = inventoryCostChange_Depot(custToBeRemoved);

                        if (CODE_CHECK)
                        {
                            double inventoryCostChange_Cust_SECURE = InventoryCostChange_AuxiliaryLists_Customer(custToBeRemoved);
                            //TODO
                            double inventoryCostChange_Depot_SECURE = InventoryCostChange_AuxiliaryLists_Depot(custToBeRemoved);

                            if (!GlobalUtils.IsEqual(inventoryCostChange_Cust_SECURE, invCostChange_Cust))
                            {
                                Console.WriteLine("DELETION DAY - Inventory Cost Inconsistency from Customer");
                            }
                            if (!GlobalUtils.IsEqual(inventoryCostChange_Depot_SECURE, invCostChange_Depot))
                            {
                                Console.WriteLine("DELETION DAY - Inventory Cost Inconsistency for Depot");
                            }
                        }
                        
                        
                        double routingCostRemoved_OriginRoute = distMatrix[rtFrom.nodes[k_from - 1].uid, rtFrom.nodes[k_from].uid] +
                            distMatrix[rtFrom.nodes[k_from].uid, rtFrom.nodes[k_from + 1].uid];

                        double routingCostAdded_OriginRoute = distMatrix[rtFrom.nodes[k_from - 1].uid, rtFrom.nodes[k_from + 1].uid];

                        double routingCostChange_OriginRoute = routingCostAdded_OriginRoute - routingCostRemoved_OriginRoute;

                        double violationCostChange = 0.0;
                        double totObjChange = routingCostChange_OriginRoute + invCostChange_Cust +
                                              invCostChange_Depot;
                        
                        
                        //Calculate change on violations
                                
                        if (perFrom.periodRoutes.Count > model.input.availableVehicles)
                        {
                            violationCostChange -= 1 * 5000; //5000: is the cost for the extra vehicles
                        }
                        
    
                        // STRICT PROMISES
                        if (!PromiseKeepingTotObjORG(custToBeRemoved, i_from, totObjChange))
                            continue;
                        
                        
                        if (Math.Abs(totObjChange + violationCostChange) < 0.0000001)
                            continue;

#if MY_CHECKS
                        //TESTING: Temporary Store and apply the move to the current solution
                        //Run the test everything function and confirm the feasibility tests
                            
                        DayDeletionMove tempMove = new DayDeletionMove();
                        StoreDeletionDayMove(i_from, j_from, k_from, routingCostChange_OriginRoute, invCostChange_Cust, invCostChange_Depot, 0.0, tempMove);
                        
                        Solution backup_sol = new Solution(solution , 0);
                        Solution old_ref = new Solution(solution , 0);
                        solution = backup_sol;
                        
                        //Apply the temp move to the current sol
                        ApplyMove(tempMove);
                            
                        bool tempFeasible = Solution.checkSolutionStatus(solution.TestEverythingFromScratch());
                        bool checkFeasible = customerInventoryFeasibility;
                            
                        if (tempFeasible != checkFeasible)
                            GlobalUtils.writeToConsole("Check wtf happened");
                            
                        if (checkFeasible)
                            StoreDeletionDayMove(i_from, j_from, k_from, routingCostChange_OriginRoute, invCostChange_Cust, invCostChange_Depot, 0.0, bestMove);
                        
                        //Bring back solution
                        solution = old_ref;
#else
                        StoreDeletionDayMove(i_from, j_from, k_from, routingCostChange_OriginRoute,
                            invCostChange_Cust, invCostChange_Depot, violationCostChange, bestMove);
#endif
                        
                    
                    }
                }
            }
#if DEBUG
            //GlobalUtils.writeToConsole("Day Deletion: Total Moves Evaluated: {0}",  total_moves_evaluated);
#endif
            return bestMove;
        }

        private static void StoreDeletionDayMove(int perFrom, int rtFrom, int ndFrom, double routingCostChange_OriginRoute,
            double inventoryCostChange_Cust, double inventoryCostChange_Depot, double violationCostChange, DayDeletionMove bestMove)
        {
            double totalObjectiveChange = routingCostChange_OriginRoute + inventoryCostWeight*(inventoryCostChange_Cust + inventoryCostChange_Depot);

            if (totalObjectiveChange + violationCostChange < bestMove.totalObjectiveChange + bestMove.violationCostChange)
            {
                bestMove.removedDay = perFrom;
                bestMove.removedRoutePosition = rtFrom;
                bestMove.removedNodePosition = ndFrom;

                bestMove.removedRouteObjectiveChange = routingCostChange_OriginRoute;

                bestMove.inventoryCustomerObjectiveChange = inventoryCostChange_Cust;
                bestMove.inventoryDepotObjectiveChange = inventoryCostChange_Depot;

                bestMove.totalObjectiveChange = totalObjectiveChange;
                bestMove.violationCostChange = violationCostChange;
            }
        }

        private static bool FeasibilityDeletionDayInventory_Customer(Node cust, int remDay)
        {
            
            //Find the new quantity to be delivered to the customer on the insDay
            //Find new visit schedule
            cust.visitSchedule.CopyTo(cust.auxiliary_visitSchedule, 0);
            cust.auxiliary_visitSchedule[remDay] = false;
            
            //Copy HorizonDeliveryServices
            Node.copyHorizonDeliveries(cust.horizonDeliveryServices, cust.auxiliary_horizonDeliveryServices, horizonDays);
            cust.auxiliary_horizonDeliveryServices[remDay].reset();
            
            //solution.SaveToFile("test_lp_sol.txt");
            bool statusheuristic = true;


            statusheuristic = cust.ApplyNewSaw(cust.auxiliary_visitSchedule, solution);
            
            //Call LP to crosscheck the heuristics
            //bool statusLP = LP.runLP(solution, cust); 
            
            //if (statusheuristic != statusLP)
            //    GlobalUtils.writeToConsole("MIsalign with LP on delete");
            
            
            if (!statusheuristic)
                return false;
            
            return true;
        }

   
        
#endregion DayDeletionMethods
        
#region DayRelocationMove

        private static DayRelocationMove FindBestDeliveryDayRelocation(bool enableArcReduction)
        {
            var arcsDaysCreated = new List<ArcDay>(5);
            var arcsDaysDeleted = new List<ArcDay>(5);

            DayRelocationMove bestMove = new DayRelocationMove();
            for (int i_from = 0; i_from < solution.periods.Count; i_from++)
            {
                Period perFrom = solution.periods[i_from];

                //Prevent Visit removals from essential visits on day 0 on boudia instances
                if (model.input.type == PRP_DATASET_VERSION.BOUDIA_FMT && i_from == 0)
                    continue;

                for (int j_from = 0; j_from < perFrom.periodRoutes.Count; j_from++)
                {
                    Route rtFrom = perFrom.periodRoutes[j_from];

                    for (int k_from = 1; k_from < rtFrom.nodes.Count - 1; k_from++)
                    {
                        Node custToBeRelocated = rtFrom.nodes[k_from];

                        for (int i_to = 0; i_to < solution.periods.Count; i_to++)
                        {
                            //Same period or already served in this period
                            if (i_from == i_to || custToBeRelocated.horizonDeliveryServices[i_to].route != null)
                                continue;

                            //FOR CHECKING
                            //prepareAuxiliaryLists_DayRelocation(custToBeRelocated, i_from, i_to);
                            
                            bool customerInventoryFeasibility = true;
                            
                            Period perTo = solution.periods[i_to];
                            for (int j_to = 0; j_to < perTo.periodRoutes.Count; j_to++)
                            {
                                Route rtTo = perTo.periodRoutes[j_to];

                                //Check Customer/Depot/Route Inventory Feasibility
                                if (!FeasibilityRelocationDayInventory_Customer(custToBeRelocated, i_from, i_to, rtTo))
                                {
                                    customerInventoryFeasibility = false;
#if MY_CHECKS
                                    //continue
#else
                                    continue;
#endif
                               }
                                
                                double invCostChange_Cust = inventoryCostChange_Customer(custToBeRelocated);
                                double invCostChange_Depot = inventoryCostChange_Depot((custToBeRelocated));
                                double inventoryCostChange = invCostChange_Cust + invCostChange_Depot;

                                if (CODE_CHECK)
                                {
                                    //Double check customer inventory cost change calculation
                                    double inventoryCostChange_Cust_Failsafe =
                                        inventoryCostChangeRelocationDay_Customer_Failsafe(custToBeRelocated, i_from, i_to);
                            
                                    double inventoryCostChange_Depot_Failsafe =
                                        inventoryCostChangeRelocationDay_Depot_Failsafe(custToBeRelocated, i_from, i_to);


                                    if (!GlobalUtils.IsEqual(invCostChange_Cust, inventoryCostChange_Cust_Failsafe))
                                    {
                                        Console.WriteLine("Wrong inventory change calculation");
                                        Console.WriteLine("Relocation from  day {0} to day {1}", i_from, i_to);
                                        inventoryCostChangeRelocationDay_Customer(custToBeRelocated, i_from, i_to);
                                        custToBeRelocated.ReportInventory();
                                    }
                                
                                    if (!GlobalUtils.IsEqual(invCostChange_Depot, inventoryCostChange_Depot_Failsafe))
                                        Console.WriteLine("Wrong depot inventory change calculation");    
                                }
                                
                                for (int k_to = 0; k_to < rtTo.nodes.Count - 1; k_to++)
                                {
                                    Node prev_to = rtTo.nodes[k_to];
                                    Node succ_to = rtTo.nodes[k_to + 1];

                                    //Calculate routing differential costs
                                    
                                    //Calculate difference from origin route
                                    double routingCostRemoved_OriginRoute = distMatrix[rtFrom.nodes[k_from - 1].uid, rtFrom.nodes[k_from].uid] +
                                                                            distMatrix[rtFrom.nodes[k_from].uid, rtFrom.nodes[k_from + 1].uid];

                                    double routingCostAdded_OriginRoute = distMatrix[rtFrom.nodes[k_from - 1].uid, rtFrom.nodes[k_from + 1].uid];

                                    double routingCostChange_OriginRoute = routingCostAdded_OriginRoute - routingCostRemoved_OriginRoute;
                                    
                                    //Calculate difference from destination route
                                    double routingCostRemoved_DestRoute = distMatrix[prev_to.uid, succ_to.uid];
                                    double routingCostAdded_DestRoute = distMatrix[prev_to.uid, custToBeRelocated.uid] +
                                        distMatrix[custToBeRelocated.uid, succ_to.uid];
                                    double routingCostChange_DestRoute = routingCostAdded_DestRoute - routingCostRemoved_DestRoute;

                                    double totObjChange = routingCostChange_DestRoute + routingCostChange_OriginRoute + invCostChange_Cust + invCostChange_Depot;
                                    double violationCostChange = 0.0;
                                    
                                    if (Math.Abs(totObjChange + violationCostChange) < 0.0000001)
                                        continue;
                                    
                                    if (!PromiseKeepingTotObjORG(custToBeRelocated, i_to, totObjChange))
                                        continue;
                                    
                                    //STRICT PROMISES
                                    if (!PromiseKeepingTotObjORG(custToBeRelocated, i_from, totObjChange))
                                        continue;

                                    if (perTo.periodRoutes.Count > model.input.availableVehicles)
                                        violationCostChange += 5000;
                                    
                                    if (perFrom.periodRoutes.Count > model.input.availableVehicles)
                                        violationCostChange -= 10000;
                                    
                                    
#if MY_CHECKS
                                    //TESTING: Temporary Store and apply the move to the current solution
                                    //Run the test everything function and cofirm the feasibility tests
                            
                                    DayRelocationMove tempMove = new DayRelocationMove();
                                    StoreRelocationDayMove(i_from, j_from, k_from, i_to, j_to, k_to, quantityInserted,
                                        routingCostChange_OriginRoute, routingCostChange_DestRoute, inventoryCostChange_Cust, inventoryCostChange_Depot, tempMove);
                                    
                                    Solution backup_sol = new Solution(solution , 0);
                                    Solution old_ref = new Solution(solution , 0);
                                    solution = backup_sol;
                                        
                                    
                                    //Apply the temp move to the current sol
                                    ApplyMove(tempMove);
                            
                                    bool tempFeasible = solution.TestEverythingFromScratch();
                                    bool checkFeasible = routeInventoryFeasibility && customerInventoryFeasibility &&
                                                         depotIventoryFeasibility;
                            
                                    if (tempFeasible != checkFeasible)
                                        GlobalUtils.writeToConsole("Check wtf happened");

                                    if (tempFeasible)
                                        StoreRelocationDayMove(i_from, j_from, k_from, i_to, j_to, k_to, quantityInserted,
                                            routingCostChange_OriginRoute, routingCostChange_DestRoute, inventoryCostChange_Cust, inventoryCostChange_Depot, bestMove);
                                    
                                    //Bring back solution
                                    solution = old_ref;
#else
                                    StoreRelocationDayMove(i_from, j_from, k_from, i_to, j_to, k_to,
                                            routingCostChange_OriginRoute, routingCostChange_DestRoute, invCostChange_Cust, invCostChange_Depot, violationCostChange, bestMove);
#endif
                                }
                            }
                        }
                    }
                }
            }

            return bestMove;
        }
        
        private static bool RelocationDayMoveConditions(Node cust, int remDay, int insDay)
        {
            int firstVisitDay = (cust.visitSchedule[0]) ? 0 : cust.nextDeliveryDay[0];
            int lastVisitDay = (cust.visitSchedule[horizonDays - 1]) ? horizonDays - 1 : cust.prevDeliveryDay[horizonDays - 1];
            int prevVisitDay = (cust.prevDeliveryDay[remDay] < 0) ? firstVisitDay : cust.prevDeliveryDay[remDay];
            int nextVisitDay = (cust.nextDeliveryDay[remDay] < 0) ? lastVisitDay : cust.nextDeliveryDay[remDay];
            
            //Do not allow the relocation of the first delivery day to a day after the 2nd delivery 
            if (remDay == firstVisitDay && nextVisitDay != firstVisitDay && insDay > nextVisitDay)
                return false;
            
            //Do not allow the relocation of any other day before the firstVisit
            if (remDay != firstVisitDay && insDay < firstVisitDay)
                return false;

            return true;
        }

        
        private static bool FeasibilityRelocationDayInventory_Customer(Node cust, int remDay, int insDay, Route insRoute)
        {
            //Find the new quantity to be delivered to the customer on the insDay
            //Find new visit schedule
            cust.visitSchedule.CopyTo(cust.auxiliary_visitSchedule, 0);
            cust.auxiliary_visitSchedule[insDay] = true;
            cust.auxiliary_visitSchedule[remDay] = false;
            
            //Copy HorizonDeliveryServices
            Node.copyHorizonDeliveries(cust.horizonDeliveryServices, cust.auxiliary_horizonDeliveryServices, horizonDays);
            cust.auxiliary_horizonDeliveryServices[insDay].quantity = 0;
            cust.auxiliary_horizonDeliveryServices[insDay].route = insRoute;
            cust.auxiliary_horizonDeliveryServices[remDay].reset();

            bool statusheuristic = cust.ApplyNewSaw(cust.auxiliary_visitSchedule, solution);
            
            //Call LP to crosscheck the heuristics
            //bool statusLP = LP.runLP(solution, cust); 
            
            //if (statusheuristic != statusLP)
            //    GlobalUtils.writeToConsole("MIsalign with LP on relocation");
            
            //DO NOT ACCEPT MOVES WITH 0 QUANTITY ON INSERTED DAYS
            if (cust.auxiliary_deliveries[insDay]  == 0)
                return false;
            
            
            if (!statusheuristic)
                return false;
            
            return true;
        }
        
        private static void StoreRelocationDayMove(int perFrom, int rtFrom, int ndFrom, int perTo, int rtTo, int ndTo, double rtObj_from, double rtObj_to,
            double invObjCust, double invObjDepot, double violationCostChange, DayRelocationMove bestMove)
        {
            double totalObjectiveChange = (rtObj_from + rtObj_to) + inventoryCostWeight*(invObjCust + invObjDepot);

            if (totalObjectiveChange + violationCostChange < bestMove.totalObjectiveChange + bestMove.violationCostChange)
            {
                bestMove.removedDay = perFrom;
                bestMove.removedRoutePosition = rtFrom;
                bestMove.removedNodePosition = ndFrom;

                bestMove.insertedDay = perTo;
                bestMove.insertedRoutePosition = rtTo;
                bestMove.insertedNodePosition = ndTo;

                bestMove.removedRouteObjectiveChange = rtObj_from;
                bestMove.insertedRouteObjectiveChange = rtObj_to;

                bestMove.inventoryCustomerObjectiveChange = invObjCust;
                bestMove.inventoryDepotObjectiveChange = invObjDepot;

                bestMove.totalObjectiveChange = totalObjectiveChange;
                bestMove.violationCostChange = violationCostChange;
            }
        }
        

        private static double quantityChangeRelocationDay(Node cust, int remDay, int insDay)
        {
            //The impact of the relocation of a visit of a customer can be considered as the accumulated cost
            //of removing the old visit and inserting the new visit
            int firstVisitDay = (cust.visitSchedule[0]) ? 0 : cust.nextDeliveryDay[0];
            int prevVisitDay_fromRemDay = (cust.prevDeliveryDay[remDay] < 0) ? firstVisitDay : cust.prevDeliveryDay[remDay];
            int prevVisitDay_fromInsDay = (cust.prevDeliveryDay[insDay] < 0) ? firstVisitDay : cust.prevDeliveryDay[insDay];
            int nextVisitDay_fromRemDay = (cust.nextDeliveryDay[remDay] < 0) ? horizonDays - 1 : cust.nextDeliveryDay[remDay];
            int nextVisitDay_fromInsDay = (cust.nextDeliveryDay[insDay] < 0) ? horizonDays - 1 : cust.nextDeliveryDay[insDay];
            
            
            double removedQuantity = cust.deliveredQuantities[remDay];
            double insertedQuantity = cust.auxiliary_deliveries[insDay];


            double change = 0.0;
            
            if (prevVisitDay_fromInsDay == prevVisitDay_fromRemDay || nextVisitDay_fromRemDay == nextVisitDay_fromInsDay)
            {
                //Relocation between two visits
                change = (remDay - prevVisitDay_fromRemDay) * removedQuantity 
                         - insertedQuantity * (insDay - Math.Min(prevVisitDay_fromRemDay, prevVisitDay_fromInsDay));
            }
            else
            {
                change = (remDay - prevVisitDay_fromRemDay) * removedQuantity
                         - (insDay - prevVisitDay_fromInsDay) * insertedQuantity;

                if (prevVisitDay_fromInsDay == remDay)
                    change -= insertedQuantity;    
            }
            
            /*
            //Naive Way
            double real_change = 0;
            
            for (int i = 0; i < solution.model.horizonDays; i++)
                real_change += cust.auxiliary_endDayInventory[i] - cust.endDayInventory[i];
            
            if (real_change != change)
                GlobalUtils.writeToConsole("BUG");
                
            */
            
            return change;
        }
        
        private static double inventoryCostChangeRelocationDay_Customer(Node cust, int remDay, int insDay)
        {
            return quantityChangeRelocationDay(cust, remDay, insDay) * cust.unitHoldingCost;
        }
        
        private static double inventoryCostChangeRelocationDay_Customer_Failsafe(Node cust, int remDay, int insDay)
        {
            double change = 0.0;
            for (int i = 0; i < horizonDays; i++)
                change += (cust.auxiliary_endDayInventory[i] - cust.endDayInventory[i]);
            return change * cust.unitHoldingCost;
        }
        
        private static double inventoryCostChangeRelocationDay_Depot(Node cust, int remDay, int insDay)
        {
            return -quantityChangeRelocationDay(cust, remDay, insDay) * solution.depot.unitHoldingCost;
        }
        
        private static double inventoryCostChangeRelocationDay_Depot_Failsafe(Node cust, int remDay, int insDay)
        {
            double change = 0.0;

            double delivery_diff = 0.0;
            for (int i = 0; i < horizonDays; i++)
            {
                delivery_diff += -cust.auxiliary_deliveries[i] + cust.deliveredQuantities[i];
                change += delivery_diff * solution.depot.unitHoldingCost;
            }
                 
            return change;
        }
        
        private static double DepotInvChangeForExtraCustQuantity(double quant, int day)
        {
            return -(horizonDays - day) * quant * solution.depot.unitHoldingCost;
        }

        private static double InventoryCostChange_AuxiliaryLists_Customer(Node cust)
        {
            double costAfter = 0;
            for (int i = 0; i < cust.auxiliary_endDayInventory.Length; i++)
            {
                costAfter += cust.auxiliary_endDayInventory[i] * cust.unitHoldingCost;
            }

            double costNow = 0;
            for (int i = 0; i < cust.endDayInventory.Length; i++)
            {
                costNow += cust.endDayInventory[i] * cust.unitHoldingCost;
            }
            return costAfter - costNow;

        }
    
        
#endregion DayRelocationMove


        private static double InventoryCostChange_AuxiliaryLists_Depot(Node custToBeRelocated)
        {
            double depotInventoryChange = 0;
            for (int i = 0; i < horizonDays; i++)
            {
                double partSum = (solution.depot.auxiliary_SignedFlowDiffs_Depot[i] * solution.depot.unitHoldingCost) * (horizonDays - i);
                depotInventoryChange += partSum;
            }

            return depotInventoryChange;
        }


        private static MOVES SelectOperatorType(Random r)
        {
            return (MOVES) r.Next(6);
        }

        private static void CreatePointersToModel(PRP irp, Solution input_sol)
        {
            //Setup new solution for LS
            solution = new Solution(input_sol, 0.0);
            
            model = irp;
            distMatrix = model.distMatrix;
            horizonDays = model.horizonDays;
            capacity = model.capacity;
        }
    }
}
