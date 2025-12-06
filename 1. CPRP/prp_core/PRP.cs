using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using System.IO;
using System.Linq;
using System.Net.Mime;
using System.Threading.Tasks;
using Accord.IO;


//Define this flag when the maximum stock capacity of a customer during a delivery should be respected
//#define RESPECT_CUSTOMER_CAPACITY


namespace PRP
{
    public struct SolutionResults
    {
        public double elapsedTime;
        public double relaxedSolutionObjective;
        public double bestSolutionObjective;
        public string productionSchedule;
        public bool feasible;

    }
    
        
    public class PRP
    {
        public DataInput input;
        public ProductionDataInput production_input;
        public double[,] distMatrix;
        public double[] minDistancesFrom;
        public double[] maxDistancesFrom;
        public double[] avgDistancesFrom;
        public int horizonDays;
        public Solution overallBestSolution;
        public double bestTimeElapsedMs;
        //public double totalTimeElapsed;
        public int capacity;
        public int vehicles;
        public string instanceName;
        public bool log;
        public System.Diagnostics.Stopwatch stopwatch;
        public StreamWriter streawriter;
        //PRP solve Parameters
        public static int restarts;

        public PRP(DataInput input, ProductionDataInput pr_input, string inst,  bool logging)
        {
            this.input = input;
            production_input = pr_input;
            instanceName = inst;
            vehicles = input.availableVehicles;
            horizonDays = input.horizonDays;
            log = logging;
            
            //Set Vehicle Capacity
            capacity = input.dayVehicleCapacity;
            
            //Generate Distance Matrix
            GenerateDistanceMatrix();
            CheckDistanceMatrixTringularities();
        }

        public SolutionResults solve() {
            
            SolutionResults res = new SolutionResults();
            res.feasible = true;
            res.productionSchedule = production_input.ID;
            res.relaxedSolutionObjective = production_input.relaxedObjective;
            res.elapsedTime = 0;
            stopwatch = new Stopwatch();

            /*
            FileStream filestream = new FileStream("ConsoleLog_" + this.instanceName+".txt", FileMode.Create);
            var streamwriter = new StreamWriter(filestream);
            streamwriter.AutoFlush = true;
            Console.SetOut(streamwriter);
            Console.SetError(streamwriter);
            */
            //redirect output
            /*
            FileStream ostrm;
            StreamWriter writer;
            TextWriter oldOut = Console.Out;
            ostrm = new FileStream("ConsoleLog_" + this.instanceName, FileMode.OpenOrCreate, FileAccess.Write);
            writer = new StreamWriter(ostrm);
            Console.SetOut(writer);
            */
            bool solveExact = false;
            bool solveHeuristic = false;

            if (MathematicalProgramming.exactParams.algorithm == "exact")
            {
                solveExact = true;
            }

            if (LocalSearch.parameters.algorithm == "ls")
            {
                solveHeuristic = true;
            }

            if (solveExact)
            {
                stopwatch.Reset();
                stopwatch.Start();
                bool feasible = false;

                // solve with MIP
                Solution optimized_sol;
                Dictionary<string, double> dict;

                if (MathematicalProgramming.exactParams.periodicity == "cyclic")
                {
                    (optimized_sol, dict) = MathematicalProgramming.SolveCPRP(this, false); 
                } 
                else if (MathematicalProgramming.exactParams.periodicity == "periodic")
                {
                    (optimized_sol, dict) = MathematicalProgramming.SolvePPRP(this, false); //First time solve with continuous
                }
                else if (MathematicalProgramming.exactParams.periodicity == "basic")
                {
                    (optimized_sol, dict) = MathematicalProgramming.SolvePPRP(this, false); //First time solve with continuous
                } 
                else
                {
                    optimized_sol = null;
                    dict = null;
                }

                bool save_sol = false;
                if (overallBestSolution == null && optimized_sol != null)
                    save_sol = true;
                else if (overallBestSolution != null && optimized_sol.isBetterThan(overallBestSolution))
                    save_sol = true;

                //optimized_sol.SaveToFile("Restart_" + i + "Sol_" + instanceName);
                if (save_sol)
                {
                    overallBestSolution = new Solution(optimized_sol, 0);
                    SolutionManager.SaveAnalyticalMIPSolutionFile(overallBestSolution, dict);
                }

                stopwatch.Stop();
                //totalTimeElapsed += stopwatch.ElapsedMilliseconds * 0.001;
                res.elapsedTime += stopwatch.ElapsedMilliseconds * 0.001; //totalTimeElapsed;

                /* TODO save the solution
                //optimized_sol.SaveToFile("Restart_" + i + "Sol_" + instanceName);
                if (save_sol)
                {
                    overallBestSolution = new Solution(optimized_sol, 0);
                    res.bestSolutionObjective = overallBestSolution.totalObjective;
                    SolutionManager.SaveAnalyticalSolutionFile(overallBestSolution);
                }

                overallBestSolution.TestEverythingFromScratch();
                UpdateLogger(optimized_sol, i);

                stopwatch.Stop();
                //totalTimeElapsed += stopwatch.ElapsedMilliseconds * 0.001;
                res.elapsedTime += stopwatch.ElapsedMilliseconds * 0.001; //totalTimeElapsed;
                */
            }

            if (solveHeuristic)
            {
                for (int i = 0; i < restarts; i++)
                {
                    stopwatch.Reset();
                    stopwatch.Start();
                    bool feasible = false;

                    Solution constructed_sol = null;
                    // TODO: fix construction heuristic
                    constructed_sol = ConstructionHeuristic.GenerateInitialSolution(this, i);

                    //Solution constructed_sol = new Solution(this);
                    //constructed_sol.ImportFromFile("russel_sol_new.txt", false);

                    if (constructed_sol == null)
                    {
                        res.elapsedTime = 0; //totalTimeElapsed;
                        res.feasible = false;
                        res.bestSolutionObjective = Double.MaxValue;
                        throw new Exception("Check this shit out");
                    }

                    //TEST MIP
                    //MIP.runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemoval(
                    //    constructed_sol);

                    //FORCE solution from file
                    //constructed_sol.ImportFromFile("Sol_RelSol_Arc_ABS1_100_2.DAT_0_1_0_1_0_1_194760.txt");

                    //Call VRP SOLVER
                    //VRP_LOCALSOLVER.SolveVRP(constructed_sol);

                    // BETA Reducing the graph 
                    //GraphReduction graphRed = new GraphReduction(this);
                    //Solution temp = graphRed.preprocess();

                    // solve with local search
                    Solution optimized_sol = LocalSearch.ApplyLocalSearch(this, constructed_sol, i);

                    bool save_sol = false;
                    if (overallBestSolution == null && optimized_sol != null)
                        save_sol = true;
                    else if (overallBestSolution != null && optimized_sol.isBetterThan(overallBestSolution))
                        save_sol = true;

                    //optimized_sol.SaveToFile("Restart_" + i + "Sol_" + instanceName);

                    if (save_sol)
                    {
                        overallBestSolution = new Solution(optimized_sol, 0);
                        res.bestSolutionObjective = overallBestSolution.totalObjective;
                        SolutionManager.SaveAnalyticalSolutionFile(overallBestSolution);
                    }

                    overallBestSolution.TestEverythingFromScratch();
                    UpdateLogger(optimized_sol, i);

                    stopwatch.Stop();
                    //totalTimeElapsed += stopwatch.ElapsedMilliseconds * 0.001;
                    res.elapsedTime += stopwatch.ElapsedMilliseconds * 0.001; //totalTimeElapsed;
                }
            }
            /*
            Console.SetOut(oldOut);
            writer.Close();
            ostrm.Close();
            */
            Console.WriteLine("Finished successfully");

            return res;
        }

        private void CreateLogFile(bool logging, bool feas)
        {
            log = logging;
            if (!log)
            {
                return;
            }
            streawriter = File.CreateText("Log_" + instanceName);
            if (!feas)
            {
                streawriter.WriteLine("Capacity Infeasibility");
            }
            streawriter.Flush();
            streawriter.Close();
        }

        private void UpdateLogger(Solution sol, int itr)
        {
            if (!log)
            {
                return;
            }
            else
            {
                streawriter = File.AppendText("Log_" + instanceName);
                streawriter.WriteLine(itr + " " + sol.totalObjectiveIncFirstPeriodInventory + " " + sol.ellapsedMs/1000 + " " + sol.restartElapsedMs/1000
                    + " " + sol.feasibleSpaceIters + " " + sol.infeasibleSpaceIters + " " + sol.totalRepairs + " " + sol.totalTimeRepairing/1000);

                streawriter.Flush();
                streawriter.Close();
            }
        }

        private void GenerateDistanceMatrix()
        {
            distMatrix = new double[input.nodes.Count, input.nodes.Count];

            for (int i = 0; i <input.nodes.Count; i++)
            {
                Node from = input.nodes[i];
                for (int j = 0; j <input.nodes.Count; j++)
                {
                    Node to = input.nodes[j];

                    distMatrix[i, j] = Distance(from.x_coord, from.y_coord, to.x_coord, to.y_coord);
                    distMatrix[i, j] = Math.Round(distMatrix[i, j], 0) * input.distanceCoeff;
                }
            }

            // calculate and store min and max distances from every node
            minDistancesFrom = new double[input.nodes.Count];
            for (int i = 0; i < input.nodes.Count; i++)
            {
                Node from = input.nodes[i];
                double dist = Double.MaxValue;
                for (int j = 0; j < input.nodes.Count; j++)
                {
                    if (i!=j)
                    {
                        Node to = input.nodes[j];
                        if (distMatrix[from.uid, to.uid] < dist)
                        {
                            dist = distMatrix[from.uid, to.uid];
                        }
                    }
                }
                minDistancesFrom[from.uid] = dist;
            }

            maxDistancesFrom = new double[input.nodes.Count];
            for (int i = 0; i < input.nodes.Count; i++)
            {
                Node from = input.nodes[i];
                double dist = 0.0; ;
                for (int j = 0; j < input.nodes.Count; j++)
                {
                    if (i != j)
                    {
                        Node to = input.nodes[j];
                        if (distMatrix[from.uid, to.uid] > dist)
                        {
                            dist = distMatrix[from.uid, to.uid];
                        }
                    }
                }
                maxDistancesFrom[from.uid] = dist;
            }

            avgDistancesFrom = new double[input.nodes.Count];
            for (int i = 0; i < input.nodes.Count; i++)
            {
                Node from = input.nodes[i];
                double dist = 0.0; ;
                for (int j = 0; j < input.nodes.Count; j++)
                {
                    if (i != j)
                    {
                        Node to = input.nodes[j];
                        dist += distMatrix[from.uid, to.uid];
                    }
                }
                dist /= input.nodes.Count - 1;
                avgDistancesFrom[from.uid] = dist;
            }
        }

        private void CheckDistanceMatrixTringularities()
        {
            for (int i = 0; i < input.nodes.Count; i++)
            {
                Node from = input.nodes[i];
                for (int j = 0; j < input.nodes.Count; j++)
                {
                    Node to = input.nodes[j];
                    for (int k = 0; k < input.nodes.Count; k++)
                    {
                        Node through = input.nodes[k];
                        if (i!=j && i!=k && j!=k)
                            if (distMatrix[i, j] > distMatrix[i, k]+ distMatrix[k, j])
                            {
                                //GlobalUtils.writeToConsole("Dist ({3},{4}) {0} > {6} dist({3},{5}){1} + ({5},{4}){2}", distMatrix[i, j], distMatrix[i, k],
                                    //distMatrix[k, j], from.uid, to.uid, through.uid, distMatrix[i, k] + distMatrix[k, j]);
                            }
                            if (distMatrix[i, j] == distMatrix[i, k] + distMatrix[k, j])
                            {
                                //GlobalUtils.writeToConsole("Dist ({3},{4}) {0} = dist({3},{5}){1} + ({5},{4}){2}", distMatrix[i, j], distMatrix[i, k],
                                //    distMatrix[k, j], from.uid, to.uid, through.uid);
                            }
                    }
                }
            }
        }

        private double Distance(double x1, double y1, double x2, double y2)
        {
            double dx = x1 - x2;
            double dy = y1 - y2;

            return Math.Sqrt(Math.Pow(dx, 2) + Math.Pow(dy, 2));
        }

        
    }
}
