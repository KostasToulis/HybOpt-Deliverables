using System;
using System.Collections.Generic;
using System.Text;
using System.Linq;
//using static F_CVRP.ClarknWright;
using F_CVRP.Local_Search;
using static F_CVRP.Setup;
using System.IO;
using System.Diagnostics;


namespace F_CVRP
{
    public struct LS_params
    {
        public int promiseIterations;
        public int maxIterations;
        public double relocateChance;
        public double swapChance;
        public double twoOptChance;
    }
    public struct LNS_params
    {
        public int restartIterations;
        public double randomRestart;
        public double percentRemoved;
    }

    public class Solver
    {

        public List<Node> selectedCustomers { get; set; }
        public List<Node> unservedCustomers { get; set; }
        public Solution sol { get; set; }

        public Model m;
        public HashSet<Node> customersInSol { get; set; }
        public double wideBudget { get; set; }
        public double narrowBudget { get; set; }


        public Solver(Model model)
        {
            this.m = model;
            this.narrowBudget = (double)m.numReq / m.numNodes;
            this.wideBudget = 1 - narrowBudget;
            this.customersInSol = new HashSet<Node>();
            
        }


        public Solution Solve(Model model, Stopwatch stopwatch)
        {
            CustomerSelection cs = new CustomerSelection(this);
            List<Node> selectedCustomers = cs.InitialSelection();
            ConstructiveHeuristic HS = new ConstructiveHeuristic(model);

            //selectedCustomers.OrderBy(c => c.id).ToList().ForEach(c => Console.Write(c.id + " "));
            (this.sol, this.unservedCustomers) = HS.Solve(selectedCustomers);
            //ReportSolution(sol);
            this.customersInSol = selectedCustomers.ToHashSet();

            sol.routes.ForEach(rt => rt.CalculateRouteCostDemand());


            PromisesSearch ls = new PromisesSearch(this);

            Solution finalSol = new Solution();

            //GuidedLocalSearch gls = new GuidedLocalSearch(this.m, this);

            GuidedPromisesSearch gps = new GuidedPromisesSearch(this);

            AdaptiveMemory AM = new AdaptiveMemory(this);

            int nonImprovements = 0;
            int i = 0;
            int j = 1;


            m.CheckSolutionFeasibility(sol);

            bool infeasible = false;

            Gurobi.MIP FCVRP_model = new Gurobi.MIP();
            int numRem = m.numNodes - m.numReq;

            string oper = "";

            do
            {

                //ls.ApplyLocalSearch(sol);

                //if (m.CheckSolutionFeasibility(ls.bestSol))
                //{
                //    UpdateNodesVars(ls.bestSol, ref finalSol, ref nonImprovements, i);
                //}

                //ApplyLNS(model, cs, ref selectedCustomers, HS, ls.bestSol, AM, FCVRP_model, numRem);



                ////if (nonImprovements >= (this.maxNonImproved * (0.2 * j)))
                ////{
                ////    UpdatePromiseIterations();
                ////    j++;
                ////}

                ////ls.Reset(model, this, finalSol.cost);

                //ls.Reset(this);
                //i++;



                gps.ApplyLocalSearch(sol);

                if (m.CheckSolutionFeasibility(gps.bestSol))
                {
                    UpdateNodesVars(gps.bestSol, ref finalSol, ref nonImprovements, i);
                }

                ApplyLNS(model, cs, ref selectedCustomers, HS, gps.bestSol, AM, FCVRP_model, numRem);


                gps.Reset(this);
                i++;


                if (stopwatch.ElapsedMilliseconds > 1000 * 3600)
                {
                    break;
                }


            } while (nonImprovements < m.runData.restartIter); //stopwatch.ElapsedMilliseconds < 1000 * 3600



            if (!m.CheckSolutionFeasibility(finalSol))
            {
                //Console.WriteLine("Invalid solution");
                finalSol.cost = 0;
            }
            
            return finalSol;
        }

        private void ApplyLNS(Model model, CustomerSelection cs, ref List<Node> selectedCustomers, ConstructiveHeuristic HS, Solution bestSol, AdaptiveMemory AM, Gurobi.MIP FCVRP_model, int numRem)
        {            

            double mathChance = m.runData.random.NextDouble();

            AM.UpdateMembers(bestSol);

            if (mathChance < 0.8)
            {
                Solution partialSol = AM.ConstructPartialSol(bestSol);
                FCVRP_model.RestorePartialSol(model, partialSol);

                if (m.CheckSolutionFeasibility(partialSol))
                {
                    this.sol = partialSol.DeepCopy();
                    this.sol.origin = "AM";
                    this.sol.feasible = true;
                }
                else
                {
                    FCVRP_model.SimulInsDel(model, bestSol, numRem);
                    this.sol = bestSol.DeepCopy();
                    this.sol.origin = "MIP";
                }

            }
            else
            {
                FCVRP_model.SimulInsDel(model, bestSol, numRem);
                this.sol = bestSol.DeepCopy();
                this.sol.origin = "MIP";
            }
            if (!m.CheckSolutionFeasibility(this.sol))
            {
                selectedCustomers = cs.RandomSelection();
                (this.sol, this.unservedCustomers) = HS.Solve(selectedCustomers);
                this.sol.origin = "Random";
            }
        
        }

        private void UpdateNodesVars(Solution bestSol, ref Solution finalSol, ref int nonImprovements, int i)
        {

            m.nodes.ForEach(n => n.isRouted = false);

            bestSol.routes.ForEach(rt => { rt.sequenceOfNodes.ForEach( n => { n.isRouted = true; }); });

            OutputBestCostToFile(bestSol);

            if (i == 0)
            {
                finalSol = bestSol.DeepCopy();
            }
            else
            {
                bestSol.routes.ForEach(rt => { rt.sequenceOfNodes.ForEach( n => { n.timesInBestSol++; }); });

                if (bestSol.cost < finalSol.cost || finalSol.cost == 0)
                {
                    UpdateFinalSol(bestSol, out finalSol, out nonImprovements);
                }
                else
                {
                    nonImprovements++;
                }
            }
        }

        private void UpdateFinalSol(Solution bestSol, out Solution finalSol, out int nonImprovements)
        {
            nonImprovements = 0;
            finalSol = bestSol.DeepCopy();
            finalSol.routes.ForEach(
                rt => { rt.sequenceOfNodes.ForEach( n => { n.timesInFinalSol++; }); });
            //OutputTrajectoriesToFile(ls);
        }

        public void OutputBestCostToFile(Solution bestSol)
        {
            using (StreamWriter costWriter = new StreamWriter("bestCosts.txt", true))
            {

                costWriter.WriteLine($"{bestSol.cost}");
                
            }

            
        }
    }




}
