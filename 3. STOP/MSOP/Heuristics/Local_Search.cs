using MSOP.Fundamentals;
using MSOP.MathematicalProgramming;
using MSOP.Operators;
using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace MSOP.Heuristics
{
    class Local_Search
    {
        static int count_restarts = 0;
        static Solution best_overall;
        public static List<Solution> sols = new List<Solution>();

        public static Solution GeneralLocalSearch(Solution starting_sol, int promise_target, int not_improved_iterations)
        {
            // params
            int maxIters = 100000;
            bool silence = true;

            Model m = Model.model;

            //starting_sol.ReportSol("Pool_Sols/Sols/restart_" + count_restarts + "_obj_" + starting_sol.total_profit + "_starting");

            //Console.WriteLine("\nRestart:" + count_restarts);
            //Console.WriteLine("Starting sol: " + starting_sol.total_profit);

            Random r = m.r;
            List<int> sol_evolution = new List<int>();  // contains the solution found in each iteration, useful for plotting the evolution of the algorithm
            sol_evolution.Add(starting_sol.total_profit);
            Solution sol = starting_sol.DeepCopy(); // shallow copy the starting_sol object into a new one
            Solution best_sol = sol.DeepCopy(); // store the best_sol encountered in the search
            OuterInsertion outerInsertion = new OuterInsertion();
            NodeRemoval nodeRemoval = new NodeRemoval();
            InOutSwap inOutSwap = new InOutSwap();
            InnerRelocation innerRelocation = new InnerRelocation();
            TwoOpt twoOpt = new TwoOpt();
            Move selected_move;

            Promises.r = r;
            double cur_t_max = m.t_max;

            // a list of the moves, of which one will be stochastically chosen to be executed
            // -outerInsertion is not included, because if a feasible insertion exists it will be
            //      executed before the stochastic selection (because it increases the profit)
            // - How the selection is done: if there is no feasible move the increases the profit,
            //      the list moves will be shuffled (using method ShuffleList) and the random order
            //      will be used as priority order to find which move will be executed
            // - The point of this procedure is to call the method FindBestMove only a necessary
            //      number of times and not for every move in every iteration
            // - CAUTION: the method FindBestMove of inOutSwap and outerInsertion is called in every
            //      iteration, in order to not lose moves that increase the profit of the solution
            List<Move> moves = new List<Move> { outerInsertion, nodeRemoval, inOutSwap, innerRelocation, twoOpt };

            Dictionary<int, int> nodesPresenceDict = InitializeNodesPresenceDict(); // a dictinary to count how many times every node was in a solution (just for inspecting purposes)
            Dictionary<int, int> setsPresenceDict = InitializeSetsPresenceDict(m); // a dictinary to count how many times every set was in a solution (just for inspecting purposes)

            int iterations = 0;
            int best_not_improved = 0;
            int pr = 0;
            int best_found_at = 0;

            bool to_apply = false;


            // watches to count the time executed
            Stopwatch watch_for_total_time = System.Diagnostics.Stopwatch.StartNew(); // counts the total time of the local search
            Stopwatch watch_for_time_of_math = new Stopwatch(); // counts the time of the Mathematical Programming parts

            while (iterations < maxIters && best_not_improved < not_improved_iterations)
            {
                // 1. Explore neighborhood and apply move
                // find best OuterInsertion and InOutSwap
                // OuterInsertion
                outerInsertion.FindBestMove(sol);

                // InOutSwap
                inOutSwap.FindBestMove(sol);

                if (outerInsertion.profit_added > 0 || inOutSwap.profit > 0) // if a move that increases the profit of the currernt solution is found
                {
                    if (outerInsertion.profit_added > inOutSwap.profit)
                        selected_move = outerInsertion;
                    else
                        selected_move = inOutSwap;
                }
                else // if no move that enhances the best solution is found, check the other moves too, and randomly select one of all of them
                {
                    ShuffleList(moves, r); // shuffle the moves

                    Move move = null;

                    // iterate the randomly placed moves and apply the first feasible of them
                    for (int i = 0; i < moves.Count; i++)
                    {
                        move = moves[i];
                        if (!move.GetMoveType().Equals("InOutSwap") && !move.GetMoveType().Equals("OuterInsertion")) // if the move is not already calculated (outerInsertion is not in the moves list)
                        {
                            move.FindBestMove(sol); // calculate the best move of the "move" type operator
                        }
                        if (move.IsMoveFound()) // if a feasible move is found don't calculate the best move of the other operators
                        {
                            break;
                        }
                    }
                    selected_move = move;
                }

                if (selected_move.IsMoveFound())
                {
                    // apply selected_move
                    selected_move.ApplyBestMove(sol);
                    if (!sol.CheckSol())
                    {
                        break;
                    }
                }
                else // if no feasible move is found reinitialize promises (Note: since node removal is a move, this branch will never be used)
                {
                    if (pr == 0)
                    {
                        return best_sol;
                    }
                    pr = 0;
                    Promises.reinit_promises();
                    to_apply = true; // apply mathematical programming
                    continue;
                }

                if (!silence)
                {
                    Console.WriteLine("#it: {0} profit(cur/best): {2} / {3}  ({1})", iterations, selected_move.GetMoveType(), sol.total_profit, best_sol.total_profit);
                }

                double percent_gap_for_math = 0.04;
                double percent_probability_for_math = 95;
                // 2. Apply mathematical programming when we've reached the promise target or when we've surpassed a pseudo local-optimum
                if (to_apply)
                {
                    watch_for_time_of_math.Start();
                    MathProgramming.SolveSimulInsDelSubproblem(m, sol, 3 * m.vehicle_number, 3 * m.vehicle_number);
                    MathProgramming.SolveShortestPath(m, sol);
                    MathProgramming.SolveTSP(m, sol);
                    watch_for_time_of_math.Stop();
                    to_apply = false;
                }
                else if ((double)(best_sol.total_profit - sol.total_profit) / best_sol.total_profit < percent_gap_for_math
                        && (double)(best_sol.total_profit - sol.total_profit) / best_sol.total_profit >= 0.00) // do not activate math when a new best solution is already found
                {
                    //watch_for_time_of_math.Start();
                    // using math here makes the search more intensive around local optima.
                    //MathProgramming.SolveShortestPath(m, sol);
                    //MathProgramming.SolveTSP(m, sol);
                    //MathProgramming.SolveShortestPath(m, sol);
                    //watch_for_time_of_math.Stop();

                    if (iterations > m.node_crowd
                        && best_not_improved > (3.00 * m.set_crowd) 
                        && r.Next(100) >= percent_probability_for_math)
                    {
                        watch_for_time_of_math.Start();
                        MathProgramming.SolveSimulInsDelSubproblem(m, sol, 3 * m.vehicle_number, 3 * m.vehicle_number);
                        MathProgramming.SolveShortestPath(m, sol);
                        MathProgramming.SolveTSP(m, sol);
                        watch_for_time_of_math.Stop();
                        to_apply = false;
                        Console.WriteLine("-MILP-");
                        // pr = 0;
                        // Promises.reinit_promises();
                    }

                }

                // 3. update best_sol
                if (sol.total_profit > best_sol.total_profit)
                {
                    if (silence)
                    {
                        Console.WriteLine(iterations + " " + sol.total_profit + " " + best_sol.total_profit);
                    }

                    best_sol = sol.DeepCopy();
                    best_found_at = iterations;
                    Promises.reinit_promises();
                    best_not_improved = -1;
                }

                // 4. reinitialize promises when needed
                if (pr >= promise_target)
                {
                    pr = 0;
                    Promises.reinit_promises();
                    to_apply = true;
                }

                // 5. update counters and auxiliary structures
                iterations++;
                pr++;
                best_not_improved++;

                sol_evolution.Add(sol.total_profit);

                UpdatePresenceDicts(nodesPresenceDict, setsPresenceDict, sol);
            }
            watch_for_total_time.Stop(); // stop the watch that counts total local search time

            Console.WriteLine("Best found at: {0}", best_found_at);
            best_sol.CheckSol();
            Console.WriteLine("Improved_sol:" + best_sol.total_profit);

            best_sol.duration_of_local_search = watch_for_total_time.ElapsedMilliseconds / 1000.0;
            best_sol.duration_of_maths = watch_for_time_of_math.ElapsedMilliseconds / 1000.0;
            best_sol.iteration_best_found = best_found_at;
            best_sol.iterations_of_local_search = iterations;

            //Solution.Report_Sol_Evolution(sol_evolution, "Reports/restart_" + count_restarts);
            //ReportStuff.ReportVisitedNodes(nodesPresenceDict, setsPresenceDict, m);
            //ReportStuff.ReportGurobiCalls(gurobi_calls, "Reports/restart_" + count_restarts);
            sols.Add(best_sol);




            //...

            return best_sol;
        }

        public static void ShuffleList<Move>(List<Move> list, Random random) // method from https://stackoverflow.com/questions/49570175/simple-way-to-randomly-shuffle-list
        {
            for (int i = list.Count - 1; i > 1; i--)
            {
                int rnd = random.Next(i + 1);

                Move value = list[rnd];
                list[rnd] = list[i];
                list[i] = value;
            }
        }

        public static Dictionary<int, int> InitializeNodesPresenceDict()
        {
            Model m = Model.model;
            Dictionary<int, int> nodesPresenceDict = new Dictionary<int, int>();
            for (int i = 0; i < m.node_crowd; i++)
            {
                nodesPresenceDict.Add(m.nodes[i].id, 0);
            }
            return nodesPresenceDict;
        }

        public static Dictionary<int, int> InitializeSetsPresenceDict(Model m)
        {
            Dictionary<int, int> setsPresenceDict = new Dictionary<int, int>();
            for (int i = 0; i < m.set_crowd; i++)
            {
                setsPresenceDict.Add(m.sets[i].id, 0);
            }
            return setsPresenceDict;
        }

        public static void UpdatePresenceDicts(Dictionary<int, int> nodesPresenceDict, Dictionary<int, int> setsPresenceDict, Solution sol)
        {
            foreach (Route route in sol.routes)
            {
                for (int i = 1; i < route.nodes_seq.Count - 1; i++)
                {
                    nodesPresenceDict[route.nodes_seq[i].id]++;
                    setsPresenceDict[route.nodes_seq[i].set_id]++;
                }
            }
        }
    }
}
