using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;

namespace SOP_Project
{
    static class Local_Search
    {
        static readonly int TABU_SIZE = 20;  // set the size of the tabu_list
        static readonly int TOTAL_RESTARTS = 20;
        static List<Move> tabu_list = new List<Move>();  // store the last moves to avoid cycling exclusively created by swap operator
        static Solution best_overall = new Solution(new Route(0, 0, new List<Node> { }, new List<Set> { }));
        public static List<Solution> sols = new List<Solution>();

        public static Solution GeneralLocalSearch(Model m, Solution starting_sol, int promise_target)
        {
            Random r = m.r;
            Promises.r = r;
            List<int> sol_evolution = new List<int>();  // contains the solution found in each iteration, useful for plotting the evolution of the algorithm
            Solution sol, best_sol;
            OuterInsertion outerInsertion = new OuterInsertion();
            Delete_Set del_move = new Delete_Set();
            InOutSwap inOutSwap = new InOutSwap();
            InnerRelocation rel_move = new InnerRelocation();
            Move selected_move;

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
            List<Move> moves = new List<Move> { del_move, inOutSwap, rel_move };

            List<int[]> local_bests = new List<int[]>();
            Solution sol_to_keep;

            //Dictionary<int, int> nodesPresenceDict = InitializeNodesPresenceDict(m); // a dictinary to count how many times every node was in a solution (just for inspecting purposes)
            //Dictionary<int, int> setsPresenceDict = InitializeSetsPresenceDict(m); // a dictinary to count how many times every set was in a solution (just for inspecting purposes)
            List<int[]> gurobi_calls = new List<int[]>();

            int best_local_profit, iterations, best_not_improved, pr, best_found_at, exact_calls;
            bool to_apply, no_imp, apply_best_local, new_best_local_found, apply_gap_from_tb;

            // watches to count the time executed
            Stopwatch watch_for_total_time; // counts the total time of the local search
            Stopwatch watch_for_time_of_math; // counts the time of the Mathematical Programming parts
            Stopwatch watch_for_constructive; // counts the time of the constructive


            for (int restart = 0; restart < TOTAL_RESTARTS; restart++)
            {
                // initialize GLS scheme
                sol_evolution.Clear();
                sol_evolution.Add(starting_sol.total_profit);
                local_bests.Clear();
                local_bests.Add(new int[] { 0, starting_sol.total_profit });
                gurobi_calls.Clear();

                sol = starting_sol.DeepCopy(m);
                best_sol = sol.DeepCopy(m);

                best_local_profit = 0;
                iterations = 0;
                best_not_improved = 0;
                pr = 0;
                best_found_at = 0;
                exact_calls = 0;
                to_apply = false;
                no_imp = false;
                apply_best_local = false;
                new_best_local_found = false;
                apply_gap_from_tb = true;

                watch_for_total_time = new Stopwatch();
                watch_for_time_of_math = new Stopwatch();
                watch_for_constructive = new Stopwatch();

                //try
                //{
                //    Directory.CreateDirectory("Pool_Sols/Sols/");
                //}
                //catch { }
                //starting_sol.ReportSol("Pool_Sols/Sols/restart_" + restart + "_obj_" + starting_sol.total_profit + "_starting", m);

                Console.WriteLine("Restart:" + restart);
                Console.WriteLine("Starting sol: " + starting_sol.total_profit);
                Console.WriteLine("Time for construction: " + starting_sol.duration_of_constructive);

                watch_for_total_time.Start();
                while (iterations < 100000 && best_not_improved < 10000)
                {
                    no_imp = false;

                    // find best OuterInsertion and InOutSwap
                    // OuterInsertion
                    outerInsertion.FindBestMove(m, sol);

                    // InOutSwap
                    inOutSwap.FindBestMove(m, sol);

                    if (outerInsertion.profit_added > 0 || inOutSwap.profit > 0) // if a move that increases the profit of the currernt solution is found
                    {
                        if (outerInsertion.profit_added > inOutSwap.profit)
                            selected_move = outerInsertion;
                        else
                            selected_move = inOutSwap;
                    }
                    else // if no move that enhances the best solution is found, check the other moves too, and randomly select one of all of them
                    {
                        no_imp = true;
                        ShuffleList(moves, r); // shuffle the moves

                        Move move = null;

                        // iterate the randomly placed moves and apply the first feasible of them
                        for (int i = 0; i < moves.Count; i++)
                        {
                            move = moves[i];
                            if (!move.GetMoveType().Equals("InOutSwap")) // if the move is not already calculated (outerInsertion is not in the moves list)
                            {
                                move.FindBestMove(m, sol); // calculate the best move of the "move" type operator
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
                    }
                    else // if no feasible move is found reinitialize promises
                    {
                        pr = 0;
                        Promises.reinit_promises(m);
                        continue;
                    }

                    //if (sol.total_profit > best_sol.total_profit)
                    //{
                    //    Console.WriteLine(iterations + " " + sol.total_profit + " " + best_sol.total_profit + " " + selected_move.GetMoveType());
                    //}
                    //Console.WriteLine(iterations + " " + sol.total_profit + " " + best_sol.total_profit + " " + selected_move.GetMoveType());

                    // run the exact when we've reached the promise target or when we've surpassed a pseudo local-optimum
                    if ((to_apply && no_imp))
                    {
                        exact_calls++;
                        gurobi_calls.Add(new int[] { iterations - 1, sol.total_profit });
                        watch_for_time_of_math.Start();
                        MathProgramming.SolveSimulInsDelSubproblem(m, sol, 10, 10);
                        MathProgramming.SolveShortestPath(m, sol);
                        MathProgramming.SolveTSP(m, sol);
                        watch_for_time_of_math.Stop();
                        to_apply = false;
                    }
                    else
                    {
                        if (apply_gap_from_tb && (double)(best_sol.total_profit - sol.total_profit) / best_sol.total_profit < 0.04 && iterations > 1000 && r.Next(100) >= 85)
                        {
                            exact_calls++;
                            watch_for_time_of_math.Start();
                            gurobi_calls.Add(new int[] { iterations - 1, sol.total_profit });
                            MathProgramming.SolveSimulInsDelSubproblem(m, sol, 4, 4);
                            MathProgramming.SolveShortestPath(m, sol);
                            MathProgramming.SolveTSP(m, sol);
                            watch_for_time_of_math.Stop();

                            apply_gap_from_tb = false;
                        }
                        if (gurobi_calls.Count > 0 && iterations >= gurobi_calls[gurobi_calls.Count - 1][0] + 25)
                        {
                            apply_gap_from_tb = true;
                        }
                    }

                    if (pr >= promise_target) // reinitialize promises when needed
                    {
                        pr = 0;
                        Promises.reinit_promises(m);
                        to_apply = true;
                    }

                    // if the current solution's profit is grater than or equal to the worst so far solution in the pool, add it to the pool
                    //if (iterations > 100 && (Pool.sol_pool.Count == 0 || sol.total_profit >= Pool.sol_pool[0].total_profit))
                    //{
                    //    Pool.UpdatePool(sol.DeepCopy(m));
                    //}

                    // update best_sol
                    if (sol.total_profit > best_sol.total_profit)
                    {
                        best_sol = sol.DeepCopy(m);
                        if (iterations > 1000) // don't call the method from the beginning in order to save some time
                        {
                            exact_calls++;
                            gurobi_calls.Add(new int[] { iterations, sol.total_profit });
                            //=========================CAUTION===============================================
                            // Maybe it is useful to call the method at the first iteration after the limit, because for different
                            // datasets the solution may never be imporved after the limit (here 1000)
                            watch_for_time_of_math.Start();
                            MathProgramming.SolveSimulInsDelSubproblem(m, sol, 3, 3);
                            MathProgramming.SolveShortestPath(m, sol);
                            MathProgramming.SolveTSP(m, sol);
                            watch_for_time_of_math.Stop();
                        }
                        if (sol.total_profit >= best_sol.total_profit)  // apply this noly when the exact gives a better solution
                        {
                            best_sol = sol.DeepCopy(m);
                        }
                        best_found_at = iterations;
                        Promises.reinit_promises(m);
                        best_not_improved = -1;
                    }

                    iterations++;
                    pr++;
                    best_not_improved++;

                    sol_evolution.Add(sol.total_profit);

                    //UpdatePresenceDicts(nodesPresenceDict, setsPresenceDict, sol);
                }
                watch_for_total_time.Stop(); // stop the watch that counts total local search time

                //Console.WriteLine("Best found at: {0}", best_found_at);
                best_sol.CheckSol(m);
                Console.WriteLine("Improved_sol:" + best_sol.total_profit);
                //best_sol.ReportSol("Pool_Sols/Sols/restart_" + restart + "_obj_" + best_sol.total_profit + "_improved", m);

                best_sol.duration_of_constructive = starting_sol.duration_of_constructive;
                best_sol.duration_of_local_search = watch_for_total_time.ElapsedMilliseconds / 1000.0;
                best_sol.duration_of_maths = watch_for_time_of_math.ElapsedMilliseconds / 1000.0;
                best_sol.iteration_best_found = best_found_at;
                best_sol.iterations_of_local_search = iterations;
                best_sol.n_exact = exact_calls;

                Console.WriteLine("Total time: " + best_sol.duration_of_local_search);
                //ReportStuff.ReportVisitedNodes(nodesPresenceDict, setsPresenceDict, m);
                //Solution.Report_Sol_Evolution(sol_evolution, "Reports/restart_" + restart);
                //ReportStuff.ReportGurobiCalls(gurobi_calls, "Reports/restart_" + restart);

                // update overall best solution foynd in all restarts
                if (best_sol.total_profit > best_overall.total_profit)
                {
                    best_overall = best_sol;
                }
                sols.Add(best_sol);

                // run pool constructive to generate a new starting solution
                if (restart < TOTAL_RESTARTS - 1)
                {
                    watch_for_constructive.Start();
                    starting_sol = Initialization.Minimum_Insertions(m);
                    //starting_sol = Pool.CreateSolutionBySetsInPool(m);
                    watch_for_constructive.Stop();
                    starting_sol.duration_of_constructive = watch_for_constructive.ElapsedMilliseconds / 1000.0;
                }
            }

            return best_overall;
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
    }
}
