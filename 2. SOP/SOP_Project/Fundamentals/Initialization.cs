using System;
using System.Collections.Generic;
using System.Linq;

namespace SOP_Project
{
    class Constructive_Move
    {
        public Node node;
        public int pos;
        public int cost_added;
        public double profit_to_cost_rate;

        public Constructive_Move(Node node, int pos, int cost_added, double profit_to_cost_rate)
        {
            this.node = node;
            this.pos = pos;
            this.cost_added = cost_added;
            this.profit_to_cost_rate = profit_to_cost_rate;
        }
    }

    class Chain_Constructive_Move
    {
        public Chain chain;
        public List<Node> nodes_seq_reversed;  // store the sequence in reverse order so that nodes are placed appropriately
        public List<int> duplicate_nodes_pos_reversed = new List<int>();  // positions of same sets with the chain that should be removed before new insertion
        public int pos;
        public int cost_added;
        public int profit_added;
        public int n_sets_added;

        public Chain_Constructive_Move(Chain chain, List<int> duplicate_nodes_pos, int pos, int connect_chains_cost, int duplicate_profit)
        {
            this.chain = chain;
            this.nodes_seq_reversed = chain.nodes_seq.ToList();  // create a deep copy of the nodes list
            this.nodes_seq_reversed.Reverse();
            for (int i = duplicate_nodes_pos.Count - 1; i >= 0 ; i--)
            {
                this.duplicate_nodes_pos_reversed.Add(duplicate_nodes_pos[i]);
            }
            this.pos = pos;
            this.cost_added = connect_chains_cost;
            this.profit_added = chain.profit - duplicate_profit;
            this.n_sets_added = chain.size;
        }
    }

    class Chain_Constructive_Move2 // for Chain_Minimum_Insertions
    {
        public Chain chain;
        public int pos;
        public int cost_added;
        public double profit_to_cost_rate;

        public Chain_Constructive_Move2(Chain chain, int pos, int cost_added, double profit_to_cost_rate)
        {
            this.chain = chain;
            this.pos = pos;
            this.cost_added = cost_added;
            this.profit_to_cost_rate = profit_to_cost_rate;
        }
    }

    class Initialization
    {
        static readonly int RCL_SIZE = 3;

        public static Solution Minimum_Insertions(Model m)
        {
            Solution sol = new Solution(new Route(new List<Node> { m.depot, m.depot }, m));
            
            //m.sets[0].in_route = true;
            int rcl_limit = 0;  // specifies the exact number of available moves to choose from
            Random r = m.r;

            while (true)
            {
                List<Constructive_Move> feasible_insertions = new List<Constructive_Move>();  // list_size: RCL
                Route route = sol.route;
                List<Node> nodes_seq = route.nodes_seq;
                int pos_of_worst;
                Constructive_Move worst_move, temp;

                // find all feasible insertions for each position
                for (int i = 1; i < nodes_seq.Count; i++)
                {
                    foreach (Node node in m.nodes)
                    {
                        if (route.sets_included.Contains(m.sets[node.set_id])) // set is already covered 
                        {
                            continue;
                        }
                        int cost_added = m.dist_matrix[nodes_seq[i - 1].id, node.id] + m.dist_matrix[node.id, nodes_seq[i].id] - m.dist_matrix[nodes_seq[i - 1].id, nodes_seq[i].id];
                        if (route.time + cost_added <= m.t_max)  // node addition results in feasible solution 
                        {
                            double profit_to_cost_rate = cost_added > 0 ? m.sets[node.set_id].profit / cost_added : Math.Pow(10, 10);

                            // add this element to the rcl list only if the list is not full or
                            // the profit to cost rate is greater than the minimum (in position 0) already in list
                            if (feasible_insertions.Count < RCL_SIZE)
                            {
                                feasible_insertions.Add(new Constructive_Move(node, i, cost_added, profit_to_cost_rate));
                            }
                            else if (profit_to_cost_rate > feasible_insertions[0].profit_to_cost_rate)
                            {
                                feasible_insertions[0] = new Constructive_Move(node, i, cost_added, profit_to_cost_rate);
                                // in order for the previous commands to work properly, we need to make sure that the move with the
                                // minimum profit to cost rate will alwatys be at position 0 of the list
                                if (feasible_insertions.Count == RCL_SIZE)
                                {
                                    worst_move = feasible_insertions.Find(x => x.profit_to_cost_rate == feasible_insertions.Min(x => x.profit_to_cost_rate));
                                    pos_of_worst = feasible_insertions.IndexOf(worst_move);
                                    if (pos_of_worst != 0)
                                    {
                                        temp = feasible_insertions[0];
                                        feasible_insertions[0] = worst_move;
                                        feasible_insertions[pos_of_worst] = temp;
                                    }
                                }
                            }
                        }
                    }
                }

                if (feasible_insertions.Count == 0) // there aren't any feasible moves
                {
                    break;
                }

                // sort feasible insertions
                var sorted_insertions = feasible_insertions.OrderByDescending(x => x.profit_to_cost_rate);

                // apply multi-restart method
                rcl_limit = Math.Min(sorted_insertions.Count(), RCL_SIZE);
                Constructive_Move selected_move = (Constructive_Move) sorted_insertions.ElementAt(r.Next(rcl_limit));
                
                // find the node and set added
                Node selected_node = selected_move.node;
                //m.sets[selected_node.set_id].in_route = true;
             
                // add selected node to the solution
                sol.route.nodes_seq.Insert(selected_move.pos, selected_node);
                sol.route.sets_included.Insert(selected_move.pos, m.sets[selected_node.set_id]);
                sol.route.time += selected_move.cost_added;
                sol.route.total_profit += m.sets[selected_node.set_id].profit;
                sol.total_time += selected_move.cost_added;
                sol.total_profit += selected_node.profit;
            }
            return sol;
        }

        public static Solution ConstructSolFromPool(Model m, Solution starting_sol = null)
        {
            Solution sol = null;
            if (starting_sol == null)
            {
                sol = new Solution(new Route(new List<Node> { m.depot, m.depot }, m));
            }
            else
            {
                sol = starting_sol;
            }

            //m.sets[0].in_route = true;
            int rcl_limit = 0;  // specifies the exact number of available moves to choose from
            bool to_do_TSP = true;
            Random r = m.r;

            while (true)
            {
                List<Constructive_Move> feasible_insertions = new List<Constructive_Move>();
                Route route = sol.route;
                List<Node> nodes_seq = route.nodes_seq;

                // find all feasible insertions for each position
                for (int i = 1; i < nodes_seq.Count; i++)
                {
                    foreach (Node node in m.nodes)
                    {
                        if (route.sets_included.Contains(m.sets[node.set_id]) || node.pool_profit < 0) // set is already covered or the node must not be
                                                                                                       // used (pool_profit < 0, and that might happen in case of creation a solution from the pool)
                        {
                            continue;
                        }
                        int cost_added = m.dist_matrix[nodes_seq[i - 1].id, node.id] + m.dist_matrix[node.id, nodes_seq[i].id] - m.dist_matrix[nodes_seq[i - 1].id, nodes_seq[i].id];
                        //Console.WriteLine("{0} {1}", cost_added, m.t_max);
                        if (route.time + cost_added <= m.t_max)  // node addition results in feasible solution 
                        {
                            double profit_to_cost_rate = cost_added > 0 ? m.sets[node.set_id].profit / cost_added : Math.Pow(10, 10);
                            feasible_insertions.Add(new Constructive_Move(node, i, cost_added, profit_to_cost_rate));
                        }
                    }
                }

                if (feasible_insertions.Count == 0) // there aren't any feasible moves
                {
                    if (to_do_TSP)
                    {
                        MathProgramming.SolveTSP(m, sol);
                        to_do_TSP = false; // don't do back to back TSPs
                        continue;
                    }
                    break;
                }

                // sort feasible insertions
                var sorted_insertions = feasible_insertions.OrderByDescending(x => x.profit_to_cost_rate);

                // apply multi-restart method
                rcl_limit = Math.Min(sorted_insertions.Count(), RCL_SIZE);
                Constructive_Move selected_move = (Constructive_Move)sorted_insertions.ElementAt(r.Next(rcl_limit));

                // find the node and set added
                Node selected_node = selected_move.node;
                //m.sets[selected_node.set_id].in_route = true;

                // add selected node to the solution
                sol.route.nodes_seq.Insert(selected_move.pos, selected_node);
                sol.route.sets_included.Insert(selected_move.pos, m.sets[selected_node.set_id]);
                sol.route.time += selected_move.cost_added;
                sol.route.total_profit += m.sets[selected_node.set_id].profit;
                sol.total_time += selected_move.cost_added;
                sol.total_profit += selected_node.profit;
                to_do_TSP = true;
            }
            sol.CheckSol(m);
            return sol;
        }

        public static Solution SolveAsKnapsack_Infeasible(Model m)
        {
            int rcl_limit;
            Random r = m.r;
            double bin_infeasibility_upper_bound = 1.3 * m.t_max;  // set the maximum allowed time a bin can reach
            double sol_infeasibility_upper_bound = 0.99 * m.t_max;  // set the maximum allowed time a solution can reach

            // Chain.exportChains(Chain.all_chains, m.dataset_name, "chains.txt");

            Dictionary<Chain, int> feasible_insertions = new Dictionary<Chain, int>();  // key: chain, value: added_cost
            List<Chain> feasible_chains = new List<Chain>();
            HashSet<Chain> route_bin = new HashSet<Chain>();
            HashSet<Node> nodes_in_bin = new HashSet<Node>();
            HashSet<string> arc_bin = new HashSet<string>();  // a bin containing all arcs added to the bin, used to erase duplicate costs
            int bin_cost = 0, added_cost = 0, common_nodes = 0;
            string chain_arc, chain_arc_reversed;

            // Phase 1: Apply the knapsack logic to create a bin with the chains of highest value
            var sorted_chains = Chain.all_chains.OrderByDescending(x => x.profit / x.size);
            while (true)
            {
                // find all feasible insertions
                foreach (Chain chain in sorted_chains)
                {
                    added_cost = chain.time;  // initialize added cost
                    // find every chain arc that is already included in the bin and remove it from the added cost
                    common_nodes = nodes_in_bin.Intersect(chain.nodes_seq.ToHashSet()).Count();
                    if (common_nodes >= 2) 
                    {
                        for (int i = 0; i < chain.nodes_seq.Count - 1; i++)
                        {
                            Node n1 = chain.nodes_seq[i];
                            Node n2 = chain.nodes_seq[i + 1];
                            chain_arc = n1.id + "," + n2.id;
                            chain_arc_reversed = n2.id + "," + n1.id;
                            if (arc_bin.Contains(chain_arc) || arc_bin.Contains(chain_arc_reversed))
                            {
                                added_cost -= m.dist_matrix[n1.id, n2.id];
                            }
                        }
                    }
                    // add the chain to the bin if the added cost doesn't exceed a predefined bound
                    if (bin_cost + added_cost <= bin_infeasibility_upper_bound && added_cost > 0)
                    {
                        //chain.useful_size = chain.size - common_nodes;
                        feasible_insertions.Add(chain, added_cost);
                        feasible_chains.Add(chain);
                    } else  // the chain will never be able to be part of the solution - remove it to reduce time
                    {
                        Chain.all_chains.Remove(chain);
                    }
                }

                if (feasible_insertions.Count == 0)  // there aren't any feasible moves
                {
                    break;
                }

                // sort feasible insertions
                //var sorted_insertions = feasible_insertions.Keys.OrderByDescending(x => Math.Pow(x.profit, 1) / x.size);  // sort by unit value of every chain

                // apply grasp logic
                rcl_limit = Math.Min(feasible_chains.Count(), RCL_SIZE);
                Chain selected_chain = (Chain)feasible_chains.ElementAt(r.Next(rcl_limit));

                // add selected chain to the route bin
                route_bin.Add(selected_chain);
                bin_cost += feasible_insertions[selected_chain];
                // update nodes_in_bin
                foreach (Node node in selected_chain.nodes_seq)
                {
                    nodes_in_bin.Add(node);
                }
                // update arc_bin
                for (int i = 0; i < selected_chain.nodes_seq.Count - 1; i++)
                {
                    arc_bin.Add(selected_chain.nodes_seq[i].id + "," + selected_chain.nodes_seq[i + 1].id);
                }

                Chain.all_chains.Remove(selected_chain);
                feasible_insertions.Clear();
                feasible_chains.Clear();
            }

            // use this if you only want to get the list of chains
            // return route_bin;

            // Phase 2: Turn the bin into a route by opening the chains and solving the TSP
            List<Node> node_seq = new List<Node>() { m.depot };
            foreach (Chain chain in route_bin)
            {
                foreach (Node node in chain.nodes_seq)
                {
                    node_seq.Add(node);
                }
            }
            node_seq.Add(m.depot);
            Solution sol = new Solution(new Route(node_seq, m));
            MathProgramming.SolveTSP(m, sol, hide_errors: true);

            // Phase 3: remove duplicate sets to construct a feasible solution
            int pos = node_seq.Count - 2;  // start counting from the last node before the depot to find duplicates
            node_seq = sol.route.nodes_seq;
            HashSet<int> sets_found = new HashSet<int>();
            int cost_to_remove;
            int cost_to_add;
            while (pos > 0)
            {
                if (sets_found.Contains(node_seq[pos].set_id))
                {
                    cost_to_remove = m.dist_matrix[node_seq[pos - 1].id, node_seq[pos].id] + m.dist_matrix[node_seq[pos].id, node_seq[pos + 1].id];
                    cost_to_add = m.dist_matrix[node_seq[pos - 1].id, node_seq[pos + 1].id];
                    sol.route.time += cost_to_add - cost_to_remove;
                    sol.total_time += cost_to_add - cost_to_remove;
                    sol.route.total_profit -= node_seq[pos].profit;
                    sol.total_profit -= node_seq[pos].profit;
                    node_seq.RemoveAt(pos);
                    sol.route.sets_included.RemoveAt(pos);
                }
                sets_found.Add(node_seq[pos].set_id);
                pos--;
            }

            MathProgramming.SolveShortestPath(m, sol, hide_errors: true);  // improve the solution
            if (sol.total_time > sol_infeasibility_upper_bound)  // if the infeasibility problem hasn't still been solved, use this method to restore infeasibility
            {
                MathProgramming.SolveSimulInsDelSubproblem(m, sol, 0, 1, hide_tmax_constraint: true,  // remove the tmax constraint so that the model doesn't crash
                    custom_tmax: sol_infeasibility_upper_bound);  
            }

            sol.CheckSol(m);
            return sol;
        }

        public static Solution SolveAsKnapsack_Feasible(Model m)
        {
            int rcl_limit = 0;
            Random r = m.r;
            int tolerance_level = 0;  // declare the number of times the algorithm will continue when we're out of feasible moves
            int i = 0;

            Solution sol = new Solution(new Route(new List<Node>() { m.depot, m.depot }, m));
            List<Chain_Constructive_Move> feasible_insertions = new List<Chain_Constructive_Move>();
            List<Node> node_seq = sol.route.nodes_seq;
            int found_at, cost_added, cost_removed, cost_difference = 0, insert_at = 0, profit_already_in_sol = 0;
            List<int> same_set_occurences = new List<int>();  // the positions in sol where common sets with the chain were found

            while (true)
            {
                // find all feasible insertions for each position
                foreach (Chain chain in Chain.all_chains)
                {
                    // check if any of the chain's sets has already been added to the bin
                    foreach (Set set in chain.set_seq)
                    {
                        found_at = sol.route.sets_included.IndexOf(set);
                        if (sol.route.sets_included.LastIndexOf(set) != found_at && chain.set_seq.Last().id == 0)
                        { 
                            // the set is located 2 times in solution, hence it's the depot
                            // if the chain contains the end depot and not the start one, we need to make the found_at value
                            // point to the last position of the solution
                            found_at = node_seq.Count - 1;
                        }
                        if (found_at != -1)
                        {
                            same_set_occurences.Add(found_at);
                            profit_already_in_sol += set.profit;
                        }
                    }

                    if (same_set_occurences.Count == 0)  // no common sets exist, so the chain can be added as a whole in the solution
                    {
                        cost_difference = Int32.MaxValue;  // find the best position to insert the chain
                        for (int pos = 1; pos < node_seq.Count; pos++)
                        {
                            cost_added = m.dist_matrix[node_seq[pos - 1].id, chain.nodes_seq[0].id] + chain.time +
                                m.dist_matrix[chain.nodes_seq.Last().id, node_seq[pos].id];
                            cost_removed = m.dist_matrix[node_seq[pos - 1].id, node_seq[pos].id];
                            if (cost_added - cost_removed < cost_difference)
                            {
                                cost_difference = cost_added - cost_removed;
                                insert_at = pos;
                            }
                        }
                    }
                    else
                    {
                        // first we need to check whether the chain's insertion to the solution is feasible, i.e. it doesn't violate the current solution's structure
                        // the common sets must be in the same order as they appear in the solution
                        bool insertion_is_valid = true;
                        int prev_common_set_pos = same_set_occurences[0];
                        for (int index = 1; index < same_set_occurences.Count; index++) 
                        {
                            if (same_set_occurences[index] != prev_common_set_pos + 1) 
                            {
                                insertion_is_valid = false;
                                break;
                            }
                            prev_common_set_pos += 1;
                        }
                        if (!insertion_is_valid)
                        {
                            same_set_occurences.Clear();
                            profit_already_in_sol = 0;
                            continue;
                        }

                        // if the chain is valid, calculate its impact to the solution
                        int first_common_set_pos = same_set_occurences[0];
                        int last_common_set_pos = same_set_occurences.Last();
                        cost_added = chain.time;
                        if (first_common_set_pos > 0)  // if the insert position has previous nodes, add its connection with the chain to the cost
                        {
                            cost_added += m.dist_matrix[node_seq[first_common_set_pos - 1].id, chain.nodes_seq[0].id];
                        }
                        if (last_common_set_pos < node_seq.Count - 1)  // if the last common set has successor nodes, add its connection with the chain to the cost
                        {
                            cost_added += m.dist_matrix[chain.nodes_seq.Last().id, node_seq[last_common_set_pos + 1].id];
                        }

                        // remove from the cost all the common sets connections
                        cost_removed = 0;
                        if (same_set_occurences[0] > 0)
                        {
                            cost_removed += m.dist_matrix[node_seq[same_set_occurences[0] - 1].id, node_seq[same_set_occurences[0]].id];
                        }
                        foreach (int pos in same_set_occurences)
                        {
                            if (pos < node_seq.Count - 1) {
                                Node pred = node_seq[pos];
                                Node succ = node_seq[pos + 1];
                                cost_removed += m.dist_matrix[pred.id, succ.id];
                            }
                        }
                        cost_difference = cost_added - cost_removed;
                        insert_at = first_common_set_pos;
                    }

                    if (sol.route.time + cost_difference <= m.t_max && chain.profit - profit_already_in_sol > 0)
                    {
                        //Console.WriteLine(chain + "" + insert_at);
                        feasible_insertions.Add(new Chain_Constructive_Move(chain, same_set_occurences, insert_at, cost_difference, profit_already_in_sol));
                    }
                    same_set_occurences.Clear();
                    profit_already_in_sol = 0;
                }

                if (feasible_insertions.Count == 0) // there aren't any feasible moves
                {
                    i++;
                    MathProgramming.SolveShortestPath(m, sol);
                    MathProgramming.SolveTSP(m, sol);
                    if (i > tolerance_level) { break; }
                    continue;
                }

                // sort feasible insertions
                var sorted_insertions = feasible_insertions.OrderByDescending(x => Math.Pow(x.profit_added, 1) / x.n_sets_added);  // sort by unit value of every chain

                // apply multi-restart method
                rcl_limit = Math.Min(sorted_insertions.Count(), RCL_SIZE);
                Chain_Constructive_Move selected_move = (Chain_Constructive_Move)sorted_insertions.ElementAt(r.Next(rcl_limit));

                //Console.WriteLine(selected_move.chain + "" + selected_move.pos + " " + selected_move.profit_added) ;

                // first remove all the duplicate sets from the solution, if necessary
                foreach (int pos in selected_move.duplicate_nodes_pos_reversed)
                {
                    Node node_to_remove = sol.route.nodes_seq[pos];
                    sol.route.nodes_seq.RemoveAt(pos);
                    sol.route.sets_included.RemoveAt(pos);
                }
                // add selected chain to the solution
                foreach (Node node in selected_move.nodes_seq_reversed)
                {
                    sol.route.nodes_seq.Insert(selected_move.pos, node);
                    sol.route.sets_included.Insert(selected_move.pos, m.sets[node.set_id]);
                }
                sol.route.time += selected_move.cost_added;
                sol.route.total_profit += selected_move.profit_added;
                sol.total_time += selected_move.cost_added;
                sol.total_profit += selected_move.profit_added;

                Chain.all_chains.Remove(selected_move.chain);
                feasible_insertions.Clear();

                if (!sol.CheckSol(m)) { break; }
            }

            return sol;
        }

        public static Solution Get_Solution_From_Chains(Model m)
        {
            List<Chain> chain_seq = Chain_Minimum_Insertions(m);
            List<Node> node_seq = Create_Node_Seq(m, chain_seq);
            return Create_Chain_Based_Solution(m, node_seq);
        }
        public static List<Chain> Chain_Minimum_Insertions(Model m)
        {
            int seq_time = 0;
            List<Chain> chain_seq = new List<Chain>();
            chain_seq.Add(Chain.all_chains[0]); // Chain.all_chains[0] contains only the depot
            chain_seq.Add(Chain.all_chains[0]);

            //m.sets[0].in_route = true;
            int rcl_limit = 0;  // specifies the exact number of available moves to choose from
            Random r = m.r;

            while (true)
            {
                List<Chain_Constructive_Move2> feasible_insertions = new List<Chain_Constructive_Move2>();
                // find all feasible insertions for each position
                foreach (Chain chain in Chain.all_chains)
                {
                    if (chain_seq.Contains(chain))
                    {
                        continue;
                    }
                    for (int i = 1; i < chain_seq.Count; i++)
                    {
                        int cost_added = Chain.chain_dist_matrix[chain_seq[i - 1].id, chain.id] + Chain.chain_dist_matrix[chain.id, chain_seq[i].id] - Chain.chain_dist_matrix[chain_seq[i - 1].id, chain_seq[i].id];
                        //Console.WriteLine("{0} {1}", cost_added, m.t_max);
                        if (seq_time + cost_added <= 1.5 * m.t_max)  // chain addition results in feasible solution 
                        {
                            double profit_to_cost_rate = cost_added > 0 ? chain.profit / cost_added : Math.Pow(10, 10);
                            feasible_insertions.Add(new Chain_Constructive_Move2(chain, i, cost_added, profit_to_cost_rate));
                        }
                    }
                }

                if (feasible_insertions.Count == 0) // there aren't any feasible moves
                {
                    break;
                }

                // sort feasible insertions
                var sorted_insertions = feasible_insertions.OrderByDescending(x => x.profit_to_cost_rate);

                // apply multi-restart method
                rcl_limit = Math.Min(sorted_insertions.Count(), RCL_SIZE);
                Chain_Constructive_Move2 selected_move = (Chain_Constructive_Move2)sorted_insertions.ElementAt(r.Next(rcl_limit));

                // find the chain
                Chain selected_chain = selected_move.chain;
                //m.sets[selected_node.set_id].in_route = true;

                // add selected node to chain_sequence
                chain_seq.Insert(selected_move.pos, selected_chain);
                seq_time += selected_move.cost_added;
            }
            return chain_seq;
        }

        public static List<Node> Create_Node_Seq(Model m, List<Chain> chain_seq) // keeps only one node from each set and turns the chain_seq to node_seq ( the sequence doesn't really matter as TSP wil be used after)
        {
            Dictionary<int, List<Node>> nodes_of_every_set = new Dictionary<int, List<Node>>();
            Random r = new Random();

            foreach (Chain chain in chain_seq)
            {
                foreach (Node node in chain.nodes_seq)
                {
                    if (node.Equals(m.depot))
                    {
                        continue;
                    }
                    if (!nodes_of_every_set.ContainsKey(node.set_id))
                    {
                        nodes_of_every_set.Add(node.set_id, new List<Node>());
                    }
                    nodes_of_every_set[node.set_id].Add(node);
                }
            }

            List<Node> node_seq = new List<Node>();
            node_seq.Add(m.depot);
            foreach (int set_id in nodes_of_every_set.Keys)
            {
                int rand_index = r.Next(0, nodes_of_every_set[set_id].Count);
                node_seq.Add(nodes_of_every_set[set_id][rand_index]);
            }
            node_seq.Add(m.depot);

            return node_seq;
        }

        public static Solution Create_Chain_Based_Solution(Model m, List<Node> nodes_seq) // uses as argument the nodes_seq produced by Create_Node_Seq method
        {
            Route chain_based_route = new Route(nodes_seq, m);
            Solution chain_based_solution = new Solution(chain_based_route);
            MathProgramming.SolveTSP(m, chain_based_solution);
            //MathProgramming.SolveShortestPath(m, chain_based_solution);

            while (chain_based_solution.total_time > m.t_max)
            {
                Delete_Set del_move = new Delete_Set();
                del_move.FindBestMove(m, chain_based_solution);
                del_move.ApplyBestMove(chain_based_solution);
            }

            MathProgramming.SolveTSP(m, chain_based_solution);
            MathProgramming.SolveShortestPath(m, chain_based_solution);
            chain_based_solution.CheckSol(m);
            return chain_based_solution;
        }
    }
}
