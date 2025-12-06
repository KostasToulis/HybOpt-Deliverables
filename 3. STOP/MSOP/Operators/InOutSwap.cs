using MSOP.Fundamentals;
using MSOP.Heuristics;
using System.Collections.Generic;

namespace MSOP.Operators
{
    class InOutSwap : Move // swaps a node of a set that is not currently in the solution with a node from the solution
    {
        public int node_adding_position;
        public int set_adding_position;
        public Route swapping_route;
        public int cost;
        public int profit;
        //public double profit_to_cost_rate; // we need to be cautious in case of positive profit and negative cost
        public Node node_to_add;
        public Node node_to_remove;
        public Set set_to_add;
        public Set set_to_remove;
        public bool is_move_found;
        const int BigNumber = 10000; // used in case of positive profit and negative cost (it gets added to profit_to_cost_rate)
                                     // it may be necessary to change it in case of a dataset with very large numbers
        public int critirion; // it is M*profit_changed - cost_changed


        public InOutSwap()
        {
            this.node_adding_position = -1;
            this.set_adding_position = -1;
            this.swapping_route = null;
            this.cost = -1;
            this.profit = -1;
            //this.profit_to_cost_rate = 0;
            this.node_to_add = null;
            this.set_to_add = null;
            this.is_move_found = false;
            this.critirion = -100000;
        }

        public InOutSwap(int node_pos, int set_pos, Route route, int cost, int profit, Node node_added, Node node_removed, Set set_added, Set set_removed, bool is_found)
        {
            this.node_adding_position = node_pos;
            this.set_adding_position = set_pos;
            this.swapping_route = route;
            this.cost = cost;
            this.profit = profit;
            //this.profit_to_cost_rate = cost != 0 ? profit / cost : BigNumber; // if cost is 0 then the swap's profit_to_rate cost is set to a very big number 
            this.node_to_add = node_added;
            this.node_to_remove = node_removed;
            this.set_to_add = set_added;
            this.set_to_remove = set_removed;
            this.is_move_found = is_found;
            this.critirion = BigNumber * this.profit - this.cost;
        }

        public void InitializeFields()
        {
            this.node_adding_position = -1;
            this.set_adding_position = -1;
            this.swapping_route = null;
            this.cost = -1;
            this.profit = -1;
            //this.profit_to_cost_rate = 0;
            this.node_to_add = null;
            this.node_to_remove = null;
            this.set_to_add = null;
            this.set_to_remove = null;
            this.is_move_found = false;
            this.critirion = -100000;
        }

        override
        public Move DeepCopy()
        {
            return new InOutSwap(this.node_adding_position, this.set_adding_position, this.swapping_route, this.cost,
                   this.profit, this.node_to_add, this.node_to_remove, this.set_to_add, this.set_to_remove, is_move_found);
        }

        override
        public void FindBestMove(Solution sol) // finds the best available swap of a node that its set is not in the solution,
                                               // with a node that is in the solution, it uses profit to cost criterion
                                               // - if we want to check if a swap that increases solution's profit was found,
                                               //   check if this.node_adding_position != -1

        {
            Model m = Model.model;
            this.InitializeFields();
            List<Set> all_sets = m.sets;
            HashSet<Set> sets_in_sol = sol.sets_included;
            int adding_cost, removing_cost, adding_profit, swap_cost, swap_profit;
            //double swap_profit_to_cost_rate, scaled_swap_cost, scaled_swap_profit;
            Node pred, current, succ;
            Set inner_set;
            int move_critirion;

            foreach (Route route in sol.routes)
            {
                List<Node> nodes_in_route = route.nodes_seq;
                foreach (Set outer_set in all_sets) // using name outer set because if the set is already in the solution then we continue to the next one
                {
                    // if the set is already in the sol's route continue to the next set
                    if (sets_in_sol.Contains(outer_set))
                    {
                        continue;
                    }
                    // if the set is not in the sol's route check the insertion of each of its nodes
                    adding_profit = outer_set.profit;
                    foreach (Node outer_node in outer_set.nodes)
                    {
                        for (int i = 1; i < nodes_in_route.Count - 1; i++) // for i starting from the second node's position until the next-to-last one's
                        {
                            // checking the swap of the outer_node with the current node
                            pred = nodes_in_route[i - 1];
                            current = nodes_in_route[i];
                            succ = nodes_in_route[i + 1];
                            inner_set = route.sets_included[i];

                            if (!Promises.MoveIsAdmissible(adding_profit - current.profit, sol.total_profit, outer_set))
                            {
                                continue;
                            }

                            adding_cost = m.dist_matrix[pred.id, outer_node.id] + m.dist_matrix[outer_node.id, succ.id];
                            removing_cost = m.dist_matrix[pred.id, current.id] + m.dist_matrix[current.id, succ.id];
                            swap_cost = adding_cost - removing_cost;
                            swap_profit = adding_profit - current.profit;

                            move_critirion = BigNumber * swap_profit - swap_cost;

                            // find the best possible move that is not part of the tabu list
                            if (move_critirion > this.critirion && route.time + swap_cost <= m.t_max)
                            {
                                this.node_adding_position = i;
                                this.set_adding_position = i;
                                this.swapping_route = route;
                                this.cost = swap_cost;
                                this.profit = swap_profit;
                                //this.profit_to_cost_rate = swap_profit_to_cost_rate;
                                this.node_to_add = outer_node;
                                this.node_to_remove = current;
                                this.set_to_add = outer_set;
                                this.set_to_remove = inner_set;
                                this.is_move_found = true;
                                this.critirion = move_critirion;
                            }
                        }
                    }
                }
            }
        }

        override
        public void ApplyBestMove(Solution sol) // a swap must have been found in order to apply it (this.node_adding_position != -1)
        {
            //Console.WriteLine("inserted_id: {0} - ({1}) removed_id: {2} - ({3}) swap_profit: {4} swap_cost: {5} rate {6}",

            Promises.MakePromise(this.swapping_route.sets_included[this.set_adding_position], sol.total_profit);

            this.swapping_route.nodes_seq[this.node_adding_position] = this.node_to_add;
            this.swapping_route.sets_included[this.set_adding_position] = set_to_add;
            this.swapping_route.time += this.cost;
            this.swapping_route.total_profit += this.profit;
            sol.total_profit += this.profit;
            sol.sets_included.Add(set_to_add);
            sol.sets_included.Remove(set_to_remove);
        }

        override
        public bool IsMoveFound()
        {
            return this.is_move_found;
        }

    }
}
