using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SOP_Project
{
    class InOutSwap : Move // swaps a node of a set that is not currently in the solution with a node from the solution
    {
        public int node_adding_position;
        public int set_adding_position;
        public int cost;
        public int profit;
        //public double profit_to_cost_rate; // we need to be cautious in case of positive profit and negative cost
        public Node node_to_add;
        public Node node_to_remove;
        public Set set_to_add;
        public bool is_move_found;
        const int BigNumber = 10000; // used in case of positive profit and negative cost (it gets added to profit_to_cost_rate)
                                       // it may be necessary to change it in case of a dataset with very large numbers
        public int critirion; // it is M*profit_changed - cost_changed


        public InOutSwap()
        {
            this.node_adding_position = -1;
            this.set_adding_position = -1;
            this.cost = -1;
            this.profit = -1;
            //this.profit_to_cost_rate = 0;
            this.node_to_add = null;
            this.set_to_add = null;
            this.is_move_found = false;
            this.critirion = -100000;
        }

        public InOutSwap(int node_pos, int set_pos, int cost, int profit, Node node_added, Node node_removed, Set set, bool is_found)
        {
            this.node_adding_position = node_pos;
            this.set_adding_position = set_pos;
            this.cost = cost;
            this.profit = profit;
            //this.profit_to_cost_rate = cost != 0 ? profit / cost : BigNumber; // if cost is 0 then the swap's profit_to_rate cost is set to a very big number 
            this.node_to_add = node_added;
            this.node_to_remove = node_removed;
            this.set_to_add = set;
            this.is_move_found = is_found;
            this.critirion = BigNumber * this.profit - this.cost;
        }

        public void InitializeFields()
        {
            this.node_adding_position = -1;
            this.set_adding_position = -1;
            this.cost = -1;
            this.profit = -1;
            //this.profit_to_cost_rate = 0;
            this.node_to_add = null;
            this.set_to_add = null;
            this.is_move_found = false;
            this.critirion = -100000;
        }

        override
        public Move ShallowCopy()
        {
            return new InOutSwap(this.node_adding_position, this.set_adding_position, this.cost,
                   this.profit, this.node_to_add, this.node_to_remove, this.set_to_add, is_move_found);
        }

        override
        public void FindBestMove(Model m, Solution sol) // finds the best available swap of a node that its set is not in the solution,
                                                             // with a node that is in the solution, it uses profit to cost criterion
                                                             // - if we want to check if a swap that increases solution's profit was found,
                                                             //   check if this.node_adding_position != -1

        {
            this.InitializeFields();
            List<Set> all_sets = m.sets;
            List<Node> nodes_in_route = sol.route.nodes_seq;
            List<Set> sets_in_route = sol.route.sets_included;
            int adding_cost, removing_cost, adding_profit, swap_cost, swap_profit;
            //double swap_profit_to_cost_rate, scaled_swap_cost, scaled_swap_profit;
            Node pred, current, succ;
            int move_critirion;

            List<InOutSwap> moves = new List<InOutSwap>();

            foreach (Set outer_set in all_sets) // using name outer set because if the set is already in the solution then we continue to the next one
            {
                // if the set is already in the sol's route continue to the next set
                if (sets_in_route.Contains(outer_set))
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

                        if (!Promises.MoveIsAdmissible(adding_profit - current.profit, sol.total_profit, outer_set))
                        //if (!Arc_Promises.MoveIsAdmissible(adding_profit - current.profit, sol.total_profit, outer_node, succ) &&
                        //    !Arc_Promises.MoveIsAdmissible(adding_profit - current.profit, sol.total_profit, pred, outer_node))
                        {
                            continue;
                        }

                        adding_cost = m.dist_matrix[pred.id, outer_node.id] + m.dist_matrix[outer_node.id, succ.id];
                        removing_cost = m.dist_matrix[pred.id, current.id] + m.dist_matrix[current.id, succ.id];
                        swap_cost = adding_cost - removing_cost;
                        swap_profit = adding_profit - current.profit;

                        // apply scaling method in order to turn swap_cost and swap_profit values into positive ones
                        //scaled_swap_cost = (swap_cost - m.min_dist_difference) / (m.max_dist_difference - m.min_dist_difference);
                        //scaled_swap_profit = (swap_profit - m.min_profit_difference) / (m.max_profit_difference - m.min_profit_difference);
                        //swap_profit_to_cost_rate = Math.Pow(scaled_swap_profit, 1) / scaled_swap_cost;
                        move_critirion = BigNumber * swap_profit - swap_cost;

                        // find the best possible move that is not part of the tabu list
                        //if (swap_profit_to_cost_rate > this.profit_to_cost_rate && sol.route.time + swap_cost <= m.t_max) // && !Local_Search.DetectCycling(new InOutSwap(i, i, swap_cost, swap_profit, outer_node, current, outer_set, true)))
                        if (move_critirion > this.critirion && sol.route.time + swap_cost <= m.t_max)
                        {
                            this.node_adding_position = i;
                            this.set_adding_position = i;
                            this.cost = swap_cost;
                            this.profit = swap_profit;
                            //this.profit_to_cost_rate = swap_profit_to_cost_rate;
                            this.node_to_add = outer_node;
                            this.node_to_remove = current;
                            this.set_to_add = outer_set;
                            this.is_move_found = true;
                            this.critirion = move_critirion;
                        }
                    }
                }
            }
        }

        override
        public void ApplyBestMove(Solution sol) // a swap must have been found in order to apply it (this.node_adding_position != -1)
        {
            //Console.WriteLine("inserted_id: {0} - ({1}) removed_id: {2} - ({3}) swap_profit: {4} swap_cost: {5} rate {6}",
            //    this.node_to_add.id, this.set_to_add.id, this.node_to_remove.id, sol.route.sets_included[this.set_adding_position],
            //     this.profit, this.cost, this.profit_to_cost_rate);

            Promises.MakePromise(sol.route.sets_included[this.set_adding_position], sol.total_profit);
            //Arc_Promises.MakePromise(sol.route.nodes_seq[this.set_adding_position], sol.route.nodes_seq[this.set_adding_position + 1], sol.total_profit);
            //Arc_Promises.MakePromise(sol.route.nodes_seq[this.set_adding_position - 1], sol.route.nodes_seq[this.set_adding_position], sol.total_profit);

            sol.route.nodes_seq[this.node_adding_position] = this.node_to_add;
            //this.set_to_add.in_route = true;
            //sol.route.sets_included[this.set_adding_position].in_route = false;
            sol.route.sets_included[this.set_adding_position] = set_to_add;
            sol.route.time += this.cost;
            sol.route.total_profit += this.profit;
            sol.total_profit += this.profit;
            sol.total_time += this.cost;

        }

        override
        public bool IsMoveFound()
        {
            return this.is_move_found;
        }

    }
}
