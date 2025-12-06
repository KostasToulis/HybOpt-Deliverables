using System;
using System.Collections.Generic;
using System.Text;

namespace SOP_Project
{
    class TwoOpt : Move // two opt move, refers only to the nodes that are currently in the solution, like a TSP Two Opt
    {
        public int node_first_position;
        public int set_first_position;
        public int node_second_position;
        public int set_second_position;
        public int cost_changed;
        public bool is_move_found;

       //---NOTICE--- instead of saving the second position it could be saved the number of nodes/sets to be reversed, as is is the argument
       // of the Reverse() method

        public TwoOpt()
        {
            this.node_first_position = -1;
            this.set_first_position = -1;
            this.node_second_position = -1;
            this.set_second_position = -1;
            this.cost_changed = 100000;
            this.is_move_found = false;
        }

        public TwoOpt(int node1_pos, int node2_pos, int set1_pos, int set2_pos, int cost, bool is_found)
        {
            this.node_first_position = node1_pos;
            this.set_first_position = set1_pos;
            this.node_second_position = node2_pos;
            this.set_second_position = set2_pos;
            this.cost_changed = cost;
            this.is_move_found = is_found;
        }

        public void InitializeFields()
        {
            this.node_first_position = -1;
            this.set_first_position = -1;
            this.node_second_position = -1;
            this.set_second_position = -1;
            this.cost_changed = 100000;
            this.is_move_found = false;
        }

        override
        public Move ShallowCopy()
        {
            return new TwoOpt(this.node_first_position, this.node_second_position,
                this.set_first_position, this.set_second_position, this.cost_changed, is_move_found);
        }

        override
        public void FindBestMove(Model m, Solution sol) // find-method based on professor's Zachariadis FindBestTwoOptMove TSP python method
        {
            this.InitializeFields();
            List<Node> nodes_in_route = sol.route.nodes_seq;
            int cost_added, cost_removed, t_o_cost; //t_o_cost refers to the currently checking two opt cost
            Node a, b, k, l;

            for (int first_index = 0; first_index < nodes_in_route.Count - 1; first_index++)
            {
                a = nodes_in_route[first_index];
                b = nodes_in_route[first_index + 1];

                for (int second_index = first_index + 2; second_index < nodes_in_route.Count - 1; second_index++)
                {
                    k = nodes_in_route[second_index];
                    l = nodes_in_route[second_index + 1];

                    if (first_index == 0 && second_index == nodes_in_route.Count - 2)
                    {
                        continue;
                    }

                    cost_added = m.dist_matrix[a.id, k.id] + m.dist_matrix[b.id, l.id];
                    cost_removed = m.dist_matrix[a.id, b.id] + m.dist_matrix[k.id, l.id];
                    t_o_cost = cost_added - cost_removed;

                    if (t_o_cost < this.cost_changed && sol.route.time + t_o_cost <= m.t_max)
                    {
                        // the interval [node_first_position, node_second_position] defines the part of the sol.nodes_seq that must be reversed
                        // respectively the interval [set_first_position, set_second_position] defines the part of sol.sets_included that must be reversed
                        this.node_first_position = first_index + 1;
                        this.set_first_position = first_index + 1;
                        this.node_second_position = second_index;
                        this.set_second_position = second_index;
                        this.cost_changed = t_o_cost;
                        this.is_move_found = true;
                    }
                }
            }

        }

        override

        public void ApplyBestMove(Solution sol) // apply-method based on professor's Zachariadis  ApplyTwoOptMove TSP python method
                                                // a two opt must have been found in order to apply it (this.node_first_position != -1 to check)
        {
            //Console.WriteLine("Applying Two_Opt operator...\n");
            int numb_of_nodes_to_reverse = this.node_second_position + 1 - this.node_first_position;
            int numb_of_sets_to_reverse = this.set_second_position + 1 - this.set_first_position;
            // reverses the nodes in the range [node_first_position, node_second_position] (third arg of Reverse is the number of elements to be reversed)
            sol.route.nodes_seq.Reverse(this.node_first_position, numb_of_nodes_to_reverse);
            // reverses the sets in the range [set_first_position, set_second_position] (third arg of Reverse is the number of elements to be reversed)
            sol.route.sets_included.Reverse(this.set_first_position, numb_of_sets_to_reverse);
            sol.route.time += this.cost_changed;
            sol.total_time += this.cost_changed;

        }

        override
        public bool IsMoveFound()
        {
            return this.is_move_found;
        }

    }
}
