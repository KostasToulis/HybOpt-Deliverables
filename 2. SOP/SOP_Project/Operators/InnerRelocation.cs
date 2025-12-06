using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SOP_Project
{
    class InnerRelocation : Move // refers to a relocation of a set that already exist in the route to another position in the route
                          // ===IMPORTANT=== 
                          // the node that will be removed may not be the same with the one inserted, the best one from the relocated set is chosen

    {
        public int node_removing_position;
        public int set_removing_position;
        public int node_insertion_position;
        public int set_insertion_position;
        public int cost_changed;
        public Node node_to_insert;
        public Node node_to_remove;
        public bool is_move_found;

        public InnerRelocation()
        {
            this.node_removing_position = -1;
            this.set_removing_position = -1;
            this.node_insertion_position = -1;
            this.set_insertion_position = -1;
            this.cost_changed = 100000;
            this.node_to_insert = null;
            this.node_to_remove = null;
            this.is_move_found = false;
        }

        public InnerRelocation(int node_rem_pos, int set_rem_pos, int node_ins_pos, int set_ins_pos, int cost, Node node_to_insert, Node node_to_remove, bool is_found)
        {
            this.node_removing_position = node_rem_pos;
            this.set_removing_position = set_rem_pos;
            this.node_insertion_position = node_ins_pos;
            this.set_insertion_position = set_ins_pos;
            this.cost_changed = cost;
            this.node_to_insert = node_to_insert;
            this.node_to_remove = node_to_remove;
            this.is_move_found = is_found;
        }

        public void InitializeFields()
        {
            this.node_removing_position = -1;
            this.set_removing_position = -1;
            this.node_insertion_position = -1;
            this.set_insertion_position = -1;
            this.cost_changed = 100000;
            this.node_to_insert = null;
            this.node_to_remove = null;
            this.is_move_found = false;
        }

        override
        public Move ShallowCopy()
        {
            return new InnerRelocation(this.node_removing_position, this.set_removing_position, this.node_insertion_position,
                   this.set_insertion_position, this.cost_changed, this.node_to_insert, this.node_to_remove, this.is_move_found);
        }

        override
        public void FindBestMove(Model m, Solution sol) // finds the best available rellocation of the sets that there are currently in the solution,
                                                                  // the best node of the relocated set is selected, not the one removed necessarily
                                                                  // ===CAUTION===
                                                                  // a relocation will be returned only if a relocation that decreases
                                                                  // the cost of the route is found
                                                                  // ---if we want to check if an insertion was found,
                                                                  //   check if this.node_removing_position != -1

        {
            this.InitializeFields();
            List<Set> sets_in_route = sol.route.sets_included; 
            List<Node> nodes_in_route = sol.route.nodes_seq;
            Set checking_set;
            int rel_cost, cost_of_insertion, cost_of_removing; // the cost of the relocation that is checked
            Node pred, succ; // the predecessor and successor in case the relocation happens

            for (int rem_pos = 1; rem_pos < sets_in_route.Count - 1; rem_pos++) // from the second position till the next-to-last, as the first and last nodes are the depot
            {
                checking_set = sets_in_route[rem_pos];
                cost_of_removing = m.dist_matrix[nodes_in_route[rem_pos - 1].id, nodes_in_route[rem_pos + 1].id] -
                    m.dist_matrix[nodes_in_route[rem_pos - 1].id, nodes_in_route[rem_pos].id] -
                    m.dist_matrix[nodes_in_route[rem_pos].id, nodes_in_route[rem_pos + 1].id]; // how much the cost will be changed because 
                                                                                               // of the removing of the relocating node
                foreach (Node checking_node in checking_set.nodes)
                {
                    for (int ins_pos = 1; ins_pos < nodes_in_route.Count; ins_pos++) // possible inserting positions, all the positions except the beginning
                                                                                   // and the end of the route
                    {
                        if (rem_pos == ins_pos || rem_pos == ins_pos - 1)
                        {
                            continue;
                        }
                        pred = nodes_in_route[ins_pos - 1];
                        succ = nodes_in_route[ins_pos];
                        cost_of_insertion = m.dist_matrix[pred.id, checking_node.id] + 
                            m.dist_matrix[checking_node.id, succ.id] -
                            m.dist_matrix[pred.id, succ.id];  // how much the cost will be changed because of the insertion of the relocating node 
                        rel_cost = cost_of_removing + cost_of_insertion;

                        if (rel_cost < this.cost_changed && sol.total_time + rel_cost <= m.t_max)
                        {
                            // find the best possible move that is not part of the tabu list
                            // if removing position is before insertion position, the removing does affects the correct insertion position
                            // and thus we must compare the tabu_move with the proper relocation move 
                            //int proper_ins_pos = ins_pos;
                            //if (rem_pos < ins_pos)
                            //{
                            //    proper_ins_pos = ins_pos - 1;
                            //}
                            //if (!Local_Search.DetectCycling(new InnerRelocation(rem_pos, rem_pos, proper_ins_pos, proper_ins_pos,
                            //    rel_cost, checking_node, nodes_in_route[rem_pos], true)))
                            //{
                            this.node_removing_position = rem_pos;
                            this.set_removing_position = rem_pos;
                            this.node_insertion_position = ins_pos;
                            this.set_insertion_position = ins_pos;
                            this.cost_changed = rel_cost;
                            this.node_to_insert = checking_node;
                            this.node_to_remove = nodes_in_route[rem_pos];
                            is_move_found = true;
                            // }
                        }
                    }
                }
            }
        }

        override
        public void ApplyBestMove(Solution sol) // a relocation must have been found in order to apply it (this.is_move_found to check)
        {
            sol.route.nodes_seq.RemoveAt(this.node_removing_position);
            Set relocated_set = sol.route.sets_included[this.set_removing_position];
            sol.route.sets_included.RemoveAt(this.set_removing_position);
            // since the removing is made, if the removing position is before the insertion position, then there is a node less in the route, than it was when
            // insertion position was calculated
            // (if removing position is after insertion position, the removing does not affect the correct insertion position)
            if (this.node_removing_position < this.node_insertion_position)
            {
                this.node_insertion_position--;
                this.set_insertion_position--;
            }
            sol.route.nodes_seq.Insert(this.node_insertion_position, this.node_to_insert);
            sol.route.sets_included.Insert(this.set_insertion_position, relocated_set);
            sol.route.time += this.cost_changed;
            sol.total_time += this.cost_changed;

            //Console.WriteLine("Insert: {0} remove: {1} ins_position: {2} rem_position: {3}", this.node_to_insert.id, this.node_to_remove.id, this.node_insertion_position, this.node_removing_position);
        }

        override
        public bool IsMoveFound()
        {
            return this.is_move_found;
        }
    }
}
