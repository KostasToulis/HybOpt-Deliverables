using MSOP.Fundamentals;
using System.Collections.Generic;

namespace MSOP.Operators
{
    class InnerRelocation : Move // refers to a relocation of a set that already exist in the route to another position in the route
                                 // ===IMPORTANT=== 
                                 // the node that will be removed may not be the same with the one inserted, the best one from the relocated set is chosen

    {
        public int node_removing_position;
        public int set_removing_position;
        public Route removing_route;
        public int node_insertion_position;
        public int set_insertion_position;
        public Route inserting_route;
        public int rem_cost_changed; // the cost change for the removing route
        public int ins_cost_changed; // the cost change for the inserting route
        public int rel_cost_change; // the cost change caused in the total solution because of the relocation 
        public int rem_profit_changed; // the profit change for the removing route
        public int ins_profit_changed; // the profit change for the inserting route
        public Node node_to_insert;
        public Node node_to_remove;
        public bool is_move_found;

        public InnerRelocation()
        {
            this.node_removing_position = -1;
            this.set_removing_position = -1;
            this.removing_route = null;
            this.node_insertion_position = -1;
            this.set_insertion_position = -1;
            this.inserting_route = null;
            this.rem_cost_changed = 100000;
            this.ins_cost_changed = 100000;
            this.rel_cost_change = 100000;
            this.rem_profit_changed = -100000;
            this.ins_profit_changed = -100000;
            this.node_to_insert = null;
            this.node_to_remove = null;
            this.is_move_found = false;
        }

        public InnerRelocation(int node_rem_pos, int set_rem_pos, Route rem_route, int node_ins_pos, int set_ins_pos, Route ins_route, int rem_cost,
            int ins_cost, int rel_cost, int rem_profit, int ins_profit, Node node_to_insert, Node node_to_remove, bool is_found)
        {
            this.node_removing_position = node_rem_pos;
            this.set_removing_position = set_rem_pos;
            this.removing_route = rem_route;
            this.node_insertion_position = node_ins_pos;
            this.set_insertion_position = set_ins_pos;
            this.inserting_route = ins_route;
            this.rem_cost_changed = rem_cost;
            this.ins_cost_changed = ins_cost;
            this.rel_cost_change = rel_cost;
            this.rem_profit_changed = rem_profit;
            this.ins_profit_changed = ins_profit;
            this.node_to_insert = node_to_insert;
            this.node_to_remove = node_to_remove;
            this.is_move_found = is_found;
        }

        public void InitializeFields()
        {
            this.node_removing_position = -1;
            this.set_removing_position = -1;
            this.removing_route = null;
            this.node_insertion_position = -1;
            this.set_insertion_position = -1;
            this.inserting_route = null;
            this.rem_cost_changed = 100000;
            this.rem_profit_changed = -100000;
            this.ins_cost_changed = 100000;
            this.rel_cost_change = 100000;
            this.rem_profit_changed = -100000;
            this.ins_profit_changed = -100000;
            this.node_to_insert = null;
            this.node_to_remove = null;
            this.is_move_found = false;
        }

        override
        public Move DeepCopy()
        {
            return new InnerRelocation(this.node_removing_position, this.set_removing_position, this.removing_route, this.node_insertion_position, this.set_insertion_position,
                   this.inserting_route, this.rem_cost_changed, this.ins_cost_changed, this.rel_cost_change, this.rem_profit_changed, this.ins_profit_changed,
                   this.node_to_insert, this.node_to_remove, this.is_move_found);
        }

        override
        public void FindBestMove(Solution sol) // finds the best available rellocation of the sets that there are currently in the solution,
                                               // the best node of the relocated set is selected, not the one removed necessarily
                                               // ===CAUTION===
                                               // a relocation will be returned only if a relocation that decreases
                                               // the cost of the route is found
                                               // ---if we want to check if an insertion was found,
                                               //   check if this.node_removing_position != -1

        {
            Model m = Model.model;
            this.InitializeFields();
            Set checking_set;
            int rel_cost, ins_route_cost_change, rem_route_cost_change, cost_of_insertion, cost_of_removing, ins_route_profit_change, rem_route_profit_change; // the cost of the relocation that is checked
            Node pred, succ; // the predecessor and successor in case the relocation happens

            foreach (Route rem_route in sol.routes)
            {
                List<Set> sets_in_rem_route = rem_route.sets_included;
                List<Node> nodes_in_rem_route = rem_route.nodes_seq;
                for (int rem_pos = 1; rem_pos < sets_in_rem_route.Count - 1; rem_pos++) // from the second position till the next-to-last, as the first and last nodes are the depot
                {
                    checking_set = sets_in_rem_route[rem_pos];
                    cost_of_removing = m.dist_matrix[nodes_in_rem_route[rem_pos - 1].id, nodes_in_rem_route[rem_pos + 1].id] -
                        m.dist_matrix[nodes_in_rem_route[rem_pos - 1].id, nodes_in_rem_route[rem_pos].id] -
                        m.dist_matrix[nodes_in_rem_route[rem_pos].id, nodes_in_rem_route[rem_pos + 1].id]; // how much the cost will be changed because 
                                                                                                           // of the removing of the relocating node
                    foreach (Node checking_node in checking_set.nodes)
                    {
                        foreach (Route ins_route in sol.routes)
                        {
                            List<Set> sets_in_ins_route = ins_route.sets_included;
                            List<Node> nodes_in_ins_route = ins_route.nodes_seq;
                            for (int ins_pos = 1; ins_pos < nodes_in_ins_route.Count; ins_pos++) // possible inserting positions, all the positions except the beginning
                                                                                                 // and the end of the route
                            {
                                if (ins_route == rem_route && (rem_pos == ins_pos || rem_pos == ins_pos - 1))
                                {
                                    continue;
                                }
                                pred = nodes_in_ins_route[ins_pos - 1];
                                succ = nodes_in_ins_route[ins_pos];
                                cost_of_insertion = m.dist_matrix[pred.id, checking_node.id] +
                                    m.dist_matrix[checking_node.id, succ.id] -
                                    m.dist_matrix[pred.id, succ.id];  // how much the cost will be changed because of the insertion of the relocating node

                                // the total cost change in the solution cause by the relocation
                                rel_cost = cost_of_removing + cost_of_insertion;

                                // calculate the cost and profit change for the removing and inserting route
                                if (ins_route == rem_route)
                                {
                                    rem_route_cost_change = cost_of_removing + cost_of_insertion;
                                    ins_route_cost_change = cost_of_removing + cost_of_insertion;
                                    rem_route_profit_change = 0;
                                    ins_route_profit_change = 0;
                                }
                                else
                                {
                                    rem_route_cost_change = cost_of_removing;
                                    ins_route_cost_change = cost_of_insertion;
                                    rem_route_profit_change = -checking_node.profit;
                                    ins_route_profit_change = checking_node.profit;
                                }

                                if (rel_cost < this.rel_cost_change && ins_route.time + ins_route_cost_change <= m.t_max)
                                {
                                    this.node_removing_position = rem_pos;
                                    this.set_removing_position = rem_pos;
                                    this.removing_route = rem_route;
                                    this.node_insertion_position = ins_pos;
                                    this.set_insertion_position = ins_pos;
                                    this.inserting_route = ins_route;
                                    this.rem_cost_changed = rem_route_cost_change;
                                    this.ins_cost_changed = ins_route_cost_change;
                                    this.rel_cost_change = rel_cost;
                                    this.rem_profit_changed = rem_route_profit_change;
                                    this.ins_profit_changed = ins_route_profit_change;
                                    this.node_to_insert = checking_node;
                                    this.node_to_remove = nodes_in_rem_route[rem_pos];
                                    is_move_found = true;
                                }
                            }
                        }
                    }
                }
            }

        }

        override
        public void ApplyBestMove(Solution sol) // a relocation must have been found in order to apply it (this.is_move_found to check)
        {
            this.removing_route.nodes_seq.RemoveAt(this.node_removing_position);
            Set relocated_set = this.removing_route.sets_included[this.set_removing_position];
            this.removing_route.sets_included.RemoveAt(this.set_removing_position);
            // since the removing is made, if the removing position is before the insertion position and inserting route is the same with the removing route, 
            // then there is a node less in the route, than it was when insertion position was calculated
            // (if removing position is after insertion position, the removing does not affect the correct insertion position)
            if (this.removing_route == this.inserting_route && this.node_removing_position < this.node_insertion_position)
            {
                this.node_insertion_position--;
                this.set_insertion_position--;
            }
            this.inserting_route.nodes_seq.Insert(this.node_insertion_position, this.node_to_insert);
            inserting_route.sets_included.Insert(this.set_insertion_position, relocated_set);
            // if inserting route is the same with removing route then a single route cost must be updated
            if (inserting_route == removing_route)
            {
                inserting_route.time += this.ins_cost_changed;
            }
            else
            {
                inserting_route.time += this.ins_cost_changed;
                removing_route.time += this.rem_cost_changed;
            }
            // update profits
            this.removing_route.total_profit += this.rem_profit_changed;
            this.inserting_route.total_profit += this.ins_profit_changed;
            //Console.WriteLine("Insert: {0} remove: {1} ins_position: {2} rem_position: {3}", this.node_to_insert.id, this.node_to_remove.id, this.node_insertion_position, this.node_removing_position);
        }

        override
        public bool IsMoveFound()
        {
            return this.is_move_found;
        }
    }
}
