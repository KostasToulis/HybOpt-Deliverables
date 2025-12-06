using MSOP.Fundamentals;
using MSOP.Heuristics;
using System;
using System.Collections.Generic;

namespace MSOP.Operators
{
    class OuterInsertion : Move //refers to an insertion in a route of a node, that its set is not currently in the route (and the node itself obviously)
    {
        public int node_insertion_position;
        public int set_insertion_position;
        public Route inserting_route;
        public int cost_added;
        public int profit_added;
        //public double profit_to_cost_rate;
        public Node node_to_insert;
        public Set set_to_insert;
        public bool is_move_found;
        const int BigNumber = 10000;
        public int critirion; // it is M*profit_added - cost_added


        public OuterInsertion()
        {
            this.node_insertion_position = -1;
            this.set_insertion_position = -1;
            this.inserting_route = null;
            this.cost_added = -1;
            this.profit_added = -1;
            //this.profit_to_cost_rate = 0;
            this.node_to_insert = null;
            this.set_to_insert = null;
            this.is_move_found = false;
            this.critirion = -100000;
        }

        public OuterInsertion(int node_pos, int set_pos, Route route, int cost, int profit, Node node, Set set, bool is_found)
        {
            this.node_insertion_position = node_pos;
            this.set_insertion_position = set_pos;
            this.inserting_route = route;
            this.cost_added = cost;
            this.profit_added = profit;
            //this.profit_to_cost_rate = cost != 0 ? profit / cost : Math.Pow(10, 10); // if cost is 0 then the insertion's profit_to_rate cost is set to a very big number 
            this.node_to_insert = node;
            this.set_to_insert = set;
            this.is_move_found = is_found;
            this.critirion = BigNumber * profit_added - cost_added;
        }

        public void InitializeFields()
        {
            this.node_insertion_position = -1;
            this.set_insertion_position = -1;
            this.inserting_route = null;
            this.cost_added = -1;
            this.profit_added = -1;
            //this.profit_to_cost_rate = 0;
            this.node_to_insert = null;
            this.set_to_insert = null;
            this.is_move_found = false;
            this.critirion = -100000;
        }

        override
        public Move DeepCopy()
        {
            return new OuterInsertion(this.node_insertion_position, this.set_insertion_position, this.inserting_route, this.cost_added,
                   this.profit_added, this.node_to_insert, this.set_to_insert, this.is_move_found);
        }

        override
        public void FindBestMove(Solution sol) // finds the best available insertion of the nodes that are not in the solution,
                                               // it uses profit to cost criterion
                                               // - if we want to check if an insertion was found,
                                               //   check if this.node_insertion_position != -1

        {
            Model m = Model.model;
            this.InitializeFields();
            List<Set> all_sets = m.sets;
            HashSet<Set> sets_in_sol = sol.sets_included;

            foreach (Route route in sol.routes)
            {
                List<Node> nodes_in_route = route.nodes_seq;
                int adding_profit, adding_cost;
                //double adding_profit_to_cost_rate;
                int move_critirion;

                Node pred, inner_node;

                foreach (Set set_not_in_sol in all_sets)
                {
                    // if the set is already in the sol's route continue to the next set
                    if (sets_in_sol.Contains(set_not_in_sol))
                    {
                        continue;
                    }

                    if (!Promises.MoveIsAdmissible(set_not_in_sol.profit, sol.total_profit, set_not_in_sol))
                    {
                        continue;
                    }

                    // if the set is not in the sol's route check the insertion of each of its nodes
                    adding_profit = set_not_in_sol.profit;
                    foreach (Node outer_node in set_not_in_sol.nodes)
                    {
                        for (int i = 1; i < nodes_in_route.Count; i++) // for i starting from the second node's position until the last one's
                        {
                            pred = nodes_in_route[i - 1];
                            inner_node = nodes_in_route[i];

                            //if (!Arc_Promises.MoveIsAdmissible(set_not_in_sol.profit, sol.total_profit, outer_node, inner_node) &&
                            //    !Arc_Promises.MoveIsAdmissible(set_not_in_sol.profit, sol.total_profit, pred, outer_node))
                            //{
                            //    continue;
                            //}

                            adding_cost = m.dist_matrix[pred.id, outer_node.id]
                            + m.dist_matrix[outer_node.id, inner_node.id] - m.dist_matrix[pred.id, inner_node.id];

                            //adding_profit_to_cost_rate = adding_cost != 0 ? Math.Pow(adding_profit, 1) / adding_cost : Math.Pow(10, 10); // if cost is 0 then the insertion's profit_to_rate cost is set to a really big number
                            move_critirion = BigNumber * adding_profit - adding_cost;

                            //if (adding_profit_to_cost_rate > this.profit_to_cost_rate && sol.route.time + adding_cost <= m.t_max)
                            if (move_critirion > this.critirion && route.time + adding_cost <= m.t_max)
                            {
                                this.node_insertion_position = i;
                                this.set_insertion_position = i;
                                this.inserting_route = route;
                                this.cost_added = adding_cost;
                                this.profit_added = adding_profit;
                                //this.profit_to_cost_rate = adding_profit_to_cost_rate;
                                this.node_to_insert = outer_node;
                                this.set_to_insert = set_not_in_sol;
                                this.is_move_found = true;
                                this.critirion = move_critirion;
                            }
                        }
                    }
                }
            }


        }

        override
        public void ApplyBestMove(Solution sol) // an insertion must have been found in order to apply it (this.is_move_found == true)
        {
            //Console.WriteLine("Id: {0} set:{4} pos:{5} profit: {1} cost: {2} rate: {3}", this.node_to_insert.id, this.profit_added, this.cost_added, this.profit_to_cost_rate, this.set_to_insert.id, this.node_insertion_position);

            this.inserting_route.nodes_seq.Insert(this.node_insertion_position, this.node_to_insert);
            this.inserting_route.sets_included.Insert(this.set_insertion_position, this.set_to_insert);
            this.inserting_route.time += this.cost_added;
            this.inserting_route.total_profit += this.profit_added;
            sol.total_profit += this.profit_added;
            sol.sets_included.Add(this.set_to_insert);
            if (!sol.CheckSol())
            {
                Console.WriteLine("Insertion error");
            }
        }

        override
        public bool IsMoveFound()
        {
            return this.is_move_found;
        }
    }
}
