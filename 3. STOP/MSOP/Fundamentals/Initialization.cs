using System;
using System.Collections.Generic;
using System.Linq;

namespace MSOP.Fundamentals
{
    class Constructive_Move
    {
        public Node node;
        public Route route;
        public int pos;
        public int cost_added;
        public double profit_to_cost_rate;

        public Constructive_Move(Node node, Route route, int pos, int cost_added, double profit_to_cost_rate)
        {
            this.node = node;
            this.route = route;
            this.pos = pos;
            this.cost_added = cost_added;
            this.profit_to_cost_rate = profit_to_cost_rate;
        }
    }

    class Initialization
    {
        //static readonly int RCL_SIZE = 4;

        public static Solution Minimum_Insertions(int RCL_SIZE)
        {
            Model m = Model.model;
            Solution sol = new Solution();

            //m.sets[0].in_route = true;
            int rcl_limit = 0;  // specifies the exact number of available moves to choose from
            Random r = m.r;

            while (true)
            {
                List<Constructive_Move> feasible_insertions = new List<Constructive_Move>();
                foreach (Route route in sol.routes)
                {
                    List<Node> nodes_seq = route.nodes_seq;

                    // find all feasible insertions for each position
                    for (int i = 1; i < nodes_seq.Count; i++)
                    {
                        foreach (Node node in m.nodes)
                        {
                            if (sol.sets_included.Contains(m.sets[node.set_id])) // set is already covered 
                            {
                                continue;
                            }
                            int cost_added = m.dist_matrix[nodes_seq[i - 1].id, node.id] + m.dist_matrix[node.id, nodes_seq[i].id] - m.dist_matrix[nodes_seq[i - 1].id, nodes_seq[i].id];
                            //Console.WriteLine("{0} {1}", cost_added, m.t_max);
                            if (route.time + cost_added <= m.t_max)  // node addition results in feasible solution 
                            {
                                double profit_to_cost_rate = cost_added > 0 ? m.sets[node.set_id].profit / cost_added : Math.Pow(10, 10);
                                feasible_insertions.Add(new Constructive_Move(node, route, i, cost_added, profit_to_cost_rate));
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
                Constructive_Move selected_move = (Constructive_Move)sorted_insertions.ElementAt(r.Next(rcl_limit));

                // find the node added
                Node selected_node = selected_move.node;
                // find the route that the addition was made
                Route selected_route = selected_move.route;
                //m.sets[selected_node.set_id].in_route = true;

                // add selected node to the solution
                selected_route.nodes_seq.Insert(selected_move.pos, selected_node);
                selected_route.sets_included.Insert(selected_move.pos, m.sets[selected_node.set_id]);
                sol.sets_included.Add(m.sets[selected_node.set_id]);
                selected_route.time += selected_move.cost_added;
                selected_route.total_profit += m.sets[selected_node.set_id].profit;
                sol.total_profit += selected_node.profit;
            }
            return sol;
        }
    }
}
