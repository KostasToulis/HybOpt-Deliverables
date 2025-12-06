using MSOP.Fundamentals;
using MSOP.Heuristics;
using System;
using System.Collections.Generic;

namespace MSOP.Operators
{
    class NodeRemoval : Move
    {
        public int removed_node_pos;
        public int removed_set_pos;
        public Route removing_route;
        public int cost_removed;
        public int profit_removed;
        public double profit_to_cost_rate;
        public Node removed_node;
        public Set removed_set;
        public bool is_move_found;

        public NodeRemoval()
        {
            removed_node_pos = -1;
            removed_set_pos = -1;
            cost_removed = 0;
            profit_removed = 0;
            profit_to_cost_rate = 100000;
            is_move_found = false;
        }

        public NodeRemoval(int removed_node_pos, int removed_set_pos, Route removing_route, int cost_removed, int profit_removed, double profit_to_cost_rate, Node removed_node)
        {
            this.removed_node_pos = removed_node_pos;
            this.removed_set_pos = removed_set_pos;
            this.removing_route = removing_route;
            this.cost_removed = cost_removed;
            this.profit_removed = profit_removed;
            this.profit_to_cost_rate = profit_to_cost_rate;
            this.removed_node = removed_node;
        }
        public void InitializeFields()
        {
            removed_node_pos = -1;
            removed_set_pos = -1;
            cost_removed = 0;
            profit_removed = 0;
            profit_to_cost_rate = 100000;
            is_move_found = false;
        }

        override
        public Move DeepCopy() // copy the elements of an object into a new one
        {
            return new NodeRemoval(this.removed_node_pos, this.removed_set_pos, this.removing_route, this.cost_removed, this.profit_removed,
            this.profit_to_cost_rate, this.removed_node);
        }

        override
        public void FindBestMove(Solution sol)  // find the node with the minimum profit_to_cost_rate
        {
            Model m = Model.model;
            this.InitializeFields();
            foreach (Route route in sol.routes)
            {
                List<Node> nodes = route.nodes_seq;
                for (int i = 1; i < nodes.Count - 1; i++)
                {
                    Node pred = nodes[i - 1];
                    Node curr_node = nodes[i];
                    Node succ = nodes[i + 1];
                    int profit_removed = curr_node.profit;
                    int cost_removed = m.dist_matrix[pred.id, curr_node.id] + m.dist_matrix[curr_node.id, succ.id] -
                        m.dist_matrix[pred.id, succ.id];
                    double profit_to_cost_rate = cost_removed > 0 ? profit_removed / Math.Pow(cost_removed, 1) : Math.Pow(10, 10);

                    // find the best possible move that is not part of the tabu list
                    if (profit_to_cost_rate < this.profit_to_cost_rate)
                    {
                        this.removed_node_pos = i;
                        this.removed_set_pos = i;
                        this.removing_route = route;
                        this.profit_removed = profit_removed;
                        this.cost_removed = cost_removed;
                        this.profit_to_cost_rate = profit_to_cost_rate;
                        this.removed_node = curr_node;
                        this.removed_set = route.sets_included[i];
                        is_move_found = true;
                    }

                    //Console.WriteLine("{0}", removed_node_pos);
                    //Console.WriteLine("{0} {1} {2} {3} {4} {5}", curr_node.id, profit_removed, cost_removed, profit_to_cost_rate, curr_node.scaled_profit, scaled_cost_removed);
                    //Console.WriteLine("{0} {1} {2} {3}", nodes[this.removed_node_pos].id, this.profit_removed, this.cost_removed, this.profit_to_cost_rate);
                    //Console.WriteLine("-----------------------------------------------");
                }
            }
        }

        override
        public void ApplyBestMove(Solution sol)
        {
            //Console.WriteLine("Id: {0} pos: {4} set:{5} profit: {1} cost: {2} rate {3}", this.removed_node.id, this.profit_removed, this.cost_removed, this.profit_to_cost_rate, this.removed_node_pos, this.removed_set.id);

            Promises.MakePromise(this.removing_route.sets_included[this.removed_set_pos], sol.total_profit);
            //Arc_Promises.MakePromise(sol.route.nodes_seq[this.removed_node_pos], sol.route.nodes_seq[this.removed_node_pos + 1], sol.total_profit);
            //Arc_Promises.MakePromise(sol.route.nodes_seq[this.removed_node_pos -1], sol.route.nodes_seq[this.removed_node_pos], sol.total_profit);

            this.removing_route.nodes_seq.RemoveAt(this.removed_node_pos);
            //sol.route.sets_included[this.removed_set_pos].in_route = false;
            this.removing_route.sets_included.RemoveAt(this.removed_set_pos);

            this.removing_route.time -= this.cost_removed;
            this.removing_route.total_profit -= this.profit_removed;
            sol.total_profit -= this.profit_removed;
            sol.sets_included.Remove(this.removed_set);
            if (!sol.CheckSol())
            {
                Console.WriteLine("Removal error");
            }
        }

        override
        public bool IsMoveFound()
        {
            return this.is_move_found;
        }
    }
}
