using MSOP.Fundamentals;
using System;
using System.Collections.Generic;
using System.Linq;

namespace MSOP.Operators
{
    class TwoOpt : Move // two opt move, refers only to the nodes that are currently in the solution, like a TSP Two Opt
    {
        public int node_first_position;
        public int set_first_position;
        public Route first_route;
        public int node_second_position;
        public int set_second_position;
        public Route second_route;
        public int first_cost_changed;
        public int first_profit_changed;
        public int second_cost_changed;
        public int second_profit_changed;
        public int total_cost_changed;
        public bool second_reverse_direction;
        public bool is_move_found;

        public TwoOpt()
        {
            this.node_first_position = -1;
            this.set_first_position = -1;
            this.first_route = null;
            this.node_second_position = -1;
            this.set_second_position = -1;
            this.second_route = null;
            this.first_cost_changed = 100000;
            this.first_profit_changed = -100000;
            this.second_cost_changed = 100000;
            this.second_profit_changed = -100000;
            this.total_cost_changed = 100000;
            this.second_reverse_direction = false;
            this.is_move_found = false;
        }

        public TwoOpt(int node1_pos, int set1_pos, Route first_route, int node2_pos, int set2_pos, Route second_route, int first_cost, int first_profit, int second_cost,
            int second_profit, int total_cost, bool second_reverse, bool is_found)
        {
            this.node_first_position = node1_pos;
            this.set_first_position = set1_pos;
            this.first_route = first_route;
            this.node_second_position = node2_pos;
            this.set_second_position = set2_pos;
            this.second_route = second_route;
            this.first_cost_changed = first_cost;
            this.first_profit_changed = first_profit;
            this.second_cost_changed = second_cost;
            this.second_profit_changed = second_profit;
            this.total_cost_changed = total_cost;
            this.second_reverse_direction = false;
            this.is_move_found = false;
        }

        public void InitializeFields()
        {
            this.node_first_position = -1;
            this.set_first_position = -1;
            this.first_route = null;
            this.node_second_position = -1;
            this.set_second_position = -1;
            this.second_route = null;
            this.first_cost_changed = 100000;
            this.first_profit_changed = -100000;
            this.second_cost_changed = 100000;
            this.second_profit_changed = -100000;
            this.total_cost_changed = 100000;
            this.second_reverse_direction = false;
            this.is_move_found = false;
        }

        override
        public Move DeepCopy()
        {
            return new TwoOpt(this.node_first_position, this.set_first_position, this.first_route, this.node_second_position, this.set_second_position, this.second_route,
                this.first_cost_changed, this.first_profit_changed, this.second_cost_changed, this.second_profit_changed, this.total_cost_changed, this.second_reverse_direction,
                this.is_move_found);
        }

        override
        public void FindBestMove(Solution sol) // find-method based on professor's Zachariadis FindBestTwoOptMove TSP python method
        {
            Model m = Model.model;
            this.InitializeFields();
            Route route_1, route_2;
            List<Node> route_1_nodes, route_2_nodes;
            int route_1_current_cost, route_1_current_profit, route_2_current_cost, route_2_current_profit, route_1_new_cost, route_2_new_cost,
                route_1_new_profit, route_2_new_profit, cost_added, cost_removed, t_o_cost; //t_o_cost refers to the currently checking two opt cost
            Node one, two, three, four, a, b, k, l;
            for (int first_route_pos = 0; first_route_pos < sol.routes.Count; first_route_pos++)
            {
                route_1 = sol.routes[first_route_pos];
                route_1_nodes = route_1.nodes_seq;

                //===== check for move within the same route =====
                for (int first_index = 0; first_index < route_1_nodes.Count - 1; first_index++)
                {
                    a = route_1_nodes[first_index];
                    b = route_1_nodes[first_index + 1];

                    for (int second_index = first_index + 2; second_index < route_1_nodes.Count - 1; second_index++)
                    {
                        k = route_1_nodes[second_index];
                        l = route_1_nodes[second_index + 1];

                        if (first_index == 0 && second_index == route_1_nodes.Count - 2)
                        {
                            continue;
                        }

                        cost_added = m.dist_matrix[a.id, k.id] + m.dist_matrix[b.id, l.id];
                        cost_removed = m.dist_matrix[a.id, b.id] + m.dist_matrix[k.id, l.id];
                        t_o_cost = cost_added - cost_removed;

                        if (t_o_cost < this.total_cost_changed && route_1.time + t_o_cost <= m.t_max)
                        {
                            // the interval [node_first_position, node_second_position] defines the part of the sol.nodes_seq that must be reversed
                            // respectively the interval [set_first_position, set_second_position] defines the part of sol.sets_included that must be reversed
                            this.node_first_position = first_index + 1;
                            this.set_first_position = first_index + 1;
                            this.first_route = route_1;
                            this.node_second_position = second_index;
                            this.set_second_position = second_index;
                            this.second_route = route_1;
                            this.first_cost_changed = t_o_cost;
                            this.first_profit_changed = 0;
                            this.second_cost_changed = t_o_cost;
                            this.second_profit_changed = 0;
                            this.total_cost_changed = t_o_cost;
                            this.second_reverse_direction = false;
                            this.is_move_found = true;
                        }
                    }
                }
                //===== check for move with another route =====
                route_1_current_cost = 0;
                route_1_current_profit = 0;
                for (int first_index = 1; first_index < route_1_nodes.Count - 1; first_index++)
                {
                    one = route_1_nodes[first_index - 1];
                    two = route_1_nodes[first_index];
                    a = route_1_nodes[first_index];
                    b = route_1_nodes[first_index + 1];
                    route_1_current_cost += m.dist_matrix[one.id, two.id];
                    route_1_current_profit += two.profit;

                    for (int second_route_pos = first_route_pos + 1; second_route_pos < sol.routes.Count - 1; second_route_pos++)
                    {
                        route_2 = sol.routes[second_route_pos];
                        route_2_nodes = route_2.nodes_seq;

                        route_2_current_cost = 0;
                        route_2_current_profit = 0;
                        for (int second_index = 1; second_index < route_2_nodes.Count - 1; second_index++)
                        {
                            three = route_2_nodes[second_index - 1];
                            four = route_2_nodes[second_index];
                            k = route_2_nodes[second_index];
                            l = route_2_nodes[second_index + 1];
                            route_2_current_cost += m.dist_matrix[three.id, four.id];
                            route_2_current_profit += four.profit;

                            if (first_index == 0 && second_index == 0)
                            {
                                continue;
                            }
                            if (first_index == route_1_nodes.Count - 2 && second_index == route_2_nodes.Count - 2)
                            {
                                continue;
                            }

                            //===== not reverse =====
                            route_1_new_cost = route_1_current_cost + route_2.time - route_2_current_cost + m.dist_matrix[a.id, l.id] - m.dist_matrix[k.id, l.id];
                            route_2_new_cost = route_2_current_cost + route_1.time - route_1_current_cost + m.dist_matrix[k.id, b.id] - m.dist_matrix[a.id, b.id];

                            if (route_1_new_cost > m.t_max || route_2_new_cost > m.t_max)
                            {
                                continue;
                            }

                            t_o_cost = m.dist_matrix[a.id, l.id] + m.dist_matrix[k.id, b.id] - m.dist_matrix[a.id, b.id] - m.dist_matrix[k.id, l.id];
                            if (t_o_cost < this.total_cost_changed)
                            {
                                this.node_first_position = first_index;
                                this.set_first_position = first_index;
                                this.first_route = route_1;
                                this.node_second_position = second_index;
                                this.set_second_position = second_index;
                                this.second_route = route_2;
                                this.first_cost_changed = route_1_new_cost - route_1.time;
                                this.first_profit_changed = route_1_current_profit + route_2.total_profit - route_2_current_profit - route_1.total_profit; // new_profit - old_profit
                                this.second_cost_changed = route_2_new_cost - route_2.time;
                                this.second_profit_changed = route_2_current_profit + route_1.total_profit - route_1_current_profit - route_2.total_profit; // new_profit - old_profit
                                this.total_cost_changed = t_o_cost;
                                this.second_reverse_direction = false;
                                this.is_move_found = true;
                            }

                            //===== reverse =====
                            route_1_new_cost = route_1_current_cost + route_2_current_cost + m.dist_matrix[a.id, k.id];
                            route_2_new_cost = route_1.time - route_1_current_cost + route_2.time - route_2_current_cost +
                                m.dist_matrix[b.id, l.id] - m.dist_matrix[a.id, b.id] - m.dist_matrix[k.id, l.id];

                            if (route_1_new_cost > m.t_max || route_2_new_cost > m.t_max)
                            {
                                continue;
                            }

                            t_o_cost = m.dist_matrix[a.id, k.id] + m.dist_matrix[b.id, l.id] - m.dist_matrix[a.id, b.id] - m.dist_matrix[k.id, l.id];
                            if (t_o_cost < this.total_cost_changed)
                            {
                                this.node_first_position = first_index;
                                this.set_first_position = first_index;
                                this.first_route = route_1;
                                this.node_second_position = second_index;
                                this.set_second_position = second_index;
                                this.second_route = route_2;
                                this.first_cost_changed = route_1_new_cost - route_1.time;
                                route_1_new_profit = route_1_current_profit + route_2_current_profit;
                                this.first_profit_changed = route_1_new_profit - route_1.total_profit;
                                this.second_cost_changed = route_2_new_cost - route_2.time;
                                route_2_new_profit = route_1.total_profit - route_1_current_profit + route_2.total_profit - route_2_current_profit;
                                this.second_profit_changed = route_2_new_profit - route_2.total_profit;
                                this.total_cost_changed = t_o_cost;
                                this.second_reverse_direction = true;
                                this.is_move_found = true;
                            }
                        }
                    }
                }

            }


        }

        override

        public void ApplyBestMove(Solution sol) // apply-method based on professor's Zachariadis  ApplyTwoOptMove TSP python method
                                                // a two opt must have been found in order to apply it (this.node_first_position != -1 to check)
        {
            //Console.WriteLine("Applying Two_Opt operator...\n");
            // ===== same route =====
            if (this.first_route == this.second_route)
            {
                int numb_of_nodes_to_reverse = this.node_second_position + 1 - this.node_first_position;
                int numb_of_sets_to_reverse = this.set_second_position + 1 - this.set_first_position;
                // reverses the nodes in the range [node_first_position, node_second_position] (third arg of Reverse is the number of elements to be reversed)
                this.first_route.nodes_seq.Reverse(this.node_first_position, numb_of_nodes_to_reverse);
                // reverses the sets in the range [set_first_position, set_second_position] (third arg of Reverse is the number of elements to be reversed)
                this.first_route.sets_included.Reverse(this.set_first_position, numb_of_sets_to_reverse);
                this.first_route.time += this.total_cost_changed;
                return;
            }

            // ===== different routes =====
            List<Node> new_first_route_nodes, new_second_route_nodes;
            List<Set> new_first_route_sets, new_second_route_sets;
            // ===== not reverse =====
            if (!this.second_reverse_direction)
            {
                new_first_route_nodes = new List<Node>(this.first_route.nodes_seq.Take(this.node_first_position + 1).Concat(this.second_route.nodes_seq.Skip(this.node_second_position + 1)));
                new_first_route_sets = new List<Set>(this.first_route.sets_included.Take(this.set_first_position + 1).Concat(this.second_route.sets_included.Skip(this.set_second_position + 1)));
                new_second_route_nodes = new List<Node>(this.second_route.nodes_seq.Take(this.node_second_position + 1).Concat(this.first_route.nodes_seq.Skip(this.node_first_position + 1)));
                new_second_route_sets = new List<Set>(this.second_route.sets_included.Take(this.set_second_position + 1).Concat(this.first_route.sets_included.Skip(this.set_first_position + 1)));
            }
            // ===== reverse ======
            else
            {
                new_first_route_nodes = new List<Node>(this.first_route.nodes_seq.Take(this.node_first_position + 1).Concat(this.second_route.nodes_seq.Take(this.node_second_position + 1).Reverse()));
                new_first_route_sets = new List<Set>(this.first_route.sets_included.Take(this.set_first_position + 1).Concat(this.second_route.sets_included.Take(this.set_second_position + 1).Reverse()));
                new_second_route_nodes = new List<Node>(this.second_route.nodes_seq.Skip(this.node_second_position + 1).Reverse().Concat(this.first_route.nodes_seq.Skip(this.node_first_position + 1)));
                new_second_route_sets = new List<Set>(this.second_route.sets_included.Skip(this.set_second_position + 1).Reverse().Concat(this.first_route.sets_included.Skip(this.set_first_position + 1)));
            }
            this.first_route.nodes_seq = new List<Node>(new_first_route_nodes);
            this.first_route.sets_included = new List<Set>(new_first_route_sets);
            this.first_route.time += this.first_cost_changed;
            this.first_route.total_profit += this.first_profit_changed;

            this.second_route.nodes_seq = new List<Node>(new_second_route_nodes);
            this.second_route.sets_included = new List<Set>(new_second_route_sets);
            this.second_route.time += this.second_cost_changed;
            this.second_route.total_profit += this.second_profit_changed;
        }

        override
        public bool IsMoveFound()
        {
            return this.is_move_found;
        }

    }
}