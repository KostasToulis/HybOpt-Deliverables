using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SOP_Project
{
    class InOut21Swap : Move // it removes 2 nodes from the current solution and adds 1 in a way that the new profit is greter
    {
        public int node_adding_position;
        public int set_adding_position;
        public Node node_to_add;
        public Set set_to_add;
        public Node node1_to_remove;
        public Set set1_to_remove;
        public Node node2_to_remove;
        public Set set2_to_remove;
        public int cost;
        public int profit;
        public bool is_move_found;
        const int BigNumber = 10000; // used in case of positive profit and negative cost (it gets added to profit_to_cost_rate)
                                     // it may be necessary to change it in case of a dataset with very large numbers
        public int critirion; // it is M*profit_changed - cost_changed


        public InOut21Swap()
        {
            this.node_adding_position = -1;
            this.set_adding_position = -1;
            this.node_to_add = null;
            this.set_to_add = null;
            this.node1_to_remove = null;
            this.set1_to_remove = null;
            this.node2_to_remove = null;
            this.set2_to_remove = null;
            this.cost = -1;
            this.profit = -1;
            this.is_move_found = false;
            this.critirion = -100000;
        }

        public InOut21Swap(int node_add_pos, int set_add_pos, Node node_added, Set set_added, Node node1_removed, Set set1_removed, 
            Node node2_removed, Set set2_removed, int cost, int profit,  bool is_found)
        {
            this.node_adding_position = node_add_pos;
            this.set_adding_position = set_add_pos;
            this.node_to_add = node_added;
            this.set_to_add = set_added;
            this.node1_to_remove = node1_removed;
            this.set1_to_remove = set1_removed;
            this.node2_to_remove = node1_removed;
            this.set2_to_remove = set1_removed;
            this.cost = cost;
            this.profit = profit;
            this.is_move_found = is_found;
            this.critirion = BigNumber * this.profit - this.cost;
        }

        public void InitializeFields()
        {
            this.node_adding_position = -1;
            this.set_adding_position = -1;
            this.node_to_add = null;
            this.set_to_add = null;
            this.node1_to_remove = null;
            this.set1_to_remove = null;
            this.node2_to_remove = null;
            this.set2_to_remove = null;
            this.cost = -1;
            this.profit = -1;
            this.is_move_found = false;
            this.critirion = -100000;
        }

        override
        public Move ShallowCopy()
        {
            return new InOut21Swap(this.node_adding_position, this.set_adding_position, this.node_to_add, this.set_to_add, this.node1_to_remove, this.set1_to_remove,
           this.node2_to_remove, this.set2_to_remove, this.cost, this.profit, this.is_move_found);
        }

        override
        public void FindBestMove(Model m, Solution sol) // finds the best available 2-1 swap of a node
                                                        // it removes 2 nodes from the current solution and adds 1 in a way that the new profit is greter


        {
            this.InitializeFields();
            List<Set> all_sets = m.sets;
            List<Node> nodes_in_route = sol.route.nodes_seq;
            List<Set> sets_in_route = sol.route.sets_included;
            int cost_change_from_node_addition, profit_change_from_node_addition, cost_change_from_node1_removal, profit_change_from_node1_removal, cost_change_from_node2_removal,
                profit_change_from_node2_removal, swap21_cost, swap21_profit;
            Node node1_rem, pred1, succ1, node2_rem, pred2, succ2, outer_node, ins_pred, ins_succ;
            int move_critirion;

            List<InOutSwap> moves = new List<InOutSwap>();

            //loop for the first node that will be removed
            for (int i = 1; i < nodes_in_route.Count - 1; i++) // in range[1, nodes_in_route.Count - 2] because depot must not be removed
            {
                node1_rem = nodes_in_route[i];
                pred1 = nodes_in_route[i - 1];
                succ1 = nodes_in_route[i + 1];
                profit_change_from_node1_removal = - node1_rem.profit;
                cost_change_from_node1_removal = m.dist_matrix[pred1.id, succ1.id] - m.dist_matrix[pred1.id, node1_rem.id] - m.dist_matrix[node1_rem.id, succ1.id];

                //loop for the second node that will be removed
                for (int j = i + 1; j < nodes_in_route.Count - 1; j++) // in range[i + 1, nodes_in_route.Count - 2] because we don't want to check two times for the same couple of nodes (i,j and j,i)
                {
                    node2_rem = nodes_in_route[j];
                    if (j - 1 != i) // if nodes_in_route[j - 1] is not to be removed set is as pred
                    {
                        pred2 = nodes_in_route[j - 1];
                    }
                    else // if nodes_in_route[j - 1] is to be removed
                    {
                        if ( j - 2 == 0)  // if nodes_in_route[j - 2] is the depot continue
                        {
                            continue;
                        }
                        // if nodes_in_route[j - 2] is  not the depot
                        pred2 = nodes_in_route[j - 2];
                    }
                    succ2 = nodes_in_route[j + 1];
                    profit_change_from_node2_removal = - node2_rem.profit;
                    cost_change_from_node2_removal = m.dist_matrix[pred2.id, succ2.id] - m.dist_matrix[pred2.id, node2_rem.id] - m.dist_matrix[node2_rem.id, succ2.id];

                    //loop for the set that will be added
                    for (int k = 0; k < all_sets.Count; k++)
                    {
                        Set outer_set = all_sets[k]; // using name outer set because if the set is already in the solution then we continue to the next one
                                                     // if the set is already in the sol's route continue to the next set
                        if (sets_in_route.Contains(outer_set))
                        {
                            continue;
                        }
                        profit_change_from_node_addition = outer_set.profit;
                        swap21_profit = profit_change_from_node_addition + profit_change_from_node1_removal + profit_change_from_node2_removal;

                        //check if Promises allow the move
                        if (!Promises.MoveIsAdmissible(swap21_profit, sol.total_profit, outer_set))
                        {
                            continue;
                        }

                        //ceck if the profit of the 2-1 swap is positive
                        if (swap21_profit < 0)
                        {
                            continue;
                        }

                        // if the set have passed the above checks, check the insertion of each of its nodes
                        for (int n = 0; n < outer_set.nodes.Count; n++)
                        {
                            outer_node = outer_set.nodes[n];
                            for (int ins_pos = 1; ins_pos < nodes_in_route.Count; ins_pos++) //in range[1, nodes_in_route.Count - 1] because it can be placed at the beginning
                            {

                                // set the ins pred
                                if (ins_pos - 1 != i && ins_pos - 1 != j) // if nodes_in_route[ins_pos - 1] is not to be removed set is as pred
                                {
                                    ins_pred = nodes_in_route[ins_pos - 1];
                                }
                                else // if nodes_in_route[ins_pos - 1] is to be removed
                                {
                                    if (ins_pos - 2 == 0)  // if nodes_in_route[ins_pos - 2] is the depot continue
                                    {
                                        continue;
                                    }
                                    else if (ins_pos - 2 != i) // check because of the case that j is after i and ins_pos after j
                                    {
                                        ins_pred = nodes_in_route[ins_pos - 2];
                                    }
                                    else // that's the case that j is after i and ins_pos after j
                                    {
                                        if (ins_pos - 3 == 0) // if nodes_in_route[ins_pos - 3] is the depot continue
                                        {
                                            continue;
                                        }
                                        else
                                        {
                                            ins_pred = nodes_in_route[ins_pos - 3];
                                        }
                                    }

                                }

                                // set the ins_succ
                                // ins_succ will be the node that will be after the inserted node, so if the nodes_in_route[ins_pos] node
                                // isn't to be removed, then it will be the successor of the inserted (in case it is not clear why ins_pos is used and not ins_pos + 1)
                                if (ins_pos != i && ins_pos != j) // if nodes_in_route[ins_pos] is not to be removed set is as succ
                                {
                                    ins_succ = nodes_in_route[ins_pos];
                                }
                                else // if nodes_in_route[ins_pos] is to be removed
                                {
                                    if (ins_pos + 1 == nodes_in_route.Count - 1)  // if nodes_in_route[ins_pos + 1] is the depot continue
                                    {
                                        continue;
                                    }
                                    else if (ins_pos + 1 != j) // check because of the case that j is after i and ins_pos before i
                                    {
                                        ins_succ = nodes_in_route[ins_pos + 1];
                                    }
                                    else // that's the case that j is after i and ins_pos before j
                                    {
                                        if (ins_pos + 2 == nodes_in_route.Count - 1) // if nodes_in_route[ins_pos + 2] is the depot continue
                                        {
                                            continue;
                                        }
                                        else
                                        {
                                            ins_succ = nodes_in_route[ins_pos + 2];
                                        }
                                    }

                                }
                                cost_change_from_node_addition = - m.dist_matrix[ins_pred.id, ins_succ.id] + m.dist_matrix[ins_pred.id, outer_node.id] + m.dist_matrix[outer_node.id, ins_succ.id];
                                swap21_cost = cost_change_from_node_addition + cost_change_from_node1_removal + cost_change_from_node2_removal;
                                move_critirion = BigNumber * swap21_profit - swap21_cost;

                                if (sol.route.time + swap21_cost <= m.t_max && move_critirion > this.critirion)
                                {
                                    this.node_adding_position = ins_pos;
                                    this.set_adding_position = ins_pos;
                                    this.node_to_add = outer_node;
                                    this.set_to_add = outer_set;
                                    this.node1_to_remove = node1_rem;
                                    this.set1_to_remove = m.sets[node1_to_remove.set_id]; // the set with id = x is in m.sets[x]
                                    this.node2_to_remove = node2_rem;
                                    this.set2_to_remove = m.sets[node2_to_remove.set_id]; // the set with id = x is in m.sets[x]
                                    this.cost = swap21_cost;
                                    this.profit = swap21_profit;
                                    this.is_move_found = true;
                                    this.critirion = move_critirion;
                                }
                            }

                        }


                    }
                }
            }
        }

        override
        public void ApplyBestMove(Solution sol) // a 2-1 swap must have been found in order to apply it
        {
            Promises.MakePromise(sol.route.sets_included[this.set_adding_position], sol.total_profit);

            // insert the node and set to be inserted
            sol.route.nodes_seq.Insert(this.node_adding_position, this.node_to_add);
            sol.route.sets_included.Insert(this.set_adding_position, this.set_to_add);

            // remove the first node and set to be removed
            sol.route.nodes_seq.Remove(this.node1_to_remove);
            sol.route.sets_included.Remove(this.set1_to_remove);

            // remove the second node and set to be removed
            sol.route.nodes_seq.Remove(this.node2_to_remove);
            sol.route.sets_included.Remove(this.set2_to_remove);

            //update costs and profits
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
