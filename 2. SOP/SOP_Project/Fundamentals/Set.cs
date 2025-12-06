using System.Collections.Generic;
using System;

// class Set
namespace SOP_Project
{
    public class Set
    {
        public int id;
        public int profit;
        public int pool_profit; // the profit that will be considered when the solution from pool is created
        public List<Node> nodes;
        internal int profit_promise;

        public Set(int id, int profit, List<Node> nodes)
        {
            this.id = id;
            this.profit = profit;
            this.nodes = nodes;
            this.pool_profit = profit;
            profit_promise = -1;
        }

        public Set DeepCopy() // generates a deep copy of a Model object
        {
            List<Node> nodesDeepCopy = this.nodes.ConvertAll(node => node.DeepCopy()); // create a deep copy of the List nodes
            return new Set(this.id, this.profit, nodesDeepCopy);
        }

        public static List<Set> CreateDeepCopyOfSetsListWithKnownNodes(List<Set> set_list, List<Node> known_nodes)
        {
            List<Set> sets_deep_copy = set_list.ConvertAll(set => new Set(set.id, set.profit, new List<Node>())); // create a deep copy of the set_list, but the list of nodes in every set is empty
            foreach (Node known_node in known_nodes)
            {
                sets_deep_copy[known_node.set_id].nodes.Add(known_node); // add the known node to correct set in the copy
            }
            return sets_deep_copy;
        }

        override
        public string ToString()
        {
            String node_ids = new String("(" + nodes[0].id);

            for (int i = 1; i < nodes.Count; i++)
            {
                node_ids += ", " + nodes[i].id;
            }
            node_ids += ")";
            return "id:" + id + " | profit:" + profit + " | nodes:" + node_ids;
        }
    }
}
