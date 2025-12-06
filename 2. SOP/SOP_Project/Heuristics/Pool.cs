using System;
using System.Collections.Generic;
using System.Text;
using System.Linq;
using System.IO;

namespace SOP_Project
{
    public class Pool
    {
        public static List<Solution> sol_pool = new List<Solution>();
        static readonly int pool_size = 20;
        static int count_restart = 0;

        public static int[] node_frequency;
        public static Dictionary<int, int> set_frequency = new Dictionary<int, int>();
        public static Dictionary<List<int>, int> chains_freq = new Dictionary<List<int>, int>();  

        // declare the acceptable limit of similarity (as a percentage) betweeen the candidate solution
        // and any other solution already in the pool
        private static readonly int POOL_SIMILARITY_BOUND = 90;
        //private static readonly int DIVERSE_POOL_INTERNAL_SIMILARITY_UPPER_BOUND = 85;  // degree of higher acceptable similarity between solutions of the diversed pool 
        //private static readonly int DIVERSE_POOL_EXTERNAL_SIMILARITY_UPPER_BOUND = 85;  // degree of lower acceptable limit of differentiation between each solution in diversed pool and the solutions in intense pool


        public static void Initialize(Model m)
        {
            sol_pool.Clear();
            node_frequency = new int[m.node_crowd];
            InitializeSetFrequencyList(m);
            chains_freq.Clear();
            SyntacticMatching.NO_OF_CHARS = m.set_crowd;
        }

        public static void InitializeSetFrequencyList(Model m)
        {
            for (int i = 1; i < m.set_crowd; i++)
            {
                // try catch because it throws exception if the set_frequency[i] already exists (from previous run), in this case we want just to set it to 0
                try
                {
                    set_frequency.Add(i, 0);
                }
                catch (Exception)
                {
                    set_frequency[i] = 0;
                }
                
            }
        }
        public static bool UpdatePool(Solution sol)  // returns true if the solutions inside the
                                                                                                // pool have changed, false otherwise
        {
            // find the position to insert the solution object so that all objects are sorted in ascending objective order
            int pos_to_insert = 0;
            Solution curr_sol = pos_to_insert < sol_pool.Count ? sol_pool[pos_to_insert] : null;
            while (curr_sol != null && (sol.total_profit > curr_sol.total_profit ||
                (curr_sol.total_profit == sol.total_profit && sol.total_time < curr_sol.total_time)))
            {
                pos_to_insert += 1;
                curr_sol = pos_to_insert < sol_pool.Count ? sol_pool[pos_to_insert] : null;
            }

            // alternatively, we can exclude solutions with exactly the same sets, in order to achieve bigger differentiation
            // we check for similarity only with solutions that have the same or better objective
            List<int> similar_sols = FindSimilarSolsWithCandidate(sol, sol_pool, POOL_SIMILARITY_BOUND);
            List<int> pos_to_remove = new List<int>() { 0}; // will always be 0 unless a new solution is similar to a pool solution with lower total profit
            if (similar_sols.Count > 0)
            {
                if (similar_sols[similar_sols.Count - 1] >= pos_to_insert)  // in this case, there is a similar sol with a bigger objective than the candidate
                {
                    return false;
                }
                else
                {
                    pos_to_remove = similar_sols;
                }
            }

            // update the pool according to the size restrictions
            if (sol_pool.Count < pool_size)  // there's still room for any solution
            {
                sol_pool.Insert(pos_to_insert, sol);
                if (similar_sols.Count > 0)
                {
                    for (int i = pos_to_remove.Count - 1; i >= 0; i--)
                    {
                        sol_pool.RemoveAt(pos_to_remove[i]);
                    }
                }
            }
            else  // the pool is full
            {
                if (pos_to_insert == 0)  // the solution is worse than any other in the pool
                { 
                    return false; 
                }
                //Console.WriteLine("NEW" + sol.total_profit + " " + sol_pool[0].total_profit);
                sol_pool.Insert(pos_to_insert, sol);
                for (int i = pos_to_remove.Count - 1; i >= 0; i--)
                {
                    sol_pool.RemoveAt(pos_to_remove[i]);
                }
            }
            return true;
        }

        static List<int> FindSimilarSolsWithCandidate(Solution candidate, List<Solution> pool, double similarity_bound)
            // returns the positions where similar sols to a candidate was found, -1 if none was found
        {
            List<int> similar_sols = new List<int>();
            for (int i=0; i < pool.Count; i++)
            {
                if (Solution.SetSimilarityPercentage(candidate, pool[i]) >= similarity_bound)
                {
                    similar_sols.Add(i);
                }
            }
            return similar_sols;
        }

        public static void UpdateFrequencies(Model m)
        {
            Pool.InitializeSetFrequencyList(m);
            foreach (Solution sol in sol_pool)
            {
                for (int pos = 0; pos < sol.route.nodes_seq.Count - 1; pos++)
                {
                    if (pos != 0)  // don't include depot's frequency
                    {
                        Node node = sol.route.nodes_seq[pos];
                        node_frequency[node.id] += 1;
                        set_frequency[node.set_id] += 1;
                    }
                }
            }

            chains_freq = UpdateChainFrequencies(sol_pool);
        }

        // Adjusts the pool_costs of the nodes and sets, based on if the set is frequent or not (and if the node belongs to a frequent set or not)
        public static void AdjustPoolCosts(Model m)
        {
            int number_of_sets_used = (int)(0.6 * Pool.sol_pool[Pool.sol_pool.Count - 1].route.sets_included.Count); // the number of the most frequntly used sets that will be used to create
                                                                                                                   // an initial solution of high quality. It is a percentage of the numbers of 
                                                                                                                   // sets that the best solution includes.
            List<KeyValuePair<int, int>> set_frequecy_list = set_frequency.ToList();
            set_frequecy_list.Sort((pair1, pair2) => pair2.Value.CompareTo(pair1.Value)); // pair2.Value compared to pair1.Value in order to sort the list in descending order
            HashSet<int> frequent_sets_ids = new HashSet<int>(); // a HashSet that contains the ids of the 'number_of_sets_used' most frequently used sets in the pool  

            for (int i = 0; i < number_of_sets_used; i++)
            {
                int frequent_set_id = set_frequecy_list[i].Key;
                frequent_sets_ids.Add(frequent_set_id);
            }

            foreach (Set set in m.sets)
            {
                // if the currently examining set is one of the most frequent sets don't change its pool_profit
                if (frequent_sets_ids.Contains(set.id)) 
                {
                    continue;
                }
                // if the currently examining set is not one of the most frequent sets set its pool_profit and all its nodes pool_profits to -1
                set.pool_profit = -1;
                foreach (Node node_not_in_frequent_set in set.nodes)
                {
                    node_not_in_frequent_set.pool_profit = -1;
                }
            }
        }

        public static Dictionary<List<int>, int> UpdateChainFrequencies(List<Solution> pool, bool avoid_intense_chains = false)
            // the bool is used to avoid searching for a diverse pool pattern when it already exisits
            // in the intense_chain_freq dict 
        {
            Dictionary<List<int>, int> pattern_freq = new Dictionary<List<int>, int>(new ListComparer());

            int min_chain_size = 2;
            int max_chain_size = (int) Math.Min(Math.Ceiling((double) sol_pool[0].route.sets_included.Count / 4), 10);  // |sets_of_worst_pool_sol| / 4 (if this value is greater than 5, set max chain size to 6)

            // create an integer array representation of the pool
            SyntacticMatching.PoolToSetSeq(pool);

            for (int chain_size = max_chain_size; chain_size >= min_chain_size; chain_size--)
            {
                // generate all possible n-sized chains and find their frequency in pool
                foreach (Solution sol in pool)
                {
                    for (int i = 1; i < sol.route.nodes_seq.Count - chain_size; i++)
                    {
                        List<int> pattern = sol.route.nodes_seq.GetRange(i, chain_size).Select(x => x.set_id).ToList();
                        if (pattern_freq.ContainsKey(pattern) || (avoid_intense_chains &&
                            ListComparer.OverlapsPattern(chains_freq.Keys.ToList(), pattern)))  // key equality is checked through ListComparer
                        {
                            continue;
                        }
                        int freq = SyntacticMatching.GetMatching(pattern);
                        if (freq >= 2)
                        {
                            pattern_freq.Add(pattern, freq);
                        }
                    }
                }
            }
            return pattern_freq;
        }

        public static void SelectMostFrequentChains(Model m)
        {
            List<Chain> chains = new List<Chain>();
            List<KeyValuePair<List<int>, int>> sorted_chains = chains_freq.OrderByDescending(x => x.Value).ToList();
            int chains_selected = (int)Math.Ceiling(0.5 * sorted_chains.Count);
            foreach (var pair in sorted_chains.GetRange(0, chains_selected))
            {
                chains.Add(new Chain(pair.Key, m));
            }
            Chain.all_chains = chains;
        }

        //Creates an initial solution using Minimum Insertions, using only the most frequnt sets
        public static Solution CreateSolutionFromMostFrequentSets(Model m)
        {
            //return Initialization.ConstructSolFromPool(m);
            //return Initialization.SolveAsKnapsack_Feasible(m);
            //return Initialization.SolveAsKnapsack_InfeasibleTest(m);
            return Initialization.SolveAsKnapsack_Infeasible(m);
        }

        // Fills the solution generated by CreateSolutionFromMostFrequentSets method, with sets that doesn't belong to the most frequent
        public static void FillSolutionFromMostFrequentSets(Model basicModel, Solution solFromMostFrequentSets, Random r)
        {
            Initialization.ConstructSolFromPool(basicModel);

        }

        public static void ReinitializePoolProfits(Model m)
        {
            foreach (Set set in m.sets)
            {
                // if the pool_profit of the set is the same with its profit move on to the next set
                if (set.pool_profit == set.profit)
                {
                    continue;
                }
                // set the pool_profit of sets and nodes back to their profit
                set.pool_profit = set.profit;
                foreach (Node node in set.nodes)
                {
                    node.pool_profit = node.profit;
                }
            }
        }

        // Creates a solution based on the information that the pool contains, regarding the most frequent sets.
        public static Solution CreateSolutionBySetsInPool(Model m)
        {
            Pool.UpdateFrequencies(m);
            //AdjustPoolCosts(m);
            SelectMostFrequentChains(m);
            Solution poolSol = CreateSolutionFromMostFrequentSets(m);
            // Chain.InitializeChainDistMatrix();
            // Solution poolSol = Initialization.Get_Solution_From_Chains(m); // create a basic solution that contains the most frequent sets
            //FillSolutionFromMostFrequentSets(m, poolSol, r); // fill the pull sol with sets outside the most frequent
            //sol_pool.Clear();  // clear the pool to get ready for the next run
            chains_freq.Clear();
            //diverse_chains_freq.Clear();
            Chain.all_chains.Clear();  // empty all chains to get ready for the next run
            //Chain.Initialize_Chains(m);
            ReinitializePoolProfits(m);
            count_restart++;

            return poolSol;
        }

        public static void ReportPoolSols(Model m)
        {
            string target_folder = "Pool_Sols/Pool_at_restart_" + count_restart;
            try
            {
                Directory.CreateDirectory(target_folder);
            }
            catch { }

            int element = 0;
            foreach(Solution sol in sol_pool)
            {
                sol.ReportSol("./" + target_folder + "/sol_" + element + "_obj_" + sol.total_profit, m);
                element++;
            }
        }
    }
}
