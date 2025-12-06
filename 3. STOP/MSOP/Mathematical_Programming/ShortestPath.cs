using MSOP.Fundamentals;
using System;
using System.Collections.Generic;

namespace MSOP.MathematicalProgramming
{
    public class ShortestPath
    {

        /**
         * Given a solution, the methods preserves the clusters and its order and optimizes the nodes visited in its cluster with respect to the distance travelled
         * Implementation adjusted from https://www.geeksforgeeks.org/shortest-path-for-directed-acyclic-graphs/
         */
        public static void SolveDAGShortestPath(Model m, Solution sol, bool hide_errors = false)
        {
            int allNodesNum = m.nodes.Count + 1; //add depot
            bool silence = true;

            // separately for each route
            foreach (Route route in sol.routes)
            {
                // initialize number of nodes (all nodes in visited sets)
                int V = allNodesNum;
                Stack<Node> stack = new Stack<Node>();

                for (int g = 0; g < route.sets_included.Count; g++)
                {
                    Set set = route.sets_included[route.sets_included.Count - 1 - g];
                    for (int i = 0; i < set.nodes.Count; i++)
                    {
                        Node node = set.nodes[i];
                        stack.Push(node);
                    }
                }

                // Preprocessing
                //edgeset
                bool[,] edgeArray = new bool[allNodesNum, allNodesNum];

                for (int g = 1; g < route.sets_included.Count; g++)
                {
                    Set set_pred = route.sets_included[g - 1];
                    Set set_suc = route.sets_included[g];

                    for (int i = 0; i < set_pred.nodes.Count; i++)
                    {
                        Node from = set_pred.nodes[i];

                        for (int j = 0; j < set_suc.nodes.Count; j++)
                        {
                            Node to = set_suc.nodes[j];
                            int toid = to.id;
                            if (toid == 0)
                            {
                                toid = allNodesNum - 1;
                            }
                            if (from.id != toid)
                            {
                                edgeArray[from.id, toid] = true;
                            }
                        }
                    }
                }

                // initialize topological ordering stack and dist array
                int[] dist = new int[V];
                Node[] parent = new Node[V];

                // Mark all the vertices as not visited
                Boolean[] visited = new Boolean[V];
                for (int i = 0; i < V; i++)
                    visited[i] = false;

                // Initialize distances to all vertices as infinite and distance to source as 0
                for (int i = 0; i < V; i++)
                    dist[i] = int.MaxValue;
                dist[0] = 0;

                for (int i = 0; i < V; i++)
                    parent[i] = null;

                // Process vertices in topological order
                while (stack.Count != 0)
                {
                    // Get the next vertex from topological order
                    Node u = stack.Pop();

                    // Update distances of all adjacent vertices
                    if (dist[u.id] != int.MaxValue)
                    {
                        for (int i = 0; i < m.nodes.Count; i++)
                        {
                            Node v = m.nodes[i];
                            int vid = v.id;
                            if (v.id == 0)
                            {
                                vid = V - 1;
                            }
                            if (edgeArray[u.id, vid])
                            {

                                if (dist[vid] > dist[u.id] + m.dist_matrix[u.id, v.id])
                                {
                                    dist[vid] = dist[u.id] + m.dist_matrix[u.id, v.id];
                                    parent[vid] = u;
                                }
                            }
                        }
                    }
                }

                // Results 
                if (!silence)
                {
                    //Console.WriteLine("Before shortest path");
                    //Console.WriteLine(route);
                }

                // 1. update route time (profits are the same)
                int oldTime = route.time;
                route.time = dist[V - 1];

                // 2. update the nodes lists
                //List<Node> oldRouteDC = sol.route.nodes_seq.ConvertAll(node => new Node(node.id, node.x, node.y, node.set_id));
                route.nodes_seq.Clear();

                route.nodes_seq.Add(m.depot);
                Node prev = parent[V - 1]; //last
                while (prev.id != 0)
                {
                    route.nodes_seq.Insert(0, prev);
                    prev = parent[prev.id];
                }
                route.nodes_seq.Insert(0, m.depot);

                // update sol and run tests (can be removed for runs)
                if (!hide_errors && !route.CheckRoute())
                {
                    Console.WriteLine("Error in shortest path");
                }
                if (!silence & oldTime != route.time)
                {
                    Console.WriteLine("Improvement from shortest path: old distance = {0} --> optimized distance = {1}", oldTime, route.time);
                }
                if (!silence)
                {
                    //Console.WriteLine("After shortest path");
                    //Console.WriteLine(route);
                }
            }
        }
    }
}
