using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using Gurobi;

namespace PRP
{
    public class VRP_LOCALSOLVER
    {


        public static void SolveVRP(Solution sol)
        {
            for (int i = 0; i < sol.periods.Count; i++)
            {
                Period pr = sol.periods[i];
                
                
                //Solve a VRP per period
                
                //Precalculate VRP instance attributes
                List<int> nodes = new List<int>();
                double vrp_cost = 0;
                nodes.Add(sol.depot.uid); //Add depot
                for (int j = 0; j < pr.periodRoutes.Count; j++)
                {
                    vrp_cost += pr.periodRoutes[j].totalRoutingCost;
                    for (int k = 1; k < pr.periodRoutes[j].nodes.Count - 1; k++)
                        nodes.Add(pr.periodRoutes[j].nodes[k].uid);
                }
                    
                GlobalUtils.writeToConsole("Start VRP Cost: {0}", vrp_cost);
                //Generate input .vrp for the localSolver
                StreamWriter SW;
                string filename = "test.vrp";
                if (File.Exists(filename) == false)
                {
                    SW = File.CreateText(filename);
                }
                else
                {
                    File.Delete(filename);
                    SW = File.CreateText(filename);
                }
            
                //Write Instance description
                SW.WriteLine("NAME: TEST INSTANCE");
                SW.WriteLine("COMMENT: GAMIESTE OLOI");
                SW.WriteLine("TYPE: CVRP");
                SW.WriteLine("DIMENSION: {0}", nodes.Count);
                SW.WriteLine("EDGE_WEIGHT_TYPE : EUC_2D");
                SW.WriteLine("CAPACITY : {0}", sol.model.input.dayVehicleCapacity);
                SW.WriteLine("NODE_COORD_SECTION");
                
                
                //Write nodes and their locations
                for (int j = 0; j < nodes.Count; j++)
                {
                    int node_id = nodes[j];
                    SW.WriteLine("{0} {1} {2}", j + 1,
                        sol.nodes[node_id].x_coord, sol.nodes[node_id].y_coord);
                }
                
                
                //Write demand section
                SW.WriteLine("DEMAND_SECTION");
                
                //Write nodes and their locations
                SW.WriteLine("1 0");
                for (int j = 1; j < nodes.Count; j++)
                {
                    int node_id = nodes[j];
                    SW.WriteLine("{0} {1}", j + 1, sol.nodes[node_id].deliveredQuantities[i]);
                }

                //Finalize
                SW.WriteLine("DEPOT_SECTION");
                SW.WriteLine("1");
                SW.WriteLine("-1");
                SW.WriteLine("EOF");

                SW.Close();
                
                //Delete previews output file if any
                if (File.Exists("test.sol"))
                    File.Delete("test.sol");
                
                //Call Solver
                System.Diagnostics.Process process = new Process();
                ProcessStartInfo startInfo = new ProcessStartInfo();
                startInfo.WindowStyle = ProcessWindowStyle.Normal;
                startInfo.UseShellExecute = true;
                //startInfo.FileName = "/bin/bash";
                //startInfo.Arguments = "-c \" " +
                //    "localsolver /opt/localsolver/examples/cvrp/cvrp.lsp inFileName=test.vrp solFileName=test.sol nbTrucks=10 lsTimeLimit=20" + " \"";
                startInfo.FileName = "localsolver";
                startInfo.Arguments = "/opt/localsolver_9_5/examples/cvrp/cvrp.lsp inFileName=test.vrp solFileName=test.sol nbTrucks=10 lsTimeLimit=20";
                process.StartInfo = startInfo;
                process.Start();
                
                while (!process.HasExited)
                {
                    try
                    {
                        while (!process.StandardOutput.EndOfStream)
                        {
                            var line = process.StandardOutput.ReadLine();
                            GlobalUtils.writeToConsole(line);
                        }    
                    } catch (Exception e) 
                    {
                        //GlobalUtils.writeToConsole("Done");
                    }
                }
                
                //Parse output file
                if (!File.Exists("test.sol"))
                {
                    GlobalUtils.writeToConsole("LocalSolver failed, probably empty VRP");
                    continue;
                }
                    
                
                StreamReader sr = new StreamReader("test.sol");
                string l = sr.ReadLine();
                List<string> l_data = new List<string>();
                l_data = GlobalUtils.SeperateStringIntoSubstrings(' ', l);

                int routeNum = int.Parse(l_data[0]);
                double vrp_obj = double.Parse(l_data[1]);

                double recalcualted_vrp_obj = 0.0;
                for (int j = 0; j < routeNum; j++)
                {
                    l = sr.ReadLine();
                    l_data = GlobalUtils.SeperateStringIntoSubstrings(' ', l);

                    Console.Write("Route {0} : 0 {1}", j, nodes[Int32.Parse(l_data[0]) - 1]);
                    
                    for (int k = 1; k < l_data.Count; k++)
                    {
                        int prev = Int32.Parse(l_data[k - 1]) - 1;
                        int next = Int32.Parse(l_data[k]) - 1;
                        
                        int node_prev = nodes[prev];
                        int node_next = nodes[next];
                        
                        Console.Write(" {0}", node_next);

                        recalcualted_vrp_obj += sol.model.distMatrix[node_prev, node_next];
                    }
                    
                    Console.Write(" 0\n");
                    
                    recalcualted_vrp_obj += sol.model.distMatrix[0, nodes[Int32.Parse(l_data[0]) - 1]];
                    recalcualted_vrp_obj += sol.model.distMatrix[0, nodes[Int32.Parse(l_data.Last()) - 1]];
                }
                
                
                
                
                GlobalUtils.writeToConsole("LocalSolver Obj: {0} vs Recalculated: {1} vs Start: {2}",
                    vrp_obj, recalcualted_vrp_obj, vrp_cost);
                






            }
            
            
            
            
        }
        
    }
}