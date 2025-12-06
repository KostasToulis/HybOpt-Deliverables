using System;
using System.Collections.Generic;
using System.Text;
using System.IO;
using System.Linq;
using static F_CVRP.Model;

namespace F_CVRP
{
    public class Setup
    {
        public struct Run_config
        {
            public string fileName;
            public Random random;

            // LS 
            public double replaceWeight;   
            public int maxIterations;
            public int promiseIter;
            public double arcPenalty;
            public int restartIter;
            public double guidedChance;

            // AM
            public double chanceAM;
            public int poolSize;
            public double minChainLen;
            public double maxChainLen;

            // Sub
            public double timesRemovedPenalty;

            public Run_config(string file, Model m)
            {
                fileName = file;
                random = new Random(1);
                replaceWeight = 0.5;
                maxIterations = m.numNodes * m.numNodes * 2;
                promiseIter = 100;
                guidedChance = 0.33;
                arcPenalty = 0.225;
                restartIter = m.numNodes/2;
                chanceAM = 0.8;
                poolSize = 5 * m.vehicles;
                minChainLen = 0.4;
                maxChainLen = 0.7;
                timesRemovedPenalty = 0.1;
            }


        }

        public static Model CreateModel(string file)
        {
            Model model = new Model();
            String line;
            try
            {
                StreamReader sr = new StreamReader(file);
                // 1st line
                line = sr.ReadLine();
                string[] parts = line.Split(' ');

                model.numNodes = Int32.Parse(parts[0]);
                model.numFam = Int32.Parse(parts[1]);
                model.numReq = Int32.Parse(parts[2]);
                model.capacity = Int32.Parse(parts[3]);
                model.vehicles = Int32.Parse(parts[4]);

                // 2nd line
                line = sr.ReadLine();
                parts = line.Split(' ');
                parts = parts.Where(s => !string.IsNullOrWhiteSpace(s)).ToArray();
                model.famMembers = new List<int>();
                foreach (string part in parts)
                {
                    model.famMembers.Add(Int32.Parse(part));
                }

                // 3rd line
                line = sr.ReadLine();
                parts = line.Split(' ');
                parts = parts.Where(s => !string.IsNullOrWhiteSpace(s)).ToArray();
                model.famReq = new List<int>();
                foreach (string part in parts)
                {
                    model.famReq.Add(Int32.Parse(part));
                }

                // 4th line
                line = sr.ReadLine();
                parts = line.Split(' ');
                parts = parts.Where(s => !string.IsNullOrWhiteSpace(s)).ToArray();
                model.famDemand = new List<float>();
                foreach (string part in parts)
                {
                    model.famDemand.Add(float.Parse(part));
                }

                // 5th line
                List<List<float>> costMatrix = new List<List<float>>();
                line = sr.ReadLine();

                while (line != null)
                {
                    List<float> nodeCosts = new List<float>();
                    parts = line.Split(' ');
                    parts = parts.Where(s => !string.IsNullOrWhiteSpace(s)).ToArray();
                    foreach (string part in parts)
                    {
                        if (float.Parse(part) < 0)
                        {
                            nodeCosts.Add(10000);
                        }
                        else
                        {
                            nodeCosts.Add(float.Parse(part));
                        }
                        
                    }
                    costMatrix.Add(nodeCosts);
                    line = sr.ReadLine();
                }
                model.costMatrix = costMatrix;

                sr.Close();

                return CreateNodesFamilies(model);
                
            }
            catch (Exception e)
            {
                Console.WriteLine("Exception: " + e.Message);
                return new Model();
            }
        }

        private static Model CreateNodesFamilies(Model model)
        {
            List<Family> families = new List<Family>();
            List<Node> nodes = new List<Node>();
            List<Node> customers = new List<Node>();

            // Family initialization
            for (int i = 0; i < model.numFam; i++)
            {
                Family family = new Family(i, new List<Node>(), model.famDemand[i], model.famReq[i]);
                families.Add(family);
            }

            // Depot initialization
            Node depot = new Node(0, null, model.costMatrix[0], 0);
            depot.isDepot = true;
            nodes.Add(depot);
            model.depot = depot;

            // Nodes and customers initialization
            for (int i=1; i < model.costMatrix.Count(); i++)
            {
                int famIndex = FindNodeFamily(model, i);              
                Node node = new Node(i, families[famIndex], model.costMatrix[i], families[famIndex].demand);      
                nodes.Add(node);
                customers.Add(node);
            }

            // Add customer nodes to families
            foreach (Node customer in customers)
            {
                families[customer.family.id].nodes.Add(customer);
            }
            model.families = families;
            model.nodes = nodes;
            model.customers = customers;

            //Run_config config = new Run_config(, model);

            return model;
        }

        private static int FindNodeFamily(Model model, int nodeID)
        {
            int c = 0;
            int prev = 0;
            foreach( int i in model.famMembers)
            {
                if (nodeID <= i + prev)
                {
                    return c;
                }
                else
                {
                    prev = prev + i;
                    c++;
                }
            }
            return c;
        }
    }
}
