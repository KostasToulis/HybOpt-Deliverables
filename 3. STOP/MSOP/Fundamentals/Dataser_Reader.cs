using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;

namespace MSOP.Fundamentals

{
    public class Dataset_Reader
    {
        public Model Read_Dataset(string dataset_path)
        {
            string dataset_name = "";
            int node_crowd = 0;
            int set_crowd = 0;
            double t_max = 0;
            int vehicle_number = 0;
            Node depot;
            List<Node> nodes = new List<Node>();
            List<Set> sets = new List<Set>();
            bool working_on_nodes = false;
            bool working_on_sets = false;

            string[] lines = File.ReadAllLines(dataset_path);
            foreach (string line in lines)
            {
                string[] sequence = line.Split(":");
                if (sequence[0].Equals("NAME"))
                {
                    dataset_name = sequence[1].Trim();
                }
                else if (sequence[0].Equals("DIMENSION"))  // set number of nodes
                {
                    node_crowd = Int16.Parse(sequence[1].Trim());
                }
                else if (sequence[0].Equals("VEHICLES"))
                {
                    vehicle_number = Int16.Parse(sequence[1].Trim());
                }
                else if (sequence[0].Equals("TMAX"))  // set t_max restriction
                {
                    t_max = Double.Parse(sequence[1].Trim());
                }
                else if (sequence[0].Equals("SETS"))  // set number of sets
                {
                    set_crowd = Int16.Parse(sequence[1].Trim());
                }
                else if (sequence[0].Equals("NODE_COORD_SECTION"))  // find the line after which the file contains the problem nodes
                {
                    working_on_nodes = true;
                }
                else if (sequence[0].Equals("GTSP_SET_SECTION"))  // find the line after which the file contains the problem sets
                {
                    working_on_nodes = false;
                    working_on_sets = true;
                }
                else if (working_on_nodes)
                {
                    string[] values = System.Text.RegularExpressions.Regex.Split(sequence[0].Trim(), @"\s+");
                    nodes.Add(new Node(Int16.Parse(values[0]), Double.Parse(values[1], CultureInfo.GetCultureInfo("en-US")), Double.Parse(values[2], CultureInfo.GetCultureInfo("en-US")), -1));
                }
                else if (working_on_sets)
                {
                    string[] values = sequence[0].Trim().Split();
                    int[] node_ids = new int[values.Length - 2];
                    List<Node> nodes_in_set = new List<Node>();
                    for (int i = 2; i < values.Length; i++)
                    {
                        node_ids[i - 2] = Int16.Parse(values[i]);
                    }
                    foreach (int id in node_ids)
                    {
                        nodes_in_set.Add(nodes[id - 1]);
                    }
                    sets.Add(new Set(Int16.Parse(values[0]), Int16.Parse(values[1]), nodes_in_set));
                }
            }
            depot = nodes[0];

            // set depot's id at 0 by reducing all nodes'ids by 1
            foreach (Node node in nodes)
            {
                node.id -= 1;
            }

            return new Model(dataset_name, node_crowd, set_crowd, vehicle_number, t_max, depot, nodes, sets);
        }
    }
}