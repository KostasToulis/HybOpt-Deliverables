using System;
using System.Collections.Generic;
using System.Text;
using Google.OrTools.ConstraintSolver;
using Google.Protobuf.WellKnownTypes;
using MSOP.Fundamentals;
using localsolver;
using System.IO;
using System.Linq;
using System.Linq.Expressions;
using static ILOG.CPLEX.Cplex.Param;
using System.Security;

namespace MSOP.Solvers
{

    //Local Solver VRP 
    public class VRPLocalSolver : IDisposable
    {
        LocalSolver localsolver;

        int truckCapacity;
        long[] demandsData;
        int nbCustomers;// Number of customers exlude depot
        int nbTrucks;
        long[][] distMatrixData;
        long[] distDepotData;         // Distances between customers and depot

        // Decision variables
        LSExpression[] customersSequences;
        LSExpression[] trucksUsed; // Are the trucks actually used
        LSExpression nbTrucksUsed; // Number of trucks used in the solution     
        LSExpression totalDistance; // Distance traveled by all the trucks

        public VRPLocalSolver()
        {
            localsolver = new LocalSolver();
        }

        public void Dispose()
        {
            if (localsolver != null)
                localsolver.Dispose();
        }

        public Solution solveVRP(Model m, Solution sop_solution, int maxTime, bool silence = true)
        {
            // declare variables 
            int[] reduced_depot_dist_matrix;
            int[,] reduced_cust_dist_matrix;
            int infeasible_customers_count = 0;
            Solution vrp_solution = new Solution();
            List<Node> nodesVRP = new List<Node>();
            List<Node> customersVRP = new List<Node>();
            List<Node> infeasible_customers = new List<Node>();

            // params 
            int timeLimit = maxTime * 60; //maxTime * 60; 
            int threads_num = 8; //Localsolver gives one token to each machine, so only one instance may be run at a time. Therefore, we can use all available threads
            int secs_between_displays = 120;
            bool ignore_infeasible_customers = true;
            bool min_total_dist_secondary = true;

            // find customers of the VRP (all contained into the visited sets of the sop solution)
            foreach (Set set in sop_solution.sets_included)
            {
                foreach (Node node in set.nodes)
                {
                    nodesVRP.Add(node);
                    customersVRP.Add(node);
                }
            }
            customersVRP.Remove(m.depot);

            // check if all customer can be served in VRP (is t_max enough for direct delivery)
            Program.runData.unserved_customers = checkedDirectDeliveryInfeasibility(m, customersVRP);
            infeasible_customers = Program.runData.unserved_customers;
            infeasible_customers_count = infeasible_customers.Count;

            // remove this customer to enable vrp solution
            if (ignore_infeasible_customers)
            {
                string output = "";
                if (!silence)
                {
                    Console.WriteLine("Warning: removing nodes the following nodes to guarantee a feasible VRP solution " +
                        "(even direct deliveries are impossible under t_max={0}):", m.t_max);
                }

                foreach (Node cust in infeasible_customers)
                {
                    output += (cust.id + " ");
                    customersVRP.Remove(cust);
                    nodesVRP.Remove(cust);
                }

                if (!silence)
                {
                    Console.WriteLine(output);
                }
            }

            // easy access
            int veh_num = customersVRP.Count; //.node_crowd vehicle_number
            int cust_num = customersVRP.Count;
            int total_nodes_num = nodesVRP.Count;

            // make new distance matrix
            (reduced_depot_dist_matrix, reduced_cust_dist_matrix) = calculateReducedMatrix(m, nodesVRP);

            // Declare the optimization model
            LSModel model = localsolver.GetModel();
            trucksUsed = new LSExpression[veh_num];
            customersSequences = new LSExpression[veh_num];
            LSExpression[] distRoutes = new LSExpression[veh_num];

            // Sequence of customers visited by each truck
            for (int k = 0; k < veh_num; ++k)
            {
                customersSequences[k] = model.List(cust_num);
            }

            // Constraint: All customers must be visited by the trucks
            model.Constraint(model.Partition(customersSequences));

            // Create LocalSolver arrays to be able to access them with an "at" operator
            //LSExpression demands = model.Array(demandsData);
            LSExpression distDepot = model.Array(reduced_depot_dist_matrix);
            LSExpression distMatrix = model.Array(reduced_cust_dist_matrix);
            //LSExpression distDepot = model.Array(Enumerable.Range(0, reduced_dist_matrix.GetLength(1)).Select(x => reduced_dist_matrix[0, x]).ToArray());
            //LSExpression distMatrix = model.Array(m.dist_matrix); // this is the customer matrix

            // for each vehicle
            for (int k = 0; k < veh_num; ++k)
            {
                LSExpression sequence = customersSequences[k];
                LSExpression c = model.Count(sequence); // counts visited customers (depots are not included)

                // A truck is used if it visits at least one customer
                trucksUsed[k] = c > 0;

                // The quantity needed in each route must not exceed the truck capacity
                //LSExpression demandLambda = model.LambdaFunction(i => demands[sequence[i]]);
                //LSExpression routeQuantity = model.Sum(model.Range(0, c), demandLambda);
                //model.Constraint(routeQuantity <= truckCapacity);

                // Distance traveled by truck k
                LSExpression distLambda = model.LambdaFunction(i => distMatrix[sequence[i - 1], sequence[i]]);
                //LSExpression distLambda = model.LambdaFunction(i => distMatrix[customersVRP[(int)sequence[i - 1].GetValue()].id, customersVRP[(int)sequence[i].GetValue()].id]);

                //distRoutes[k] = model.Sum(model.Range(1, c), distLambda)
                //    + model.If(c > 0, distMatrix[0, sequence[0]] + distMatrix[sequence[c - 1], 0], 0);
                distRoutes[k] = model.Sum(model.Range(1, c), distLambda)
                    + model.If(c > 0, distDepot[sequence[0]] + distDepot[sequence[c - 1]], 0);
                //+ model.If(c > 0, distMatrix[0, customersVRP[(int)sequence[0].GetIntValue()].id] 
                //+ distMatrix[customersVRP[(int) sequence[c - 1].GetIntValue()].id, 0], 0);

                // Tmax constraints
                model.Constraint(distRoutes[k] <= m.t_max);
            }

            nbTrucksUsed = model.Sum(trucksUsed);
            totalDistance = model.Sum(distRoutes);

            // Objective: minimize the number of trucks used, then minimize the distance traveled
            model.Minimize(nbTrucksUsed);
            if (min_total_dist_secondary)
            {
                model.Minimize(totalDistance);
            }

            model.Close();

            // Parametrize the solver
            localsolver.GetParam().SetTimeLimit(timeLimit);
            localsolver.GetParam().SetNbThreads(threads_num);
            localsolver.GetParam().SetTimeBetweenDisplays(secs_between_displays);

            // solve
            localsolver.Solve();

            // update the solution object
            vrp_solution.routes.Clear();
            for (int i = 0; i < nbTrucksUsed.GetValue(); i++)
            {
                vrp_solution.routes.Add(new Route(i));
            }

           int cur_veh = -1;
            for (int k = 0; k < veh_num; ++k)
            {
                if (trucksUsed[k].GetValue() != 1)
                    continue;
                else
                    cur_veh++;

                // Values in sequence are in [0..nbCustomers-1]. +2 is to put it back in [2..nbCustomers+1]
                // as in the data files (1 being the depot)
                LSCollection customersCollection = customersSequences[k].GetCollectionValue();
                Route rt = vrp_solution.routes[cur_veh];
                for (int i = 0; i < customersCollection.Count(); ++i)
                {
                    Node node = customersVRP[(int)customersCollection[i]];
                    rt.nodes_seq.Insert(rt.nodes_seq.Count - 1, node);
                    rt.sets_included.Insert(rt.sets_included.Count - 1, m.sets[node.set_id]);
                }
                rt.time = (int) distRoutes[k].GetValue();
            }
            //var x = localsolver.GetStatistics();
            Program.runData.status = localsolver.GetState().ToString();

            localsolver.Dispose();

            return vrp_solution;
        }

        public (int[] reduced_depot_dist_matrix, int[,] reduced_cust_dist_matrix) calculateReducedMatrix(Model m, List<Node> nodes)
        {
            int[] reduced_depot_dist_matrix = new int[nodes.Count - 1];
            int[,] reduced_cust_dist_matrix = new int[nodes.Count - 1, nodes.Count - 1];

            // depot matrix
            for (int i = 1; i < nodes.Count; i++)
            {
                Node n1 = nodes[i];
                reduced_depot_dist_matrix[i-1] = m.dist_matrix[0, n1.id];
            }

            // distance from self 
            for (int i = 1; i < nodes.Count; i++)
            {
                for (int j = 1; j < nodes.Count; j++)
                {
                    reduced_cust_dist_matrix[i-1, j-1] = 0;
                }
            }

            // symmetric calculation
            for (int i = 1; i < nodes.Count; i++)
            {
                for (int j = i + 1; j < nodes.Count; j++)
                {
                    Node n1 = nodes[i];
                    Node n2 = nodes[j];

                    //double d_x = Math.Abs(n1.x - n2.x);
                    //double d_y = Math.Abs(n1.y - n2.y);
                    //double dist = Math.Sqrt(Math.Pow(d_x, 2) + Math.Pow(d_y, 2));

                    reduced_cust_dist_matrix[i-1, j-1] = m.dist_matrix[n1.id, n2.id]; //(int)Math.Ceiling(dist);
                    reduced_cust_dist_matrix[j-1, i-1] = m.dist_matrix[n2.id, n1.id];                    
                }
            }
            return (reduced_depot_dist_matrix, reduced_cust_dist_matrix);
        }

        public List<Node> checkedDirectDeliveryInfeasibility(Model m, List<Node> customersVRP)
        {
            List<Node> infeasible_customers = new List<Node>();

            foreach (Node cust in customersVRP)
            {
                if (m.dist_matrix[0,cust.id] + m.dist_matrix[cust.id, 0] > m.t_max)
                {
                    // found customer that cannot be served
                    infeasible_customers.Add(cust);
                }
            }
            return infeasible_customers;
        }

        /* Write the solution in a file with the following format:
         *  - number of trucks used and total distance
         *  - for each truck the customers visited (omitting the start/end at the depot) */
        //public void WriteSolution(string fileName)
        //{
        //    using (StreamWriter output = new StreamWriter(fileName))
        //    {
        //        output.WriteLine(nbTrucksUsed.GetValue() + " " + totalDistance.GetValue());
        //        for (int k = 0; k < nbTrucks; ++k)
        //        {
        //            if (trucksUsed[k].GetValue() != 1)
        //                continue;
        //            // Values in sequence are in [0..nbCustomers-1]. +2 is to put it back in [2..nbCustomers+1]
        //            // as in the data files (1 being the depot)
        //            LSCollection customersCollection = customersSequences[k].GetCollectionValue();
        //            for (int i = 0; i < customersCollection.Count(); ++i)
        //            {
        //                output.Write((customersCollection[i] + 2) + " ");
        //            }
        //            output.WriteLine();
        //        }
        //    }
        //}
    }
}

