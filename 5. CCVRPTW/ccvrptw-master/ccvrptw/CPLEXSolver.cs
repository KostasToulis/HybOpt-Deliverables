using CCVRPTW;
using Google.OrTools.ConstraintSolver;
using Google.OrTools.Sat;
using ILOG.Concert;
using ILOG.CPLEX;
using System;
using System.Globalization;
using System.Text;
using System.Threading.Tasks;

namespace ccvrptw
{
    public class CPLEXSolver
    {
        public static Solution? SolveCumulativePartialVRPTW(Solution input_sol, int[] route_ids, float timeLimit = 120.0f, int workerNum = 1)
        {
            Cplex cplex = new Cplex();

            Model m = ProblemConfiguration.model;

            List<Node> _customers = new();
            int route_num = route_ids.Length;

            //Generate Inverval variables for all customers
            for (int i = 0; i < route_ids.Length; i++)
                for (int j = 1; j < input_sol.routes[route_ids[i]].sequence.Count - 1; j++)
                    _customers.Add(input_sol.routes[route_ids[i]].sequence[j]);

            //Add Univisited
            foreach (Node n in input_sol.unvisited)
                _customers.Add(n);

            Console.WriteLine("Customer Report");
            foreach (Node n in _customers)
                Console.WriteLine($"{n.id}");

            
            //Decision variables
            int node_num = _customers.Count + 2;
            //Arc variables
            IIntVar[][][] x = new IIntVar[route_num][][];
            for (int k = 0; k < route_num; k++)
            {
                x[k] = new IIntVar[node_num][];
                for (int i = 0; i < node_num; i++)
                {
                    x[k][i] = new IIntVar[node_num];
                    for (int j = 0; j < node_num; j++)
                    {
                        x[k][i][j] = cplex.IntVar(0, 1);
                    }
                }
            }

            //Service Start Times
            IIntVar[][] s = new IIntVar[route_num][];
            for (int i = 0; i < route_num; i++)
                s[i] = cplex.IntVarArray(node_num, 0, 10000);

            //Parameters
            double maxTimeBetweenCustomers = 0.0;
            for (int i = 0; i < node_num; i++)
            {
                Node n_i = (i == 0 || i == node_num - 1) ? m.depot : _customers[i - 1];
                for (int j = 0; j < node_num; j++)
                {
                    Node n_j = (j == 0 || j == node_num - 1) ? m.depot : _customers[j - 1];
                    
                    maxTimeBetweenCustomers = Math.Max(maxTimeBetweenCustomers, 
                        (n_i.windowEnd + m.distances[n_i.id, n_j.id] + n_i.serviceTime - n_j.windowStart));
                }

            }
            maxTimeBetweenCustomers = 10000.0;
                
            //Constraints

            //Disable self-arcs
            for (int i = 0; i < route_num; i++)
                for (int j = 0; j < node_num; j++)
                    cplex.Add(cplex.Eq(x[i][j][j], 0));

            //Each customer should be visited once
            for (int i = 1; i < node_num - 1; i++)
            {
                List<INumExpr> terms1 = new();
                for (int k = 0; k < route_num; k++)
                    for (int j = 0; j < node_num; j++)
                    {
                        terms1.Add(x[k][i][j]);
                    }
                        
                cplex.Add(cplex.Le(cplex.Sum(terms1.ToArray()), 1));
            }

            //Depots should be visited route_num times
            for (int k = 0; k < route_num; k++)
            {
                List<INumExpr> terms1 = new();
                for (int j = 0; j < node_num; j++)
                    terms1.Add(x[k][0][j]);
                
                cplex.Add(cplex.Eq(cplex.Sum(terms1.ToArray()), 1));
            }

            for (int k = 0; k < route_num; k++)
            {
                List<INumExpr> terms = new();
                for (int j = 0; j < node_num; j++)
                {
                    terms.Add(x[k][j][node_num - 1]);
                }
                    
                cplex.Add(cplex.Eq(cplex.Sum(terms.ToArray()), 1));
                cplex.Add(cplex.Eq(x[k][node_num - 1][0], 1));
            }

            //Visit continuity constraints
            for (int k = 0; k < route_num; k++)
            {
                for (int i = 0; i < node_num; i++)
                {
                    List<INumExpr> terms1 = new();
                    List<INumExpr> terms2 = new();
                
                    for (int j = 0; j < node_num; j++)
                    {
                        terms1.Add(x[k][i][j]);
                        terms2.Add(x[k][j][i]);
                    }
                    cplex.Add(cplex.Eq(cplex.Sum(terms1.ToArray()), cplex.Sum(terms2.ToArray())));
                }
            }


            //Vehicle Capacity constraints
            for (int k = 0; k < route_num; k++)
            {
                List<INumExpr> terms = new();
                for (int i = 1; i < node_num - 1; i++) //Iterate in customers
                {
                    for (int j = 0; j < node_num; j++)
                        terms.Add(cplex.Prod(x[k][i][j], _customers[i - 1].demand));
                }
                cplex.Add(cplex.Ge(m.capacity, cplex.Sum(terms.ToArray())));
            }

            //Calculate correct departure times
            for (int k = 0; k < route_num; k++)
                for (int i = 0; i < node_num - 1; i++)
                    for (int j = 1; j < node_num; j++)
                    {
                        Node n_i = (i == node_num - 1 || i == 0) ? m.depot : _customers[i - 1];
                        Node n_j = (j == node_num - 1 || j == 0) ? m.depot : _customers[j - 1];

                        INumExpr t1 = cplex.Sum(s[k][i], cplex.Constant(m.distances[n_i.id, n_j.id] + n_i.serviceTime - maxTimeBetweenCustomers));
                        INumExpr t2 = cplex.Prod(maxTimeBetweenCustomers, x[k][i][j]);
                        cplex.Add(cplex.Le(cplex.Sum(t1,t2), s[k][j]));
                    }


            //Time Windows
            for (int k = 0; k < route_num; k++)
            {
                for (int i = 1; i < node_num - 1; i++) 
                {
                    
                    cplex.Add(cplex.Ge(_customers[i - 1].windowEnd, s[k][i]));
                    cplex.Add(cplex.Le(_customers[i - 1].windowStart, s[k][i]));
                    
                }
            }

            //Warm Start
            for (int i = 0; i < route_ids.Length; i++)
            {
                Route rt = input_sol.routes[route_ids[i]];
                List<INumVar> vars = new();
                List<double> values = new();
                for (int j = 1; j < rt.sequence.Count - 1; j++)
                {
                    Node u = rt.sequence[j];
                    Node v = rt.sequence[j + 1];

                    int u_id = _customers.IndexOf(u) + 1;
                    int v_id = _customers.IndexOf(v) + 1;

                    vars.Add(x[i][u_id][v_id]);
                    values.Add(1.0);
                }
                cplex.AddMIPStart(vars.ToArray(), values.ToArray());
            }


            //Distance Obj
            INumExpr dist_obj;
            {
                List<INumExpr> terms = new();
                for (int k = 0; k < route_num; k++)
                    for (int i = 0; i < node_num; i++)
                        for (int j = 0; j < node_num; j++)
                        {
                            Node n_i = (i == node_num - 1 || i == 0) ? m.depot : _customers[i - 1];
                            Node n_j = (j == node_num - 1 || j == 0) ? m.depot : _customers[j - 1];
                            
                            terms.Add(cplex.Prod(x[k][i][j], m.distances[n_i.id, n_j.id]));
                        }
                      
                dist_obj = cplex.Sum(terms.ToArray());
            }

            //Unvisited Customers Obj
            {
                List<INumExpr> terms = new();
                for (int k = 0; k < route_num; k++)
                    for (int i = 1; i < node_num - 1; i++)
                        for (int j = 1; j < node_num; j++)
                        {
                            terms.Add(x[k][i][j]);
                        }

                INumExpr obj_expr = cplex.Sum(terms.ToArray());
                cplex.Add(cplex.Le(obj_expr, cplex.Constant(_customers.Count)));
                IObjective obj = cplex.Minimize(cplex.Sum(cplex.Prod(10000, cplex.Diff(cplex.Constant(node_num - 2), obj_expr)), 
                                                          dist_obj));
                cplex.Add(obj);
            }

            cplex.SetParam(Cplex.DoubleParam.TimeLimit, 300.0f);
            //cplex.ExportModel("cplex_model.lp");
            if (cplex.Solve())
            {
                Console.WriteLine($"Objective {cplex.ObjValue}");
                //Solution Report
                StreamWriter sw = new StreamWriter("temp.txt", false);
                for (int k = 0; k < route_num; k++)
                    for (int i = 0; i < node_num; i++)
                        for (int j = 0; j < node_num; j++)
                            if (cplex.GetValue(x[k][i][j]) > 0.5)
                                sw.WriteLine($"x[{k}][{i}][{j}] =  {cplex.GetValue(x[k][i][j])}");
                sw.Close();

                for (int k = 0; k < route_num; k++)
                {
                    Console.Write($"Route {k}: ");
                    for (int i = 0; i < node_num; i++)
                        for (int j = 0; j < node_num; j++)
                        {
                            if (cplex.GetValue(x[k][i][j]) > 0.95)
                            {
                                Node n_i = (i == 0 || i == node_num - 1) ? m.depot : _customers[i - 1];
                                Node n_j = (j == 0 || j == node_num - 1) ? m.depot : _customers[j - 1];
                                
                                Console.Write($"[{n_i.id} ({cplex.GetValue(s[k][i])}) ({n_i.windowStart} - {n_i.windowEnd})] -> {n_j.id}");
                            }
                                
                        }
                    Console.Write("\n");
                }

                Solution output_sol = new Solution(input_sol);

                //ClearNodes
                for (int i = 0; i < route_num; i++)
                    output_sol.routes[route_ids[i]].ClearNodes();

                List<Node> added_customers = new();
                for (int k = 0; k < route_num; k++)
                {
                    Route rt = output_sol.routes[route_ids[k]];
                    int current_node = 0;
                    while (true)
                    {
                        for (int j = 0; j < node_num; j++)
                            if (cplex.GetValue(x[k][current_node][j]) > 0.98)
                            {
                                if (current_node != 0)
                                {
                                    Node new_n = new(_customers[current_node - 1]);
                                    rt.sequence.Insert(rt.sequence.Count - 1, new_n);
                                    added_customers.Add(new_n);
                                }
                                current_node = j;
                            }

                        if (current_node == node_num - 1)
                            break;
                    }
                    rt.UpdateRouteNodes();

                }

                //Add Unvisited
                output_sol.unvisited.Clear();
                foreach (Node n in _customers)
                {
                    if (added_customers.Find(x=> x.id == n.id) is null)
                        output_sol.unvisited.Add(new(n));
                }
                    
                //Recalculate solution objective
                output_sol.cost = output_sol.ComputeCumulativeDistances(true);
                return output_sol;
            }
            else
            {
                Console.WriteLine($"Status {cplex.GetStatus().ToString()}");
            }
            
            return null;

        }


    }
}
