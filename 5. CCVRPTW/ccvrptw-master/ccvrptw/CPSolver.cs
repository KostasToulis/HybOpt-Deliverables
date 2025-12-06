using ILOG.CP;
using ILOG.Concert;
using System.ComponentModel;

namespace CCVRPTW
{

    public class CPSolver
    {
        public static Solution? SolveVRPTW(float timeLimit = 120.0f, int workerNum = 1)
        {
            CP cp = new CP();
            Model m = ProblemConfiguration.model;

            System.Diagnostics.Debug.Assert(m != null);
            
            int eff_vehicle_num = m.vehicles + 2;

            //Generate Inverval variables for all customers
            List<IIntervalVar> customer_interval_vars = new();
            for (int i = 0; i < m.customers.Count; i++)
            {
                customer_interval_vars.Add(cp.IntervalVar());
            }
            
            //Generate Sequence Variables for all vehicles and Init
            List<IIntervalVar>[] vehicle_customer_vars = new List<IIntervalVar>[eff_vehicle_num];
            for (int i = 0; i < eff_vehicle_num; i++)
            {
                vehicle_customer_vars[i] = new();
            }
                
            //Generate Cumulative FUnction expressions to hold vehicle capacities
            IIntExpr[] vehicle_capacity_expressions = new IIntExpr[eff_vehicle_num];
            List<IIntervalVar> short_window_vars = new();
            List<IIntervalVar> long_window_vars = new();

            for (int j = 0; j < m.customers.Count; j++)
            {
                Node cust = m.customers[j];

                //Add customer to each vehicle and also tie with the main interval
                //variable of the customer using alternative global constraint

                List<IIntervalVar> options = new();
                for (int i = 0; i < eff_vehicle_num; i++)
                {
                    IIntervalVar customer_vehicle_var = cp.IntervalVar((int) cust.serviceTime, "veh_" + i + "_cust_" + j + "_intvar");
                    customer_vehicle_var.SetOptional();
                    vehicle_customer_vars[i].Add(customer_vehicle_var);

                    
                    
                    if (cust.windowEnd - cust.windowStart <= 20)
                    {
                        short_window_vars.Add(customer_vehicle_var);
                        customer_vehicle_var.StartMin = Math.Max((int) cust.windowStart - 10, 0);
                        customer_vehicle_var.StartMax = (int) cust.windowEnd + 10;
                        
                    } else
                    {
                        long_window_vars.Add(customer_vehicle_var);
                        customer_vehicle_var.StartMin = (int) cust.windowStart;
                        customer_vehicle_var.StartMax = (int) cust.windowEnd;
                    }
                    
                    customer_vehicle_var.EndMin = customer_vehicle_var.StartMin + (int) cust.serviceTime;
                    customer_vehicle_var.EndMax = customer_vehicle_var.StartMax + (int) cust.serviceTime;
                    
                    options.Add(customer_vehicle_var);
                }

                //Add alternative constraint
                cp.Add(cp.Alternative(customer_interval_vars[j], options.ToArray()));
            }

            //Vehicle capacity constraints
            for (int i = 0; i < eff_vehicle_num; i++)
            {
                IIntExpr[] cust_expr = new IIntExpr[m.customers.Count];
                for (int j=0; j < m.customers.Count; j++)
                    cust_expr[j] = cp.Prod(cp.PresenceOf(vehicle_customer_vars[i][j]), cp.Constant(m.customers[j].demand));
                vehicle_capacity_expressions[i] = cp.Sum(cust_expr);
                cp.Add(cp.Ge(m.capacity, vehicle_capacity_expressions[i]));
            }

            //Set variable types equal to the node ids
            int[] ntypes = new int[m.customers.Count + 2];
            for (int i = 0; i < m.customers.Count; i++)
                ntypes[i + 1] = m.customers[i].id;
            ntypes[0] = 0;
            ntypes[m.customers.Count + 1] = 0;

            //Prepare TransitionDistance Matrix
            int[][] int_d_mat = new int[m.customers.Count + 1][];
            for (int i = 0; i < m.customers.Count + 1; i++)
            {
                int_d_mat[i] = new int[m.customers.Count + 1];
                for (int j = 0; j < m.customers.Count + 1; j++)
                {
                    int_d_mat[i][j] = (int) Math.Ceiling(m.distances[i, j]);
                }
            }

            ITransitionDistance td = cp.TransitionDistance(int_d_mat, "td");


            INumExpr[] route_cost_expr = new INumExpr[eff_vehicle_num];

            //Add two depot interval variables for each vehicle
            for (int i = 0; i < eff_vehicle_num; i++)
            {
                IIntervalVar depot_start = cp.IntervalVar(0);
                vehicle_customer_vars[i].Insert(0, depot_start);
                IIntervalVar depot_end = cp.IntervalVar(0);
                depot_end.EndMax = (int) m.depot.windowEnd;
                vehicle_customer_vars[i].Add(depot_end);

                IIntervalSequenceVar var = cp.IntervalSequenceVar(vehicle_customer_vars[i].ToArray(), ntypes, "vehicle_seq_" + i);

                cp.Add(cp.First(var, depot_start));
                cp.Add(cp.Last(var, depot_end));

                //Add no overlap constraint
                cp.Add(cp.NoOverlap(var, td));

                //Alternative way of calculating route costs
                INumExpr[] cust_expr = new INumExpr[m.customers.Count + 1];
                
                
                for (int j = 0; j < m.customers.Count + 1; j++)
                {
                    double[] cust_distances = new double[m.customers.Count + 1];
                    IIntExpr typeId;
                    if (j == m.customers.Count)
                    {
                        //Depot End
                        for (int k = 0; k < m.customers.Count + 1; k++)
                            cust_distances[m.nodes[k].id] = m.distances[0, m.nodes[k].id];

                        typeId = cp.TypeOfPrevious(var, vehicle_customer_vars[i][j + 1],
                        0, 0);

                    } else
                    {
                        for (int k = 0; k < m.customers.Count + 1; k++)
                            cust_distances[m.nodes[k].id] = m.distances[m.customers[j].id, m.nodes[k].id];
                        
                        typeId = cp.TypeOfPrevious(var, vehicle_customer_vars[i][j + 1],
                        m.customers[j].id, m.customers[j].id);
                    }
                    
                    cust_expr[j] = cp.Element(cust_distances, typeId);

                }
                route_cost_expr[i] = cp.Sum(cust_expr);
            }

            INumExpr[] cust_expr_window_violations = new INumExpr[m.customers.Count];
            for (int j = 0; j < m.customers.Count; j++)
            {
                cust_expr_window_violations[j] = cp.Sum(cp.Max(0, cp.Diff(cp.Constant((int)m.customers[j].windowStart), cp.StartOf(customer_interval_vars[j]))),
                                                        cp.Max(0, cp.Diff(cp.StartOf(customer_interval_vars[j]), cp.Constant((int)m.customers[j].windowEnd))));
                    
            }
            
            //Add Objective
            //IObjective obj = cp.Minimize(cp.Sum(cp.Sum(route_cost_expr), 
            //                                    cp.Prod(1000.0, cp.Sum(cust_expr_window_violations)),
            //                                    cp.Prod(1000.0, vehicle_capacity_expressions[m.vehicles - 1])));


            IObjective obj = cp.Minimize(cp.Sum(cp.Prod(1.0, cp.Sum(cust_expr_window_violations)),
                                                cp.Prod(10.0, vehicle_capacity_expressions[eff_vehicle_num - 1]),
                                                cp.Prod(10.0, vehicle_capacity_expressions[eff_vehicle_num - 2])));

            //IObjective obj = cp.Minimize(cp.Sum(route_cost_expr));

            //obj = cp.Minimize(cp.Sum(serviceStarts.ToArray())); //FORCE OBJECTIVE

            cp.Add(obj);
            cp.SetParameter(CP.IntParam.Workers, workerNum);
            cp.SetParameter(CP.IntParam.LogVerbosity, CP.ParameterValues.Terse);
            cp.SetParameter(CP.DoubleParam.TimeLimit, timeLimit);

            //cp.ExportModel("test_export.cpo");
            Solution? sol = null;

            //Set Search Phases
            ISearchPhase[] phases = new ISearchPhase[2];
            phases[0] = cp.SearchPhase(long_window_vars.ToArray());
            phases[1] = cp.SearchPhase(short_window_vars.ToArray());
            cp.SetSearchPhases(phases);

            if (cp.Solve())
            {
                Console.WriteLine($"CP Solution Cost: {cp.GetObjValue(0)}");
                sol = new Solution();
                //Populate Routes

                if (cp.GetValue(vehicle_capacity_expressions[eff_vehicle_num - 1]) > 0)
                {
                    Console.WriteLine("Extra vehicle still has load");
                    return null;
                }
                    
                if (cp.GetValue(vehicle_capacity_expressions[eff_vehicle_num - 2]) > 0)
                {
                    Console.WriteLine("Extra vehicle still has load");
                    return null;
                }
                
                for (int i = 0; i < m.vehicles; i++)
                {
                    Route rt = new Route();
                    rt.cost = cp.GetValue(route_cost_expr[i]);
                    Console.WriteLine($"Route {i} Cost from CP : {rt.cost}");

                    Dictionary<int, float> service_starts = new();
                    Dictionary<int, float> dist_starts = new();
                    for (int j = 0; j < m.customers.Count; j++)
                    {
                        IIntervalVar var = cp.GetIIntervalVar("veh_" + i + "_cust_" + j + "_intvar");

                        if (cp.IsPresent(var))
                        {
                            Node nn = new Node()
                            {
                                id = m.customers[j].id,
                                serviceTime = 0,
                                windowStart = m.customers[j].windowStart,
                                windowEnd = m.customers[j].windowEnd,
                                x = m.customers[j].x,
                                y = m.customers[j].y,
                                demand = m.customers[j].demand,
                            };
                            service_starts[nn.id] = cp.GetStart(var);

                            rt.sequence.Add(nn);
                        }
                    }

                    //Sort sequence based on service starts
                    rt.sequence.Sort((Node n1, Node n2) =>
                    {
                        return service_starts[n1.id].CompareTo(service_starts[n2.id]);
                    });

                    //Add Depot nodes per route
                    Node depot_start = m.depots[i, 0];
                    Node depot_end = m.depots[i, 1];
                    rt.sequence.Insert(0, depot_start);
                    rt.sequence.Add(depot_end);

                    //Try to recalculate waiting and arrival times from scratch

                    double current_time = 0.0;
                    double current_distance = 0.0;
                    for (int j = 1; j < rt.sequence.Count; j++)
                    {
                        Node prev_node = rt.sequence[j - 1];
                        Node current_node = rt.sequence[j];

                        //Calculate arrival time at current_node
                        current_time += m.distances[prev_node.id, current_node.id];
                        current_distance += m.distances[prev_node.id, current_node.id];

                        current_node.arrivalTime = current_time;

                        System.Diagnostics.Debug.Assert(current_time <= current_node.windowEnd);

                        //Clamp to window start of current if needed
                        current_time = Math.Max(current_time, current_node.windowStart);
                        current_node.waitingTime = Math.Max(0, current_time - current_node.arrivalTime);
                        //current_time += current_node.serviceTime;
                        
                        if (j < rt.sequence.Count - 1)
                        {
                            //    rt.cost += current_node.arrivalTime + current_node.waitingTime;
                            //Cumulative Distance Tests
                            rt.cost += current_distance;
                        }
                        
                        rt.load += current_node.demand;

                        //Report
                        Console.WriteLine($"Arrived at Node {current_node.id} ({current_node.windowStart}, {current_node.windowEnd}) at {current_node.arrivalTime}, travelled for {m.distances[prev_node.id, current_node.id]} waited for {current_node.waitingTime}");
                    }
                    
                    rt.cost = sol.ComputeRouteCumulativeDistances(rt);
                    
                    Console.WriteLine($"Using {rt.load}/{m.capacity}. Route Cost: {rt.cost}");
                    System.Diagnostics.Debug.Assert(rt.load <= m.capacity);
                    sol.cost += rt.cost;
                    sol.routes.Add(rt);
                }
                
                Console.WriteLine($"Real Solution Cost: {sol.cost}");
            }

            return sol;
        }

        public static Solution? SolveVRPTW_Alternative(Model m, float timeLimit = 120.0f, int workerNum = 1)
        {
            CP cp = new CP();

            double distance_prec = 1.0;

            //Define Integer variables for customer - vehicle assignments
            List<IIntVar> customer_vehicle_vars = new();
            for (int i = 0; i < m.customers.Count; i++)
                customer_vehicle_vars.Add(cp.IntVar(1, m.vehicles));

            //Add dummy variables for depot visits 
            for (int i = 0; i < m.vehicles; i++)
            {
                customer_vehicle_vars.Add(cp.IntVar(i + 1, i + 1)); //Depot Start
                customer_vehicle_vars.Add(cp.IntVar(i + 1, i + 1)); //Depot End
            }

            //Define integer variables for customer successors on their vehicle
            List<IIntVar> customer_vehicle_pred_vars = new();
            for (int i = 0; i < m.customers.Count; i++)
                customer_vehicle_pred_vars.Add(cp.IntVar(0, m.customers.Count + 2 * m.vehicles - 1));
            for (int i = 0; i < m.vehicles; i++)
            {
                customer_vehicle_pred_vars.Add(cp.IntVar(m.customers.Count + 2 * i + 1, m.customers.Count + 2 * i + 1));
                customer_vehicle_pred_vars.Add(cp.IntVar(0, m.customers.Count + 2 * m.vehicles - 1));
            }

            //Define integer variables for vehicle visit times
            List<IIntVar> customer_visit_vars = new();
            for (int i = 0; i < m.customers.Count; i++)
                customer_visit_vars.Add(cp.IntVar((int) (m.customers[i].windowStart * distance_prec), (int) (m.customers[i].windowEnd * distance_prec)));
            
            for (int i = 0; i < m.vehicles; i++)
            {
                customer_visit_vars.Add(cp.IntVar(0, 0));
                customer_visit_vars.Add(cp.IntVar(0, (int) (m.depot.windowEnd * distance_prec)));
            }

            //Define integer variables for vehicle visit capacity
            List<IIntVar> customer_visit_capacity_vars = new();
            for (int i = 0; i < m.customers.Count; i++)
                customer_visit_capacity_vars.Add(cp.IntVar(0, m.capacity));
            for (int i = 0; i < m.vehicles; i++)
            {
                customer_visit_capacity_vars.Add(cp.IntVar(0, 0));
                customer_visit_capacity_vars.Add(cp.IntVar(0, m.capacity));
            }

            //Prepare TransitionDistance Matrix
            double[,] d = m.ComputeDistances();
            int[][] int_d_mat = new int[m.customers.Count + 2 * m.vehicles][];
            int[] int_service_times = new int[m.customers.Count + 2 * m.vehicles];
            for (int i = 0; i < m.customers.Count; i++)
            {
                int_service_times[i] = (int) (m.customers[i].serviceTime * distance_prec);
            }

            for (int i = 1; i < m.nodes.Count; i++)
            {
                int_d_mat[i - 1] = new int[m.customers.Count + 2 * m.vehicles];
                for (int j = 1; j < m.nodes.Count; j++)
                {
                    int_d_mat[i - 1][j - 1] = (int)Math.Ceiling(d[i, j] * distance_prec);
                }

                for (int j = 0; j < 2 * m.vehicles; j++)
                {
                    int_d_mat[i - 1][m.customers.Count + j] = (int) Math.Ceiling(d[i, 0] * distance_prec);
                }
            }

            //Fill symmetrical values for distance from the depot to any customer
            for (int j = 0; j < 2 * m.vehicles; j++)
            {
                int_d_mat[m.customers.Count + j] = new int[m.customers.Count + 2 * m.vehicles];
                for (int i = 0; i < m.customers.Count; i++)
                {
                    int_d_mat[m.customers.Count + j][i] = int_d_mat[i][m.customers.Count + j];
                }
            }


            ITransitionDistance td = cp.TransitionDistance(int_d_mat, "td");

            //CONSTRAINTS

            //Make sure all node relations are unique
            cp.Add(cp.AllDiff(customer_vehicle_pred_vars.ToArray()));

            //Make sure node and its pred belong to the same route
            for (int i = 0; i < m.customers.Count + 2 * m.vehicles; i++)
            {
                cp.Add(cp.Eq(customer_vehicle_vars[i],
                             cp.Element(customer_vehicle_vars.ToArray(), customer_vehicle_pred_vars[i])));
            }

            //Calculate node visit times

            for (int i = 0; i < m.customers.Count; i++)
            {
                cp.Add(cp.Ge(customer_visit_vars[i],
                             cp.Sum(cp.Element(customer_visit_vars.ToArray(), customer_vehicle_pred_vars[i]),
                                    cp.Element(int_d_mat[i], customer_vehicle_pred_vars[i]),
                                    cp.Element(int_service_times, customer_vehicle_pred_vars[i]) ) ));
            }

            //Same constraints should be applied to the last depot visits
            for (int i = 0; i < m.vehicles; i++)
            {
                int index = m.customers.Count + 2 * i + 1;
                cp.Add(cp.Ge(customer_visit_vars[index],
                             cp.Sum(cp.Element(customer_visit_vars.ToArray(), customer_vehicle_pred_vars[index]),
                                    cp.Element(int_d_mat[index], customer_vehicle_pred_vars[index]),
                                    cp.Element(int_service_times, customer_vehicle_pred_vars[index])) ));
            }
            
            //Calculate vehicle capacities at visit
            for (int i = 0; i < m.customers.Count; i++)
            {
                cp.Add(cp.Eq(customer_visit_capacity_vars[i],
                             cp.Sum(cp.Element(customer_visit_capacity_vars.ToArray(), customer_vehicle_pred_vars[i]),
                                    cp.Constant(m.customers[i].demand))));
            }

            //Same constraints should be applied to the last depot visits
            for (int i = 0; i < m.vehicles; i++)
            {
                int index = m.customers.Count + 2 * i + 1;
                cp.Add(cp.Eq(customer_visit_capacity_vars[index],
                             cp.Element(customer_visit_capacity_vars.ToArray(), customer_vehicle_pred_vars[index])));
            }

            INumExpr[] cust_expr = new INumExpr[m.customers.Count + m.vehicles];
            //Add two depot interval variables for each vehicle
            for (int i = 0; i < m.customers.Count; i++)
                cust_expr[i] = cp.Element(int_d_mat[i], customer_vehicle_pred_vars[i]);
            for (int i = 0; i < m.vehicles; i++)
            {
                int index = m.customers.Count + 2 * i + 1;
                cust_expr[m.customers.Count + i] = cp.Element(int_d_mat[index], customer_vehicle_pred_vars[index]);
            }

            //Add Objective
            List<IIntVar> depotVisits = new();
            for (int i = 0; i < m.vehicles; i++)
            {
                int index = m.customers.Count + 2 * i + 1;
                depotVisits.Add(customer_visit_vars[index]);
            }
            IObjective obj = cp.Minimize(cp.Sum(cp.Sum(cust_expr.ToArray()), 
                                                cp.Prod(1000.0 * distance_prec, customer_visit_capacity_vars[m.customers.Count + 2 * (m.vehicles - 1) + 1])));

            cp.Add(obj);
            cp.SetParameter(CP.IntParam.Workers, workerNum);
            cp.SetParameter(CP.IntParam.LogVerbosity, CP.ParameterValues.Terse);
            cp.SetParameter(CP.DoubleParam.TimeLimit, timeLimit);

            cp.ExportModel("test_export.cpo");
            Solution? sol = null;
            if (cp.Solve())
            {
                Console.WriteLine($"CP Solution Cost: {cp.GetObjValue(0)}");
                sol = new Solution();

                //Report Solution
                for (int i = 0; i < m.vehicles; i++)
                {
                    Console.WriteLine($"Route {i + 1}");
                    for (int j = 0; j < m.customers.Count + 2 * m.vehicles; j++)
                    {
                        int veh_id = (int)cp.GetValue(customer_vehicle_vars[j]);
                        if (veh_id == i + 1)
                            Console.WriteLine($"Node {j} Vehicle {veh_id} " +
                                $"Pred {cp.GetValue(customer_vehicle_pred_vars[j])} " +
                                $"Veh Load {cp.GetValue(customer_visit_capacity_vars[j])} " +
                                $"Visit Time {cp.GetValue(customer_visit_vars[j])} ");
                    }
                }

                for (int i = 0; i < m.vehicles; i++)
                {
                    Route rt = new Route();
                    
                    //Add dummy nodes

                    //Add Depot nodes per route
                    Node depot_start = new()
                    {
                        id = m.depot.id,
                        serviceTime = 0,
                        windowStart = m.depot.windowStart,
                        windowEnd = m.depot.windowEnd,
                        isDepot = true,
                        waitingTime = 0,
                        arrivalTime = 0,
                        x = m.depot.x,
                        y = m.depot.y,
                        demand = m.depot.demand
                    };

                    Node depot_end = new()
                    {
                        id = m.depot.id,
                        serviceTime = 0,
                        windowStart = m.depot.windowStart,
                        windowEnd = m.depot.windowEnd,
                        isDepot = true,
                        x = m.depot.x,
                        y = m.depot.y,
                        demand = m.depot.demand
                    };

                    rt.sequence.Add(depot_start);
                    rt.sequence.Add(depot_end);

                    IIntVar last_depot_visit_var = customer_visit_vars[m.customers.Count + 2 * i + 1];
                    rt.cost = cp.GetValue(last_depot_visit_var) / 1.0;
                    
                    Console.WriteLine($"Route {i} Cost from CP : {rt.cost}");

                    Dictionary<int, float> service_starts = new();
                    Dictionary<int, float> dist_starts = new();

                    int visited_customer = (int) cp.GetValue(customer_vehicle_pred_vars[m.customers.Count + 2 * i + 1]);
                    while (visited_customer != m.customers.Count + 2 * i)
                    {
                        Node nn = new Node()
                        {
                            id = m.customers[visited_customer].id,
                            serviceTime = 0,
                            windowStart = m.customers[visited_customer].windowStart,
                            windowEnd = m.customers[visited_customer].windowEnd,
                            x = m.customers[visited_customer].x,
                            y = m.customers[visited_customer].y,
                            demand = m.customers[visited_customer].demand,
                        };
                        
                        rt.sequence.Insert(1, nn);
                        visited_customer = (int)cp.GetValue(customer_vehicle_pred_vars[visited_customer]);
                    }

                    //Try to recalculate waiting and arrival times from scratch

                    double current_time = 0.0;
                    double current_distance = 0.0;
                    for (int j = 1; j < rt.sequence.Count; j++)
                    {
                        Node prev_node = rt.sequence[j - 1];
                        Node current_node = rt.sequence[j];

                        //Calculate arrival time at current_node
                        current_time += d[prev_node.id, current_node.id];
                        current_distance += d[prev_node.id, current_node.id];

                        current_node.arrivalTime = current_time;

                        //System.Diagnostics.Debug.Assert(current_time <= current_node.windowEnd);

                        //Clamp to window start of current if needed
                        current_time = Math.Max(current_time, current_node.windowStart);
                        current_node.waitingTime = Math.Max(0, current_time - current_node.arrivalTime);
                        current_time += current_node.serviceTime;

                        if (j < rt.sequence.Count - 1)
                            rt.cost += current_node.arrivalTime + current_node.waitingTime;

                        rt.load += current_node.demand;

                        //Report
                        Console.WriteLine($"Arrived at Node {current_node.id} ({current_node.windowStart}, {current_node.windowEnd}) at {current_node.arrivalTime}, travelled for {d[prev_node.id, current_node.id]} waited for {current_node.waitingTime}");
                    }

                    rt.cost = current_distance;

                    Console.WriteLine($"Using {rt.load}/{m.capacity}. Route Cost: {rt.cost}");
                    System.Diagnostics.Debug.Assert(rt.load <= m.capacity);
                    sol.cost += rt.cost;
                    sol.routes.Add(rt);
                }

            }

            return sol;
        }


        public static Solution? SolveCumulativeVRPTW(Model m, float timeLimit = 120.0f, int workerNum = 1)
        {
            double distance_precision = 10000.0;
            CP cp = new CP();

            //Generate Inverval variables for all customers
            List<IIntervalVar> customer_interval_vars = new();
            for (int i = 0; i < m.customers.Count; i++)
            {
                customer_interval_vars.Add(cp.IntervalVar());
            }

            //Generate Sequence Variables for all vehicles and Init
            List<IIntervalVar>[] vehicle_customer_vars = new List<IIntervalVar>[m.vehicles];
            List<IIntervalVar>[] vehicle_customer_vars_dist = new List<IIntervalVar>[m.vehicles];
            for (int i = 0; i < m.vehicles; i++)
            {
                vehicle_customer_vars[i] = new();
                vehicle_customer_vars_dist[i] = new();
            }

            //Generate Cumulative FUnction expressions to hold vehicle capacities
            IIntExpr[] vehicle_capacity_expressions = new IIntExpr[m.vehicles];

            for (int j = 0; j < m.customers.Count; j++)
            {
                Node cust = m.customers[j];

                //Add customer to each vehicle and also tie with the main interval
                //variable of the customer using alternative global constraint

                List<IIntervalVar> options = new();
                List<IIntervalVar> distoptions = new();
                for (int i = 0; i < m.vehicles; i++)
                {
                    IIntervalVar customer_vehicle_var = cp.IntervalVar((int) (cust.serviceTime * distance_precision), "veh_" + i + "_cust_" + j + "_intvar");
                    IIntervalVar customer_vehicle_var_dist = cp.IntervalVar(0, "dist_veh_" + i + "_cust_" + j + "_intvar");
                    
                    customer_vehicle_var.SetOptional();
                    customer_vehicle_var_dist.SetOptional();
                    vehicle_customer_vars[i].Add(customer_vehicle_var);
                    vehicle_customer_vars_dist[i].Add(customer_vehicle_var_dist);

                    customer_vehicle_var.StartMin = (int)(cust.windowStart * distance_precision);
                    customer_vehicle_var.StartMax = (int)(cust.windowEnd * distance_precision);

                    options.Add(customer_vehicle_var);
                    distoptions.Add(customer_vehicle_var_dist);

                    cp.Add(cp.Eq(cp.PresenceOf(customer_vehicle_var_dist), cp.PresenceOf(customer_vehicle_var)));
                }

                //Add alternative constraint
                cp.Add(cp.Alternative(customer_interval_vars[j], options.ToArray()));
            }

            //Vehicle capacity constraints
            for (int i = 0; i < m.vehicles; i++)
            {
                IIntExpr[] cust_expr = new IIntExpr[m.customers.Count];
                for (int j = 0; j < m.customers.Count; j++)
                    cust_expr[j] = cp.Prod(cp.PresenceOf(vehicle_customer_vars[i][j]), cp.Constant(m.customers[j].demand));
                vehicle_capacity_expressions[i] = cp.Sum(cust_expr);
                cp.Add(cp.Ge(m.capacity, vehicle_capacity_expressions[i]));
            }

            //Set variable types equal to the node ids
            int[] ntypes = new int[m.customers.Count + 2];
            for (int i = 0; i < m.customers.Count; i++)
                ntypes[i + 1] = m.customers[i].id;
            ntypes[0] = 0;
            ntypes[m.customers.Count + 1] = 0;


            //Prepare TransitionDistance Matrix
            double[,] d = m.ComputeDistances();
            int[][] int_d_mat = new int[m.nodes.Count][];
            int[][] int_d_mat_prec = new int[m.nodes.Count][];
            for (int i = 0; i < m.nodes.Count; i++)
            {
                int_d_mat[i] = new int[m.nodes.Count];
                int_d_mat_prec[i] = new int[m.nodes.Count];
                for (int j = 0; j < m.nodes.Count; j++)
                {
                    int_d_mat[i][j] = (int)Math.Ceiling(d[i, j]);
                    int_d_mat_prec[i][j] = (int) Math.Ceiling(d[i, j] * distance_precision);
                }
            }

            ITransitionDistance td = cp.TransitionDistance(int_d_mat, "td");
            ITransitionDistance td_pred = cp.TransitionDistance(int_d_mat_prec, "td");

            INumExpr[] route_cost_expr = new INumExpr[m.vehicles];

            //Add two depot interval variables for each vehicle
            for (int i = 0; i < m.vehicles; i++)
            {
                IIntervalVar depot_start = cp.IntervalVar(0);
                IIntervalVar dstart = cp.IntervalVar(0);
                vehicle_customer_vars[i].Insert(0, depot_start);
                vehicle_customer_vars_dist[i].Insert(0, dstart);
                IIntervalVar depot_end = cp.IntervalVar(0);
                IIntervalVar dend = cp.IntervalVar(0);
                depot_end.EndMax = (int) (m.depot.windowEnd * distance_precision);
                vehicle_customer_vars[i].Add(depot_end);
                vehicle_customer_vars_dist[i].Add(dend);
                
                IIntervalSequenceVar var = cp.IntervalSequenceVar(vehicle_customer_vars[i].ToArray(), ntypes, "vehicle_seq_" + i);
                IIntervalSequenceVar dvar = cp.IntervalSequenceVar(vehicle_customer_vars_dist[i].ToArray(), ntypes, "dist_vehicle_seq_" + i);

                cp.Add(cp.First(var, depot_start));
                cp.Add(cp.Last(var, depot_end));

                //Add no overlap constraint
                cp.Add(cp.NoOverlap(var, td_pred));
                cp.Add(cp.NoOverlap(dvar, td_pred));

                cp.Add(cp.SameSequence(var, dvar));
                
                //Alternative way of calculating route costs
                INumExpr[] cust_expr = new INumExpr[m.customers.Count];

                for (int j = 0; j < m.customers.Count; j++)
                    cust_expr[j] = cp.Prod(cp.StartOf(vehicle_customer_vars_dist[i][j + 1]), cp.PresenceOf(vehicle_customer_vars_dist[i][j + 1]));
                //route_cost_expr[i] = cp.Prod(1000.0, cp.Sum(cust_expr));
                route_cost_expr[i] = cp.Sum(cust_expr);
            }

            IObjective obj = cp.Minimize(cp.Sum(route_cost_expr)); //FORCE OBJECTIVE

            cp.Add(obj);
            cp.SetParameter(CP.IntParam.Workers, workerNum);
            cp.SetParameter(CP.IntParam.LogVerbosity, CP.ParameterValues.Terse);
            cp.SetParameter(CP.DoubleParam.TimeLimit, timeLimit);

            cp.ExportModel("test_export.cpo");
            Solution? sol = null;
            if (cp.Solve())
            {
                Console.WriteLine($"CP Solution Cost: {cp.GetObjValue(0) / distance_precision}");
                sol = new Solution();
                //Populate Routes

                for (int i = 0; i < m.vehicles; i++)
                {
                    Route rt = new Route();
                    
                    Dictionary<int, float> service_starts = new();
                    Dictionary<int, float> dist_starts = new();
                    for (int j = 0; j < m.customers.Count; j++)
                    {
                        IIntervalVar var = cp.GetIIntervalVar("veh_" + i + "_cust_" + j + "_intvar");

                        if (cp.IsPresent(var))
                        {
                            Node nn = new Node()
                            {
                                id = m.customers[j].id,
                                serviceTime = m.customers[j].serviceTime,
                                windowStart = m.customers[j].windowStart,
                                windowEnd = m.customers[j].windowEnd,
                                x = m.customers[j].x,
                                y = m.customers[j].y,
                                demand = m.customers[j].demand,
                            };
                            service_starts[nn.id] = cp.GetStart(var);
                            rt.sequence.Add(nn);
                        }
                    }

                    //Sort sequence based on service starts
                    rt.sequence.Sort((Node n1, Node n2) =>
                    {
                        return service_starts[n1.id].CompareTo(service_starts[n2.id]);
                    });

                    //Add Depot nodes per route
                    Node depot_start = new()
                    {
                        id = m.depot.id,
                        serviceTime = 0,
                        windowStart = m.depot.windowStart,
                        windowEnd = m.depot.windowEnd,
                        isDepot = true,
                        waitingTime = 0,
                        arrivalTime = 0,
                        x = m.depot.x,
                        y = m.depot.y,
                        demand = m.depot.demand
                    };

                    Node depot_end = new()
                    {
                        id = m.depot.id,
                        serviceTime = 0,
                        windowStart = m.depot.windowStart,
                        windowEnd = m.depot.windowEnd,
                        isDepot = true,
                        x = m.depot.x,
                        y = m.depot.y,
                        demand = m.depot.demand
                    };

                    rt.sequence.Insert(0, depot_start);
                    rt.sequence.Add(depot_end);


                    //Try to recalculate waiting and arrival times from scratch

                    double current_time = 0.0;
                    double current_distance = 0.0;
                    for (int j = 1; j < rt.sequence.Count; j++)
                    {
                        Node prev_node = rt.sequence[j - 1];
                        Node current_node = rt.sequence[j];

                        //Calculate arrival time at current_node
                        current_time += d[prev_node.id, current_node.id];
                        current_distance += (rt.sequence.Count - 1 - j) * d[prev_node.id, current_node.id];

                        current_node.arrivalTime = current_time;

                        System.Diagnostics.Debug.Assert(current_time <= current_node.windowEnd);

                        //Clamp to window start of current if needed
                        current_time = Math.Max(current_time, current_node.windowStart);
                        current_node.waitingTime = Math.Max(0, current_time - current_node.arrivalTime);
                        current_time += current_node.serviceTime;

                        if (j < rt.sequence.Count - 1)
                            rt.cost += current_node.arrivalTime + current_node.waitingTime;

                        rt.load += current_node.demand;

                        //Report
                        Console.WriteLine($"Arrived at Node {current_node.id} ({current_node.windowStart}, {current_node.windowEnd}) at {current_node.arrivalTime}, travelled for {d[prev_node.id, current_node.id]} waited for {current_node.waitingTime}");
                    }

                    rt.cost = current_distance;

                    Console.WriteLine($"Using {rt.load}/{m.capacity}. Route Cost: {rt.cost}");
                    System.Diagnostics.Debug.Assert(rt.load <= m.capacity);
                    sol.cost += rt.cost;
                    sol.routes.Add(rt);
                }


                Console.WriteLine($"Real Solution Cost: {sol.cost}");
            }

            return sol;
        }


        public static Solution? SolveCumulativePartialVRPTW(Solution input_sol, int[] route_ids, float timeLimit = 120.0f, int workerNum = 1)
        {
            double distance_precision = 10000.0;
            CP cp = new CP();

            Model m = ProblemConfiguration.model;

            List<Node> _customers = new();
            int route_num = route_ids.Length;
            
            //Generate Inverval variables for all customers
            List<IIntervalVar> customer_interval_vars = new();
            for (int i=0;i<route_ids.Length; i++)
                for (int j = 1; j < input_sol.routes[route_ids[i]].sequence.Count - 1; j++)
                    _customers.Add(input_sol.routes[route_ids[i]].sequence[j]);
            
            //Add Univisited
            foreach (Node n in input_sol.unvisited)
                _customers.Add(n);
            
            for (int i = 0; i < _customers.Count; i++)
                customer_interval_vars.Add(cp.IntervalVar());

            //Generate Sequence Variables for all vehicles and Init
            List<IIntervalVar>[] vehicle_customer_vars = new List<IIntervalVar>[route_num];
            List<IIntervalVar>[] vehicle_customer_vars_dist = new List<IIntervalVar>[route_num];
            for (int i = 0; i < route_num; i++)
            {
                vehicle_customer_vars[i] = new();
                vehicle_customer_vars_dist[i] = new();
            }

            //Generate Cumulative Function expressions to hold vehicle capacities
            IIntExpr[] vehicle_capacity_expressions = new IIntExpr[route_num];

            for (int j = 0; j < _customers.Count; j++)
            {
                Node cust = _customers[j];

                //Add customer to each vehicle and also tie with the main interval
                //variable of the customer using alternative global constraint

                List<IIntervalVar> options = new();
                List<IIntervalVar> distoptions = new();
                for (int i = 0; i < route_num; i++)
                {
                    IIntervalVar customer_vehicle_var = cp.IntervalVar((int)(cust.serviceTime * distance_precision), "veh_" + i + "_cust_" + j + "_intvar");
                    IIntervalVar customer_vehicle_var_dist = cp.IntervalVar(0, "dist_veh_" + i + "_cust_" + j + "_intvar");

                    customer_vehicle_var.SetOptional();
                    customer_vehicle_var_dist.SetOptional();
                    vehicle_customer_vars[i].Add(customer_vehicle_var);
                    vehicle_customer_vars_dist[i].Add(customer_vehicle_var_dist);

                    customer_vehicle_var.StartMin = (int)(cust.windowStart * distance_precision);
                    customer_vehicle_var.StartMax = (int)(cust.windowEnd * distance_precision);

                    options.Add(customer_vehicle_var);
                    distoptions.Add(customer_vehicle_var_dist);

                    cp.Add(cp.Eq(cp.PresenceOf(customer_vehicle_var_dist), cp.PresenceOf(customer_vehicle_var)));
                }

                //Add alternative constraint
                cp.Add(cp.Alternative(customer_interval_vars[j], options.ToArray()));
            }

            //Vehicle capacity constraints
            for (int i = 0; i < route_num; i++)
            {
                IIntExpr[] cust_expr = new IIntExpr[_customers.Count];
                for (int j = 0; j < _customers.Count; j++)
                    cust_expr[j] = cp.Prod(cp.PresenceOf(vehicle_customer_vars[i][j]), cp.Constant(_customers[j].demand));
                vehicle_capacity_expressions[i] = cp.Sum(cust_expr);
                cp.Add(cp.Ge(m.capacity, vehicle_capacity_expressions[i]));
            }

            //Set variable types equal to the node ids
            int[] ntypes = new int[_customers.Count + 2];
            for (int i = 0; i < _customers.Count; i++)
                ntypes[i + 1] = _customers[i].id;
            ntypes[0] = 0;
            ntypes[_customers.Count + 1] = 0;


            //Prepare TransitionDistance Matrix
            double[,] d = m.ComputeDistances();
            int[][] int_d_mat = new int[m.nodes.Count][];
            int[][] int_d_mat_prec = new int[m.nodes.Count][];
            for (int i = 0; i < m.nodes.Count; i++)
            {
                int_d_mat[i] = new int[m.nodes.Count];
                int_d_mat_prec[i] = new int[m.nodes.Count];
                for (int j = 0; j < m.nodes.Count; j++)
                {
                    int_d_mat[i][j] = (int)Math.Ceiling(d[i, j]);
                    int_d_mat_prec[i][j] = (int)Math.Ceiling(d[i, j] * distance_precision);
                }
            }

            ITransitionDistance td = cp.TransitionDistance(int_d_mat, "td");
            ITransitionDistance td_pred = cp.TransitionDistance(int_d_mat_prec, "td");

            INumExpr[] route_cost_expr = new INumExpr[route_num];

            //Add two depot interval variables for each vehicle
            for (int i = 0; i < route_num; i++)
            {
                IIntervalVar depot_start = cp.IntervalVar(0);
                IIntervalVar dstart = cp.IntervalVar(0);
                vehicle_customer_vars[i].Insert(0, depot_start);
                vehicle_customer_vars_dist[i].Insert(0, dstart);
                IIntervalVar depot_end = cp.IntervalVar(0);
                IIntervalVar dend = cp.IntervalVar(0);
                depot_end.EndMax = (int)(m.depot.windowEnd * distance_precision);
                vehicle_customer_vars[i].Add(depot_end);
                vehicle_customer_vars_dist[i].Add(dend);

                IIntervalSequenceVar var = cp.IntervalSequenceVar(vehicle_customer_vars[i].ToArray(), ntypes, "vehicle_seq_" + i);
                IIntervalSequenceVar dvar = cp.IntervalSequenceVar(vehicle_customer_vars_dist[i].ToArray(), ntypes, "dist_vehicle_seq_" + i);

                cp.Add(cp.First(var, depot_start));
                cp.Add(cp.Last(var, depot_end));

                //Add no overlap constraint
                cp.Add(cp.NoOverlap(var, td_pred));
                cp.Add(cp.NoOverlap(dvar, td_pred));
                //cp.Add(cp.NoOverlap(var));
                //cp.Add(cp.NoOverlap(dvar));

                cp.Add(cp.SameSequence(var, dvar));

                //Alternative way of calculating route costs
                INumExpr[] cust_expr = new INumExpr[_customers.Count];

                for (int j = 0; j < _customers.Count; j++)
                    cust_expr[j] = cp.Prod(cp.StartOf(vehicle_customer_vars_dist[i][j + 1]), cp.PresenceOf(vehicle_customer_vars_dist[i][j + 1]));
                //route_cost_expr[i] = cp.Prod(1000.0, cp.Sum(cust_expr));
                route_cost_expr[i] = cp.Sum(cust_expr);
            }

            IObjective obj = cp.Minimize(cp.Sum(route_cost_expr)); //FORCE OBJECTIVE

            cp.Add(obj);
            cp.SetParameter(CP.IntParam.Workers, workerNum);
            cp.SetParameter(CP.IntParam.LogVerbosity, CP.ParameterValues.Terse);
            cp.SetParameter(CP.DoubleParam.TimeLimit, timeLimit);

            cp.ExportModel("test_export.cpo");
            Solution? sol = null;
            if (cp.Solve())
            {
                Console.WriteLine($"CP Solution Cost: {cp.GetObjValue(0) / distance_precision}");
                sol = new Solution();
                //Populate Routes

                for (int i = 0; i < m.vehicles; i++)
                {
                    Route rt = new Route();

                    Dictionary<int, float> service_starts = new();
                    Dictionary<int, float> dist_starts = new();
                    for (int j = 0; j < m.customers.Count; j++)
                    {
                        IIntervalVar var = cp.GetIIntervalVar("veh_" + i + "_cust_" + j + "_intvar");

                        if (cp.IsPresent(var))
                        {
                            Node nn = new Node()
                            {
                                id = m.customers[j].id,
                                serviceTime = m.customers[j].serviceTime,
                                windowStart = m.customers[j].windowStart,
                                windowEnd = m.customers[j].windowEnd,
                                x = m.customers[j].x,
                                y = m.customers[j].y,
                                demand = m.customers[j].demand,
                            };
                            service_starts[nn.id] = cp.GetStart(var);
                            rt.sequence.Add(nn);
                        }
                    }

                    //Sort sequence based on service starts
                    rt.sequence.Sort((Node n1, Node n2) =>
                    {
                        return service_starts[n1.id].CompareTo(service_starts[n2.id]);
                    });

                    //Add Depot nodes per route
                    Node depot_start = new()
                    {
                        id = m.depot.id,
                        serviceTime = 0,
                        windowStart = m.depot.windowStart,
                        windowEnd = m.depot.windowEnd,
                        isDepot = true,
                        waitingTime = 0,
                        arrivalTime = 0,
                        x = m.depot.x,
                        y = m.depot.y,
                        demand = m.depot.demand
                    };

                    Node depot_end = new()
                    {
                        id = m.depot.id,
                        serviceTime = 0,
                        windowStart = m.depot.windowStart,
                        windowEnd = m.depot.windowEnd,
                        isDepot = true,
                        x = m.depot.x,
                        y = m.depot.y,
                        demand = m.depot.demand
                    };

                    rt.sequence.Insert(0, depot_start);
                    rt.sequence.Add(depot_end);


                    //Try to recalculate waiting and arrival times from scratch

                    double current_time = 0.0;
                    double current_distance = 0.0;
                    for (int j = 1; j < rt.sequence.Count; j++)
                    {
                        Node prev_node = rt.sequence[j - 1];
                        Node current_node = rt.sequence[j];

                        //Calculate arrival time at current_node
                        current_time += d[prev_node.id, current_node.id];
                        current_distance += (rt.sequence.Count - 1 - j) * d[prev_node.id, current_node.id];

                        current_node.arrivalTime = current_time;

                        System.Diagnostics.Debug.Assert(current_time <= current_node.windowEnd);

                        //Clamp to window start of current if needed
                        current_time = Math.Max(current_time, current_node.windowStart);
                        current_node.waitingTime = Math.Max(0, current_time - current_node.arrivalTime);
                        current_time += current_node.serviceTime;

                        if (j < rt.sequence.Count - 1)
                            rt.cost += current_node.arrivalTime + current_node.waitingTime;

                        rt.load += current_node.demand;

                        //Report
                        Console.WriteLine($"Arrived at Node {current_node.id} ({current_node.windowStart}, {current_node.windowEnd}) at {current_node.arrivalTime}, travelled for {d[prev_node.id, current_node.id]} waited for {current_node.waitingTime}");
                    }

                    rt.cost = current_distance;

                    Console.WriteLine($"Using {rt.load}/{m.capacity}. Route Cost: {rt.cost}");
                    System.Diagnostics.Debug.Assert(rt.load <= m.capacity);
                    sol.cost += rt.cost;
                    sol.routes.Add(rt);
                }


                Console.WriteLine($"Real Solution Cost: {sol.cost}");
            }

            return sol;
        }


    }

}
