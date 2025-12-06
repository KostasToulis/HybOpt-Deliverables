using System;
using System.Collections.Generic;
using Google.OrTools.ConstraintSolver;
using Google.Protobuf.WellKnownTypes;

namespace CCVRPTW
{
    public class ORToolsSolver
    {
        
        public static Solution SolveVRPTW_Partial(Solution input_sol, int[] route_ids)
        {
            List<Node> _nodes = new();
            _nodes.Add(ProblemConfiguration.model.depot);
            int route_num = route_ids.Length;
            
            //Generate Inverval variables for all customers
            for (int i = 0; i < route_ids.Length; i++)
                for (int j = 1; j < input_sol.routes[route_ids[i]].sequence.Count - 1; j++)
                    _nodes.Add(input_sol.routes[route_ids[i]].sequence[j]);

            //Add Univisited
            foreach (Node n in input_sol.unvisited)
                _nodes.Add(n);

            //Create Routing Mamanger
            RoutingIndexManager manager = new RoutingIndexManager(_nodes.Count, route_num, 0);

            //Create Routing Model
            RoutingModel routingModel = new RoutingModel(manager);


            int transitCallbackIndex = routingModel.RegisterTransitCallback((long fromIndex, long toIndex) =>
            {
                // Convert from routing variable Index to time
                // matrix NodeIndex.
                int fromNode = manager.IndexToNode(fromIndex);
                int toNode = manager.IndexToNode(toIndex);
                Node a = _nodes[fromNode];
                Node b = _nodes[toNode];

                return (long) (ProblemConfiguration.model.distances[a.id, b.id] * 10);
            });

            // Define cost of each arc.
            routingModel.SetArcCostEvaluatorOfAllVehicles(transitCallbackIndex);


            int demandCallbackIndex = routingModel.RegisterUnaryTransitCallback((long fromIndex) =>
            {
                // Convert from routing variable Index to
                // demand NodeIndex.
                int fromNode = manager.IndexToNode(fromIndex);
                return _nodes[fromNode].demand;
            });

            long[] vehicle_capacities = new long[route_num];
            for (int i = 0; i < route_num; i++)
                vehicle_capacities[i] = ProblemConfiguration.model.capacity;

            routingModel.AddDimensionWithVehicleCapacity(demandCallbackIndex, 0, // null capacity slack
                                                    vehicle_capacities, // vehicle maximum capacities
                                                    true,                   // start cumul to zero
                                                    "Capacity");


            // Add Time constraint.
            routingModel.AddDimension(transitCallbackIndex,           // transit callback
                                 30,                                  // allow waiting time
                                 ProblemConfiguration.model.capacity, // vehicle maximum capacities
                                 false,                               // start cumul to zero
                                 "Time");
            RoutingDimension timeDimension = routingModel.GetMutableDimension("Time");

            // Add time window constraints for each location except depot.
            for (int i = 1; i < _nodes.Count; ++i)
            {
                long index = manager.NodeToIndex(i);
                Node n = _nodes[i];
                long ws = (long) (0 * n.windowStart);
                long we = (long) (10 * n.windowEnd);
                
                timeDimension.CumulVar(index).SetRange(ws, we);
            }
            
            // Add time window constraints for each vehicle start node.
            for (int i = 0; i < route_num; ++i)
            {
                long index = routingModel.Start(i);
                timeDimension.CumulVar(index).SetRange(0, 30000);
            }

            // Instantiate route start and end times to produce feasible times.
            for (int i = 0; i < route_num; ++i)
            {
                routingModel.AddVariableMinimizedByFinalizer(timeDimension.CumulVar(routingModel.Start(i)));
                routingModel.AddVariableMinimizedByFinalizer(timeDimension.CumulVar(routingModel.End(i)));
            }

            RoutingSearchParameters searchParameters =
            operations_research_constraint_solver.DefaultRoutingSearchParameters();
            searchParameters.FirstSolutionStrategy = FirstSolutionStrategy.Types.Value.PathCheapestArc;
            searchParameters.LocalSearchMetaheuristic = LocalSearchMetaheuristic.Types.Value.Automatic;
            searchParameters.TimeLimit = new Duration { Seconds = 600 };
            searchParameters.LogSearch = true;
            
            Assignment solution = routingModel.SolveWithParameters(searchParameters);
            Console.WriteLine($"Objective {solution.ObjectiveValue()}:");
            Solution sol = new Solution();
            return sol;
        }





    }
}
