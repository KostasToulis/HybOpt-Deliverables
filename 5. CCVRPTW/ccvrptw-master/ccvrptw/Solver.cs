using ccvrptw;
using ScottPlot.Control.EventProcess;
using System.Collections;
using System.Collections.Generic;
using System.Reflection.Metadata;

namespace CCVRPTW { 
    public class Arc
    {
        public Node firstNode;
        public Node secondNode;

        public Arc(Node firstNode, Node secondNode)
        {
            this.firstNode = firstNode;
            this.secondNode = secondNode;
        }
    }
    class Solver
    {
        public int seed;
        public static Random random;
        public List<Node> nodes;
        public List<Node> customers;
        public int vehicles;
        public int capacity;
        public Solution solution;
        public double[,] promisesMatrix;
        public FeasibleSegment[,] segments;
        public List<Arc> toBeCreated;
        public double[,] timeWindowCompatibilityMatrix;
        public int[] customersLeftOut;
        private HashSet<int> blacklist;
        public int promisesLastImprovedLimit;
        public int promisesRestart;
        public float promisesPrecision;
        public List<double> costProgression;
        public int rclSize;
        public int restarts;
        public long elapsedTime;
        public int bestRestart;
        public int totalIterations;
        
        public Solver(int r)
        {
            seed = (int) (DateTime.Now.ToBinary() & 0xFFFFFFFF);
            random = new Random(seed);
            //random = new Random(0x17);

            nodes = ProblemConfiguration.model.nodes;
            customers = ProblemConfiguration.model.customers;
            vehicles = ProblemConfiguration.model.vehicles;
            capacity = ProblemConfiguration.model.capacity;
            solution = new Solution();
            promisesMatrix = new double[nodes.Count, nodes.Count];
            segments = new FeasibleSegment[nodes.Count + ProblemConfiguration.model.depots.Length, nodes.Count + ProblemConfiguration.model.depots.Length];

            //Initialize Segments
            for (int i=0;i< nodes.Count + ProblemConfiguration.model.depots.Length; i++)
                for (int j=0; j< nodes.Count + ProblemConfiguration.model.depots.Length;j++)
                    segments[i,j] = new FeasibleSegment();
            
            toBeCreated = new List<Arc>();
            timeWindowCompatibilityMatrix = new double[nodes.Count, nodes.Count];
            promisesLastImprovedLimit = 30000;
            promisesRestart = 600;
            promisesPrecision = 0.1f; //0.1f
            costProgression = new List<double>();
            rclSize = 10;
            restarts = r;
            bestRestart = -1;
            totalIterations = 0;
            customersLeftOut = new int[nodes.Count];
            blacklist = new HashSet<int>();
        }

        // h methodologia pou akolouthoume gia th lush tou provlhmatos
        // me thn proseggish multiple restarts.
        public void SolveWithRestarts(Objective objective)
        {
            var watch = new System.Diagnostics.Stopwatch();
            // initiliaze Promises
            createPromisesMatrix();
            blacklist.Clear();
            // initialize TWC
            createTimeWindowCompatibilityMatrix();
            Solution bestSolutionFromRestarts = new Solution();
            bestSolutionFromRestarts.cost = Math.Pow(10, 12);
            watch.Start();
            // gia kathe restart pou theloume na treksoume
            for (int j = 0; j < restarts; j++)
            {
                costProgression = new List<double>();
                clearNodes();
                solution = new Solution();
                // pare to initial solution
                Construct(objective);
                Console.WriteLine(solution);

                if (!solution.CheckSolution(objective))
                    throw new Exception("Invalid Solution");

                //CPLEXSolver.SolveCumulativePartialVRPTW(solution, new int[]{ 1, 2, 3});

                // mpes sto local search
                PromisesImprove(objective);
                // an h kaluterh lush pou vrhkes se auto to restart einai kaluterh
                // apo thn best pou exeis vrei mexri stigmhs, tote update best.
                
                //Warning: CheckSolution will fail when swapping objectives is on and the LS failed to switch objectives
                if (!solution.CheckSolution(objective))
                    throw new Exception("Invalid Solution");
                
                if (solution.cost < bestSolutionFromRestarts.cost)
                {
                    Console.WriteLine("New best cost found in restarts");
                    Console.WriteLine($"Old = {bestSolutionFromRestarts.cost} New = {solution.cost}");
                    // fulakse th best lush me diaforetika node instances apo ta main.
                    bestSolutionFromRestarts = new Solution(solution);
                    bestRestart = j;
                }
                
#if DEBUG
                solution.PlotCostProgression(costProgression, solution.cost % ProblemConfiguration.model.extraNodePenalty, solution.lastImprovedIteration, j);
#endif
            }
            watch.Stop();
            elapsedTime = watch.ElapsedMilliseconds / 1000 / restarts;
            solution = new Solution(bestSolutionFromRestarts);
            // kane elegxo egkurothtas sth lush pou vgalame
            if (!solution.CheckSolution(objective))
                throw new Exception("Invalid Solution");
            Console.WriteLine($"Cost = {solution.ComputeCumulativeDistances()}");
#if DEBUG
            Console.Write(solution);
#endif
            Console.WriteLine($"Execution time = {watch.ElapsedMilliseconds / (float)1000 / (float)restarts} s");

            solution.ExportSimple($"sol_{ProblemConfiguration.model.Name}_{seed}_.txt");

        }
        
        // katharise tis idiothtes twn main Node instances.
        // to theloume gia na ksekinaei ek neou se kathe restart.
        public void clearNodes()
        {
            foreach (Node n in nodes)
            {
                n.arrivalTime = 0;
                n.waitingTime = 0;
                n.pushForward = 0;
                n.isRouted = false;
            }
        }

        // initialize Promises
        public void createPromisesMatrix()
        {
            for (int i = 0; i < promisesMatrix.GetLength(0); i++)
            {
                for (int j = 0; j < promisesMatrix.GetLength(1); j++)
                {
                    promisesMatrix[i, j] = Math.Pow(10,9);
                }
            }
        }

        // initiliaze TWC
        public void createTimeWindowCompatibilityMatrix()
        {
            foreach (Node u in this.nodes)
            {
                foreach (Node v in this.nodes)
                {
                    double twc;
                    double distance = ProblemConfiguration.model.distances[u.id, v.id];
                    double arrVEarliest = u.windowStart + u.serviceTime + distance;
                    double arrVLatest = u.windowEnd + u.serviceTime + distance;
                    if (v.windowEnd - arrVEarliest >= 0)
                    {
                        twc = Math.Min(arrVLatest, v.windowEnd) - Math.Max(arrVEarliest, v.windowStart);
                    }
                    else
                    {
                        if (u == v)
                        {
                            // isos na prepei na ginei diaforetika 
                            twc = -Math.Pow(10, 5);
                        }
                        else
                        {
                            twc = -Math.Pow(10, 6);
                        }
                    }
                    timeWindowCompatibilityMatrix[u.id, v.id] = twc;
                }
            }
        }

        // h diadikasia local search
        public void PromisesImprove(Objective objective)
        {
            int iter = 0;
            Solution BestSol = new Solution(solution);
            Objective localObjective = objective;
            bool feasible_space_trigger = false;
            promisesRestart = 300;

            // mexri kapoio termination_condition = true
            while (true)
            {
                //Apply settings for feasible space
                //if (solution.unvisited.Count ==0 && !feasible_space_trigger)
                //{
                //    promisesRestart = 600;
                //    feasible_space_trigger = true;
                //}

                if (solution.unvisited.Count == 0 && localObjective != objective)
                {
                    Console.WriteLine($"SWITCHING TO MAIN OBJECTIVE: {objective}");
                    localObjective = objective;
                    if (objective == Objective.CUMULATIVE_DISTANCE)
                    {
                        solution.cost = solution.ComputeCumulativeDistances(true);
                    } else
                    {
                        throw new Exception($"Unsupported objective: {objective}");
                    }
                    
                    BestSol = new Solution(solution);
                    Console.WriteLine($"Iteration {iter} New Best Cost: {BestSol.cost}");
                    
                    createPromisesMatrix();
                    CreateSegments();

                }
                
                //Console.WriteLine($"Iteration {iter} Solution Cost: {solution.cost}");
                // pote kanoume restart ta promises.
                if ((iter - solution.lastImprovedIteration) % promisesRestart == 0)
                {
                    createPromisesMatrix();
                }
                // pote pianoume to limit twn consecutive non improving moves.
                if ((iter - solution.lastImprovedIteration) == promisesLastImprovedLimit)
                {
                    break;
                }
                
                
                // vres thn kaluterh kinish gia to trexon iteration.
                BestMove(localObjective);
                //solution.PrintSolution();
                // an h lush pou vrhkes sto trexon iteration einai h kaluterh
                // sth diadikasia local search, kane update th best.
                if (solution.cost < BestSol.cost - 0.001)
                {
                    // kane restart ta promises.
                    createPromisesMatrix();
                    BestSol = new Solution(solution);
                    // bres to megalytero keno metaxi dyo kalyteron lyseon
                    if (iter - solution.lastImprovedIteration > solution.biggestIterationsGap)
                    {
                        solution.biggestIterationsGap = iter - solution.lastImprovedIteration;
                    }
                    solution.lastImprovedIteration = iter;
                    Console.WriteLine($"Iteration {iter} New Best Cost: {BestSol.cost}" + (BestSol.IsFeasible() ? " **FEASIBLE**" : ""));
#if DEBUG
                    //solution.PlotRoutes(iter);
#endif
                }
                //Console.WriteLine($"Iteration {iter} Solution Cost: {solution.cost} Vehicle Count: {solution.routes.Count}/{model.vehicles}");

                iter++;

                costProgression.Add(solution.cost % ProblemConfiguration.model.extraNodePenalty);

                //Perturbation
                if ((iter - solution.lastImprovedIteration) % 10000 == 0 && iter > 0 && solution.unvisited.Count > 0)
                {
                    //ORToolSolver.SolveVRPTW_Partial(solution, route_ids);
                    //Solution? new_sol = CPLEXSolver.SolveCumulativePartialVRPTW(solution,
                    //    new int[10] { route_ids[0], route_ids[1], route_ids[2], route_ids[3], route_ids[4],
                    //                  route_ids[5], route_ids[6], route_ids[7], route_ids[8], route_ids[9]});
                    
                    //DestroySolutionTW(0.2f, localObjective, iter);
                    //createPromisesMatrix();
                }

                //if (solution.routes.Count == model.vehicles)
                //    solution.CheckSolution(objective);

                //Report Left Out Customers
                //if (iter % 500 == 0)
                //{
                //    for (int i = 0; i < nodes.Count; i++)
                //    {
                //        if (customersLeftOut[i] > 0)
                //        {
                //            Console.WriteLine($"Customer {i} left out {customersLeftOut[i]} / {iter} times. Time Window {nodes[i].windowStart} - {nodes[i].windowEnd} ");
                //        }
                //    }
                //    Console.WriteLine("--------------");
                //}

            }
            solution = new Solution(BestSol);
            totalIterations = iter;
        }


        public void DestroySolutionTW(float destruction_perc, Objective objective, int iterations)
        {
            Console.WriteLine($"TW Destruction Operator. Deconstruction Threshold {destruction_perc}");
            float[] customer_weights = new float[nodes.Count];
            
            for (int i = 0; i < customersLeftOut.Length; i++)
            {
                customer_weights[i] = ((float) customersLeftOut[i]) / iterations;
                Console.WriteLine($"Customer {i} Out of solution {customersLeftOut[i]} times.");
            }

            //Select one customer at random
            List<Node> oldUnvisited = new();
            foreach (Node n in solution.unvisited)
                oldUnvisited.Add(n);
            
            Node rand = solution.unvisited[random.Next(solution.unvisited.Count)];

            int NodesToRemoveCount = (int) Math.Floor(destruction_perc * customers.Count);
            List<Tuple<Node, int>> Candidates = new();

            //Select all customers that fall into rand's time window
            for (int i = 0; i < solution.routes.Count; i++)
            {
                Route rt = solution.routes[i];
                for (int j = 1; j < rt.sequence.Count - 1; j++)
                {
                    Node nd = rt.sequence[j];
                    if (blacklist.Contains(nd.id))
                        continue;
                    
                    double nd_service_start = nd.arrivalTime + nd.waitingTime;
                    if (nd.windowStart >= rand.windowStart && nd.windowStart <= rand.windowEnd)
                        Candidates.Add(new Tuple<Node, int>(nd, i));
                    else if (nd.windowEnd >= rand.windowStart && nd.windowEnd <= rand.windowEnd)
                        Candidates.Add(new Tuple<Node, int>(nd, i));
                    else if (nd.windowStart <= rand.windowStart && nd.windowEnd >= rand.windowEnd)
                        Candidates.Add(new Tuple<Node, int>(nd, i));
                }
            }

            //Shuffle List
            Candidates.OrderBy(item => random.Next());
            
            //Remove candidates from the routes
            for (int i = 0; i < Math.Min(NodesToRemoveCount, Candidates.Count); i++)
            {
                Node cand = Candidates[i].Item1;
                int route_id = Candidates[i].Item2;
                solution.routes[route_id].sequence.Remove(cand);
                solution.routes[route_id].load -= cand.demand;
                solution.unvisited.Add(cand);
            }

            //Update Route Node Info    
            for (int i = 0; i < solution.routes.Count; i++)
                solution.routes[i].UpdateRouteNodes();
            
            //Recalculate Route Objective and update solution objective
            if (objective == Objective.CUMULATIVE_DISTANCE)
                solution.cost = solution.ComputeCumulativeDistances(true);
            else if (objective == Objective.DISTANCE)
                solution.cost = solution.ComputeDistances(true);
            else
                throw new Exception("NOT SUPPORTED OBJECTIVE");


            //Sort unvisited based on the weight
            solution.unvisited.Sort( (Node x, Node y) => 
            {
                return customer_weights[y.id].CompareTo(customer_weights[x.id]);
            });

            ////Call Solution Constructor
            //List<int> route_ids = new();
            //for (int i = 0; i < solution.routes.Count; i++)
            //{
            //    route_ids.Add(i);
            //    Console.WriteLine($"Route {i} Push Forward Time {solution.routes[i].maxPushForward}");
            //}

            //route_ids.Sort((int a, int b) =>
            //{
            //    return solution.routes[b].maxPushForward.CompareTo(solution.routes[a].maxPushForward);
            //});

            //Solution? new_sol = CPLEXSolver.SolveCumulativePartialVRPTW(solution, route_ids.GetRange(0, 4).ToArray());

            BlindConstruct(objective);
            Console.WriteLine($"Reconstructed Solution Cost: {solution.cost}");

            //Clear customer stats
            for (int i = 0; i < customersLeftOut.Length; i++)
                customersLeftOut[i] = 0;

            //Set Blacklists
            blacklist.Clear();
            foreach (Node n in oldUnvisited)
            {
                if (!solution.unvisited.Contains(n))
                {
                    blacklist.Add(n.id);
                    Console.WriteLine($"Adding {n.id} to the blacklist");
                }
            }
        }

        public int findMinCostMove(Move[] moves, out double min_cost)
        {
            int id = 0;
            min_cost = Math.Pow(10, 9);
            for (int i = 0; i < moves.Length; i++)
            {
                if (moves[i].cost + moves[i].penalty < min_cost)
                {
                    id = i;
                    min_cost = moves[i].cost;
                }
            }
            
            return id;
        }
        
        // edw eksetazoume tis candidate moves apo kathe operator
        public void BestMove(Objective objective)
        {
            // ftiakse ta Segments gia thn trexousa morfh ths lushs.
            CreateSegments();
            Move[] moves = new Move[5];
            moves[0] = FindRelocate(objective, segments);
            moves[1] = FindSwap(objective, segments);
            moves[2] = FindTwoOpt(objective, segments);
            moves[3] = FindInsertion(objective, segments);
            moves[4] = FindReplace(objective, segments);

            int min_cost_move_id = findMinCostMove(moves, out double min_cost);
            
            // Beltiotiki kinisi
            if (min_cost < 0)
            {
                ApplyMove(moves[min_cost_move_id]);
            }
            // yparkti kinisi
            else if (min_cost < Math.Pow(10,9))
            {
                Move[] valid_moves = moves.Where(x => x.cost < Math.Pow(10, 9)).ToArray();
                Move random_move = valid_moves[random.Next(0, valid_moves.Length)];
                ApplyMove(random_move);
            }
            else
            {
                //Console.WriteLine("No Move Exists!");
            }


            if (!solution.CheckSolution(objective))
                Console.WriteLine("MALAKIA APPLICATION");

            //Report Customers outside of the solution

            for (int i=0; i < solution.unvisited.Count; i++)
            {
                Node nd = solution.unvisited[i];
                customersLeftOut[nd.id]++;
            }
        
        }


        public void ApplyMove(Move move)
        {
            Type t = move.GetType();
            //Console.WriteLine($"Applying Move {move.GetType()} with Cost: {move.cost}");

            if (t == typeof(Relocation))
                ApplyMove((Relocation)move);
            else if (t == typeof(Insertion))
                ApplyMove((Insertion)move);
            else if (t == typeof(Swap))
                ApplyMove((Swap)move);
            else if (t == typeof(TwoOptMove))
                ApplyMove((TwoOptMove)move);
            else if (t == typeof(Replace))
                ApplyMove((Replace)move);
        }

        public void ApplyMove(TwoOptMove topt)
        {
            //Console.WriteLine("Applying TWO OPT");
            ApplyTwoOptMove(topt);
        }
        public void ApplyMove(Relocation reloc)
        {
            //Console.WriteLine("Applying Relocation");
            ApplyRelocation(reloc);
        }
        public void ApplyMove(Insertion ins)
        {
            //Console.WriteLine("Applying Insertion");
            ApplyInsertion(ins);
        }
        public void ApplyMove(Swap swap)
        {
            //Console.WriteLine("Applying SWAP");
            ApplySwap(swap);
        }

        public void ApplyMove(Replace repl)
        {
            //Console.WriteLine("Applying Replace");
            ApplyReplace(repl);
        }

        // elegxei an h ulopoihsh mias kinhshs prepei na blockaristei me vash ta promises.
        // an to epipedo ths antikeimenikhs gia th lush pou tha prokupsei, den ikanopoiei
        // to promise estw kai enos apo ta arcs pou tha ftiaxtoun apo to move, tote h kinhsh block.
        public bool promisesBroken(double moveCost)
        {
            foreach (Arc arc in toBeCreated)
            {
                if (promisesMatrix[arc.firstNode.id, arc.secondNode.id] <= solution.cost + promisesPrecision + moveCost)
                {
                    return true;
                }
            }
            return false;
        }

        // dhmiourgei ta Segments gia thn trexousa morfh ths lushs.
        public void CreateSegments()
        {
            // Ta Segments einai stored se ena (n x n) pinaka (me n plhthos komvwn mazi me ta nea depots)
            // To stoixeio (i, j) exei mesa to Segment pou ksekina apo ton komvo i kai teleiwnei ston j.
            // Clear array.
            //for (int i=0; i < segments.GetLength(0); i++)
            //{
            //    for (int j = 0; j < segments.GetLength(1); j++)
            //    {
            //        segments[i, j] = null;
            //    }
            //}
               
            
            // Ftiaxnoume ta Segments pou prokuptoun apo thn trexousa morfh ths lushs.
            // Gia kathe dromologio sth lush.
            foreach (Route route in solution.routes)
            {
                // Ftiaxnoume ena SingleSegment gia kathe komvo tou dromologiou.
                for (int i = 0; i < route.sequence.Count; i++)
			    {
                    Node n = route.sequence[i];
                    FeasibleSegment.CreateSingleSegment(segments[n.id, n.id], n);
                }
                
                // Ftiaxnoume ta Segments gia kathe uposeira komvwn tou dromologiou.
                for (int i = 0; i < route.sequence.Count - 1; i++)
			    {
                    // Gia kathe uposeira komvwn me ekkinish ton komvo me index i sto dromologio.
                    Node u = route.sequence[i];
                    for (int j = i + 1; j < route.sequence.Count; j++)
			        {
                        Node v = route.sequence[j];
                        Node vPrevious = route.sequence[j - 1];
                        FeasibleSegment seg = segments[u.id, vPrevious.id];
                        double dxy = ProblemConfiguration.model.distances[vPrevious.id, v.id];
                        double twc = timeWindowCompatibilityMatrix[vPrevious.id, v.id];

                        FeasibleSegment.MergeSegments(segments[u.id, v.id], seg, segments[v.id, v.id], dxy, twc, this.capacity);
                        //if (!segments[u.id, v.id].feasibility)
                        //    Console.WriteLine("WARNING! Segments not feasible");
                    }
			    }
            }
            
        }

        // upologizei analutika to objetive Cumulative Service Times (Min Sum) gia ena dromologio.
        public double calculateMinSum(List<Node> seq)
        {
            double cumulativeServiceTimes = 0;
            for (int i = 1; i < seq.Count - 1; i++)
            {
                Node v = seq[i];
                cumulativeServiceTimes += (v.arrivalTime + v.waitingTime);
            }

            return cumulativeServiceTimes;
        }

        // to xrhsimopoioume otan kanoume mia kinhsh pou diaforopoiei
        // ena dromologio meta apo th thesh 'from_node'. Dhladh prpepei
        // na upologisoume tis allages stous xronous gia tous komvous meta apo auton.
        public void updateArrivalTimes(List<Node> seq, int fromNode)
        {
            for (int j = fromNode; j < seq.Count - 1; j++)
            {
                Node u = seq[j];
                Node v = seq[j + 1];
                v.arrivalTime = u.arrivalTime + u.waitingTime + u.serviceTime + ProblemConfiguration.model.distances[u.id, v.id];
                if (v.arrivalTime > v.windowEnd)
                {
                    v.waitingTime = v.windowEnd - v.arrivalTime;
                }
                else
                {
                    v.waitingTime = Math.Max(0, v.windowStart - v.arrivalTime);
                }
            }
        }

        public void BlindConstruct(Objective objective)
        {
            int u_id = 0;
            while (u_id < solution.unvisited.Count)
            {
                Node un = solution.unvisited[u_id];

                // segments for the routes
                CreateSegments();
                foreach (Node n in solution.unvisited)
                {
                    segments[n.id, n.id] = FeasibleSegment.SingleSegment(n);
                }
                
                List<Insertion> allPossibleInsertions = new List<Insertion>();
                
                for (int routeIndex = 0; routeIndex < ProblemConfiguration.model.vehicles; routeIndex++)
                {
                    Route route = solution.routes[routeIndex];
                    Node routeDepotStart = route.sequence[0];
                    Node routeDepotEnd = route.sequence[route.sequence.Count - 1];
                    FeasibleSegment routeSegment = segments[routeDepotStart.id, routeDepotEnd.id];
                    for (int i = 0; i < route.sequence.Count - 1; i++)
                    {
                        if (timeWindowCompatibilityMatrix[route.sequence[i].id, un.id] == -Math.Pow(10, 6)
                            || timeWindowCompatibilityMatrix[un.id, route.sequence[i + 1].id] == -Math.Pow(10, 6)) continue;
                        FeasibleSegment segForRoute = new FeasibleSegment(segments[route.sequence[0].id, route.sequence[i].id]); // depotStart to i
                        FeasibleSegment seg = segments[un.id, un.id]; // n
                        double dxy = ProblemConfiguration.model.distances[route.sequence[i].id, un.id];
                        double twc = timeWindowCompatibilityMatrix[route.sequence[i].id, un.id];
                        segForRoute = FeasibleSegment.MergeSegments(segForRoute, seg, dxy, twc, capacity); // add n
                        seg = segments[route.sequence[i + 1].id, route.sequence[route.sequence.Count - 1].id]; // i + 1 to depotEnd
                        dxy = ProblemConfiguration.model.distances[un.id, route.sequence[i + 1].id];
                        twc = timeWindowCompatibilityMatrix[un.id, route.sequence[i + 1].id];
                        segForRoute = FeasibleSegment.MergeSegments(segForRoute, seg, dxy, twc, capacity); // add i + 1 to depot

                        if (segForRoute.load > capacity) continue;

                        Insertion insertion = new Insertion();
                        insertion.penalty = -ProblemConfiguration.model.extraNodePenalty;
                        if (objective == Objective.CUMULATIVE_DISTANCE) // cumulative distances
                        {
                            double routeCumDistBefore = routeSegment.cumulativeDistances;
                            double routeCumDistAfter = segForRoute.cumulativeDistances;
                            double moveCost = routeCumDistAfter - routeCumDistBefore;
                            insertion.StoreMove(un, routeIndex, i, moveCost);
                        }
                        else if (objective == Objective.CUMULATIVE_SERVICE_TIMES) // min sum begin service time
                        {
                            double routeMinSumBefore = route.cost;

                            // make a copy of route sequence
                            // List<Node> newSequence = new List<Node>(route.sequence); changes the node arrival and waiting times
                            List<Node> newSequence = new List<Node>();
                            foreach (Node v in route.sequence)
                            {
                                newSequence.Add(new Node(v));
                            }
                            newSequence.Insert(i + 1, un);

                            updateArrivalTimes(newSequence, i);
                            double routeMinSumAfter = this.calculateMinSum(newSequence);

                            double moveCost = routeMinSumAfter - routeMinSumBefore;
                            insertion.StoreMove(un, routeIndex, i, moveCost);

                        }
                        else if (objective == Objective.DISTANCE)
                        {
                            double routeTotalDistBefore = routeSegment.distances;
                            double routeTotalDistAfter = segForRoute.distances;
                            double moveCost = routeTotalDistAfter - routeTotalDistBefore;
                            insertion.StoreMove(un, routeIndex, i, moveCost);
                        }

                        allPossibleInsertions.Add(insertion);
                    }
                }


                if (allPossibleInsertions.Count != 0)
                {
                    //Sort moves based on min cost
                    allPossibleInsertions.Sort((Insertion A, Insertion B) =>
                    {
                        double cost_a = A.cost + A.penalty;
                        double cost_b = B.cost + B.penalty;
                        return cost_a.CompareTo(cost_b);
                    });

                    int randomInsertionIndex = random.Next(0, Math.Min(allPossibleInsertions.Count, rclSize));
                    Insertion insertionToApply = allPossibleInsertions[randomInsertionIndex];

                    ApplyInsertion(insertionToApply);
                }
                else
                    u_id++;
            }

            
            for (int i = 0; i < solution.routes.Count; i++)
            {
                solution.routes[i].UpdateRouteNodes();
                Console.WriteLine($"Route {i} Slack Time {solution.routes[i].totalSlack}");
            }

            //solution.cost += solution.unvisited.Count * model.extraNodePenalty;
        }


        public void Construct(Objective objective)
        {
            while (solution.unvisited.Count > 0)
            {
                // segments for the routes
                CreateSegments();
                foreach (Node n in solution.unvisited)
                {
                    segments[n.id, n.id] = FeasibleSegment.SingleSegment(n);
                }
                List<Insertion> allPossibleInsertions = new List<Insertion>();
                foreach (Node n in solution.unvisited)
                {
                    for (int routeIndex = 0; routeIndex < ProblemConfiguration.model.vehicles; routeIndex++)
                    {
                        Route route = solution.routes[routeIndex];
                        Node routeDepotStart = route.sequence[0];
                        Node routeDepotEnd = route.sequence[route.sequence.Count - 1];
                        FeasibleSegment routeSegment = segments[routeDepotStart.id, routeDepotEnd.id];
                        for (int i = 0; i < route.sequence.Count - 1; i++)
                        {
                            //if (routeIndex == 10 && n.id == 64 && i == 1 && solution.cost < 1510000)
                            //{
                            //    Console.WriteLine("TEST");
                            //}

                            //if (timeWindowCompatibilityMatrix[route.sequence[i].id, n.id] == -Math.Pow(10, 6)
                            //    || timeWindowCompatibilityMatrix[n.id, route.sequence[i + 1].id] == -Math.Pow(10, 6)) continue;
                            FeasibleSegment segForRoute = new FeasibleSegment(segments[route.sequence[0].id, route.sequence[i].id]); // depotStart to i
                            FeasibleSegment seg = segments[n.id, n.id]; // n
                            double dxy = ProblemConfiguration.model.distances[route.sequence[i].id, n.id];
                            double twc = timeWindowCompatibilityMatrix[route.sequence[i].id, n.id];
                            segForRoute = FeasibleSegment.MergeSegments(segForRoute, seg, dxy, twc, capacity); // add n
                            seg = segments[route.sequence[i + 1].id, route.sequence[route.sequence.Count - 1].id]; // i + 1 to depotEnd
                            dxy = ProblemConfiguration.model.distances[n.id, route.sequence[i + 1].id];
                            twc = timeWindowCompatibilityMatrix[n.id, route.sequence[i + 1].id];
                            segForRoute = FeasibleSegment.MergeSegments(segForRoute, seg, dxy, twc, capacity); // add i + 1 to depot

                            if (segForRoute.load > capacity) continue;

                            Insertion insertion = new Insertion();
                            insertion.penalty = -ProblemConfiguration.model.extraNodePenalty;
                            if (objective == Objective.CUMULATIVE_DISTANCE) // cumulative distances
                            {
                                double routeCumDistBefore = routeSegment.cumulativeDistances + FeasibleSegment.timeWindowPenaltyWeight * routeSegment.timeWindowPenalty;
                                double routeCumDistAfter = segForRoute.cumulativeDistances + FeasibleSegment.timeWindowPenaltyWeight * segForRoute.timeWindowPenalty;
                                double moveCost = routeCumDistAfter - routeCumDistBefore;

                                insertion.StoreMove(n, routeIndex, i, moveCost);
                            }
                            else if (objective == Objective.CUMULATIVE_SERVICE_TIMES) 
                            {
                                double routeCumServiceBefore = route.cost;

                                // make a copy of route sequence
                                // List<Node> newSequence = new List<Node>(route.sequence); changes the node arrival and waiting times
                                List<Node> newSequence = new List<Node>();
                                foreach (Node v in route.sequence)
                                {
                                    newSequence.Add(new Node(v));
                                }
                                newSequence.Insert(i + 1, n);

                                updateArrivalTimes(newSequence, i);

                                double routeCumServiceAfter = calculateMinSum(newSequence) + FeasibleSegment.timeWindowPenaltyWeight * segForRoute.timeWindowPenalty;

                                double moveCost = routeCumServiceAfter - routeCumServiceBefore;
                                insertion.StoreMove(n, routeIndex, i, moveCost);

                            }
                            else if (objective == Objective.DISTANCE)
                            {
                                double routeTotalDistBefore = routeSegment.distances + FeasibleSegment.timeWindowPenaltyWeight * routeSegment.timeWindowPenalty;
                                double routeTotalDistAfter = segForRoute.distances + FeasibleSegment.timeWindowPenaltyWeight * segForRoute.timeWindowPenalty;
                                double moveCost = routeTotalDistAfter - routeTotalDistBefore;
                                insertion.StoreMove(n, routeIndex, i, moveCost);
                            }

                            allPossibleInsertions.Add(insertion);
                        }
                    }
                }
                
                if (allPossibleInsertions.Count == 0)
                {
                    break;
                } else
                {
                    //Sort moves based on min cost
                    allPossibleInsertions.Sort((Insertion A, Insertion B) =>
                    {
                        double cost_a = A.cost + A.penalty;
                        double cost_b = B.cost + B.penalty;
                        return cost_a.CompareTo(cost_b);
                    });

                    int randomInsertionIndex = random.Next(0, Math.Min(allPossibleInsertions.Count, rclSize));
                    Insertion insertionToApply = allPossibleInsertions[randomInsertionIndex];

                    //Console.WriteLine(solution);
                    ApplyInsertion(insertionToApply);

                    //Console.WriteLine(solution);

                    solution.CheckSolution(objective);

                }
            }



            for (int i = 0; i < solution.routes.Count; i++)
            {
                solution.routes[i].UpdateRouteNodes();
                Console.WriteLine($"Route {i} Slack Time {solution.routes[i].totalSlack}");
            }

            //solution.cost += solution.unvisited.Count * model.extraNodePenalty;
        }
        
        public Relocation FindRelocate(Objective objective, FeasibleSegment[,] segments)
        {
            Relocation relocation = new Relocation();
            FeasibleSegment segForSourceRoute = new();
            FeasibleSegment segForTargetRoute = new();
            List<Node> originRouteSequence = new List<Node>();
            List<Node> targetRouteSequence = new List<Node>();
            
            for (int originRouteIndex = 0; originRouteIndex < ProblemConfiguration.model.vehicles; originRouteIndex++)
            {
                Route originRoute = solution.routes[originRouteIndex];
                
                for (int targetRouteIndex = 0; targetRouteIndex < ProblemConfiguration.model.vehicles; targetRouteIndex++)
                {
                    Route targetRoute = solution.routes[targetRouteIndex];
                    for (int nodeIndex = 1; nodeIndex < originRoute.sequence.Count - 1; nodeIndex++)
                    {
                        for (int relocationIndex = 0; relocationIndex < targetRoute.sequence.Count - 1; relocationIndex++)
                        {
                            Node nPrevious = originRoute.sequence[nodeIndex - 1];
                            Node n = originRoute.sequence[nodeIndex];
                            Node nNext = originRoute.sequence[nodeIndex + 1];
                            Node v = targetRoute.sequence[relocationIndex];
                            Node vNext = targetRoute.sequence[relocationIndex + 1];


                            //if (timeWindowCompatibilityMatrix[v.id, n.id] == -Math.Pow(10, 6)
                            //   || timeWindowCompatibilityMatrix[n.id, vNext.id] == -Math.Pow(10, 6)) continue;

                            double movePenalty = 0.0;
                                                  
                            if (originRouteIndex == targetRouteIndex) // an to relocation afora to idio route
                            {
                                if (nodeIndex == relocationIndex || nodeIndex - 1 == relocationIndex) continue;
                                FeasibleSegment seg1;
                                double dxy;
                                double twc;
                                if (nodeIndex < relocationIndex)
                                {
                                    // apo depot mexri nPrevious
                                    segForSourceRoute.From(segments[originRoute.sequence[0].id, nPrevious.id]);
                                    // apo nNext mexri v
                                    seg1 = segments[nNext.id, v.id];
                                    dxy = ProblemConfiguration.model.distances[nPrevious.id, nNext.id];
                                    twc = timeWindowCompatibilityMatrix[nPrevious.id, nNext.id];
                                    FeasibleSegment.MergeSegments(segForSourceRoute, segForSourceRoute, seg1, dxy, twc, capacity);
                                    // o customer n se segment
                                    seg1 = segments[n.id, n.id];
                                    dxy = ProblemConfiguration.model.distances[v.id, n.id];
                                    twc = timeWindowCompatibilityMatrix[v.id, n.id];
                                    FeasibleSegment.MergeSegments(segForSourceRoute, segForSourceRoute, seg1, dxy, twc, capacity);
                                    // apo vNext mexri depot
                                    seg1 = segments[vNext.id, originRoute.sequence[originRoute.sequence.Count - 1].id];
                                    dxy = ProblemConfiguration.model.distances[n.id, vNext.id];
                                    twc = timeWindowCompatibilityMatrix[n.id, vNext.id];
                                    FeasibleSegment.MergeSegments(segForSourceRoute, segForSourceRoute, seg1, dxy, twc, capacity);
                                }
                                else // nodeIndex > relocationIndex
                                {
                                    // apo depot mexri v
                                    segForSourceRoute.From(segments[originRoute.sequence[0].id, v.id]);
                                    // o customer n se segment
                                    seg1 = segments[n.id, n.id];
                                    dxy = ProblemConfiguration.model.distances[v.id, n.id];
                                    twc = timeWindowCompatibilityMatrix[v.id, n.id];
                                    FeasibleSegment.MergeSegments(segForSourceRoute, segForSourceRoute, seg1, dxy, twc, capacity);
                                    // apo ton vNext ston nPrevious
                                    seg1 = segments[vNext.id, nPrevious.id];
                                    dxy = ProblemConfiguration.model.distances[n.id, vNext.id];
                                    twc = timeWindowCompatibilityMatrix[n.id, vNext.id];
                                    FeasibleSegment.MergeSegments(segForSourceRoute, segForSourceRoute, seg1, dxy, twc, capacity);
                                    // apo nNext mexri depot
                                    seg1 = segments[nNext.id, originRoute.sequence[originRoute.sequence.Count - 1].id];
                                    dxy = ProblemConfiguration.model.distances[nPrevious.id, nNext.id];
                                    twc = timeWindowCompatibilityMatrix[nPrevious.id, nNext.id];
                                    FeasibleSegment.MergeSegments(segForSourceRoute, segForSourceRoute, seg1, dxy, twc, capacity);
                                }
                                if (segForSourceRoute.load > capacity) continue;
                                double routeCostBefore = 0.0;
                                double routeCostAfter = 0.0;
                                
                                if (objective == Objective.CUMULATIVE_DISTANCE)
                                {
                                    routeCostBefore = originRoute.cost;
                                    routeCostAfter = segForSourceRoute.cumulativeDistances + FeasibleSegment.timeWindowPenaltyWeight * segForSourceRoute.timeWindowPenalty;
                                }
                                else if (objective == Objective.CUMULATIVE_SERVICE_TIMES)
                                {
                                    routeCostBefore = originRoute.cost;

                                    originRouteSequence.Clear();
                                    foreach (Node x in originRoute.sequence)
                                    {
                                        originRouteSequence.Add(new Node(x));
                                    }
                                    Node relNode = originRouteSequence[nodeIndex];
                                    originRouteSequence.Remove(relNode);// TODO: na to kanei meso index
                                    if (nodeIndex < relocationIndex)
                                    {
                                        originRouteSequence.Insert(relocationIndex, relNode);
                                    }
                                    else
                                    {
                                        originRouteSequence.Insert(relocationIndex + 1, relNode);
                                    }
                                    updateArrivalTimes(originRouteSequence, Math.Min(nodeIndex - 1, relocationIndex));

                                    routeCostAfter = calculateMinSum(originRouteSequence) + FeasibleSegment.timeWindowPenaltyWeight * segForSourceRoute.timeWindowPenalty;
                                }
                                else if (objective == Objective.DISTANCE)
                                {
                                    routeCostBefore = originRoute.cost;
                                    routeCostAfter = segForSourceRoute.distances + FeasibleSegment.timeWindowPenaltyWeight * segForSourceRoute.timeWindowPenalty;
                                }

                                double moveCost = routeCostAfter - routeCostBefore;
                                toBeCreated.Clear();
                                toBeCreated.Add(new Arc(nPrevious, nNext));
                                toBeCreated.Add(new Arc(v, n));
                                toBeCreated.Add(new Arc(n, vNext));
                                if (promisesBroken(moveCost)) continue;
                                if (moveCost >= relocation.cost) continue;
                                relocation.storeMove(originRouteIndex, targetRouteIndex, nodeIndex, relocationIndex, moveCost, moveCost, moveCost, movePenalty);
                            }
                            else // to relocation afora diaforetika routes
                            {
                                segForSourceRoute.From(segments[originRoute.sequence[0].id, nPrevious.id]);

                                FeasibleSegment seg2 = segments[nNext.id, originRoute.sequence[originRoute.sequence.Count - 1].id];
                                double dyx = ProblemConfiguration.model.distances[nPrevious.id, nNext.id];
                                double twc = timeWindowCompatibilityMatrix[nPrevious.id, nNext.id];
                                FeasibleSegment.MergeSegments(segForSourceRoute, segForSourceRoute, seg2, dyx, twc, capacity);

                                segForTargetRoute.From(segments[targetRoute.sequence[0].id, v.id]);
                                
                                dyx = ProblemConfiguration.model.distances[v.id, n.id];
                                twc = timeWindowCompatibilityMatrix[v.id, n.id];
                                FeasibleSegment.MergeSegments(segForTargetRoute, segForTargetRoute, segments[n.id, n.id], dyx, twc, capacity);
                                dyx = ProblemConfiguration.model.distances[n.id, vNext.id];
                                twc = timeWindowCompatibilityMatrix[n.id, vNext.id];
                                seg2 = segments[vNext.id, targetRoute.sequence[targetRoute.sequence.Count - 1].id];
                                FeasibleSegment.MergeSegments(segForTargetRoute, segForTargetRoute, seg2, dyx, twc, capacity);

                                if (segForSourceRoute.load > capacity) continue;
                                if (segForTargetRoute.load > capacity) continue;
                                double originRouteCostBefore, originRouteCostAfter, originRouteCostDiff = 0;
                                double targetRouteCostBefore, targetRouteCostAfter, targetRouteCostDiff = 0;

                                double moveCost = 0;
                                if (objective == Objective.CUMULATIVE_DISTANCE)
                                {
                                    originRouteCostBefore = originRoute.cost;
                                    originRouteCostAfter = segForSourceRoute.cumulativeDistances + FeasibleSegment.timeWindowPenaltyWeight * segForSourceRoute.timeWindowPenalty;
                                    originRouteCostDiff = originRouteCostAfter - originRouteCostBefore;
                                    targetRouteCostBefore = targetRoute.cost;
                                    targetRouteCostAfter = segForTargetRoute.cumulativeDistances + FeasibleSegment.timeWindowPenaltyWeight * segForTargetRoute.timeWindowPenalty;
                                    targetRouteCostDiff = targetRouteCostAfter - targetRouteCostBefore;
                                    moveCost = originRouteCostDiff + targetRouteCostDiff;
                                }
                                else if (objective == Objective.CUMULATIVE_SERVICE_TIMES)
                                {
                                    originRouteCostBefore = originRoute.cost;
                                    originRouteSequence.Clear();
                                    targetRouteSequence.Clear();
                                    foreach (Node x in originRoute.sequence)
                                        originRouteSequence.Add(new Node(x));
                                    Node relNode = originRouteSequence[nodeIndex];
                                    originRouteSequence.Remove(relNode);
                                    updateArrivalTimes(originRouteSequence, nodeIndex - 1);

                                    originRouteCostAfter = calculateMinSum(originRouteSequence) + FeasibleSegment.timeWindowPenaltyWeight * segForSourceRoute.timeWindowPenalty;
                                    originRouteCostDiff = originRouteCostAfter - originRouteCostBefore;

                                    targetRouteCostBefore = targetRoute.cost;
                                    foreach (Node x in targetRoute.sequence)
                                        targetRouteSequence.Add(new Node(x));
                                    targetRouteSequence.Insert(relocationIndex + 1, relNode);
                                    updateArrivalTimes(targetRouteSequence, relocationIndex);

                                    targetRouteCostAfter = calculateMinSum(targetRouteSequence) + FeasibleSegment.timeWindowPenaltyWeight * segForTargetRoute.timeWindowPenalty;
                                    targetRouteCostDiff = targetRouteCostAfter - targetRouteCostBefore;
                                    moveCost = originRouteCostDiff + targetRouteCostDiff;

                                }
                                else if (objective == Objective.DISTANCE)
                                {
                                    originRouteCostBefore = originRoute.cost;
                                    originRouteCostAfter = segForSourceRoute.distances + FeasibleSegment.timeWindowPenaltyWeight * segForSourceRoute.timeWindowPenalty;
                                    originRouteCostDiff = originRouteCostAfter - originRouteCostBefore;
                                    targetRouteCostBefore = targetRoute.cost;
                                    targetRouteCostAfter = segForTargetRoute.distances + FeasibleSegment.timeWindowPenaltyWeight * segForTargetRoute.timeWindowPenalty;
                                    targetRouteCostDiff = targetRouteCostAfter - targetRouteCostBefore;
                                    moveCost = originRouteCostDiff + targetRouteCostDiff;
                                }

                                toBeCreated.Clear();
                                toBeCreated.Add(new Arc(nPrevious, nNext));
                                toBeCreated.Add(new Arc(v, n));
                                toBeCreated.Add(new Arc(n, vNext));
                                if (promisesBroken(moveCost)) continue;
                                if (moveCost >= relocation.cost) continue;
                                relocation.storeMove(originRouteIndex, targetRouteIndex, nodeIndex, relocationIndex, moveCost, originRouteCostDiff, targetRouteCostDiff, 0.0);
                            }
                        }
                    }
                }
            }
            return relocation;
        }

        public Insertion FindInsertion(Objective objective, FeasibleSegment[,] segments)
        {
            Insertion insertion = new();
            insertion.penalty = -ProblemConfiguration.model.extraNodePenalty;
            
            foreach (Node n in solution.unvisited)
            {
                for (int routeIndex = 0; routeIndex < ProblemConfiguration.model.vehicles; routeIndex++)
                {
                    Route route = solution.routes[routeIndex];
                    Node routeDepotStart = route.sequence[0];
                    Node routeDepotEnd = route.sequence[route.sequence.Count - 1];
                    FeasibleSegment routeSegment = segments[routeDepotStart.id, routeDepotEnd.id];
                    for (int i = 0; i < route.sequence.Count - 1; i++)
                    {
                        //if (timeWindowCompatibilityMatrix[route.sequence[i].id, n.id] == -Math.Pow(10, 6)
                        //    || timeWindowCompatibilityMatrix[n.id, route.sequence[i + 1].id] == -Math.Pow(10, 6)) continue;

                        FeasibleSegment segForRoute = new FeasibleSegment(segments[route.sequence[0].id, route.sequence[i].id]); // depotStart to i
                        FeasibleSegment seg = segments[n.id, n.id]; // n
                        double dxy = ProblemConfiguration.model.distances[route.sequence[i].id, n.id];
                        double twc = timeWindowCompatibilityMatrix[route.sequence[i].id, n.id];
                        segForRoute = FeasibleSegment.MergeSegments(segForRoute, seg, dxy, twc, capacity); // add n
                        seg = segments[route.sequence[i + 1].id, route.sequence[route.sequence.Count - 1].id]; // i + 1 to depotEnd
                        dxy = ProblemConfiguration.model.distances[n.id, route.sequence[i + 1].id];
                        twc = timeWindowCompatibilityMatrix[n.id, route.sequence[i + 1].id];
                        segForRoute = FeasibleSegment.MergeSegments(segForRoute, seg, dxy, twc, capacity); // add i + 1 to depot

                        if (segForRoute.load > capacity) continue;

                        //Calculate move cost
                        double moveCost = 0.0;
                        
                        if (objective == Objective.CUMULATIVE_DISTANCE) // cumulative distances
                        {
                            double routeCumDistBefore = routeSegment.cumulativeDistances + FeasibleSegment.timeWindowPenaltyWeight * routeSegment.timeWindowPenalty;
                            double routeCumDistAfter = segForRoute.cumulativeDistances + FeasibleSegment.timeWindowPenaltyWeight * segForRoute.timeWindowPenalty;
                            moveCost += routeCumDistAfter - routeCumDistBefore;
                        }
                        else if (objective == Objective.CUMULATIVE_SERVICE_TIMES) // min sum begin service time
                        {
                            double routeMinSumBefore = route.cost;
                            
                            // make a copy of route sequence
                            // List<Node> newSequence = new List<Node>(route.sequence); changes the node arrival and waiting times
                            List<Node> newSequence = new List<Node>();
                            foreach (Node v in route.sequence)
                            {
                                newSequence.Add(new Node(v));
                            }
                            newSequence.Insert(i + 1, n);

                            updateArrivalTimes(newSequence, i);
                            double routeMinSumAfter = calculateMinSum(newSequence) + FeasibleSegment.timeWindowPenaltyWeight * segForRoute.timeWindowPenalty;

                            moveCost += routeMinSumAfter - routeMinSumBefore;
                        }
                        else if (objective == Objective.DISTANCE)
                        {
                            double routeTotalDistBefore = routeSegment.distances + FeasibleSegment.timeWindowPenaltyWeight * routeSegment.timeWindowPenalty; 
                            double routeTotalDistAfter = segForRoute.distances + FeasibleSegment.timeWindowPenaltyWeight * segForRoute.timeWindowPenalty;
                            moveCost += routeTotalDistAfter - routeTotalDistBefore;
                        }

                        if (moveCost >= insertion.cost) continue;
                        insertion.StoreMove(n, routeIndex, i, moveCost);
                    }
                }
            }

            return insertion;
        }

        public Replace FindReplace(Objective objective, FeasibleSegment[,] segments)
        {
            Replace repl = new Replace();
            foreach (Node n in solution.unvisited)
            {
                for (int targetRouteIndex = 0; targetRouteIndex < ProblemConfiguration.model.vehicles; targetRouteIndex++)
                {
                    Route targetRoute = solution.routes[targetRouteIndex];
                    
                    for (int secondNodeIndex = 1; secondNodeIndex < targetRoute.sequence.Count - 1; secondNodeIndex++)
                    {
                        Node vPrevious = targetRoute.sequence[secondNodeIndex - 1];
                        Node v = targetRoute.sequence[secondNodeIndex];
                        Node vNext = targetRoute.sequence[secondNodeIndex + 1];

                        if (blacklist.Contains(v.id))
                            continue;

                        //if (timeWindowCompatibilityMatrix[vPrevious.id, n.id] == -Math.Pow(10, 6)
                        //    || timeWindowCompatibilityMatrix[n.id, vNext.id] == -Math.Pow(10, 6)) continue;

                        FeasibleSegment segForTargetRoute = new FeasibleSegment(segments[targetRoute.sequence[0].id, vPrevious.id]); // depotStart to vPrevious
                        FeasibleSegment seg = segments[n.id, n.id]; // n
                        double dxy = ProblemConfiguration.model.distances[vPrevious.id, n.id];
                        double twc = timeWindowCompatibilityMatrix[vPrevious.id, n.id];
                        segForTargetRoute = FeasibleSegment.MergeSegments(segForTargetRoute, seg, dxy, twc, capacity); // add n
                        seg = segments[vNext.id, targetRoute.sequence[targetRoute.sequence.Count - 1].id]; // vNext to depotEnd
                        dxy = ProblemConfiguration.model.distances[n.id, vNext.id];
                        twc = timeWindowCompatibilityMatrix[n.id, vNext.id];
                        segForTargetRoute = FeasibleSegment.MergeSegments(segForTargetRoute, seg, dxy, twc, capacity); // add vNext to depotEnd

                        if (segForTargetRoute.load > capacity) continue;
                        double targetRouteCostBefore, targetRouteCostAfter, targetRouteCostDiff = 0;
                        double moveCost = 0;
                        if (objective == Objective.CUMULATIVE_DISTANCE)
                        {
                            targetRouteCostBefore = targetRoute.cost;
                            targetRouteCostAfter = segForTargetRoute.cumulativeDistances + FeasibleSegment.timeWindowPenaltyWeight * segForTargetRoute.timeWindowPenalty;
                            targetRouteCostDiff = targetRouteCostAfter - targetRouteCostBefore;
                            moveCost = targetRouteCostDiff;
                        }
                        else if (objective == Objective.CUMULATIVE_SERVICE_TIMES)
                        {
                            targetRouteCostBefore = targetRoute.cost;
                            List<Node> newSequenceTarget = new List<Node>();
                            foreach (Node x in targetRoute.sequence)
                            {
                                newSequenceTarget.Add(new Node(x));
                            }
                            newSequenceTarget[secondNodeIndex] = new Node(n);
                            updateArrivalTimes(newSequenceTarget, secondNodeIndex - 1);

                            targetRouteCostAfter = this.calculateMinSum(newSequenceTarget) + FeasibleSegment.timeWindowPenaltyWeight * segForTargetRoute.timeWindowPenalty;
                            targetRouteCostDiff = targetRouteCostAfter - targetRouteCostBefore;
                            moveCost = targetRouteCostDiff;

                        }
                        else if (objective == Objective.DISTANCE)
                        {
                            targetRouteCostBefore = targetRoute.cost;
                            targetRouteCostAfter = segForTargetRoute.distances + FeasibleSegment.timeWindowPenaltyWeight * segForTargetRoute.timeWindowPenalty;
                            targetRouteCostDiff = targetRouteCostAfter - targetRouteCostBefore;
                            moveCost = targetRouteCostDiff;
                        }
                        
                        toBeCreated.Clear();
                        toBeCreated.Add(new Arc(vPrevious, n));
                        toBeCreated.Add(new Arc(n, vNext));
                        if (promisesBroken(moveCost)) continue;

                        if (moveCost >= repl.cost) continue;
                        repl.storeMove(n, targetRouteIndex, secondNodeIndex, moveCost, 0);
                    }
                }
            }
            
            
            return repl;
        }

        public void ApplyInsertion(Insertion ins)
        {
            System.Diagnostics.Debug.Assert(ins.customer != null);
            Node n = ins.customer;
            n.isRouted = true;
            Route targetRoute = solution.routes[ins.targetRouteIndex];
            Node v = targetRoute.sequence[ins.targetNodeIndex];
            Node vNext = targetRoute.sequence[ins.targetNodeIndex + 1];

            promisesMatrix[v.id, vNext.id] = solution.cost;

            solution.cost += ins.cost + ins.penalty; //penalty refers to the unvisited nodes not the time window penalties

            targetRoute.cost += ins.cost;
            targetRoute.load += n.demand;
            targetRoute.sequence.Insert(ins.targetNodeIndex + 1, n);
            targetRoute.UpdateRouteNodes();
            solution.unvisited.Remove(n);
        }

        public void ApplyRelocation(Relocation relocation)
        {
            Route originRoute = solution.routes[relocation.originRouteIndex];
            Route targetRoute = solution.routes[relocation.targetRouteIndex];
            Node nPrevious = originRoute.sequence[relocation.originNodeIndex - 1];
            Node node = originRoute.sequence[relocation.originNodeIndex];
            Node nNext = originRoute.sequence[relocation.originNodeIndex + 1];
            Node v = targetRoute.sequence[relocation.targetNodeIndex];
            Node vNext = targetRoute.sequence[relocation.targetNodeIndex + 1];

            promisesMatrix[v.id, vNext.id] = solution.cost;
            promisesMatrix[nPrevious.id, node.id] = solution.cost;
            promisesMatrix[node.id, nNext.id] = solution.cost;

            solution.cost += relocation.cost;
            if (relocation.originRouteIndex == relocation.targetRouteIndex)
            {
                originRoute.cost += relocation.cost;
                originRoute.sequence.RemoveAt(relocation.originNodeIndex);
                if (relocation.originNodeIndex > relocation.targetNodeIndex)
                {
                    originRoute.sequence.Insert(relocation.targetNodeIndex + 1, node);
                }
                else
                {
                    originRoute.sequence.Insert(relocation.targetNodeIndex, node);
                }
                originRoute.UpdateRouteNodes();
            }
            else 
            {
                originRoute.cost += relocation.originRouteCostDiff;
                originRoute.load -= node.demand;
                targetRoute.cost += relocation.targetRouteCostDiff;
                targetRoute.load += node.demand;
                originRoute.sequence.RemoveAt(relocation.originNodeIndex);
                targetRoute.sequence.Insert(relocation.targetNodeIndex + 1, node);
                originRoute.UpdateRouteNodes();
                targetRoute.UpdateRouteNodes();
            }
        }
        
        public Swap FindSwap(Objective objective, FeasibleSegment[,] segments)
        {
            Swap swap = new Swap();
            List<Node> originRouteSequence = new List<Node>();
            List<Node> targetRouteSequence = new List<Node>();
            for (int originRouteIndex = 0; originRouteIndex < solution.routes.Count; originRouteIndex++)
            {
                Route originRoute = solution.routes[originRouteIndex];
                for (int targetRouteIndex = originRouteIndex; targetRouteIndex < solution.routes.Count; targetRouteIndex++)
                {
                    Route targetRoute = solution.routes[targetRouteIndex];
                    for (int firstNodeIndex = 1; firstNodeIndex < originRoute.sequence.Count - 1; firstNodeIndex++)
                    {
                        for (int secondNodeIndex = (originRouteIndex == targetRouteIndex)? firstNodeIndex + 1 : 1; secondNodeIndex < targetRoute.sequence.Count - 1; secondNodeIndex++)
                        {
                            Node nPrevious = originRoute.sequence[firstNodeIndex - 1];
                            Node n = originRoute.sequence[firstNodeIndex];
                            Node nNext = originRoute.sequence[firstNodeIndex + 1];

                            Node vPrevious = targetRoute.sequence[secondNodeIndex - 1];
                            Node v = targetRoute.sequence[secondNodeIndex];
                            Node vNext = targetRoute.sequence[secondNodeIndex + 1];

                            //if (timeWindowCompatibilityMatrix[vPrevious.id, n.id] == -Math.Pow(10, 6)
                            //   || timeWindowCompatibilityMatrix[n.id, vNext.id] == -Math.Pow(10, 6)
                            //   || timeWindowCompatibilityMatrix[nPrevious.id, v.id] == -Math.Pow(10, 6)
                            //   || timeWindowCompatibilityMatrix[v.id, nNext.id] == -Math.Pow(10, 6)) continue;
                            
                            if (originRouteIndex == targetRouteIndex)
                            {
                                FeasibleSegment segForRoute, seg;
                                double dxy;
                                double twc;
                                if (secondNodeIndex == firstNodeIndex + 1)
                                {
                                    segForRoute = new FeasibleSegment(segments[originRoute.sequence[0].id, nPrevious.id]); // depotStart to nPrevious
                                    seg = segments[v.id, v.id]; // v
                                    dxy = ProblemConfiguration.model.distances[nPrevious.id, v.id];
                                    twc = timeWindowCompatibilityMatrix[nPrevious.id, v.id];
                                    segForRoute = FeasibleSegment.MergeSegments(segForRoute, seg, dxy, twc, capacity); // add v
                                    seg = segments[n.id, n.id]; // n
                                    dxy = ProblemConfiguration.model.distances[v.id, n.id];
                                    twc = timeWindowCompatibilityMatrix[v.id, n.id];
                                    segForRoute = FeasibleSegment.MergeSegments(segForRoute, seg, dxy, twc, capacity); // add n
                                    seg = segments[vNext.id, originRoute.sequence[originRoute.sequence.Count - 1].id]; // vNext to depotEnd
                                    dxy = ProblemConfiguration.model.distances[n.id, vNext.id];
                                    twc = timeWindowCompatibilityMatrix[n.id, vNext.id];
                                    segForRoute = FeasibleSegment.MergeSegments(segForRoute, seg, dxy, twc, capacity); // add vNext to depotEnd
                                }
                                else if (firstNodeIndex == secondNodeIndex + 2)
                                {
                                    segForRoute = new FeasibleSegment(segments[originRoute.sequence[0].id, nPrevious.id]); // depotStart to nPrevious
                                    seg = segments[v.id, v.id]; // v
                                    dxy = ProblemConfiguration.model.distances[nPrevious.id, v.id];
                                    twc = timeWindowCompatibilityMatrix[nPrevious.id, v.id];
                                    segForRoute = FeasibleSegment.MergeSegments(segForRoute, seg, dxy, twc, this.capacity); // add v
                                    seg = segments[nNext.id, nNext.id]; // nNext = vPrevious
                                    dxy = ProblemConfiguration.model.distances[v.id, nNext.id];
                                    twc = timeWindowCompatibilityMatrix[v.id, nNext.id];
                                    segForRoute = FeasibleSegment.MergeSegments(segForRoute, seg, dxy, twc, this.capacity); // add nNext
                                    seg = segments[n.id, n.id]; // n
                                    dxy = ProblemConfiguration.model.distances[nNext.id, n.id];
                                    twc = timeWindowCompatibilityMatrix[nNext.id, n.id];
                                    segForRoute = FeasibleSegment.MergeSegments(segForRoute, seg, dxy, twc, this.capacity); // add n
                                    seg = segments[vNext.id, originRoute.sequence[originRoute.sequence.Count - 2].id]; // vNext to depotEnd
                                    dxy = ProblemConfiguration.model.distances[n.id, vNext.id];
                                    twc = timeWindowCompatibilityMatrix[n.id, vNext.id];
                                    segForRoute = FeasibleSegment.MergeSegments(segForRoute, seg, dxy, twc, this.capacity); // add vNext to depotEnd
                                } else
                                {
                                    segForRoute = new FeasibleSegment(segments[originRoute.sequence[0].id, nPrevious.id]); // depotStart to nPrevious
                                    seg = segments[v.id,v.id]; // v
                                    dxy = ProblemConfiguration.model.distances[nPrevious.id, v.id];
                                    twc = timeWindowCompatibilityMatrix[nPrevious.id, v.id];
                                    segForRoute = FeasibleSegment.MergeSegments(segForRoute, seg, dxy, twc, this.capacity); // add v
                                    seg = segments[nNext.id,vPrevious.id]; // nNext to vPrevious
                                    dxy = ProblemConfiguration.model.distances[v.id, nNext.id];
                                    twc = timeWindowCompatibilityMatrix[v.id, nNext.id];
                                    segForRoute = FeasibleSegment.MergeSegments(segForRoute, seg, dxy, twc, this.capacity); // add nNext to vPrevious
                                    seg = segments[n.id,n.id]; // n
                                    dxy = ProblemConfiguration.model.distances[vPrevious.id, n.id];
                                    twc = timeWindowCompatibilityMatrix[vPrevious.id, n.id];
                                    segForRoute = FeasibleSegment.MergeSegments(segForRoute, seg, dxy, twc, this.capacity); // add n
                                    seg = segments[vNext.id,originRoute.sequence[originRoute.sequence.Count - 1].id]; // vNext to depotEnd
                                    dxy = ProblemConfiguration.model.distances[n.id, vNext.id];
                                    twc = timeWindowCompatibilityMatrix[n.id, vNext.id];
                                    segForRoute = FeasibleSegment.MergeSegments(segForRoute, seg, dxy, twc, this.capacity); // add vNext to depotEnd
                                }

                                if (segForRoute.load > capacity) continue;
                                double routeCostBefore, routeCostAfter, moveCost = 0;
                                if (objective == Objective.CUMULATIVE_DISTANCE)
                                {
                                    routeCostBefore = originRoute.cost;
                                    routeCostAfter = segForRoute.cumulativeDistances + FeasibleSegment.timeWindowPenaltyWeight * segForRoute.timeWindowPenalty;
                                    moveCost = routeCostAfter - routeCostBefore;
                                }
                                else if (objective == Objective.CUMULATIVE_SERVICE_TIMES)
                                {
                                    routeCostBefore = originRoute.cost;

                                    List<Node> newSequence = new List<Node>();
                                    foreach (Node x in originRoute.sequence)
                                    {
                                        newSequence.Add(new Node(x));
                                    }

                                    newSequence[firstNodeIndex] = new Node(v);
                                    newSequence[secondNodeIndex] = new Node(n);

                                    updateArrivalTimes(newSequence, firstNodeIndex-1);

                                    routeCostAfter = calculateMinSum(newSequence) + FeasibleSegment.timeWindowPenaltyWeight * segForRoute.timeWindowPenalty;
                                    moveCost = routeCostAfter - routeCostBefore;
                                }
                                else if (objective == Objective.DISTANCE)
                                {
                                    routeCostBefore = originRoute.cost;
                                    routeCostAfter = segForRoute.distances + FeasibleSegment.timeWindowPenaltyWeight * segForRoute.timeWindowPenalty;
                                    moveCost = routeCostAfter - routeCostBefore;
                                }
                                
                                toBeCreated.Clear();
                                toBeCreated.Add(new Arc(nPrevious, v));
                                toBeCreated.Add(new Arc(v, nNext));
                                toBeCreated.Add(new Arc(n, vNext));
                                toBeCreated.Add(new Arc(vPrevious, n));
                                
                                if (promisesBroken(moveCost)) continue;
                                if (moveCost >= swap.cost) continue;
                                swap.storeMove(originRouteIndex, targetRouteIndex, firstNodeIndex, secondNodeIndex, moveCost, moveCost, moveCost);
                                
                            }
                            else
                            {
                                FeasibleSegment segForOriginRoute = new FeasibleSegment(segments[originRoute.sequence[0].id,nPrevious.id]); // from depotStart to nPrevious
                                FeasibleSegment seg = segments[v.id,v.id]; // v
                                double dxy = ProblemConfiguration.model.distances[nPrevious.id, v.id];
                                double twc = timeWindowCompatibilityMatrix[nPrevious.id, v.id];
                                segForOriginRoute = FeasibleSegment.MergeSegments(segForOriginRoute, seg, dxy, twc, capacity); // add v
                                seg = segments[nNext.id,originRoute.sequence[originRoute.sequence.Count - 1].id]; // nNext to depotEnd
                                dxy = ProblemConfiguration.model.distances[v.id, nNext.id];
                                twc = timeWindowCompatibilityMatrix[v.id, nNext.id];
                                segForOriginRoute = FeasibleSegment.MergeSegments(segForOriginRoute, seg, dxy, twc, capacity); // add nNext to depotEnd

                                FeasibleSegment segForTargetRoute = new FeasibleSegment(segments[targetRoute.sequence[0].id,vPrevious.id]); // depotStart to vPrevious
                                seg = segments[n.id,n.id]; // n
                                dxy = ProblemConfiguration.model.distances[vPrevious.id, n.id];
                                twc = timeWindowCompatibilityMatrix[vPrevious.id, n.id];
                                segForTargetRoute = FeasibleSegment.MergeSegments(segForTargetRoute, seg, dxy, twc, capacity); // add n
                                seg = segments[vNext.id,targetRoute.sequence[targetRoute.sequence.Count - 1].id]; // vNext to depotEnd
                                dxy = ProblemConfiguration.model.distances[n.id, vNext.id];
                                twc = timeWindowCompatibilityMatrix[n.id, vNext.id];
                                segForTargetRoute = FeasibleSegment.MergeSegments(segForTargetRoute, seg, dxy, twc, capacity); // add vNext to depotEnd

                                if (segForOriginRoute.load > capacity) continue;
                                if (segForTargetRoute.load > capacity) continue;
                                double originRouteCostBefore, originRouteCostAfter, originRouteCostDiff = 0;
                                double targetRouteCostBefore, targetRouteCostAfter, targetRouteCostDiff = 0;
                                double moveCost = 0;
                                if (objective == Objective.CUMULATIVE_DISTANCE)
                                {
                                    originRouteCostBefore = originRoute.cost;
                                    originRouteCostAfter = segForOriginRoute.cumulativeDistances + FeasibleSegment.timeWindowPenaltyWeight * segForOriginRoute.timeWindowPenalty;
                                    originRouteCostDiff = originRouteCostAfter - originRouteCostBefore;
                                    targetRouteCostBefore = targetRoute.cost;
                                    targetRouteCostAfter = segForTargetRoute.cumulativeDistances + FeasibleSegment.timeWindowPenaltyWeight * segForTargetRoute.timeWindowPenalty;
                                    targetRouteCostDiff = targetRouteCostAfter - targetRouteCostBefore;
                                    moveCost = originRouteCostDiff + targetRouteCostDiff;
                                }
                                else if (objective == Objective.CUMULATIVE_SERVICE_TIMES)
                                {
                                    originRouteCostBefore = originRoute.cost;
                                    originRouteSequence.Clear();
                                    targetRouteSequence.Clear();
                                    
                                    foreach (Node x in originRoute.sequence)
                                        originRouteSequence.Add(new Node(x));
                                    originRouteSequence[firstNodeIndex] = new Node(v);
                                    updateArrivalTimes(originRouteSequence, firstNodeIndex - 1);
                                    
                                    originRouteCostAfter = calculateMinSum(originRouteSequence) + FeasibleSegment.timeWindowPenaltyWeight * segForOriginRoute.timeWindowPenalty;
                                    originRouteCostDiff = originRouteCostAfter - originRouteCostBefore;

                                    targetRouteCostBefore = targetRoute.cost;
                                    foreach (Node x in targetRoute.sequence)
                                        targetRouteSequence.Add(new Node(x));
                                    targetRouteSequence[secondNodeIndex] = new Node(n);
                                    updateArrivalTimes(targetRouteSequence, secondNodeIndex - 1);

                                    targetRouteCostAfter = this.calculateMinSum(targetRouteSequence) + FeasibleSegment.timeWindowPenaltyWeight * segForTargetRoute.timeWindowPenalty; ;
                                    targetRouteCostDiff = targetRouteCostAfter - targetRouteCostBefore;
                                    moveCost = originRouteCostDiff + targetRouteCostDiff;

                                }
                                else if (objective == Objective.DISTANCE)
                                {
                                    originRouteCostBefore = originRoute.cost;
                                    originRouteCostAfter = segForOriginRoute.distances + FeasibleSegment.timeWindowPenaltyWeight * segForOriginRoute.timeWindowPenalty;
                                    originRouteCostDiff = originRouteCostAfter - originRouteCostBefore;
                                    targetRouteCostBefore = targetRoute.cost;
                                    targetRouteCostAfter = segForTargetRoute.distances + FeasibleSegment.timeWindowPenaltyWeight * segForTargetRoute.timeWindowPenalty;
                                    targetRouteCostDiff = targetRouteCostAfter - targetRouteCostBefore;
                                    moveCost = originRouteCostDiff + targetRouteCostDiff;
                                }
                                
                                toBeCreated.Clear();
                                toBeCreated.Add(new Arc(nPrevious, v));
                                toBeCreated.Add(new Arc(v, nNext));
                                toBeCreated.Add(new Arc(n, vNext));
                                toBeCreated.Add(new Arc(vPrevious, n));
                                if (promisesBroken(moveCost)) continue;
                                if (moveCost >= swap.cost) continue;
                                swap.storeMove(originRouteIndex, targetRouteIndex, firstNodeIndex, secondNodeIndex, moveCost, originRouteCostDiff, targetRouteCostDiff);
                            }
                        }
                    }
                }
            }
            return swap;
        }

        public void ApplySwap(Swap swap)
        {
            Route originRoute = solution.routes[swap.originRouteIndex];
            Route targetRoute = solution.routes[swap.targetRouteIndex];
            Node nPrevious = originRoute.sequence[swap.originNodeIndex - 1];
            Node firstNode = originRoute.sequence[swap.originNodeIndex];
            Node nNext = originRoute.sequence[swap.originNodeIndex + 1];

            Node vPrevious = targetRoute.sequence[swap.targetNodeIndex - 1];
            Node secondNode = targetRoute.sequence[swap.targetNodeIndex];
            Node vNext = targetRoute.sequence[swap.targetNodeIndex + 1];

            promisesMatrix[nPrevious.id, firstNode.id] = solution.cost;
            promisesMatrix[firstNode.id, nNext.id] = solution.cost;
            promisesMatrix[vPrevious.id, secondNode.id] = solution.cost;
            promisesMatrix[secondNode.id, vNext.id] = solution.cost;

            solution.cost += swap.cost;

            if (swap.originRouteIndex == swap.targetRouteIndex)
            {
                originRoute.sequence[swap.originNodeIndex] = secondNode;
                originRoute.sequence[swap.targetNodeIndex] = firstNode;
                originRoute.cost += swap.originRouteCostDiff;
                originRoute.UpdateRouteNodes();
            }
            else
            {
                originRoute.sequence[swap.originNodeIndex] = secondNode;
                targetRoute.sequence[swap.targetNodeIndex] = firstNode;
                originRoute.cost += swap.originRouteCostDiff;
                originRoute.load += secondNode.demand - firstNode.demand;
                targetRoute.cost += swap.targetRouteCostDiff;
                targetRoute.load += firstNode.demand - secondNode.demand;
                originRoute.UpdateRouteNodes();
                targetRoute.UpdateRouteNodes();    
            }
        }

        public void ApplyReplace(Replace repl)
        {
            System.Diagnostics.Debug.Assert(repl.n != null);
            
            Route targetRoute = solution.routes[repl.targetRouteIndex];
            
            Node firstNode = repl.n;
            Node vPrevious = targetRoute.sequence[repl.targetNodeIndex - 1];
            Node secondNode = targetRoute.sequence[repl.targetNodeIndex];
            Node vNext = targetRoute.sequence[repl.targetNodeIndex + 1];

            promisesMatrix[vPrevious.id, secondNode.id] = solution.cost;
            promisesMatrix[secondNode.id, vNext.id] = solution.cost;

            solution.cost += repl.cost;

            targetRoute.sequence[repl.targetNodeIndex] = firstNode;
            targetRoute.cost += repl.cost; //cost already includes just the cost diff of the target route
            targetRoute.load += firstNode.demand - secondNode.demand;
            targetRoute.UpdateRouteNodes();
            solution.unvisited.Remove(repl.n);
            solution.unvisited.Add(secondNode);
        }

        public TwoOptMove FindTwoOpt(Objective objective, FeasibleSegment[,] segments)
        {
            TwoOptMove top = new TwoOptMove();
            for (int rtInd1 = 0; rtInd1 < ProblemConfiguration.model.vehicles; rtInd1++)
            {
                Route rt1 = solution.routes[rtInd1];
                for (int rtInd2 = rtInd1; rtInd2 < ProblemConfiguration.model.vehicles; rtInd2++)
                {
                    Route rt2 = solution.routes[rtInd2];
                    for (int nodeInd1 = 0; nodeInd1 < rt1.sequence.Count - 1; nodeInd1++)
                    {
                        int start2 = 0;
                        if (rtInd1 == rtInd2)
                        {
                            start2 = nodeInd1 + 2;
                        }
                        for (int nodeInd2 = start2; nodeInd2 < rt2.sequence.Count - 1; nodeInd2++)
                        {
                            //Origin route nodes
                            Node A = rt1.sequence[nodeInd1];
                            Node B = rt1.sequence[nodeInd1 + 1];

                            //Target route nodes
                            Node K = rt2.sequence[nodeInd2];
                            Node L = rt2.sequence[nodeInd2 + 1];

                            // initialize parameters
                            double route1CostAfter = Math.Pow(10, 10);
                            double route2CostAfter = Math.Pow(10, 10);
                            double cost = Math.Pow(10, 10);
                            int rt1Load = 0;
                            int rt2Load = 0;


                            // A-->B-->C-->K-->L
                            //
                            //
                            // /-----------\
                            // |           |
                            // |           V
                            // A   B<--C<--K   L
                            //     |           ^
                            //     |           |
                            //     \-----------/
                            //
                            //
                            // A-->K-->C-->B-->L
                            if (rtInd1 == rtInd2)
                            {
                                //if (timeWindowCompatibilityMatrix[A.id, K.id] == -Math.Pow(10, 6)
                                //|| timeWindowCompatibilityMatrix[B.id, L.id] == -Math.Pow(10, 6)) continue;

                                if (nodeInd1 == 0 && nodeInd2 == rt1.sequence.Count - 2) continue;

                                FeasibleSegment segA, segK, segL;
                                // depot mexri A
                                segA = new FeasibleSegment(segments[rt1.sequence[0].id, A.id]);

                                // K segment
                                segK = new FeasibleSegment(segments[K.id, K.id]);
                                // Expanding K segment mexri to B
                                double twc;
                                for (int i = nodeInd2 - 1; i > nodeInd1; i--)
                                {
                                    Node k = rt1.sequence[i + 1];
                                    Node c = rt1.sequence[i];
                                    twc = timeWindowCompatibilityMatrix[k.id, c.id];
                                    //segK = FeasibleSegment.ExpandRightWithCustomer(segK, c, ProblemConfiguration.model.distances[k.id, c.id], twc, this.capacity);
                                    segK = FeasibleSegment.MergeSegments(segK, segments[c.id, c.id], ProblemConfiguration.model.distances[k.id, c.id], twc, this.capacity);
                                }

                                // L mexri to telos
                                segL = new FeasibleSegment(segments[L.id, rt1.sequence[rt1.sequence.Count - 1].id]);
                                // kataskebh neoy route
                                twc = timeWindowCompatibilityMatrix[A.id, K.id];
                                FeasibleSegment seg1 = FeasibleSegment.MergeSegments(segA, segK, ProblemConfiguration.model.distances[A.id, K.id], twc, this.capacity);
                                twc = timeWindowCompatibilityMatrix[B.id, L.id];
                                seg1 = FeasibleSegment.MergeSegments(seg1, segL, ProblemConfiguration.model.distances[B.id, L.id], twc, this.capacity);

                                if (seg1.load > capacity) continue;

                                if (objective == Objective.CUMULATIVE_DISTANCE)
                                {
                                    double routeCostBefore = rt1.cost;
                                    route1CostAfter = seg1.cumulativeDistances + FeasibleSegment.timeWindowPenaltyWeight * seg1.timeWindowPenalty;
                                    cost = route1CostAfter - routeCostBefore;
                                    rt1Load = seg1.load;
                                }
                                else if (objective == Objective.CUMULATIVE_SERVICE_TIMES)
                                {
                                    double routeCostBefore = rt1.cost;
                                    List<Node> newSequence = new List<Node>();
                                    foreach (Node v in rt1.sequence)
                                    {
                                        newSequence.Add(new Node(v));
                                    }
                                    // depot mexri A
                                    List<Node> firstSegmentOfRt1 = new List<Node>();
                                    for (int i = 0; i < nodeInd1 + 1; i++)
                                    {
                                        firstSegmentOfRt1.Add(newSequence[i]);
                                    }
                                    // K mexri B reversed
                                    List<Node> secondSegmentOfRt1 = new List<Node>();
                                    for (int i = nodeInd2; i > nodeInd1; i--)
                                    {
                                        secondSegmentOfRt1.Add(newSequence[i]);
                                    }
                                    // L mexri telos
                                    List<Node> thirdSegmentOfRt1 = new List<Node>();
                                    for (int i = nodeInd2 + 1; i < newSequence.Count; i++)
                                    {
                                        thirdSegmentOfRt1.Add(newSequence[i]);
                                    }
                                    firstSegmentOfRt1.AddRange(secondSegmentOfRt1);
                                    firstSegmentOfRt1.AddRange(thirdSegmentOfRt1);
                                    newSequence = firstSegmentOfRt1;
                                    this.updateArrivalTimes(newSequence, nodeInd1);

                                    route1CostAfter = this.calculateMinSum(newSequence) + FeasibleSegment.timeWindowPenaltyWeight * seg1.timeWindowPenalty;
                                    cost = route1CostAfter - routeCostBefore;
                                    rt1Load = seg1.load;

                                }
                                else if (objective == Objective.DISTANCE)
                                {
                                    double routeCostBefore = rt1.cost;
                                    route1CostAfter = seg1.distances + FeasibleSegment.timeWindowPenaltyWeight * seg1.timeWindowPenalty;
                                    cost = route1CostAfter - routeCostBefore;
                                    rt1Load = seg1.load;
                                }

                                toBeCreated.Clear();
                                toBeCreated.Add(new Arc(A, K));
                                toBeCreated.Add(new Arc(B, L));
                                if (promisesBroken(cost)) continue;
                            }
                            else
                            // A-\ /->L
                            //    X
                            // K-/ \->B
                            //
                            //
                            // A-->L
                            //
                            // K-->B
                            {

                                //if (timeWindowCompatibilityMatrix[A.id, L.id] == -Math.Pow(10, 6)
                                //|| timeWindowCompatibilityMatrix[K.id, B.id] == -Math.Pow(10, 6)) continue;

                                if (nodeInd1 == 0 && nodeInd2 == 0) continue;

                                if (nodeInd1 == rt1.sequence.Count - 2 && nodeInd2 == rt2.sequence.Count - 2) continue;

                                // proti periprosi
                                FeasibleSegment segA, segB, segK, segL;

                                // depot mexri A
                                segA = new FeasibleSegment(segments[rt1.sequence[0].id, A.id]);
                                // B mexri telos
                                segB = new FeasibleSegment(segments[B.id, rt1.sequence[rt1.sequence.Count - 1].id]);
                                // depot mexri K
                                segK = new FeasibleSegment(segments[rt2.sequence[0].id, K.id]);
                                // L mexri telos
                                segL = new FeasibleSegment(segments[L.id, rt2.sequence[rt2.sequence.Count - 1].id]);

                                // kataskebh dyo neon route
                                double twc = timeWindowCompatibilityMatrix[A.id, L.id];
                                FeasibleSegment seg1 = FeasibleSegment.MergeSegments(segA, segL, ProblemConfiguration.model.distances[A.id, L.id], twc, this.capacity);
                                twc = timeWindowCompatibilityMatrix[K.id, B.id];
                                FeasibleSegment seg2 = FeasibleSegment.MergeSegments(segK, segB, ProblemConfiguration.model.distances[K.id, B.id], twc, this.capacity);

                                double routesCostBefore = rt1.cost + rt2.cost;

                                if (seg1.load > capacity) continue;
                                if (seg2.load > capacity) continue;


                                if (objective == Objective.CUMULATIVE_DISTANCE)
                                {
                                    route1CostAfter = seg1.cumulativeDistances + FeasibleSegment.timeWindowPenaltyWeight * seg1.timeWindowPenalty;
                                    route2CostAfter = seg2.cumulativeDistances + FeasibleSegment.timeWindowPenaltyWeight * seg2.timeWindowPenalty;

                                    rt1Load = seg1.load;
                                    rt2Load = seg2.load;

                                    cost = route1CostAfter + route2CostAfter - routesCostBefore;
                                }
                                else if (objective == Objective.CUMULATIVE_SERVICE_TIMES)
                                {
                                    List<Node> newSequencert1 = new List<Node>();
                                    foreach (Node v in rt1.sequence)
                                    {
                                        newSequencert1.Add(new Node(v));
                                    }
                                    List<Node> firstSegmentOfRt1 = new List<Node>();
                                    List<Node> relocatedSegmentOfRt1 = new List<Node>();
                                    for (int i = 0; i < newSequencert1.Count; i++)
                                    {
                                        if (i < nodeInd1 + 1)
                                        {
                                            // depot mexri A
                                            firstSegmentOfRt1.Add(newSequencert1[i]);
                                        }
                                        else
                                        {
                                            // B mexri telos
                                            relocatedSegmentOfRt1.Add(newSequencert1[i]);
                                        }
                                    }

                                    List<Node> newSequencert2 = new List<Node>();
                                    foreach (Node v in rt2.sequence)
                                    {
                                        newSequencert2.Add(new Node(v));
                                    }
                                    List<Node> firstSegmentOfRt2 = new List<Node>();
                                    List<Node> relocatedSegmentOfRt2 = new List<Node>();
                                    for (int i = 0; i < newSequencert2.Count; i++)
                                    {
                                        if (i < nodeInd2 + 1)
                                        {
                                            // depot mexri K
                                            firstSegmentOfRt2.Add(newSequencert2[i]);
                                        }
                                        else
                                        {
                                            // L mexri telos
                                            relocatedSegmentOfRt2.Add(newSequencert2[i]);
                                        }
                                    }

                                    firstSegmentOfRt1.AddRange(relocatedSegmentOfRt2);
                                    newSequencert1 = firstSegmentOfRt1;
                                    updateArrivalTimes(newSequencert1, nodeInd1);

                                    firstSegmentOfRt2.AddRange(relocatedSegmentOfRt1);
                                    newSequencert2 = firstSegmentOfRt2;
                                    updateArrivalTimes(newSequencert2, nodeInd2);

                                    route1CostAfter = calculateMinSum(newSequencert1) + FeasibleSegment.timeWindowPenaltyWeight * seg1.timeWindowPenalty;
                                    route2CostAfter = calculateMinSum(newSequencert2) + FeasibleSegment.timeWindowPenaltyWeight * seg2.timeWindowPenalty;

                                    rt1Load = seg1.load;
                                    rt2Load = seg2.load;

                                    cost = route1CostAfter + route2CostAfter - routesCostBefore;
                                }

                            }

                            if (cost > top.cost) continue;
                            
                            // sthn perisptosh toy enos route secondRouteCostDiff secondRouteLoad agnooynte
                            top.StoreMove(rtInd1, rtInd2, nodeInd1, nodeInd2, cost, 0.0,
                                route1CostAfter - rt1.cost, route2CostAfter - rt2.cost, rt1Load, rt2Load);
                        }
                    }
                }
            }
            return top;
        }
        public void ApplyTwoOptMove(TwoOptMove top)
        {
            Route rt1 = solution.routes[top.originRouteIndex];
            Route rt2 = solution.routes[top.targetRouteIndex];

            Node A = rt1.sequence[top.originNodeIndex];
            Node B = rt1.sequence[top.originNodeIndex + 1];

            //Target route nodes
            Node K = rt2.sequence[top.targetNodeIndex];
            Node L = rt2.sequence[top.targetNodeIndex + 1];

            promisesMatrix[A.id, B.id] = solution.cost;
            promisesMatrix[K.id, L.id] = solution.cost;

            solution.cost += top.cost;

            if (top.originRouteIndex == top.targetRouteIndex)
            {
                // depot mexri A
                List<Node> firstSegmentOfRt1 = new List<Node>();
                for (int i = 0; i < top.originNodeIndex + 1; i++)
                {
                    firstSegmentOfRt1.Add(rt1.sequence[i]);
                }
                // K mexri B reversed
                List<Node> secondSegmentOfRt1 = new List<Node>();
                for (int i = top.targetNodeIndex; i > top.originNodeIndex; i--)
                {
                    secondSegmentOfRt1.Add(rt1.sequence[i]);
                }
                // L mexri telos
                List<Node> thirdSegmentOfRt1 = new List<Node>();
                for (int i = top.targetNodeIndex + 1; i < rt1.sequence.Count; i++)
                {
                    thirdSegmentOfRt1.Add(rt1.sequence[i]);
                }

                rt1.load = top.originRouteLoad;
                rt1.cost += top.originRouteCostDiff;
                firstSegmentOfRt1.AddRange(secondSegmentOfRt1);
                firstSegmentOfRt1.AddRange(thirdSegmentOfRt1);
                rt1.sequence = firstSegmentOfRt1;
                rt1.UpdateRouteNodes();
            }
            else
            {
                List<Node> firstSegmentOfRt1 = new List<Node>();
                List<Node> relocatedSegmentOfRt1 = new List<Node>();
                for (int i = 0; i < rt1.sequence.Count; i++)
                {
                    if (i < top.originNodeIndex + 1)
                    {
                        // depot mexri A
                        firstSegmentOfRt1.Add(rt1.sequence[i]);
                    }
                    else
                    {
                        // B mexri telos
                        relocatedSegmentOfRt1.Add(rt1.sequence[i]);
                    }
                }
                List<Node> firstSegmentOfRt2 = new List<Node>();
                List<Node> relocatedSegmentOfRt2 = new List<Node>();
                for (int i = 0; i < rt2.sequence.Count; i++)
                {
                    if (i < top.targetNodeIndex + 1)
                    {
                        // depot mexri K
                        firstSegmentOfRt2.Add(rt2.sequence[i]);
                    }
                    else
                    {
                        // L mexri telos
                        relocatedSegmentOfRt2.Add(rt2.sequence[i]);
                    }
                }

                rt1.load = top.originRouteLoad;
                rt1.cost += top.originRouteCostDiff;
                firstSegmentOfRt1.AddRange(relocatedSegmentOfRt2);
                rt1.sequence = firstSegmentOfRt1;
                rt1.UpdateRouteNodes();

                rt2.load = top.targetRouteLoad;
                rt2.cost += top.targetRouteCostDiff;
                firstSegmentOfRt2.AddRange(relocatedSegmentOfRt1);
                rt2.sequence = firstSegmentOfRt2;
                rt2.UpdateRouteNodes();
                
            }
        }
    }

}
