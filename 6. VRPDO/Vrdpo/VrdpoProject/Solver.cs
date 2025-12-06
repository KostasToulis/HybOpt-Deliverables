using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Linq;
using Microsoft.VisualBasic.FileIO;
using System.Text.Json;
using static System.Net.Mime.MediaTypeNames;
using System.Diagnostics;

namespace VrdpoProject
{
    public class Solver
    {
        string instance;
        private CustomerInsertionAllPositions bestInsertion = new();
        Solution globalBestSol = new Solution();
        LocalSearch ls = new();
        Random rnd2 = new Random(42);
        TimeSpan globalBestTime = TimeSpan.Zero;
        public void Solve()
        {
            string jsonContent = File.ReadAllText("settings.json");
            var settings = JsonSerializer.Deserialize<Settings>(jsonContent);
            var totalTimer = new Stopwatch();
            totalTimer.Start();
            if (settings.multiRestart) {

                List<Solution> feasibleSolutions = new List<Solution>();
                feasibleSolutions = ConstructFeasibleSolutions().OrderBy(x => x.Cost).ToList();
                CalculateSimilarity(feasibleSolutions);
                foreach (Solution sol in feasibleSolutions)
                {
                    RemoveEmptyRoutes(sol);
                    PrintSolution(sol);
                }

                globalBestSol.Cost = Math.Pow(10, 9);
                int count = 1;
                Solution lc_sol = new Solution();
                foreach (Solution s in feasibleSolutions)
                {
                    Console.WriteLine("Solution Number " + count);
                    Console.WriteLine("------------------");
                    lc_sol = LocalSearch(s);
                    if (lc_sol.Cost < globalBestSol.Cost)
                    {
                        globalBestSol = lc_sol.DeepCopy(lc_sol);
                    }
                    Console.WriteLine("------------------");
                    Console.WriteLine("------------------");
                    count++;
                }
                totalTimer.Stop();
                Console.WriteLine("The best solution's cost: " + globalBestSol.Cost);
                CalculateServiceLevel(globalBestSol);
                ReportSolution(globalBestSol, globalBestTime, totalTimer.Elapsed);

            }
            else
            {

                Solution lc_sol = new Solution();
                lc_sol = LocalSearch();
                PrintSolution(lc_sol);
                Console.WriteLine("The best solution's cost: " + lc_sol.Cost);
                CalculateServiceLevel(lc_sol);
                //Export lc_sol to JSON
                //lc_sol.ExportToJson("./solution_data.json");
                totalTimer.Stop();
                ReportSolution(lc_sol, globalBestTime, totalTimer.Elapsed);
            }
        }


        Solution LocalSearch(Solution startingSolution = null)
        {
            string jsonContent = File.ReadAllText("settings.json");
            var settings = JsonSerializer.Deserialize<Settings>(jsonContent);
            Solution bestSol = new();
            bestSol.Cost = Math.Pow(10, 9);
            int numberOfRestarts = settings.restarts;
            Solution currentSol;
            int timesFailedFindFeasible = 0;
            var restartTimer = new Stopwatch();
            int restartCounter = -1;
            
            for (int restart = 0; restart < numberOfRestarts; restart++)
            {
                restartTimer.Reset();
                restartTimer.Start();
                Random rnd = new(restart);
                Random rnd5 = new(restart);
                int reinitCount = -1;
                int c = 0;
                int lastImprovement = 0;
                Relocation rm = new();
                Swap sm = new();
                TwoOpt top = new();
                Flip flip = new();
                PrioritySwap psm = new();
                Solution localBest = new();
                localBest.Cost = double.MaxValue;
                currentSol = new();
                //double temperature = 500; Remove comment for Simulated Annealing

                if (settings.multiRestart)
                {
                    currentSol = startingSolution.DeepCopy(startingSolution);
                }
                else
                {

                    SetRoutedToFalse(currentSol.Customers);
                    SetServedToFalse(currentSol.Options);
                    if (!MinimumInsertions(currentSol, rnd5, timesFailedFindFeasible))
                    {
                        timesFailedFindFeasible++;
                        numberOfRestarts++;
                        continue;
                    }
                    RemoveEmptyRoutes(currentSol);
                    timesFailedFindFeasible = 0;
                }

                restartCounter++;
                for (int i = 0; i < settings.repetitions; i++)
                {
                    if (i - lastImprovement > 3000)
                    {
                        break;
                    }

                    reinitCount++;
                    rm.ReinitializeVariables();
                    sm.ReinitializeVariables();
                    top.ReinitializeVariables();
                    flip.ReinitializeVariables();
                    psm.ReinitializeVariables();

                    if (reinitCount == currentSol.Options.Count * settings.promisesRestartRatio)
                    {
                        currentSol.InitPromises();
                        reinitCount = 0;
                    }
                    if (settings.schema == "greedy" && localBest.Cost != double.MaxValue)
                    {
                        if (i - lastImprovement > 1000 && rnd.NextDouble() > 0.98 && (currentSol.Cost - localBest.Cost) / localBest.Cost <= 0.08)
                        {
                            psm = ls.FindBestPrioritySwapMove(psm, currentSol);
                            Console.WriteLine("Apply random priority swap");
                            ls.ApplyPrioritySwapMove(psm, currentSol);
                        }

                        sm = ls.FindBestSwapMove(sm, currentSol);
                        rm = ls.FindBestRelocationMove(rm, currentSol);
                        top = ls.FindBestTwoOptMove(top, currentSol);
                        psm = ls.FindBestPrioritySwapMove(psm, currentSol);

                        if (i > 2000 && ((i - c) > 500))//2000 and 500 1000 and 200 better
                        {
                            c = i;
                            flip = ls.FindBestFlipMove(flip, currentSol);
                        }

                        var mincost = double.MaxValue;
                        mincost = FindMinMoveCost(sm, rm, top, flip, psm);
                        
                        /* Remove Comment for Simulated Annealing
                        if (mincost > 0) {
                            double delta = mincost;
                            double probability = Math.Exp(-delta / temperature);
                            Console.WriteLine("Temperature: " + temperature + "Delta: " + delta);
                            Console.WriteLine("Probability: " + probability);
                            double pt = rnd.Nextdouble();
                            if (pt >= probability) {
                                continue;
                            }
                        }
                        */
                        
                        //if (mincost > 2000)
                        //{
                        //    continue;
                        //}
                        
                        if (mincost == sm.TotalCost)
                        {
                            ls.ApplySwapMove(sm, currentSol);
                        }
                        else if (mincost == rm.TotalCost)
                        {
                            ls.ApplyRelocationMove(rm, currentSol);
                        }
                        else if (mincost == top.TotalCost)
                        {
                            ls.ApplyTwoOptMove(top, currentSol);
                        }
                        else if (mincost == flip.TotalCost)
                        {
                            ls.ApplyFlipMove(flip, currentSol);
                        }
                        else if (mincost == psm.TotalCost)
                        {
                            ls.ApplyPrioritySwapMove(psm, currentSol);
                        }
                        RemoveEmptyRoutes(currentSol);
                        currentSol.SolutionUtilizationMetric = currentSol.CalculateUtilizationMetric();
                        //Console.WriteLine("Utilization Metric: " + currentSol.SolutionUtilizationMetric);
                        
                        //temperature *= 0.9997; //Remove comment for Simulated Annealing
                    }
                    else if (settings.schema == "random" || localBest.Cost == double.MaxValue)
                    {
                        int k = rnd.Next(1, 6);
                        if (k == 4)
                        {
                            sm = ls.FindBestSwapMove(sm, currentSol);
                            ls.ApplySwapMove(sm, currentSol);
                        }
                        else if (k == 2)
                        {
                            rm = ls.FindBestRelocationMove(rm, currentSol);
                            ls.ApplyRelocationMove(rm, currentSol);
                        }
                        else if (k == 3)
                        {
                            top = ls.FindBestTwoOptMove(top, currentSol);
                            ls.ApplyTwoOptMove(top, currentSol);
                        }
                        else if (k == 1)
                        {
                            psm = ls.FindBestPrioritySwapMove(psm, currentSol);
                            ls.ApplyPrioritySwapMove(psm, currentSol);
                        }
                        else if (k == 5)
                        {
                            if ((i - c) > 1 && i > 1)
                            {
                                c = i;
                                flip = ls.FindBestFlipMove(flip, currentSol);
                                var service_level = CalculateServiceLevel(currentSol, false);
                                if (service_level[0] > 0.8 && service_level[1] > 0.9)
                                {
                                    if (flip.MoveCost < 0)
                                    {
                                        ls.ApplyFlipMove(flip, currentSol);
                                    }
                                }
                                else
                                {
                                    ls.ApplyFlipMove(flip, currentSol);
                                }
                            }
                            else
                            {
                                sm = ls.FindBestSwapMove(sm, currentSol);
                                rm = ls.FindBestRelocationMove(rm, currentSol);
                                top = ls.FindBestTwoOptMove(top, currentSol);
                                if (rm.TotalCost < sm.TotalCost && rm.TotalCost < top.TotalCost)
                                {
                                    ls.ApplyRelocationMove(rm, currentSol);
                                }
                                else if (sm.TotalCost < top.TotalCost && sm.TotalCost < rm.TotalCost)
                                {
                                    ls.ApplySwapMove(sm, currentSol);
                                }
                                else
                                {
                                    ls.ApplyTwoOptMove(top, currentSol);

                                }
                            }
                        }
                    }
                    RemoveEmptyRoutes(currentSol);
                    if (!currentSol.CheckEverything(currentSol))
                    {
                        Console.WriteLine("Infeasible Solution!!!");
                    }

                    var serviceLevel = CalculateServiceLevel(currentSol, false);
                    int openRoutes = currentSol.Routes.Count(x => x.SequenceOfLocations.Count > 2);
                    if (currentSol.Cost + openRoutes * 10000 < localBest.Cost + localBest.Routes.Count * 10000 + 0.001 && currentSol.Cost < 100000 && serviceLevel[0] >= 0.8 && serviceLevel[1] >= 0.9) {
                        currentSol.Repetition = i;
                        localBest = currentSol.DeepCopy(currentSol);
                        localBest.RemoveEmptyRoutes();
                        lastImprovement = i;
                        if (!localBest.CheckEverything(localBest))
                        {
                            Console.WriteLine();
                        }
                        currentSol.Routes = currentSol.Routes.Where(rt => rt.SequenceOfLocations.Count != 2).ToList();
                        Console.WriteLine("{0} {1} {2} {3} {4}", i, currentSol.Cost, localBest.Cost, localBest.Routes.Count(x => x.SequenceOfLocations.Count > 2), currentSol.LastMove);
                    }
                    if (settings.verbal)
                    {
                        Console.WriteLine("{0} {1} {2} {3}", i, (double)currentSol.Cost, localBest.Cost, localBest.Routes.Count(x => x.SequenceOfLocations.Count > 2));
                    }
                }
                restartTimer.Stop();
                CalculateServiceLevel(localBest);
                PrintSolution(localBest);

                if (localBest.Cost + localBest.Routes.Count * 10000 < bestSol.Cost + bestSol.Routes.Count * 10000 + 0.001 && localBest.Cost < 100000)
                {
                    bestSol = localBest.DeepCopy(localBest);
                    bestSol.Restart = restartCounter;
                    bestSol.Repetition = localBest.Repetition;
                    globalBestTime = restartTimer.Elapsed;
                }

                Console.WriteLine("///////////////////////");
                Console.WriteLine(localBest.Cost + " " + bestSol.Cost);
            }
            CalculateServiceLevel(bestSol);
            return bestSol;
        }

        void RemoveEmptyRoutes(Solution sol)
        {
            if (sol.Routes[sol.Routes.Count - 1].Load == 0)
            {
                sol.Routes.RemoveAt(sol.Routes.Count - 1);
            }
        }

        List<Solution> ConstructFeasibleSolutions()
        {
            List<Solution> solutionList = new List<Solution>();
            Random rnd = new Random(42);
            int Solutions = 10;
            int timesFailedFindFeasible = 0;

            for (int i = 0; i < Solutions; i++)
            {
                Solution sol = new Solution();
                SetRoutedToFalse(sol.Customers);
                SetServedToFalse(sol.Options);
                solutionList.Add(sol);
            
                Console.WriteLine("----");
                var selectedOptions = new List<Option>();

                foreach (Customer cus in sol.Customers)
                {
                    if (cus.Options.Count == 1)
                    {
                        selectedOptions.Add(cus.Options[0]);
                        cus.IsRouted = true;
                    }
                }
                while (CalculateServiceLevel(selectedOptions, sol)[0] < 0.8)
                {
                    InsertBestFirstOption2(selectedOptions, sol);
                    Console.WriteLine(CalculateServiceLevel(selectedOptions, sol)[0] + "  " + CalculateServiceLevel(selectedOptions, sol)[1]);
                }
                /**while (CalculateServiceLevel(selectedOptions, sol)[1] < 0.9)
                {
                    InsertBestFirstOption2(selectedOptions, sol);
                    Console.WriteLine(CalculateServiceLevel(selectedOptions, sol)[0] + "  " + CalculateServiceLevel(selectedOptions, sol)[1]);
                }**/ 
                Dictionary<Option, double> sc = new Dictionary<Option, double>();
                List<Option> opt;
                Option tempOpt;
                // add customers that are not served
                foreach (Customer cus in sol.Customers)
                {
                    if (!cus.IsRouted)
                    {
                        sc = CalculateObjective(selectedOptions, sol, cus.Options);
                        opt = sc.OrderBy(kvp => kvp.Value).Select(pair => pair.Key).ToList();
                        tempOpt = CountOptionsAndCheckMaxCapacity(selectedOptions, opt);
                        selectedOptions.Add(tempOpt);
                        tempOpt.Cust.IsRouted = true;
                        sc.Clear();
                    }
                }

                // Also, check for each option if teh capacity is violated if the option is added
                SetRoutedToFalse(sol.Customers);
                SetServedToFalse(selectedOptions);
                if (!MinimumInsertions(sol, rnd, timesFailedFindFeasible, selectedOptions))
                {
                    timesFailedFindFeasible++;
                    solutionList.Remove(sol);
                    Solutions++;
                    continue;
                };
                timesFailedFindFeasible = 0;
                Console.WriteLine("Solution " + i);
                Console.WriteLine("------------");
            }
            return solutionList;
        }

        public Option CountOptionsAndCheckMaxCapacity(List<Option> options, List<Option> opts)
        {
            var groupedByLocation = options
                .Where(option => option.Location.Type == 1)
                .GroupBy(option => option.Location)
                .Select(group => new
                {
                    LocationId = group.Key.Id,
                    Count = group.Count(),
                    MaxCapacity = group.Key.MaxCap
                });

            foreach (var opt in opts)
            {
                if (opt.Location.Type == 1)
                {
                    var tup = groupedByLocation.Where(x => x.LocationId == opt.Location.Id && x.Count < x.MaxCapacity).Any();
                    if (tup)
                    {
                        return opt;
                    }
                }
                else
                {
                    return opt;
                }
            }
            return opts[0];
        }

        void CalculateSimilarity(List<Solution> solutions)
        {
            Dictionary<int, List<Option>> dict = new Dictionary<int, List<Option>>();
            int count = 0;
            foreach (Solution sol in solutions)
            {
                List<Option> temp = new List<Option>();
                for (int k = 0; k < sol.Routes.Count; k++)
                {
                    temp.AddRange(sol.Routes[k].SequenceOfOptions.GetRange(1, sol.Routes[k].SequenceOfOptions.Count - 2));
                }
                dict.Add(count, temp);
                count++;
            }

            double similarity = 0;
            for (int i = 0; i < solutions.Count - 1; i++)
            {
                for (int j = i + 1; j < solutions.Count; j++)
                {
                    similarity = CalculateJaccardIndex(dict[i], dict[j]);
                    Console.WriteLine($"Similarity between solution {i + 1} and solution {j + 1}: {similarity}");
                }
            }
        }

        double CalculateJaccardIndex(List<Option> listA, List<Option> listB)
        {
            int matchingCount = 0;
            int count = listA.Count;
            for (int i = 0; i < count; i++)
            {
                if (listA[i].Id == listB[i].Id)
                {
                    matchingCount++;
                }
            }

            return matchingCount / count;
        }


        void InsertBestFirstOption2(List<Option> selectedOptions, Solution sol)
        {
            double bestOptionObjective = double.MaxValue;

            Dictionary<Option, double> optionScores = new Dictionary<Option, double>();

            optionScores = CalculateObjective(selectedOptions, sol, AvailableFirstOptions(sol, selectedOptions));

            if (optionScores.Any())
            { 
                var topThreeOptions = optionScores.OrderBy(kvp => kvp.Value).Take(3).ToDictionary(kvp => kvp.Key, kvp => kvp.Value);

                // Random selection among the top options
                var keysList = topThreeOptions.Keys.ToList();
                Option bestOption = keysList[rnd2.Next(keysList.Count)];
                bestOptionObjective = topThreeOptions[bestOption];

                selectedOptions.Add(bestOption);
                bestOption.Cust.IsRouted = true;
                Console.WriteLine($"Best Option: {bestOption.Id} Best Option's Objective: {bestOptionObjective}");
            }
        }

        Dictionary<Option, double> CalculateObjective(List<Option> selectedOptions, Solution sol, List<Option> candOptions)
        {
            double weightDistance = 1;
            double weightDueReadyDiff = 0.25;
            double weightOverlap = 0.25;
            List<Option> nearestOptions;
            double weightedSum;
            Dictionary<Option, double> scores = new Dictionary<Option, double>();
            double maxDistance = sol.DistanceMatrix.Cast<double>().Max();
            double maxDuration = sol.Depot.Due - sol.Depot.Ready;
            foreach (Option option in candOptions)
            {
                nearestOptions = FindXnearest(option, sol, selectedOptions);
                weightedSum = (weightDistance * DistanceFromXnearest(nearestOptions, option, sol)) / (5 * maxDistance) -
                (weightDueReadyDiff * (option.Location.Due - option.Location.Ready) / (5 * maxDuration))
                                       + (weightOverlap * (CalculateOverlap(option, nearestOptions) / (5 * maxDuration)));
                scores[option] = weightedSum;
            }

            return scores;
        }


        List<Option> AvailableFirstOptions(Solution sol, List<Option> selectedOptions)
        {
            var groupedByLocation = selectedOptions
                .Where(option => option.Location.Type == 1)
                .GroupBy(option => option.Location)
                .Select(group => new
                {
                    LocationId = group.Key.Id,
                    Count = group.Count(),
                    MaxCapacity = group.Key.MaxCap
                });
            List<Option> tempOptions = new List<Option>();
            foreach (Customer cus in sol.Customers)
            {
                if (!cus.IsRouted)
                {
                    var firstOpt = cus.Options.OrderBy(x => x.Prio).FirstOrDefault();
                    if (firstOpt.Location.Type == 1)
                    {
                        var tup = groupedByLocation.Where(x => x.LocationId == firstOpt.Location.Id && x.Count < x.MaxCapacity).Any();
                        if (tup)
                        {
                            tempOptions.Add(firstOpt);
                        }
                    } else
                    {
                        tempOptions.Add(firstOpt);
                    }
                }
            }
            return tempOptions;
        }

        public List<Option> AvailableSecondOptions(Solution sol)
        {
            List<Option> tempOptions = new List<Option>();
            foreach (Customer cus in sol.Customers)
            {
                if (!cus.IsRouted)
                {
                    // Skip the first option and take the next one
                    var secondOption = cus.Options.OrderBy(x => x.Prio).Skip(1).FirstOrDefault();
                }
            }
            return tempOptions;
        }
        public List<Option> AvailableOptionsRandomly(Solution sol)
        {
            List<Option> tempOptions = new List<Option>();
            Random random = new Random(42); // Random number generator

            foreach (Customer cus in sol.Customers)
            {
                if (!cus.IsRouted && cus.Options.Any())
                {
                    // Determine the number of options to consider (either 1 or 2)
                    int optionsToConsider = 2;

                    // Randomly choose between the first and the second option (if available)
                    var chosenOption = cus.Options.OrderBy(x => x.Prio).Take(optionsToConsider).ElementAt(random.Next(optionsToConsider));

                    tempOptions.Add(chosenOption);
                }
            }
            return tempOptions;
        }

        double CalculateOverlap(Option target, List<Option> nrOptions)
        {
            double overlap = 0;
            foreach (var opt in nrOptions)
            {
                overlap += Math.Max(0, Math.Min(target.Due, opt.Due) - Math.Max(target.Ready, opt.Ready));
            }
            return overlap;
        }

        List<Option> FindXnearest(Option targetOpt, Solution sol, List<Option> selectedOptions)
        {
            // Sort the points based on distance to central point ** maybe it should be calculated candidate option distance to already selected options
            //List<Option> sortedPoints = availableFirstOptions(sol).OrderBy(point => sol.CalculateDistance(targetOpt.Location, point.Location)).ToList();
            List<Option> sortedPoints = selectedOptions.OrderBy(point => sol.CalculateDistance(targetOpt.Location, point.Location)).ToList();

            // Take the first k points as the nearest neighbors
            List<Option> nearestNeighbors = sortedPoints.Take(5).ToList();

            return nearestNeighbors;
        }


        double DistanceFromXnearest(List<Option> opts, Option target, Solution sol)
        {
            double sumOfDistances = opts.Sum(neighbor => sol.CalculateDistance(target.Location, neighbor.Location));

            // Calculate the average distance
            double averageDistance = sumOfDistances / opts.Count;

            return averageDistance;
        }


        void RestoreFeasibility(Solution sol, Flip flip)
        {
            flip.ReinitializeVariables();
            while (CalculateServiceLevel(sol)[0] < 80 || CalculateServiceLevel(sol)[1] < 90)
            {
                flip.ReinitializeVariables();
                flip = ls.FindBestFlipMove(flip, sol, false);
                if (!flip.IsValid())
                {
                    break;
                }
                ls.ApplyFlipMove(flip, sol);
                Console.WriteLine("cost: " + sol.Cost);
            }
        }

        void PrintSolution(Solution sol)
        {
            Console.WriteLine("///////////////////");
            if (sol.CheckEverything(sol)) { Console.WriteLine("Feasible"); }
            else { Console.WriteLine("INFEASIBLE!"); }

            Console.WriteLine("Solution Cost: {0}", sol.Cost);

            foreach (Route r in sol.Routes)
            {
                Console.WriteLine("--------------");
                Console.WriteLine("Route {0}:", r.Id); Console.WriteLine();
                if (r.SequenceOfLocations.Count == 2)
                {
                    sol.Cost -= r.Cost;
                    continue;
                }
                Console.WriteLine("LOCATION | CUSTOMER");
                for (int i = 0; i < r.SequenceOfOptions.Count; i++)
                {
                    Console.WriteLine("{0} {1}", r.SequenceOfOptions[i].Location.Id, r.SequenceOfOptions[i].Cust.Id);
                }
                Console.WriteLine("Max capacity: {0} Load:{1}", r.Capacity, r.Load);
                if (!sol.CheckRouteFeasibility(r)) { Console.WriteLine("INFEASIBLE!"); }
                Console.WriteLine("--------------");
            }
            Console.WriteLine("Solution Cost: " + sol.Cost);
            CalculateServiceLevel(sol);
            Console.WriteLine("///////////////////");
        }

        private double FindMinMoveCost(Swap sm, Relocation rm, TwoOpt top, Flip flip, PrioritySwap psm) => Math.Min(Math.Min(Math.Min(Math.Min(sm.TotalCost, rm.TotalCost), top.TotalCost), flip.TotalCost), psm.TotalCost);
        private double FindMinMoveCost(Swap sm, Relocation rm, TwoOpt top, Flip flip) => Math.Min(Math.Min(Math.Min(sm.TotalCost, rm.TotalCost), top.TotalCost), flip.TotalCost);
        private double FindMinMoveCost(Relocation rm, Flip flip, PrioritySwap psm) => Math.Min(Math.Min(rm.TotalCost, flip.TotalCost), psm.TotalCost);
        private double FindMinMoveCost(Swap sm, Relocation rm, Flip flip, PrioritySwap psm) => Math.Min(Math.Min(Math.Min(sm.TotalCost, rm.TotalCost), flip.TotalCost), psm.TotalCost);
        void SetRoutedToFalse(List<Customer> customers)
        {
            foreach(Customer customer1 in customers)
            {
                customer1.IsRouted = false;
            }
        }

        void SetServedToFalse(List<Option> options)
        {
            foreach (Option option1 in options)
            {
                option1.IsServed = false;
            }
        }

        void AlwaysKeepAnEmptyRoute(Solution sol)
        {
            if (sol.Routes.Count < 100)
            {
                if (sol.Routes.Count == 0)
                {
                    Route newRoute = new(sol.Routes.Count, sol.Cap, sol.Depot);
                    sol.Routes.Add(newRoute);
                    sol.Cost += newRoute.Cost;
                }
                else
                {
                    if (sol.Routes.Last().SequenceOfLocations.Count > 2)
                    {
                        Route newRoute = new(sol.Routes.Count, sol.Cap, sol.Depot);
                        sol.Routes.Add(newRoute);
                        sol.Cost += newRoute.Cost;
                    }
                }
            }
        }


        void ReportSolution(Solution sol, TimeSpan restartTime, TimeSpan totalTime)
        {
            string jsonContent = File.ReadAllText("settings.json");
            var settings = JsonSerializer.Deserialize<Settings>(jsonContent);
            StreamWriter writetext = new(instance.Replace(".txt", settings.schema + ".txt"));

            if (sol.CheckEverything(sol)) { writetext.WriteLine("Feasible"); }
            else { writetext.WriteLine("INFEASIBLE!"); }

            writetext.WriteLine("Restart {0}, Repetition {1} \n", sol.Restart, sol.Repetition);
            writetext.WriteLine("Total time: {0}, Restart time: {1} \n", totalTime.ToString(), restartTime.ToString());
            writetext.WriteLine("Total cost: " + sol.Cost + "\n");
            writetext.WriteLine("Number of Routes: {0}", sol.Routes.Count(list => list.SequenceOfLocations.Count > 2));

            var priorities = CalculateServiceLevel(sol, false);
            writetext.WriteLine("Priority 1: " + priorities[0] + "\n");
            writetext.WriteLine("Priority 2: " + priorities[1] + "\n");

            writetext.WriteLine("\n");

            for (int i = 0; i < sol.Routes.Count; i++)
            {
                writetext.WriteLine("Route " + Convert.ToString(i) + "\n" + "Location " + "Option " + "Customer" + "\n");
                Route rt = sol.Routes[i];
                for (int j = 0; j < rt.SequenceOfOptions.Count; j++)
                {
                    if (j == 0 | j == rt.SequenceOfOptions.Count - 1)
                    {
                        writetext.WriteLine(rt.SequenceOfLocations[j].Id + " " + "-" + " " + "-" + "\n");
                    }
                    else
                    {
                        writetext.WriteLine(rt.SequenceOfLocations[j].Id + " " + rt.SequenceOfOptions[j].Id + " " + rt.SequenceOfCustomers[j].Id + "\n");
                    }
                }
            }
            writetext.Close();
            writetext = new("log.txt", true);
            writetext.WriteLine("{0} {1} {2} {3}", instance.Replace(".txt", " "), sol.Cost, sol.Routes.Count(list => list.SequenceOfLocations.Count > 2), DateTime.Now.ToString());
            writetext.Close();
        }


        Option candidateOpt;
        Location A, B;
        double timeAdded, timeRemoved, trialTime;
        double costAdded, costRemoved, trialCost, costAddedPenalized, costRemovedPenalized, trialCostPenalized;
        double[] tw;

        public string Instance { get => instance; set => instance = value; }

        List<CustomerInsertionAllPositions> IdentifyMinimumCostInsertion(CustomerInsertionAllPositions bestInsertion, Solution sol, List<Option> selectedOptions = null)
        {

            List<CustomerInsertionAllPositions> topThree = new List<CustomerInsertionAllPositions>
            {
                bestInsertion
            };
            int options;
            if (selectedOptions == null)
            {
                selectedOptions = sol.Options; // maybe deep copy
            }

            for (int i = 0; i < selectedOptions.Count; i++)
            {
                candidateOpt = selectedOptions[i];
                if (candidateOpt.Cust.IsRouted == false & candidateOpt.IsServed == false)
                {
                    foreach (Route rt in sol.Routes)
                    {
                        if (rt.Load + candidateOpt.Cust.Dem <= rt.Capacity)
                        {
                            for (int j = 0; j < rt.SequenceOfLocations.Count - 1; j++)
                            {
                                A = rt.SequenceOfLocations[j];
                                B = rt.SequenceOfLocations[j + 1];
                                timeAdded = sol.CalculateTime(A, candidateOpt.Location) + sol.CalculateTime(candidateOpt.Location, B);
                                timeRemoved = sol.CalculateTime(A, B);
                                costAdded = sol.CalculateDistance(A, candidateOpt.Location) + sol.CalculateDistance(candidateOpt.Location, B);
                                costRemoved = sol.CalculateDistance(A, B);
                                trialCost = costAdded - costRemoved;
                                trialTime = timeAdded - timeRemoved + candidateOpt.Location.ServiceTime;
                                var t = sol.RespectsTimeWindow2(rt, j, candidateOpt.Location);

                                if (t.Item1)
                                {
                                    if (trialCost <= topThree.Last().Cost || topThree.Count < 3)
                                    {
                                        if (candidateOpt.Location.Type == 2 | candidateOpt.Location.Cap < candidateOpt.Location.MaxCap)
                                        {
                                            bestInsertion.Option = candidateOpt;
                                            bestInsertion.Customer = candidateOpt.Cust;
                                            bestInsertion.Location = candidateOpt.Location;
                                            bestInsertion.Route = rt;
                                            bestInsertion.InsertionPosition = j + 1;
                                            bestInsertion.Duration = trialTime;
                                            bestInsertion.Cost = trialCost;
                                            bestInsertion.Ect = t.Item2[j + 1];
                                            bestInsertion.Lat = t.Item3[j + 1];
                                            CustomerInsertionAllPositions custTemp = new CustomerInsertionAllPositions(bestInsertion);
                                            topThree.Add(custTemp);
                                            topThree = topThree.OrderBy(o=>o.Cost).ToList();
                                            topThree = topThree.Take(3).ToList();
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            return topThree;
        }

        List<CustomerInsertionAllPositions> IdentifyMinimumPenalizedCostInsertion(CustomerInsertionAllPositions bestInsertion, Solution sol, List<Option> selectedOptions = null)
        {

            List<CustomerInsertionAllPositions> topThree = new List<CustomerInsertionAllPositions>
            {
                bestInsertion
            };
            int options;
            if (selectedOptions == null)
            {
                selectedOptions = sol.Options;
            }

            for (int i = 0; i < selectedOptions.Count; i++)
            {
                candidateOpt = selectedOptions[i];
                if (candidateOpt.Cust.IsRouted == false & candidateOpt.IsServed == false)
                {
                    foreach (Route rt in sol.Routes)
                    {
                        if (rt.Load + candidateOpt.Cust.Dem <= rt.Capacity)
                        {
                            for (int j = 0; j < rt.SequenceOfLocations.Count - 1; j++)
                            {
                                A = rt.SequenceOfLocations[j];
                                B = rt.SequenceOfLocations[j + 1];
                                timeAdded = sol.CalculateTime(A, candidateOpt.Location) + sol.CalculateTime(candidateOpt.Location, B);
                                timeRemoved = sol.CalculateTime(A, B);
                                int routeIndex = sol.Routes.IndexOf(rt);
                                double loadFactor = (rt.Load + candidateOpt.Cust.Dem) / rt.Capacity; // Use this to fill the routes in a balanced way. Noticed that works better in practice. 
                                double routePenalty = routeIndex * 400 * (1 + loadFactor);
                                costAddedPenalized = sol.CalculateDistance(A, candidateOpt.Location) + sol.CalculateDistance(candidateOpt.Location, B) + routePenalty; // for every new route that opens the penalty is higher
                                costAdded = sol.CalculateDistance(A, candidateOpt.Location) + sol.CalculateDistance(candidateOpt.Location, B);
                                costRemovedPenalized = sol.CalculateDistance(A, B);
                                costRemoved = sol.CalculateDistance(A, B);
                                trialCostPenalized = costAddedPenalized - costRemovedPenalized;
                                trialCost = costAdded - costRemoved;
                                trialTime = timeAdded - timeRemoved + candidateOpt.Location.ServiceTime;

                                var t = sol.RespectsTimeWindow2(rt, j, candidateOpt.Location);
                                //Console.WriteLine("Candidate options with id " + candidateOpt.Id + " and penalized cost " + trialCostPenalized);

                                if (t.Item1)
                                {
                                    if (trialCostPenalized <= topThree.Last().CostPenalized || topThree.Count < 3)
                                    {
                                        if (candidateOpt.Location.Type == 2 | candidateOpt.Location.Cap < candidateOpt.Location.MaxCap)
                                        {
                                            bestInsertion.Option = candidateOpt;
                                            bestInsertion.Customer = candidateOpt.Cust;
                                            bestInsertion.Location = candidateOpt.Location;
                                            bestInsertion.Route = rt;
                                            bestInsertion.InsertionPosition = j + 1;
                                            bestInsertion.Duration = trialTime;
                                            bestInsertion.Cost = trialCost;
                                            bestInsertion.CostPenalized = trialCostPenalized;
                                            bestInsertion.Ect = t.Item2[j + 1];
                                            bestInsertion.Lat = t.Item3[j + 1];

                                            CustomerInsertionAllPositions custTemp = new CustomerInsertionAllPositions(bestInsertion);
                                            if (!topThree.Contains(custTemp))
                                            {
                                                topThree.Add(custTemp);
                                                topThree = topThree.OrderBy(o => o.CostPenalized).ToList();
                                                topThree = topThree.Take(3).ToList();
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            return topThree;
        }



        void ApplyCustomerInsertionAllPositions(CustomerInsertionAllPositions insertion, Solution sol)
        {
            insertion.Route.SequenceOfLocations.Insert(insertion.InsertionPosition, insertion.Location);
            insertion.Route.SequenceOfCustomers.Insert(insertion.InsertionPosition, insertion.Customer);
            insertion.Route.SequenceOfOptions.Insert(insertion.InsertionPosition, insertion.Option);
            insertion.Route.Duration += insertion.Duration;
            insertion.Route.Cost += insertion.Cost;
            insertion.Route.Load += insertion.Customer.Dem;
            insertion.Route.RouteUtilizationMetric = Math.Pow(Convert.ToDouble(insertion.Route.Capacity - insertion.Route.Load), 2);
            insertion.Customer.IsRouted = true;
            insertion.Option.IsServed = true;
            sol.Cost += insertion.Cost;
            sol.Duration += insertion.Duration;
            insertion.Location.Cap += 1;
            insertion.Route.SequenceOfEct.Insert(insertion.InsertionPosition, insertion.Ect);
            insertion.Route.SequenceOfLat.Insert(insertion.InsertionPosition, insertion.Lat);
            sol.UpdateTimes(insertion.Route);
        }

        bool MinimumInsertions(Solution sol, Random rnd, int timesFailedFindFeasible, List<Option> selectedOptions = null)
        {
            bool modelIsFeasible = true;
            bool failed = false;
            if (timesFailedFindFeasible < 100)
            {
                failed = true;
            }

            while (sol.Customers.Any(x => !x.IsRouted))
            {   
                bestInsertion = new CustomerInsertionAllPositions();
                List<CustomerInsertionAllPositions> topThree = new List<CustomerInsertionAllPositions>();
                List<CustomerInsertionAllPositions> topThreeOfPenalized = new List<CustomerInsertionAllPositions>();
                AlwaysKeepAnEmptyRoute(sol);
                if (selectedOptions != null)
                {
                    if (failed == true)
                    {
                        topThreeOfPenalized = IdentifyMinimumPenalizedCostInsertion(bestInsertion, sol, selectedOptions);
                    }
                    else
                    {
                        topThree = IdentifyMinimumCostInsertion(bestInsertion, sol, selectedOptions);
                    }
                } else
                {
                    if (failed == true)
                    {
                        topThreeOfPenalized = IdentifyMinimumPenalizedCostInsertion(bestInsertion, sol);
                    }
                    else
                    {
                        topThree = IdentifyMinimumCostInsertion(bestInsertion, sol);
                    }
                }
                if (failed == true)
                {
                    bestInsertion = topThreeOfPenalized[rnd.Next(topThreeOfPenalized.Count)];
                } else
                {
                    bestInsertion = topThree[rnd.Next(topThree.Count)];
                }

                if (bestInsertion.Customer != null)
                {
                    ApplyCustomerInsertionAllPositions(bestInsertion, sol);
                } else
                {
                    //Console.WriteLine("Initial solution is not feasible. Try again !");
                    modelIsFeasible = false;
                    if (failed == true)
                    {
                        timesFailedFindFeasible++;
                    }
                    return modelIsFeasible;
                }
            }
            return modelIsFeasible;
        }
        double[] CalculateServiceLevel(Solution sol, bool verbal = true)
        {
            int po0Sum = 0;
            int po1Sum = 0;
            int po2Sum = 0;
            double sum = 0;
            int po = -1;

            for (int r = 0; r < sol.Routes.Count; r++)
            {
                for (int c = 1; c < sol.Routes[r].SequenceOfOptions.Count - 1; c++)
                {
                    po = sol.Routes[r].SequenceOfOptions[c].Prio;
                    switch (po)
                    {
                        case 0:
                            po0Sum++;
                            break;
                        case 1:
                            po1Sum++;
                            break;
                        case 2:
                            po2Sum++;
                            break;
                    }
                }
            }
            sum = po0Sum + po1Sum + po2Sum;
            var sl0 = po0Sum / sum;
            var sl1 = (po0Sum + po1Sum) / sum;
            if (verbal) {
                Console.WriteLine("Priority 1: {0}", sl0);
                Console.WriteLine("Priority 2: {0}", sl1);
            }

            return new double[] {sl0, sl1};
        }

        double[] CalculateServiceLevel(List<Option> selectedOptions, Solution sol)
        {
            int po0Sum = 0;
            int po1Sum = 0;
            int po2Sum = 0;
            double sum = 0;
            int po = -1;

            for (int c = 0; c < selectedOptions.Count - 1; c++)
            {
                po = selectedOptions[c].Prio;
                switch (po)
                {
                    case 0:
                        po0Sum++;
                        break;
                    case 1:
                        po1Sum++;
                        break;
                    case 2:
                        po2Sum++;
                        break;
                }
            }
            sum = po0Sum + po1Sum + po2Sum;
            var sl0 = po0Sum / sol.Customers.Count;
            var sl1 = (po0Sum + po1Sum) / sol.Customers.Count;
            Console.WriteLine("Priority 1: {0}", sl0);
            Console.WriteLine("Priority 2: {0}", sl1);
            return new double[] { sl0, sl1 };
        }
    }
}
