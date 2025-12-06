using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection.Metadata.Ecma335;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace VrdpoProject
{

    public class LocalSearch
    {

        private Route rt1, rt2;
        public Relocation FindBestRelocationMove(Relocation rm, Solution sol)
        {
            int openRoutes;
            for (int originRouteIndex = 0; originRouteIndex < sol.Routes.Count; originRouteIndex++)
            {
                rt1 = sol.Routes[originRouteIndex];

                for (int targetRouteIndex = 0; targetRouteIndex < sol.Routes.Count; targetRouteIndex++)
                {
                    rt2 = sol.Routes[targetRouteIndex];

                    for (int originOptionIndex = 1; originOptionIndex < rt1.SequenceOfOptions.Count - 1; originOptionIndex++)
                    {
                        for (int targetOptionIndex = 0; targetOptionIndex < rt2.SequenceOfOptions.Count - 1; targetOptionIndex++)
                        {
                            openRoutes = sol.Routes.Count;
                            if (originRouteIndex == targetRouteIndex && (targetOptionIndex == originOptionIndex || targetOptionIndex == originOptionIndex - 1))
                            {
                                continue;
                            }

                            var tw = sol.RespectsTimeWindow2(rt2, targetOptionIndex,
                                            rt1.SequenceOfLocations[originOptionIndex]);

                            if (!tw.Item1) { continue; };

                            Option A = rt1.SequenceOfOptions[originOptionIndex - 1];
                            Option B = rt1.SequenceOfOptions[originOptionIndex];
                            Option C = rt1.SequenceOfOptions[originOptionIndex + 1];

                            Option F = rt2.SequenceOfOptions[targetOptionIndex];
                            Option G = rt2.SequenceOfOptions[targetOptionIndex + 1];
                            if (rt1 != rt2)
                            {
                                if (rt2.Load + B.Cust.Dem > rt2.Capacity)
                                {
                                    continue;
                                }
                            }
                            if (rt1.Load - B.Cust.Dem == 0) { // if route becomes empty
                                //Console.WriteLine("This RELOCATION move empties a route");
                                //Console.WriteLine("Routes before : " + openRoutes);
                                openRoutes--;
                                //Console.WriteLine("Routes after : " + openRoutes);
                            }

                            double costAdded = sol.CalculateDistance(A.Location, C.Location) + sol.CalculateDistance(F.Location, B.Location)
                                                + sol.CalculateDistance(B.Location, G.Location);
                            double costRemoved = sol.CalculateDistance(A.Location, B.Location) + sol.CalculateDistance(B.Location, C.Location)
                                                + sol.CalculateDistance(F.Location, G.Location);
                            double moveCost = costAdded - costRemoved;

                            double costChangeOriginRt = sol.CalculateDistance(A.Location, C.Location) - sol.CalculateDistance(A.Location, B.Location)
                                                - sol.CalculateDistance(B.Location, C.Location);
                            double costChangeTargetRt = sol.CalculateDistance(F.Location, B.Location) + sol.CalculateDistance(B.Location, G.Location)
                                                - sol.CalculateDistance(F.Location, G.Location);

                            var newUtilizationMetricRoute1 = Math.Pow(Convert.ToDouble(rt1.Capacity - (rt1.Load - B.Cust.Dem)), 2);
                            var newUtilizationMetricRoute2 = Math.Pow(Convert.ToDouble(rt2.Capacity - (rt2.Load + B.Cust.Dem)), 2);
                            var newSolUtilizationMetric = sol.SolutionUtilizationMetric - rt1.RouteUtilizationMetric - rt2.RouteUtilizationMetric + newUtilizationMetricRoute1 + newUtilizationMetricRoute2;
                            var ratio = (sol.SolutionUtilizationMetric + 1) / (newSolUtilizationMetric + 1);
                            if (sol.Routes.Count == sol.LowerBoundRoutes)
                            {
                                ratio = 1;
                            }
                            sol.RatioCombinedMoveCost = ratio * moveCost;

                            //favor relocations from very small routes
                            //int bonus = 0;
                            //if (rt1.SequenceOfLocations.Count <= 4)
                            //{
                            //    bonus = -2000;
                            //}
                            ////prevent relocating to empty/small routes
                            //if (rt2.SequenceOfLocations.Count <= 4)
                            //{
                            //    continue;
                            //}
                            if (sol.RatioCombinedMoveCost + openRoutes * 10000 < rm.TotalCost + 0.001 & targetRouteIndex != 0 & moveCost != 0) // + bpnus
                            {
                                // Console.WriteLine("Total cost : " + rm.TotalCost + " Open Routes : " + openRoutes);
                                if (PromiseIsBroken(F.Id,B.Id, moveCost + sol.Cost + 0.001, sol))
                                {
                                    continue;
                                }
                                if (PromiseIsBroken(B.Id, G.Id, moveCost + sol.Cost + 0.001, sol))
                                {
                                    continue;
                                }
                                if (PromiseIsBroken(A.Id, C.Id, moveCost + sol.Cost + 0.001, sol))
                                {
                                    continue;
                                }
                                
                                rm.TotalCost = moveCost + openRoutes * 10000;
                                rm.MoveCost = moveCost;
                                rm.OriginRoutePosition = originRouteIndex;
                                rm.TargetRoutePosition = targetRouteIndex;
                                rm.OriginOptionPosition = originOptionIndex;
                                rm.TargetOptionPosition = targetOptionIndex;
                                rm.CostChangeOriginRt = costChangeOriginRt;
                                rm.CostChangeTargetRt = costChangeTargetRt;
                            }
                        }
                    }
                }
            }
            return rm;
        }
        public void ApplyRelocationMove(Relocation rm, Solution sol)
        {
            if (rm.IsValid())
            {
                sol.LastMove = "relocate";
                Route originRt = sol.Routes[rm.OriginRoutePosition];
                Route targetRt = sol.Routes[rm.TargetRoutePosition];

                if (!sol.CheckRouteFeasibility(originRt) || !sol.CheckRouteFeasibility(targetRt))
                {
                    Console.WriteLine("-----");
                }
                Option A = originRt.SequenceOfOptions[rm.OriginOptionPosition - 1];
                Option B = originRt.SequenceOfOptions[rm.OriginOptionPosition];
                Option C = originRt.SequenceOfOptions[rm.OriginOptionPosition + 1];
                Option F = targetRt.SequenceOfOptions[rm.TargetOptionPosition];
                Option G = targetRt.SequenceOfOptions[rm.TargetOptionPosition + 1];

                if (originRt == targetRt)
                {
                    originRt.SequenceOfOptions.RemoveAt(rm.OriginOptionPosition);
                    originRt.SequenceOfCustomers.RemoveAt(rm.OriginOptionPosition);
                    originRt.SequenceOfLocations.RemoveAt(rm.OriginOptionPosition);
                    if (rm.OriginOptionPosition < rm.TargetOptionPosition)
                    {
                        targetRt.SequenceOfOptions.Insert(rm.TargetOptionPosition, B);
                        targetRt.SequenceOfCustomers.Insert(rm.TargetOptionPosition, B.Cust);
                        targetRt.SequenceOfLocations.Insert(rm.TargetOptionPosition, B.Location);
                    }
                    else
                    {
                        targetRt.SequenceOfOptions.Insert(rm.TargetOptionPosition + 1, B);
                        targetRt.SequenceOfCustomers.Insert(rm.TargetOptionPosition + 1, B.Cust);
                        targetRt.SequenceOfLocations.Insert(rm.TargetOptionPosition + 1, B.Location);
                    }
                    sol.UpdateTimes(originRt);
                    originRt.Cost += rm.MoveCost;
                    UpdateRouteCostAndLoad(originRt, sol);
                }
                else
                {
                    originRt.SequenceOfOptions.RemoveAt(rm.OriginOptionPosition);
                    originRt.SequenceOfCustomers.RemoveAt(rm.OriginOptionPosition);
                    originRt.SequenceOfLocations.RemoveAt(rm.OriginOptionPosition);
                    originRt.SequenceOfEct.RemoveAt(rm.OriginOptionPosition);
                    originRt.SequenceOfLat.RemoveAt(rm.OriginOptionPosition);
                    targetRt.SequenceOfOptions.Insert(rm.TargetOptionPosition + 1, B);
                    targetRt.SequenceOfCustomers.Insert(rm.TargetOptionPosition + 1, B.Cust);
                    targetRt.SequenceOfLocations.Insert(rm.TargetOptionPosition + 1, B.Location);
                    targetRt.SequenceOfEct.Insert(rm.TargetOptionPosition + 1, 0);
                    targetRt.SequenceOfLat.Insert(rm.TargetOptionPosition + 1, 0);
                    originRt.Cost += rm.CostChangeOriginRt;
                    targetRt.Cost += rm.CostChangeTargetRt;
                    originRt.Load -= B.Cust.Dem;
                    targetRt.Load += B.Cust.Dem;
                    originRt.RouteUtilizationMetric = Math.Pow(Convert.ToDouble(originRt.Capacity - originRt.Load), 2);
                    targetRt.RouteUtilizationMetric = Math.Pow(Convert.ToDouble(targetRt.Capacity - targetRt.Load), 2);
                    sol.UpdateTimes(originRt);
                    sol.UpdateTimes(targetRt);
                    UpdateRouteCostAndLoad(originRt, sol);
                    UpdateRouteCostAndLoad(targetRt, sol);
                }
                sol.Cost += rm.MoveCost;
                sol.Promises[A.Id, C.Id] = sol.Cost;
                sol.Promises[F.Id, B.Id] = sol.Cost;
                sol.Promises[B.Id, G.Id] = sol.Cost;
                if (!sol.CheckRouteFeasibility(originRt))
                {
                    Console.WriteLine("-----");
                    sol.CheckRouteFeasibility(originRt);
                }
                if (!sol.CheckRouteFeasibility(targetRt))
                {
                    Console.WriteLine("-----");
                }
            }
        }

        public Swap FindBestSwapMove(Swap sm, Solution sol)
        {
            Route rt1, rt2;
            int openRoutes;
            int startOfSecondOptionIndex;
            Option a1, b1, c1, a2, b2, c2;
            for (int firstRouteIndex = 0; firstRouteIndex < sol.Routes.Count; firstRouteIndex++)
            {
                rt1 = sol.Routes[firstRouteIndex];
                for (int secondRouteIndex = firstRouteIndex; secondRouteIndex < sol.Routes.Count; secondRouteIndex++)
                {
                    rt2 = sol.Routes[secondRouteIndex];
                    for (int firstOptionIndex = 1; firstOptionIndex < rt1.SequenceOfOptions.Count - 1; firstOptionIndex++)
                    {
                        startOfSecondOptionIndex = 1;
                        if (rt1 == rt2)
                        {
                            startOfSecondOptionIndex = firstOptionIndex + 1;
                        }
                        for (int secondOptionIndex = startOfSecondOptionIndex; secondOptionIndex < rt2.SequenceOfOptions.Count - 1; secondOptionIndex++)
                        {
                            openRoutes = sol.Routes.Count;
                            a1 = rt1.SequenceOfOptions[firstOptionIndex - 1];
                            b1 = rt1.SequenceOfOptions[firstOptionIndex];
                            c1 = rt1.SequenceOfOptions[firstOptionIndex + 1];
                            a2 = rt2.SequenceOfOptions[secondOptionIndex - 1];
                            b2 = rt2.SequenceOfOptions[secondOptionIndex];
                            c2 = rt2.SequenceOfOptions[secondOptionIndex + 1];

                            double moveCost;
                            double costChangeFirstRoute = 0;
                            double costChangeSecondRoute = 0;
                            double ratio = 1;

                            var tw1 = sol.RespectsTimeWindow2(rt1, firstOptionIndex, b2.Location);
                            var tw2 = sol.RespectsTimeWindow2(rt2, secondOptionIndex, b1.Location);

                            if (!tw1.Item1 || !tw2.Item1) { continue; }

                            if (rt1 == rt2)
                            {
                                tw1 = sol.RespectsTimeWindow2(rt1, firstOptionIndex, b2.Location);
                                if (!tw1.Item1)
                                {
                                    continue;
                                }
                                Route rtTemp = rt1.getTempCopy(rt1, sol.Options.Select(x => x.Location).ToHashSet().ToList());
                                rtTemp.SequenceOfOptions[firstOptionIndex] = b2;
                                rtTemp.SequenceOfCustomers[firstOptionIndex] = b2.Cust;
                                rtTemp.SequenceOfLocations[firstOptionIndex] = b2.Location;
                                tw2 = sol.RespectsTimeWindow2(rtTemp, secondOptionIndex, b1.Location);
                                if (!tw1.Item1 || !tw2.Item1)
                                {
                                    continue;
                                }
                                if (firstOptionIndex == secondOptionIndex - 1)
                                {
                                    double costRemoved = sol.CalculateDistance(a1.Location, b1.Location) + sol.CalculateDistance(b1.Location, b2.Location) + sol.CalculateDistance(b2.Location, c2.Location);
                                    double costAdded = sol.CalculateDistance(a1.Location, b2.Location) + sol.CalculateDistance(b2.Location, b1.Location) + sol.CalculateDistance(b1.Location, c2.Location);
                                    moveCost = costAdded - costRemoved;
                                } else {
                                    double costRemoved1 = sol.CalculateDistance(a1.Location, b1.Location) + sol.CalculateDistance(b1.Location, c1.Location);
                                    double costAdded1 = sol.CalculateDistance(a1.Location, b2.Location) + sol.CalculateDistance(b2.Location, c1.Location);
                                    double costRemoved2 = sol.CalculateDistance(a2.Location, b2.Location) + sol.CalculateDistance(b2.Location, c2.Location);
                                    double costAdded2 = sol.CalculateDistance(a2.Location, b1.Location) + sol.CalculateDistance(b1.Location, c2.Location);
                                    moveCost = costAdded1 + costAdded2 - (costRemoved1 + costRemoved2);
                                }
                            } else {
                                if (rt1.Load - b1.Cust.Dem + b2.Cust.Dem > rt1.Capacity) { continue; }
                                if (rt2.Load - b2.Cust.Dem + b1.Cust.Dem > rt2.Capacity) { continue; }
                                double costRemoved1 = sol.CalculateDistance(a1.Location, b1.Location) + sol.CalculateDistance(b1.Location, c1.Location);
                                double costAdded1 = sol.CalculateDistance(a1.Location, b2.Location) + sol.CalculateDistance(b2.Location, c1.Location);
                                double costRemoved2 = sol.CalculateDistance(a2.Location, b2.Location) + sol.CalculateDistance(b2.Location, c2.Location);
                                double costAdded2 = sol.CalculateDistance(a2.Location, b1.Location) + sol.CalculateDistance(b1.Location, c2.Location);
                                costChangeFirstRoute = costAdded1 - costRemoved1;
                                costChangeSecondRoute = costAdded2 - costRemoved2;
                                moveCost = costAdded1 + costAdded2 - (costRemoved1 + costRemoved2);
                                var newUtilizationMetricRoute1 = Math.Pow(Convert.ToDouble(rt1.Capacity - (rt1.Load - b1.Cust.Dem + b2.Cust.Dem)), 2);
                                var newUtilizationMetricRoute2 = Math.Pow(Convert.ToDouble(rt2.Capacity - (rt2.Load - b2.Cust.Dem + b1.Cust.Dem)), 2);
                                var newSolUtilizationMetric = sol.SolutionUtilizationMetric - rt1.RouteUtilizationMetric - rt2.RouteUtilizationMetric + newUtilizationMetricRoute1 + newUtilizationMetricRoute2;
                                ratio = (sol.SolutionUtilizationMetric + 1) / (newSolUtilizationMetric + 1);
                                if (sol.Routes.Count == sol.LowerBoundRoutes)
                                {
                                    ratio = 1;
                                }
                            }

                            if (ratio * moveCost < sm.MoveCost + 0.001 & moveCost !=0)
                            {
                                if (PromiseIsBroken(a1.Id, b2.Id, moveCost + sol.Cost + 0.001, sol))
                                {
                                    continue;
                                }
                                if (PromiseIsBroken(b2.Id, c1.Id, moveCost + sol.Cost + 0.001, sol))
                                {
                                    continue;
                                }
                                if (PromiseIsBroken(a2.Id, b1.Id, moveCost + sol.Cost + 0.001, sol))
                                {
                                    continue;
                                }                                    
                                if (PromiseIsBroken(b1.Id, c2.Id, moveCost + sol.Cost + 0.001, sol))
                                {
                                    continue;
                                }
                                if (rt1 == rt2)
                                {
                                    sm.TotalCost = moveCost + openRoutes * 10000;
                                    sm.PositionOfFirstRoute = firstRouteIndex;
                                    sm.PositionOfSecondRoute = secondRouteIndex;
                                    sm.PositionOfFirstOption = firstOptionIndex;
                                    sm.PositionOfSecondOption = secondOptionIndex;
                                    sm.MoveCost = moveCost;
                                } else {
                                    sm.TotalCost = moveCost + openRoutes * 10000;
                                    sm.PositionOfFirstRoute = firstRouteIndex;
                                    sm.PositionOfSecondRoute = secondRouteIndex;
                                    sm.PositionOfFirstOption = firstOptionIndex;
                                    sm.PositionOfSecondOption = secondOptionIndex;
                                    sm.CostChangeFirstRt = costChangeFirstRoute;
                                    sm.CostChangeSecondRt = costChangeSecondRoute;
                                    sm.MoveCost = moveCost;
                                }
                            }
                        }
                    }
                }
            }
            return sm;
        }

        public void ApplySwapMove(Swap sm, Solution sol)
        {
            if (sm.IsValid())
            {
                sol.LastMove = "swap";
                Route rt1 = sol.Routes[sm.PositionOfFirstRoute];
                Route rt2 = sol.Routes[sm.PositionOfSecondRoute];
                if (!sol.CheckRouteFeasibility(rt1) || !sol.CheckRouteFeasibility(rt2))
                {
                    Console.WriteLine("-----");
                }
                Option a1 = rt1.SequenceOfOptions[sm.PositionOfFirstOption - 1];
                Option a2 = rt2.SequenceOfOptions[sm.PositionOfSecondOption - 1];
                Option b1 = rt1.SequenceOfOptions[sm.PositionOfFirstOption];
                Option b2 = rt2.SequenceOfOptions[sm.PositionOfSecondOption];
                Option c1 = rt1.SequenceOfOptions[sm.PositionOfFirstOption + 1];
                Option c2 = rt2.SequenceOfOptions[sm.PositionOfSecondOption + 1];
                rt1.SequenceOfOptions[sm.PositionOfFirstOption] = b2;
                rt1.SequenceOfCustomers[sm.PositionOfFirstOption] = b2.Cust;
                rt1.SequenceOfLocations[sm.PositionOfFirstOption] = b2.Location;
                rt2.SequenceOfOptions[sm.PositionOfSecondOption] = b1;
                rt2.SequenceOfCustomers[sm.PositionOfSecondOption] = b1.Cust;
                rt2.SequenceOfLocations[sm.PositionOfSecondOption] = b1.Location;
                if (rt1 == rt2)
                {
                    rt1.Cost += sm.MoveCost;
                    UpdateRouteCostAndLoad(rt1, sol);
                    sol.UpdateTimes(rt1);
                }
                else
                {
                    rt1.Cost += sm.CostChangeFirstRt;
                    rt2.Cost += sm.CostChangeSecondRt;
                    rt1.Load = rt1.Load - b1.Cust.Dem + b2.Cust.Dem;
                    rt2.Load = rt2.Load + b1.Cust.Dem - b2.Cust.Dem;
                    rt1.RouteUtilizationMetric = Math.Pow(Convert.ToDouble(rt1.Capacity - rt1.Load), 2);
                    rt2.RouteUtilizationMetric = Math.Pow(Convert.ToDouble(rt2.Capacity - rt2.Load), 2);
                    UpdateRouteCostAndLoad(rt1, sol);
                    UpdateRouteCostAndLoad(rt2, sol);
                    sol.UpdateTimes(rt1);
                    sol.UpdateTimes(rt2);
                }
                sol.Cost += sm.MoveCost;
                sol.Promises[a1.Id, b2.Id] = sol.Cost;
                sol.Promises[b2.Id, c1.Id] = sol.Cost;
                sol.Promises[a2.Id, b1.Id] = sol.Cost;
                sol.Promises[b1.Id, c2.Id] = sol.Cost;
                if (!sol.CheckRouteFeasibility(rt1) || !sol.CheckRouteFeasibility(rt2))
                {
                    Console.WriteLine("-----");
                }
            }
        }

        public TwoOpt FindBestTwoOptMove(TwoOpt top, Solution sol) {
            int openRoutes;
            for (int rtInd1 = 0; rtInd1 < sol.Routes.Count; rtInd1++) {
                Route rt1 = sol.Routes[rtInd1];
                for (int rtInd2 = 0; rtInd2 < sol.Routes.Count; rtInd2++) {
                    Route rt2 = sol.Routes[rtInd2];
                    for (int optInd1 = 0; optInd1 < rt1.SequenceOfOptions.Count - 1; optInd1++) {
                        int start2 = 0;
                        if (rt1 == rt2) {
                            start2 = optInd1 + 2;
                        }
                        for (int optInd2 = start2; optInd2 < rt2.SequenceOfOptions.Count - 1; optInd2++) {
                            openRoutes = sol.Routes.Count;

                            double moveCost;
                            double costAdded;
                            double costRemoved;

                            Option A = rt1.SequenceOfOptions[optInd1];
                            Option B = rt1.SequenceOfOptions[optInd1 + 1];
                            Option K = rt2.SequenceOfOptions[optInd2];
                            Option L = rt2.SequenceOfOptions[optInd2 + 1];

                            var tw1 = sol.RespectsTimeWindow(rt1, optInd1,
                                            rt2.SequenceOfLocations.GetRange(optInd2 + 1, rt2.SequenceOfLocations.Count - (optInd2 + 1)));
                            var tw2 = sol.RespectsTimeWindow(rt2, optInd2,
                                            rt1.SequenceOfLocations.GetRange(optInd1 + 1, rt1.SequenceOfLocations.Count - (optInd1 + 1)));

                            bool respectsTw1 = tw1.Item1;
                            bool respectsTw2 = tw2.Item1;
                            
                            if (!respectsTw1 || !respectsTw2) { continue; }

                            if (rt1 == rt2) {
                                if (optInd1 == 0 & optInd2 == rt1.SequenceOfOptions.Count - 2) { continue; }
                                tw1 = sol.RespectsTimeWindow(rt1, optInd1,
                                            rt1.SequenceOfLocations.GetRange(optInd2 + 1, rt2.SequenceOfLocations.Count - (optInd2 + 1)));
                                tw2 = sol.RespectsTimeWindow(rt1, optInd2, rt1.SequenceOfLocations.GetRange(optInd1 + 1, rt1.SequenceOfLocations.Count - (optInd1 + 1)));
                                respectsTw1 = tw1.Item1;
                                respectsTw2 = tw2.Item1;
                                if (!respectsTw1 || !respectsTw2) { continue; }

                                Route rtTemp = rt1.getTempCopy(rt1, sol.Options.Select(x => x.Location).ToHashSet().ToList());
                                int frombase = optInd1 + 1;
                                int fromend = optInd2 + 1;
                                List<Option> reversedSegment = Enumerable.Reverse(rtTemp.SequenceOfOptions.GetRange(frombase, fromend - frombase)).ToList();
                                List<Location> reversedLocations = Enumerable.Reverse(rtTemp.SequenceOfLocations.GetRange(frombase, fromend - frombase)).ToList();
                                List<Customer> reversedCustomers = Enumerable.Reverse(rtTemp.SequenceOfCustomers.GetRange(frombase, fromend - frombase)).ToList();
                                rtTemp.SequenceOfOptions.RemoveRange(frombase, fromend - frombase);
                                rtTemp.SequenceOfOptions.InsertRange(frombase, reversedSegment);
                                rtTemp.SequenceOfLocations.RemoveRange(frombase, fromend - frombase);
                                rtTemp.SequenceOfLocations.InsertRange(frombase, reversedLocations);
                                rtTemp.SequenceOfCustomers.RemoveRange(frombase, fromend - frombase);
                                rtTemp.SequenceOfCustomers.InsertRange(frombase, reversedCustomers);
                                rtTemp.SequenceOfEct = new List<double>(rt1.SequenceOfEct);
                                rtTemp.SequenceOfLat = new List<double>(rt1.SequenceOfLat);
                                // Update rtTemp times and check if it respects time windows
                                // Parse the sequence of options and update the list of ect and lat
                                for (int i = 0; i < rtTemp.SequenceOfOptions.Count - 1; i++)
                                {
                                    rtTemp.SequenceOfEct[i] = rtTemp.SequenceOfOptions[i].Due;
                                    rtTemp.SequenceOfLat[i] = rtTemp.SequenceOfOptions[i].Ready;
                                }

                                
                                if (!rtTemp.CheckTimeWindowsFeasibility()) 
                                { 
                                    continue; 
                                }

                                costAdded = sol.CalculateDistance(A.Location, K.Location) + sol.CalculateDistance(B.Location, L.Location);
                                costRemoved = sol.CalculateDistance(A.Location, B.Location) + sol.CalculateDistance(K.Location, L.Location);
                                moveCost = costAdded - costRemoved;
                                sol.RatioCombinedMoveCost = moveCost;
                            } else {
                                if (optInd1 == 0 && optInd2 == 0) { continue; }

                                if (optInd1 == rt1.SequenceOfOptions.Count - 2 & optInd2 == rt2.SequenceOfOptions.Count - 2) { continue; }

                                if (CapacityIsViolated(rt1, optInd1, rt2, optInd2)) { continue; }

                                costAdded = sol.CalculateDistance(A.Location, L.Location) + sol.CalculateDistance(B.Location, K.Location);
                                costRemoved = sol.CalculateDistance(A.Location, B.Location) + sol.CalculateDistance(K.Location, L.Location);
                                moveCost = costAdded - costRemoved;
                                if (rt1.Load - B.Cust.Dem == 0 || rt2.Load - K.Cust.Dem == 0) {
                                    //Console.WriteLine("This TWO-OPT move empties a route");
                                    openRoutes--;
                                }
                                var newUtilizationMetricRoute1 = Math.Pow(Convert.ToDouble(rt1.Capacity - (rt1.Load - B.Cust.Dem)), 2);
                                var newUtilizationMetricRoute2 = Math.Pow(Convert.ToDouble(rt2.Capacity - (rt2.Load - K.Cust.Dem)), 2);
                                var newSolUtilizationMetric = sol.SolutionUtilizationMetric - rt1.RouteUtilizationMetric - rt2.RouteUtilizationMetric + newUtilizationMetricRoute1 + newUtilizationMetricRoute2;
                                var ratio = (sol.SolutionUtilizationMetric + 1) / (newSolUtilizationMetric + 1);
                                if (sol.Routes.Count == sol.LowerBoundRoutes)
                                {
                                    ratio = 1;
                                }
                                sol.RatioCombinedMoveCost = ratio * moveCost;
                            }

                            if (sol.RatioCombinedMoveCost + openRoutes * 10000 < top.TotalCost + 0.001 & moveCost != 0)
                            {

                                if (PromiseIsBroken(A.Id, L.Id, moveCost + sol.Cost + 0.001, sol))
                                {
                                    continue;
                                }
                                if (PromiseIsBroken(B.Id, K.Id, moveCost + sol.Cost + 0.001, sol))
                                {
                                    continue;
                                }

                                top.TotalCost = moveCost + openRoutes * 10000;
                                top.PositionOfFirstRoute = rtInd1;
                                top.PositionOfSecondRoute = rtInd2;
                                top.PositionOfFirstOption = optInd1;
                                top.PositionOfSecondOption = optInd2;
                                top.Ect1 = tw1.Item2;
                                top.Ect2 = tw2.Item2;
                                top.Lat1 = tw1.Item3;
                                top.Lat2 = tw2.Item3;
                                top.MoveCost = moveCost;
                            }
                        }
                    }
                }
            }
            return top;
        }

        public bool CapacityIsViolated(Route rt1, int optionInd1, Route rt2, int optionInd2) {
            double rt1FirstSegmentLoad = 0;
            for (int i = 0; i < optionInd1 + 1; i++) {
                Option n = rt1.SequenceOfOptions[i];
                rt1FirstSegmentLoad += n.Cust.Dem;
            }
            double rt1SecondSegmentLoad = rt1.Load - rt1FirstSegmentLoad;
            double rt2FirstSegmentLoad = 0;
            for (int i = 0; i < optionInd2 + 1; i++) {
                Option n = rt2.SequenceOfOptions[i];
                rt2FirstSegmentLoad += n.Cust.Dem;
            }
            double rt2SecondSegmentLoad = rt2.Load - rt2FirstSegmentLoad;
            if (rt1FirstSegmentLoad + rt2SecondSegmentLoad > rt1.Capacity) {
                return true;
            }
            if (rt2FirstSegmentLoad + rt1SecondSegmentLoad > rt2.Capacity) {
                return true;
            }
            return false;
        }

        public void ApplyTwoOptMove(TwoOpt top, Solution sol) {
            if (!top.IsValid()) { return; }
            sol.LastMove = "two opt";
            Route rt1 = sol.Routes[top.PositionOfFirstRoute];
            Route rt2 = sol.Routes[top.PositionOfSecondRoute];
            if (!sol.CheckRouteFeasibility(rt1) || !sol.CheckRouteFeasibility(rt2))
            {
                Console.WriteLine("-----");
            }
            Option A = rt1.SequenceOfOptions[top.PositionOfFirstOption];
            Option B = rt1.SequenceOfOptions[top.PositionOfFirstOption + 1];
            Option K = rt2.SequenceOfOptions[top.PositionOfSecondOption];
            Option L = rt2.SequenceOfOptions[top.PositionOfSecondOption + 1];
            if (rt1 == rt2)
            {
                // reverses the nodes in the segment [positionOfFirstNode + 1,  top.positionOfSecondNode]
                int frombase = top.PositionOfFirstOption + 1;
                int fromend = top.PositionOfSecondOption + 1;
                List<Option> reversedSegment = Enumerable.Reverse(rt1.SequenceOfOptions.GetRange(frombase, fromend - frombase)).ToList();
                List<Location> reversedLocations = Enumerable.Reverse(rt1.SequenceOfLocations.GetRange(frombase, fromend - frombase)).ToList();
                List<Customer> reversedCustomers = Enumerable.Reverse(rt1.SequenceOfCustomers.GetRange(frombase, fromend - frombase)).ToList();
                rt1.SequenceOfOptions.RemoveRange(frombase, fromend - frombase);
                rt1.SequenceOfOptions.InsertRange(frombase, reversedSegment);
                rt1.SequenceOfLocations.RemoveRange(frombase, fromend - frombase);
                rt1.SequenceOfLocations.InsertRange(frombase, reversedLocations);
                rt1.SequenceOfCustomers.RemoveRange(frombase, fromend - frombase);
                rt1.SequenceOfCustomers.InsertRange(frombase, reversedCustomers);
                rt1.Cost += top.MoveCost;
                sol.UpdateTimes(rt1);
                UpdateRouteCostAndLoad(rt1, sol);
                rt1.RouteUtilizationMetric = Math.Pow(Convert.ToDouble(rt1.Capacity - rt1.Load), 2);
            }
            else
            {
                int frombase = top.PositionOfFirstOption + 1;
                int fromend = top.PositionOfSecondOption + 1;
                // slice with the nodes from position top.positionOfFirstNode + 1 onwards
                List<Option> relocatedSegmentOfRt1 = rt1.SequenceOfOptions.GetRange(frombase, rt1.SequenceOfOptions.Count - frombase).ToList();
                List<Location> relocatedLocations1 = rt1.SequenceOfLocations.GetRange(frombase, rt1.SequenceOfLocations.Count - frombase).ToList();
                List<Customer> relocatedCustomers1 = rt1.SequenceOfCustomers.GetRange(frombase, rt1.SequenceOfCustomers.Count - frombase).ToList();
                // slice with the nodes from position top.positionOfFirstNode + 1 onwards
                List<Option> relocatedSegmentOfRt2 = rt2.SequenceOfOptions.GetRange(fromend, rt2.SequenceOfOptions.Count - fromend).ToList();
                List<Location> relocatedLocations2 = rt2.SequenceOfLocations.GetRange(fromend, rt2.SequenceOfLocations.Count - fromend).ToList();
                List<Customer> relocatedCustomers2 = rt2.SequenceOfCustomers.GetRange(fromend, rt2.SequenceOfCustomers.Count - fromend).ToList();

                int length = rt1.SequenceOfOptions.Count - 1;
                for (int i = length; i >= top.PositionOfFirstOption + 1; i--)
                {
                    rt1.SequenceOfOptions.RemoveAt(i);
                    rt1.SequenceOfLocations.RemoveAt(i);
                    rt1.SequenceOfCustomers.RemoveAt(i);
                }
                length = rt2.SequenceOfOptions.Count - 1;
                for (int i = length; i >= top.PositionOfSecondOption + 1; i--)
                {
                    rt2.SequenceOfOptions.RemoveAt(i);
                    rt2.SequenceOfLocations.RemoveAt(i);
                    rt2.SequenceOfCustomers.RemoveAt(i);
                }
                rt1.SequenceOfOptions.AddRange(relocatedSegmentOfRt2);
                rt2.SequenceOfOptions.AddRange(relocatedSegmentOfRt1);
                rt1.SequenceOfLocations.AddRange(relocatedLocations2);
                rt2.SequenceOfLocations.AddRange(relocatedLocations1);
                rt1.SequenceOfCustomers.AddRange(relocatedCustomers2);
                rt2.SequenceOfCustomers.AddRange(relocatedCustomers1);
                UpdateRouteCostAndLoad(rt1, sol);
                UpdateRouteCostAndLoad(rt2, sol);
                rt1.RouteUtilizationMetric = Math.Pow(Convert.ToDouble(rt1.Capacity - rt1.Load), 2);
                rt2.RouteUtilizationMetric = Math.Pow(Convert.ToDouble(rt2.Capacity - rt2.Load), 2);
                rt1.SequenceOfEct = top.Ect1.ToList();
                rt2.SequenceOfEct = top.Ect2.ToList();
                rt1.SequenceOfLat = top.Lat1.ToList();
                rt2.SequenceOfLat = top.Lat2.ToList();

            }
            sol.Cost += top.MoveCost;
            sol.Promises[A.Id, L.Id] = sol.Cost;
            sol.Promises[B.Id, K.Id] = sol.Cost;
            if (!sol.CheckRouteFeasibility(rt1) || !sol.CheckRouteFeasibility(rt2))
            {
                Console.WriteLine("-----");
            }
        }

        public Flip FindBestFlipMove(Flip flip, Solution sol, bool cond = false)
        {
            int openRoutes;
            for (int rtInd1 = 0; rtInd1 < sol.Routes.Count; rtInd1++)
            {
                Route rt1 = sol.Routes[rtInd1];

                for (int custInd1 = 1; custInd1 < rt1.SequenceOfCustomers.Count - 1; custInd1++)
                {
                    Customer custA = rt1.SequenceOfCustomers[custInd1 - 1];
                    Customer custB = rt1.SequenceOfCustomers[custInd1];
                    Customer custC = rt1.SequenceOfCustomers[custInd1 + 1];
                    //Route rt1_copy = new Route(rt1);
                    Route rt1_copy = rt1.getTempCopy(rt1, sol.Options.Select(x => x.Location).ToHashSet().ToList());
                    rt1_copy.SequenceOfCustomers.RemoveAt(custInd1);
                    rt1_copy.SequenceOfOptions.RemoveAt(custInd1);
                    rt1_copy.SequenceOfLocations.RemoveAt(custInd1);
                    rt1_copy.SequenceOfEct.RemoveAt(custInd1);
                    rt1_copy.SequenceOfLat.RemoveAt(custInd1);
                    sol.UpdateTimes(rt1_copy);
                    rt1_copy.Load = rt1_copy.Load - custB.Dem;

                    // remove customer and update time windows and capacity
                    for (int optInd = 0; optInd < custB.Options.Count; optInd++)
                    {
                        for (int rtInd2 = 0; rtInd2 < sol.Routes.Count; rtInd2++)
                        {
                            openRoutes = sol.Routes.Count;
                            Route rt2 = sol.Routes[rtInd2];
                            int indCust = rt1.SequenceOfCustomers.IndexOf(custB);
                            int targetRouteIndex = 0;

                            if (rt2 == rt1)
                            {
                                //continue;
                                targetRouteIndex = custInd1 + 1;
                            }

                            if (custB.Options[optInd] == rt1.SequenceOfOptions[indCust])
                            {
                                continue;
                            }
                            if (rt2.Load + custB.Dem > rt2.Capacity)
                            {
                                continue;
                            }
                            if (custB.Options[optInd].Location.MaxCap == custB.Options[optInd].Location.Cap)
                            {
                                // if shared location's max capacity is already reached continue
                                continue;
                            }

                            for (int targetOptionIndex = targetRouteIndex; targetOptionIndex < rt2.SequenceOfOptions.Count - 1; targetOptionIndex++) //-1
                            {

                                var tw = sol.RespectsTimeWindow2(rt2, targetOptionIndex, custB.Options[optInd].Location);
                                
                                if (!tw.Item1) { continue; }

                                if (rt1.SequenceOfOptions[custInd1].Prio < custB.Options[optInd].Prio)
                                {
                                    continue;
                                }

                                Option A = rt1.SequenceOfOptions[custInd1 - 1];
                                Option B1 = rt1.SequenceOfOptions[custInd1];
                                Option C = rt1.SequenceOfOptions[custInd1 + 1];

                                Option F = rt2.SequenceOfOptions[targetOptionIndex];
                                Option B2 = custB.Options[optInd];
                                Option G = rt2.SequenceOfOptions[targetOptionIndex + 1];

                                if (rt1 != rt2)
                                {
                                    if (rt2.Load + custB.Dem > rt2.Capacity)
                                    {
                                        continue;
                                    }
                                }

                                if (rt1.Load - B1.Cust.Dem == 0)
                                {
                                    //Console.WriteLine("This FLIP move empties a route");
                                    openRoutes--;
                                }

                                double costAdded = sol.CalculateDistance(A.Location, C.Location) + sol.CalculateDistance(F.Location, B2.Location)
                                                    + sol.CalculateDistance(B2.Location, G.Location);
                                double costRemoved = sol.CalculateDistance(A.Location, B1.Location) + sol.CalculateDistance(B1.Location, C.Location)
                                                    + sol.CalculateDistance(F.Location, G.Location);
                                double moveCost = costAdded - costRemoved;
                                var newUtilizationMetricRoute1 = Math.Pow(Convert.ToDouble(rt1.Capacity - (rt1.Load - B1.Cust.Dem)), 2);
                                var newUtilizationMetricRoute2 = Math.Pow(Convert.ToDouble(rt2.Capacity - (rt2.Load + B2.Cust.Dem)), 2);
                                var newSolUtilizationMetric = sol.SolutionUtilizationMetric - rt1.RouteUtilizationMetric - rt2.RouteUtilizationMetric + newUtilizationMetricRoute1 + newUtilizationMetricRoute2;
                                var ratio = (sol.SolutionUtilizationMetric + 1) / (newSolUtilizationMetric + 1);
                                if (sol.Routes.Count == sol.LowerBoundRoutes)
                                {
                                    ratio = 1;
                                }
                                sol.RatioCombinedMoveCost = ratio * moveCost;

                                double costChangeOriginRt = sol.CalculateDistance(A.Location, C.Location) - sol.CalculateDistance(A.Location, B1.Location)
                                                    - sol.CalculateDistance(B1.Location, C.Location);
                                double costChangeTargetRt = sol.CalculateDistance(F.Location, B2.Location) + sol.CalculateDistance(B2.Location, G.Location)
                                                    - sol.CalculateDistance(F.Location, G.Location);


                                if (sol.RatioCombinedMoveCost + openRoutes * 10000 < flip.TotalCost + 0.001 & rtInd2 != 0)
                                {
                                    if (PromiseIsBroken(F.Id, B2.Id, moveCost + sol.Cost + 0.001, sol))
                                    {
                                        continue;
                                    }
                                    if (PromiseIsBroken(B2.Id, G.Id, moveCost + sol.Cost + 0.001, sol))
                                    {
                                        continue;
                                    }
                                    if (PromiseIsBroken(A.Id, C.Id, moveCost + sol.Cost + 0.001, sol))
                                    {
                                        continue;
                                    }
                                    flip.TotalCost = moveCost + openRoutes * 10000;
                                    flip.MoveCost = moveCost;
                                    flip.OriginRoutePosition = rtInd1;
                                    flip.TargetRoutePosition = rtInd2;
                                    flip.TargetOptionPosition = targetOptionIndex;
                                    flip.OriginOptionPosition = custInd1;
                                    flip.CostChangeOriginRt = costChangeOriginRt;
                                    flip.CostChangeTargetRt = costChangeTargetRt;
                                    flip.NewOptionIndex = optInd;
                                }
                            }
                        }
                    }
                    sol.Routes[rtInd1] = rt1;
                }
            }
            return flip;
        }

        public void ApplyFlipMove(Flip flip, Solution sol)
        { 
            if (flip.IsValid())
            {
                sol.LastMove = "flip";
                Route originRt = sol.Routes[flip.OriginRoutePosition];
                Route targetRt = sol.Routes[flip.TargetRoutePosition];
                if (!sol.CheckRouteFeasibility(targetRt) || !sol.CheckRouteFeasibility(originRt))
                {
                    Console.WriteLine("-----");
                }
                Option A = originRt.SequenceOfOptions[flip.OriginOptionPosition - 1];
                Option B1 = originRt.SequenceOfOptions[flip.OriginOptionPosition];
                Option B2 = originRt.SequenceOfCustomers[flip.OriginOptionPosition].Options[flip.NewOptionIndex];//new option to be placed in place of B1
                Option C = originRt.SequenceOfOptions[flip.OriginOptionPosition + 1];
                Option F = targetRt.SequenceOfOptions[flip.TargetOptionPosition];
                Option G = targetRt.SequenceOfOptions[flip.TargetOptionPosition + 1];

                
                if (originRt == targetRt)
                {
                    originRt.SequenceOfOptions.RemoveAt(flip.OriginOptionPosition);
                    originRt.SequenceOfCustomers.RemoveAt(flip.OriginOptionPosition);
                    originRt.SequenceOfLocations.RemoveAt(flip.OriginOptionPosition);
                    if (flip.OriginOptionPosition < flip.TargetOptionPosition)
                    {
                        targetRt.SequenceOfOptions.Insert(flip.TargetOptionPosition, B2);
                        targetRt.SequenceOfCustomers.Insert(flip.TargetOptionPosition, B2.Cust);
                        targetRt.SequenceOfLocations.Insert(flip.TargetOptionPosition, B2.Location);
                    }
                    else
                    {
                        targetRt.SequenceOfOptions.Insert(flip.TargetOptionPosition + 1, B2);
                        targetRt.SequenceOfCustomers.Insert(flip.TargetOptionPosition + 1, B2.Cust);
                        targetRt.SequenceOfLocations.Insert(flip.TargetOptionPosition + 1, B2.Location);
                    }
                    sol.UpdateTimes(originRt);
                    originRt.Cost += flip.MoveCost;
                    UpdateRouteCostAndLoad(originRt, sol);
                    originRt.RouteUtilizationMetric = Math.Pow(Convert.ToDouble(originRt.Capacity - originRt.Load), 2);
                }
                else
                {
                originRt.SequenceOfOptions.RemoveAt(flip.OriginOptionPosition);
                originRt.SequenceOfCustomers.RemoveAt(flip.OriginOptionPosition);
                originRt.SequenceOfLocations.RemoveAt(flip.OriginOptionPosition);
                originRt.SequenceOfEct.RemoveAt(flip.OriginOptionPosition);
                originRt.SequenceOfLat.RemoveAt(flip.OriginOptionPosition);

                targetRt.SequenceOfOptions.Insert(flip.TargetOptionPosition + 1, B2);
                targetRt.SequenceOfCustomers.Insert(flip.TargetOptionPosition + 1, B2.Cust);
                targetRt.SequenceOfLocations.Insert(flip.TargetOptionPosition + 1, B2.Location);
                targetRt.SequenceOfEct.Insert(flip.TargetOptionPosition + 1, 0);
                targetRt.SequenceOfLat.Insert(flip.TargetOptionPosition + 1, 0);

                originRt.Cost += flip.CostChangeOriginRt;
                targetRt.Cost += flip.CostChangeTargetRt;
                originRt.Load -= B1.Cust.Dem;
                targetRt.Load += B2.Cust.Dem;
                UpdateRouteCostAndLoad(originRt, sol);
                UpdateRouteCostAndLoad(targetRt, sol);
                sol.UpdateTimes(originRt);
                sol.UpdateTimes(targetRt);
                originRt.RouteUtilizationMetric = Math.Pow(Convert.ToDouble(originRt.Capacity - originRt.Load), 2);
                targetRt.RouteUtilizationMetric = Math.Pow(Convert.ToDouble(targetRt.Capacity - targetRt.Load), 2);
                }
                sol.Cost += flip.MoveCost;
                B1.IsServed = false; B2.IsServed = true;
                //adjust capacity for shared locations
                B2.Location.Cap++;
                B1.Location.Cap--;
                sol.Promises[A.Id, C.Id] = sol.Cost;
                sol.Promises[F.Id, B2.Id] = sol.Cost;
                sol.Promises[B2.Id, G.Id] = sol.Cost;
                if (!sol.CheckRouteFeasibility(targetRt) || !sol.CheckRouteFeasibility(originRt))
                {
                    Console.WriteLine("-----");
                }
            }
        }

        public PrioritySwap FindBestPrioritySwapMove(PrioritySwap psm, Solution sol)
        {
            Dictionary<int, List<Option>> optionsPerCustomer = sol.OptionsPerCustomer; //get a Dict with all the options that regard the customer with id=key
            Dictionary<int, List<int>> optionsPrioritiesPerCustomer = sol.OptionsPrioritiesPerCustomer; //get a Dict with the corresponding level of priorities available for each customer
            Route rt1, rt2;
            Option a1, b1, c1, d1, a2, b2, c2, d2; // d1, d2 the alternative priorities for cust1 and cust 1
            int offset = 0;
            int openRoutes;
            for (int firstRouteIndex = 0; firstRouteIndex < sol.Routes.Count; firstRouteIndex++)
            {
                rt1 = sol.Routes[firstRouteIndex];
                for (int firstOptionIndex = 1; firstOptionIndex < rt1.SequenceOfOptions.Count - 1; firstOptionIndex++)
                {
                    b1 = rt1.SequenceOfOptions[firstOptionIndex];
                    var customer1 = sol.Customers.Where(x => x.Id == rt1.SequenceOfCustomers[firstOptionIndex].Id).ToList()[0];
                    //var customer1 = rt1.SequenceOfCustomers[firstOptionIndex];
                    int custID1 = b1.Cust.Id;
                    if (custID1 == 1000) { continue; }
                    if (customer1.Options.Count > 1) //if optionsPerCustomer[custID1].Count <= 1 there is no reason to check if there is any chance to do a priority swap with another customer
                    {
                        //foreach (Option opt1 in optionsPerCustomer[custID1]) //Check all the available Options of Customer 1
                        foreach (Option opt1 in customer1.Options) //Check all the available Options of Customer 1
                        {
                            if ((opt1.Location.Cap < opt1.Location.MaxCap && opt1.Location.Type == 1) || opt1.Location.Type == 2)
                            {
                                for (int secondRouteIndex = firstRouteIndex; secondRouteIndex < sol.Routes.Count; secondRouteIndex++)
                                {
                                    rt2 = sol.Routes[secondRouteIndex];
                                    int startOfSecondOptionIndex = 1;
                                    if (rt1 == rt2)
                                    {
                                        startOfSecondOptionIndex = firstOptionIndex + 1;
                                    }
                                    for (int secondOptionIndex = startOfSecondOptionIndex; secondOptionIndex < rt2.SequenceOfOptions.Count - 1; secondOptionIndex++)
                                    {
                                        openRoutes = sol.Routes.Count;
                                        b2 = rt2.SequenceOfOptions[secondOptionIndex];
                                        var customer2 = sol.Customers.Where(x => x.Id == rt2.SequenceOfCustomers[secondOptionIndex].Id).ToList()[0];
                                        //var customer2 = rt2.SequenceOfCustomers[secondOptionIndex];
                                        int custID2 = b2.Cust.Id;
                                        if (custID2 == 1000) { continue; }
                                        if (customer2.Options.Count > 1) //same as in the line 770
                                        {
                                            //foreach (Option opt2 in optionsPerCustomer[custID2])
                                            foreach (Option opt2 in customer2.Options)
                                            {
                                                psm.TimeWindowsError = false;
                                                //check if a shared location will be used two times with this swap
                                                offset = 0;
                                                if (opt1.Location.Id == opt2.Location.Id)
                                                {
                                                    offset = 1;
                                                }
                                                if ((sol.Options.Where(x => x.Id == opt2.Id).ToList()[0].Location.Cap < opt2.Location.MaxCap - offset && opt2.Location.Type == 1) || opt2.Location.Type == 2)
                                                {

                                                    if (rt1.Load - b1.Cust.Dem + b2.Cust.Dem > rt1.Capacity)
                                                    {
                                                        continue;
                                                    }
                                                    if (rt2.Load - b2.Cust.Dem + b1.Cust.Dem > rt2.Capacity)
                                                    {
                                                        continue;
                                                    }

                                                    a1 = rt1.SequenceOfOptions[firstOptionIndex - 1];
                                                    d1 = opt1;
                                                    c1 = rt1.SequenceOfOptions[firstOptionIndex + 1];
                                                    a2 = rt2.SequenceOfOptions[secondOptionIndex - 1];
                                                    d2 = opt2;
                                                    c2 = rt2.SequenceOfOptions[secondOptionIndex + 1];
                                                    double moveCost = 0;
                                                    double costChangeFirstRoute = 0;
                                                    double costChangeSecondRoute = 0;
                                                    double newUtilizationMetricRoute1 = 0;
                                                    double newUtilizationMetricRoute2 = 0;
                                                    double newSolUtilizationMetric = 0;
                                                    double ratio = 1;
                                                    //If cust1 has priority level 0 and cust 2 has priority level 2 then if customer 1 does not have a 3rd option(prior level= 2) you can not do the priority swap
                                                    //So the current priority level of cust 1 must exist in the priority options of cust 2 and the current priority level of cust 2 must exist in the priority options of cust 1
                                                    if (optionsPrioritiesPerCustomer[custID2].Contains(b1.Prio) && (b1.Prio != b2.Prio) && optionsPrioritiesPerCustomer[custID1].Contains(b2.Prio))
                                                    {
                                                        if (opt1.Prio == b2.Prio && (opt1.Id != b1.Id) && (opt2.Id != b2.Id) && opt2.Prio == b1.Prio)
                                                        { //Check that the alternative option of cust1 has same level of priority with the current level of customer 2 and ensure that opt1 does not refer to the option of cust 1 that is already in the route
                                                            if (rt1 == rt2 && (firstOptionIndex == secondOptionIndex - 1 || secondOptionIndex == firstOptionIndex - 1))
                                                            {
                                                                if (firstOptionIndex == secondOptionIndex - 1)
                                                                {
                                                                    var tw1 = sol.RespectsTimeWindow2(rt1, firstOptionIndex, d1.Location);
                                                                    if (!tw1.Item1)
                                                                    {
                                                                        continue;
                                                                    }
                                                                    //Route rtTemp =  new Route(rt1);
                                                                    Route rtTemp = rt1.getTempCopy(rt1, sol.Options.Select(x => x.Location).ToHashSet().ToList());
                                                                    rtTemp.SequenceOfOptions[firstOptionIndex] = d1;
                                                                    rtTemp.SequenceOfCustomers[firstOptionIndex] = d1.Cust;
                                                                    rtTemp.SequenceOfLocations[firstOptionIndex] = d1.Location;
                                                                    var tw2 = sol.RespectsTimeWindow2(rtTemp, secondOptionIndex, d2.Location);
                                                                    if (!tw1.Item1 || !tw2.Item1)
                                                                    {
                                                                        psm.TimeWindowsError = true;
                                                                        continue;
                                                                    }
                                                                    //in this case c1=b2 and a2=b1, so do not remove twice distance between the options that might change priority level
                                                                    double costRemoved = sol.CalculateDistance(a1.Location, b1.Location) + sol.CalculateDistance(b1.Location, c1.Location) + sol.CalculateDistance(b2.Location, c2.Location);
                                                                    double costAdded = sol.CalculateDistance(a1.Location, d1.Location) + sol.CalculateDistance(d1.Location, d2.Location) + sol.CalculateDistance(d2.Location, c2.Location);
                                                                    moveCost = costAdded - costRemoved;
                                                                    newUtilizationMetricRoute1 = Math.Pow(Convert.ToDouble(rt1.Capacity - (rt1.Load - b1.Cust.Dem - c1.Cust.Dem + d1.Cust.Dem + d2.Cust.Dem)), 2);
                                                                    newSolUtilizationMetric = sol.SolutionUtilizationMetric - rt1.RouteUtilizationMetric + newUtilizationMetricRoute1;
                                                                }
                                                                else if (secondOptionIndex == firstOptionIndex - 1)
                                                                {
                                                                    var tw2 = sol.RespectsTimeWindow2(rt2, secondOptionIndex, d2.Location);
                                                                    if (!tw2.Item1)
                                                                    {
                                                                        continue;
                                                                    }
                                                                    //var rtTemp = new Route(rt2);
                                                                    Route rtTemp = rt2.getTempCopy(rt2, sol.Options.Select(x => x.Location).ToHashSet().ToList());
                                                                    rtTemp.SequenceOfOptions[secondOptionIndex] = d2;
                                                                    rtTemp.SequenceOfCustomers[secondOptionIndex] = d2.Cust;
                                                                    rtTemp.SequenceOfLocations[secondOptionIndex] = d2.Location;
                                                                    var tw1 = sol.RespectsTimeWindow2(rtTemp, firstOptionIndex, d1.Location);
                                                                    if (!tw1.Item1 || !tw2.Item1)
                                                                    {
                                                                        psm.TimeWindowsError = true;
                                                                        continue;
                                                                    }
                                                                    double costRemoved = sol.CalculateDistance(a2.Location, b2.Location) + sol.CalculateDistance(b2.Location, c2.Location) + sol.CalculateDistance(b1.Location, c1.Location);
                                                                    double costAdded = sol.CalculateDistance(a2.Location, d2.Location) + sol.CalculateDistance(d2.Location, d1.Location) + sol.CalculateDistance(d1.Location, c1.Location);
                                                                    moveCost = costAdded - costRemoved;
                                                                    newUtilizationMetricRoute1 = Math.Pow(Convert.ToDouble(rt1.Capacity - (rt1.Load - b2.Cust.Dem - c2.Cust.Dem + d2.Cust.Dem + d1.Cust.Dem)), 2);
                                                                    newSolUtilizationMetric = sol.SolutionUtilizationMetric - rt1.RouteUtilizationMetric + newUtilizationMetricRoute1;
                                                                }
                                                            }
                                                            else
                                                            {
                                                                if (rt1 == rt2)
                                                                {
                                                                    if (firstOptionIndex < secondOptionIndex)
                                                                    {
                                                                        var tw1 = sol.RespectsTimeWindow2(rt1, firstOptionIndex, d1.Location);
                                                                        if (!tw1.Item1)
                                                                        {
                                                                            continue;
                                                                        }
                                                                        //var rtTemp = new Route(rt1);
                                                                        Route rtTemp = rt1.getTempCopy(rt1, sol.Options.Select(x => x.Location).ToHashSet().ToList());
                                                                        rtTemp.SequenceOfOptions[firstOptionIndex] = d1;
                                                                        rtTemp.SequenceOfCustomers[firstOptionIndex] = d1.Cust;
                                                                        rtTemp.SequenceOfLocations[firstOptionIndex] = d1.Location;
                                                                        var tw2 = sol.RespectsTimeWindow2(rtTemp, secondOptionIndex, d2.Location);
                                                                        if (!tw1.Item1 || !tw2.Item1)
                                                                        {
                                                                            psm.TimeWindowsError = true;
                                                                            continue;
                                                                        }
                                                                    }
                                                                    else if (firstOptionIndex > secondOptionIndex)
                                                                    {
                                                                        var tw2 = sol.RespectsTimeWindow2(rt2, secondOptionIndex, d2.Location);
                                                                        if (!tw2.Item1)
                                                                        {
                                                                            continue;
                                                                        }
                                                                        //var rtTemp = new Route(rt2);
                                                                        Route rtTemp = rt2.getTempCopy(rt2, sol.Options.Select(x => x.Location).ToHashSet().ToList());
                                                                        rtTemp.SequenceOfOptions[secondOptionIndex] = d2;
                                                                        rtTemp.SequenceOfCustomers[secondOptionIndex] = d2.Cust;
                                                                        rtTemp.SequenceOfLocations[secondOptionIndex] = d2.Location;
                                                                        var tw1 = sol.RespectsTimeWindow2(rtTemp, firstOptionIndex, d1.Location);
                                                                        if (!tw1.Item1 || !tw2.Item1)
                                                                        {
                                                                            psm.TimeWindowsError = true;
                                                                            continue;
                                                                        }
                                                                    }
                                                                }
                                                                else
                                                                {
                                                                    var tw1 = sol.RespectsTimeWindow2(rt1, firstOptionIndex, d1.Location);
                                                                    var tw2 = sol.RespectsTimeWindow2(rt2, secondOptionIndex, d2.Location);
                                                                    if (!tw1.Item1 || !tw2.Item1)
                                                                    {
                                                                        psm.TimeWindowsError = true;
                                                                        continue;
                                                                    }
                                                                }
                                                                //In any other case whether intra or inter route you break 4 arcs and create 4 new arcs
                                                                double costRemoved1 = sol.CalculateDistance(a1.Location, b1.Location) + sol.CalculateDistance(b1.Location, c1.Location);
                                                                double costRemoved2 = sol.CalculateDistance(a2.Location, b2.Location) + sol.CalculateDistance(b2.Location, c2.Location);
                                                                double costAdded1 = sol.CalculateDistance(a1.Location, d1.Location) + sol.CalculateDistance(d1.Location, c1.Location);
                                                                double costAdded2 = sol.CalculateDistance(a2.Location, d2.Location) + sol.CalculateDistance(d2.Location, c2.Location);
                                                                newUtilizationMetricRoute1 = Math.Pow(Convert.ToDouble(rt1.Capacity - (rt1.Load - b1.Cust.Dem + d1.Cust.Dem)), 2);
                                                                newUtilizationMetricRoute2 = Math.Pow(Convert.ToDouble(rt2.Capacity - (rt2.Load - b2.Cust.Dem + d2.Cust.Dem)), 2);
                                                                newSolUtilizationMetric = sol.SolutionUtilizationMetric - rt1.RouteUtilizationMetric - rt2.RouteUtilizationMetric + newUtilizationMetricRoute1 + newUtilizationMetricRoute2;
                                                                if (rt1 != rt2)
                                                                {
                                                                    costChangeFirstRoute = costAdded1 - costRemoved1;
                                                                    costChangeSecondRoute = costAdded2 - costRemoved2;
                                                                }
                                                                moveCost = costAdded1 + costAdded2 - (costRemoved1 + costRemoved2);
                                                            }
                                                        }

                                                    }
                                                    ratio = (sol.SolutionUtilizationMetric + 1) / (newSolUtilizationMetric + 1);
                                                    if (sol.Routes.Count == sol.LowerBoundRoutes)
                                                    {
                                                        ratio = 1;
                                                    }
                                                    sol.RatioCombinedMoveCost = ratio * moveCost;
                                                    if (sol.RatioCombinedMoveCost < psm.MoveCost + 0.001 & moveCost != 0)
                                                    {
                                                        if (rt1 == rt2 && (firstOptionIndex == secondOptionIndex - 1 || secondOptionIndex == firstOptionIndex - 1))
                                                        {
                                                            if (firstOptionIndex == secondOptionIndex - 1)
                                                            {
                                                                if (PromiseIsBroken(a1.Id, d1.Id, moveCost + sol.Cost + 0.001, sol))
                                                                {
                                                                    continue;
                                                                }
                                                                if (PromiseIsBroken(d1.Id, d2.Id, moveCost + sol.Cost + 0.001, sol))
                                                                {
                                                                    continue;
                                                                }
                                                                if (PromiseIsBroken(d2.Id, c2.Id, moveCost + sol.Cost + 0.001, sol))
                                                                {
                                                                    continue;
                                                                }
                                                            }
                                                            else if (secondOptionIndex == firstOptionIndex - 1)
                                                            {
                                                                if (PromiseIsBroken(a2.Id, d2.Id, moveCost + sol.Cost + 0.001, sol))
                                                                {
                                                                    continue;
                                                                }
                                                                if (PromiseIsBroken(d2.Id, d1.Id, moveCost + sol.Cost + 0.001, sol))
                                                                {
                                                                    continue;
                                                                }
                                                                if (PromiseIsBroken(d1.Id, c1.Id, moveCost + sol.Cost + 0.001, sol))
                                                                {
                                                                    continue;
                                                                }
                                                            }
                                                        }
                                                        else
                                                        {
                                                            if (PromiseIsBroken(a1.Id, d1.Id, moveCost + sol.Cost + 0.001, sol))
                                                            {
                                                                continue;
                                                            }
                                                            if (PromiseIsBroken(d1.Id, c1.Id, moveCost + sol.Cost + 0.001, sol))
                                                            {
                                                                continue;
                                                            }
                                                            if (PromiseIsBroken(a2.Id, d2.Id, moveCost + sol.Cost + 0.001, sol))
                                                            {
                                                                continue;
                                                            }
                                                            if (PromiseIsBroken(d2.Id, c2.Id, moveCost + sol.Cost + 0.001, sol))
                                                            {
                                                                continue;
                                                            }
                                                        }
                                                 
                                                        psm.TotalCost = moveCost + openRoutes * 10000;
                                                        psm.PositionOfFirstRoute = firstRouteIndex;
                                                        psm.PositionOfSecondRoute = secondRouteIndex;
                                                        psm.PositionOfFirstOption = firstOptionIndex;
                                                        psm.PositionOfSecondOption = secondOptionIndex;
                                                        psm.CostChangeFirstRt = costChangeFirstRoute;
                                                        psm.CostChangeSecondRt = costChangeSecondRoute;
                                                        psm.MoveCost = moveCost;
                                                        //psm.AltOption1 = opt1;
                                                        //psm.AltOption2 = opt2;
                                                        psm.AltOption1 = sol.Options.Single(x => x.Id == opt1.Id);
                                                        psm.AltOption2 = (Option)sol.Options.Single(x => x.Id == opt2.Id);
                                                    }

                                                }

                                            }

                                        }

                                    }
                                }
                            }

                        }
                    }
                }
            }
            return psm;
        }

        public void ApplyPrioritySwapMove(PrioritySwap psm, Solution sol)
        {
            if (psm.IsValid()) //&& psm.MoveCost < 0) 
            {
                sol.LastMove = "psm";
                Route rt1 = sol.Routes[psm.PositionOfFirstRoute];
                Route rt2 = sol.Routes[psm.PositionOfSecondRoute];
                if (!sol.CheckRouteFeasibility(rt1) || !sol.CheckRouteFeasibility(rt2))
                {
                    Console.WriteLine("-----");
                }
                Option a1 = rt1.SequenceOfOptions[psm.PositionOfFirstOption - 1];
                Option b1 = rt1.SequenceOfOptions[psm.PositionOfFirstOption];
                Option c1 = rt1.SequenceOfOptions[psm.PositionOfFirstOption + 1];
                Option d1 = psm.AltOption1;
                Option a2 = rt2.SequenceOfOptions[psm.PositionOfSecondOption - 1];
                Option b2 = rt2.SequenceOfOptions[psm.PositionOfSecondOption];
                Option c2 = rt2.SequenceOfOptions[psm.PositionOfSecondOption + 1];
                Option d2 = psm.AltOption2;
                /*
                Console.WriteLine("This Priority Swap Move reduces cost by {0} ", psm.MoveCost);
                Console.WriteLine("I change customer {0} from location {1} and priority {2} to location {3} and priority {4} ", b1.Cust.Id, b1.Location.Id, b1.Prio, d1.Location.Id, d1.Prio);
                Console.WriteLine("I change customer {0} from location {1} and priority {2} to location {3} and priority {4} ", b2.Cust.Id, b2.Location.Id, b2.Prio, d2.Location.Id, d2.Prio);
                if (rt1==rt2)
                {
                    Console.WriteLine("Same route");
                    if (psm.PositionOfFirstOption == psm.PositionOfSecondOption + 1 || psm.PositionOfFirstOption == psm.PositionOfSecondOption - 1) {
                        Console.WriteLine("Next to each other");
                    }
                    Console.WriteLine("FirstOptionIndex {0} SecondOptionIndex {1} ", psm.PositionOfFirstOption, psm.PositionOfSecondOption);
                    Console.WriteLine("b1 ID = {0} and a2 ID = {1} a1 ID = {2}", b1.Cust.Id, a2.Cust.Id, a1.Cust.Id);
                    Console.WriteLine("c1 ID = {0} and b2 ID = {1} c2 ID = {2} ", c1.Cust.Id, b2.Cust.Id, c2.Cust.Id);
                }
                else
                {
                    Console.WriteLine("Different route");
                }
                */
                rt1.SequenceOfOptions[psm.PositionOfFirstOption] = d1;
                rt1.SequenceOfCustomers[psm.PositionOfFirstOption] = d1.Cust;
                rt1.SequenceOfLocations[psm.PositionOfFirstOption] = d1.Location;
                rt2.SequenceOfOptions[psm.PositionOfSecondOption] = d2;
                rt2.SequenceOfCustomers[psm.PositionOfSecondOption] = d2.Cust;
                rt2.SequenceOfLocations[psm.PositionOfSecondOption] = d2.Location;
                b1.Location.Cap -= 1;
                b2.Location.Cap -= 1;
                d1.Location.Cap += 1;
                d2.Location.Cap += 1;
                b1.IsServed = false;
                b2.IsServed = false;
                d1.IsServed = true;
                d2.IsServed = true;
                if (rt1 == rt2)
                {
                    rt1.Cost += psm.MoveCost;
                    UpdateRouteCostAndLoad(rt1, sol);
                    rt1.RouteUtilizationMetric = Math.Pow(Convert.ToDouble(rt1.Capacity - rt1.Load), 2);
                    sol.UpdateTimes(rt1);
                    sol.Cost += psm.MoveCost;
                    if (rt1 == rt2 && (psm.PositionOfFirstOption == psm.PositionOfSecondOption - 1 || psm.PositionOfSecondOption == psm.PositionOfFirstOption - 1))
                    {
                        if (psm.PositionOfFirstOption == psm.PositionOfSecondOption - 1)
                        {
                            sol.Promises[a1.Id, d1.Id] = sol.Cost;
                            sol.Promises[d1.Id, d2.Id] = sol.Cost;
                            sol.Promises[d2.Id, c2.Id] = sol.Cost;
                        }
                        else if (psm.PositionOfSecondOption == psm.PositionOfFirstOption - 1)
                        {
                            sol.Promises[a2.Id, d2.Id] = sol.Cost;
                            sol.Promises[d2.Id, d1.Id] = sol.Cost;
                            sol.Promises[d1.Id, c1.Id] = sol.Cost;
                        }
                    }
                    else
                    {
                        sol.Promises[a1.Id, d1.Id] = sol.Cost;
                        sol.Promises[d1.Id, c1.Id] = sol.Cost;
                        sol.Promises[a2.Id, d2.Id] = sol.Cost;
                        sol.Promises[d2.Id, c2.Id] = sol.Cost;
                    }
                    if (!sol.CheckRouteFeasibility(rt1) || !sol.CheckRouteFeasibility(rt2))
                    {
                        Console.WriteLine("-----");
                    }
                }
                else
                {
                    rt1.Cost += psm.CostChangeFirstRt;
                    rt2.Cost += psm.CostChangeSecondRt;
                    UpdateRouteCostAndLoad(rt1, sol);
                    UpdateRouteCostAndLoad(rt2, sol);
                    rt1.RouteUtilizationMetric = Math.Pow(Convert.ToDouble(rt1.Capacity - rt1.Load), 2);
                    rt2.RouteUtilizationMetric = Math.Pow(Convert.ToDouble(rt2.Capacity - rt2.Load), 2);
                    sol.UpdateTimes(rt1);
                    sol.UpdateTimes(rt2);
                    sol.Cost += psm.MoveCost;
                    sol.Promises[a1.Id, d1.Id] = sol.Cost;
                    sol.Promises[d1.Id, c1.Id] = sol.Cost;
                    sol.Promises[a2.Id, d2.Id] = sol.Cost;
                    sol.Promises[d2.Id, c2.Id] = sol.Cost;
                    if (!sol.CheckRouteFeasibility(rt1) || !sol.CheckRouteFeasibility(rt2))
                    {
                        Console.WriteLine("-----");
                    }
                }
            }
        }


        public void UpdateRouteCostAndLoad(Route rt, Solution sol) {
            double tc = 0;
            double tl = 0;
            for (int i = 0; i < rt.SequenceOfOptions.Count - 1; i++) {
                Option A = rt.SequenceOfOptions[i];
                Option B = rt.SequenceOfOptions[i + 1];
                tc += sol.CalculateDistance(A.Location, B.Location);
                tl += A.Cust.Dem;
            }
            rt.Load = tl;
            rt.Cost = tc;
        }

        //! make these to accept only tuples of ids not whole new options
        bool CheckPromises(List<Option[]> arcs, double newCost, Solution sol)
        {
            foreach(Option[] arc in arcs)
            {
                if (newCost >= sol.Promises[arc[0].Id, arc[1].Id])
                {
                    return false;
                }
            }
            return true;
        }

        bool PromiseIsBroken(int a, int b, double newCost, Solution sol)
        {
            if (newCost >= sol.Promises[a, b])
            {
                return true;
            }

            return false;
        }
    }
}
