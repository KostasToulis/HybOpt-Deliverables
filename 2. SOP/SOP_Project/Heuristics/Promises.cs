using System;
using System.Collections.Generic;
using System.Text;

namespace SOP_Project
{
    public class Promises
    {
        public static double T = 1;
        public static Random r;

        public static void MakePromise(Set removed, int current_profit)
        {
            removed.profit_promise = current_profit;
        }

        public static bool MoveIsAdmissible(int profitAdded, int currentSolProfit, Set toBeAdded)
        {
            int d_z = currentSolProfit + profitAdded - toBeAdded.profit_promise;
            if (d_z > 0)
            {
                return true;
            }
            //else  // find the probability of the certain move overriding the promise and becoming admissible
            //      // we use the simulated annealing formula to calculate that value
            //{
            //    double override_prob = Math.Exp(d_z / T);
            //    if (r.NextDouble() < override_prob)
            //    {
            //        return true;
            //    }
            //}
            return false;
        }

        public static void reinit_promises(Model m)
        {
            for (int i = 0; i < m.sets.Count; i++)
            {
                m.sets[i].profit_promise = -1;
            }
        }
    }
}
