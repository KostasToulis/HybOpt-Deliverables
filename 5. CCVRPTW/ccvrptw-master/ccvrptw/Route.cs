using CCVRPTW;

namespace CCVRPTW
{
    public class Route
    {
        public List<Node> sequence;
        public int load;
        public double cost; // to cost enos route(vehicle) einai h xronikh stigmh pou oloklhrwthike to service tou last customer.
        public double totalSlack; //the total slack time of the vehicle
        public double maxPushForward; //the total time that the route time can be streched
        public double timeWindowPenalty;
        public Route()
        {
            sequence = new List<Node>();
        }

        public Route(Node dStart, Node dEnd, bool extra)
        {
            sequence = new List<Node> { dStart, dEnd };
            load = 0;
            cost = 0;
        }

        public string ToString(bool simple = false)
        {
            if (!simple)
                return ToString();

            StringWriter sw = new StringWriter();

            for (int i = 1; i < sequence.Count - 1; i++)
            {
                Node n = sequence[i];

                double d = Math.Round(n.arrivalTime + n.waitingTime, 2);

                sw.Write($"{n.id} ");

            }

            return sw.ToString();
        }

        public void UpdateRouteNodes()
        {
            totalSlack = 0.0;
            timeWindowPenalty = 0.0;
            maxPushForward = 1000000.0;
            for (int i = 0; i < sequence.Count - 1; i++)
            {
                Node u = sequence[i];
                Node v = sequence[i + 1];
                v.arrivalTime = u.arrivalTime + u.waitingTime + u.serviceTime + ProblemConfiguration.model.distances[u.id, v.id];
                if (v.arrivalTime > v.windowEnd)
                {
                    v.waitingTime = v.windowEnd - v.arrivalTime;
                    timeWindowPenalty += FeasibleSegment.timeWindowPenaltyWeight * Math.Abs(v.waitingTime);
                }
                else
                {
                    v.waitingTime = Math.Max(0, v.windowStart - v.arrivalTime);
                }
                
                totalSlack += v.waitingTime;
                maxPushForward = Math.Min(maxPushForward, v.windowEnd - v.arrivalTime + v.waitingTime);
            }
        }

        public void ClearNodes()
        {
            while(sequence.Count > 2) 
                sequence.RemoveAt(1);
            UpdateRouteNodes();
        }
        
        public override string ToString()
        {
            StringWriter sw = new StringWriter();

            foreach (Node n in sequence)
            {
                double d = Math.Round(n.arrivalTime + n.waitingTime, 2);

                if (ProblemConfiguration.UseTimeWindows)
                {
                    double w1 = Math.Round(n.windowStart);
                    double w2 = Math.Round(n.windowEnd);
                    sw.Write($"{n.id}({d})({w1}-{w2})  ");
                }
                else
                {
                    sw.Write($"{n.id}({d})  ");
                }

            }
            double c = Math.Round(cost, 2);
            sw.WriteLine($"Cost {c} Load {load} Wrap {timeWindowPenalty} ");

            return sw.ToString();
        }

        public void printRoute()
        {
            Console.WriteLine(ToString());
        }

    }

}
