namespace CHRVRP
{
    public class Route
    {
        public List<Node> sequence;
        public int id;
        public int load;
        public double totalDistance;
        public double cumDistance;
        public double max;
        public double min;
        public Route()
        {
            sequence = new List<Node>();
        }

        public Route(Node dStart, int id)
        {
            sequence = new List<Node> { dStart };
            this.id = id;
            dStart.routeIndex = id;
            load = 0;
            totalDistance = 0;
            cumDistance = 0;
            max = 0;
            min = 10_000_000;
        }

        public override string ToString()
        {
            StringWriter sw = new StringWriter();
            sw.Write($"[ ");
            foreach (Node n in sequence)
            {
                
                sw.Write($"{n.id} ");

            }
            sw.Write($"] ");
            double completed = Math.Round(totalDistance, 2);
            sw.Write($"Time Completed: {totalDistance}");
            sw.Write($" Max: {max} Min: {min}");
            return sw.ToString();
        }

        public void printRoute()
        {
            Console.WriteLine(ToString());
        }

        public void Add(Node node, int position, double objective)
        {
            if (position == this.sequence.Count)
            {
                this.sequence.Add(node);
            }
            else
            {
                this.sequence.Insert(position, node);
            }
            this.totalDistance += objective;
            this.load++;
            node.isRouted = true;
            node.routeIndex = this.id;
        }

        public void IndexInRoute()
        {
            for (int i = 2; i < this.sequence.Count; i++)
            {
                this.sequence[i].indexInRoute = i;
            }
        }
        
    }

}