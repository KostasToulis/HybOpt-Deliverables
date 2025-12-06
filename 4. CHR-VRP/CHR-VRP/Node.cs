namespace CHRVRP
{
    public class Node
    {

        public string id;
        public int serialNumber;
        public double x;
        public double y;
        public Model.Category category;
        public bool isRouted;
        public int routeIndex;
        public int indexInRoute;
        public double arrivalTime;
        public double averageDistanceToAllNodes;
        public List<Node> nearestNodes;
        // na doyme mipos xriazetai attribute me lista distances 

        public Node()
        {

        }

        public Node(Node n)
        {
            this.id = n.id;
            this.serialNumber = n.serialNumber;
            this.x = n.x;
            this.y = n.y;
            this.category = n.category;
            this.isRouted = n.isRouted;
            this.routeIndex = n.routeIndex;
            this.indexInRoute = n.indexInRoute;
            this.arrivalTime = n.arrivalTime;
            this.averageDistanceToAllNodes = n.averageDistanceToAllNodes;
            nearestNodes = n.nearestNodes;
        }

        public Node(string id, int serialNumber, double x, double y, Model.Category category)
        {
            this.id = id;
            this.serialNumber = serialNumber;
            this.x = x;
            this.y = y;
            this.category = category;
            this.isRouted = false;
            this.arrivalTime = 0;
            this.averageDistanceToAllNodes = 0;
            nearestNodes = new List<Node>();
        }

        public override string ToString()
        {
            return String.Format("Node({0}, {1}, {2}, {3}, {4}, Arrival Time: {5})",
                this.id, this.serialNumber, this.x, this.y, this.category, this.arrivalTime);
        }


    }

    public class SupplyPoint : Node
    {/*
        public Dictionary<Node, double> vehiclesArrivalTimes;

        public SupplyPoint(string id, int serialNumber, double x, double y, Model.Category category) 
            : base( id, serialNumber, x, y, category)
        {
            vehiclesArrivalTimes = new Dictionary<Node, double>();
        }*/
    }
}