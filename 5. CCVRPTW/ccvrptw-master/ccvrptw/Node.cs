
namespace CCVRPTW
{
    public class Node
    {

        public int id;
        public double x;
        public double y;
        public int demand;
        public double serviceTime; // o xronos pou xreiazetai gia na oloklhrwthei to service tou pelath
        public double windowStart; // enarksh parathurou
        public double windowEnd; // lhksh parathurou
        public double arrivalTime; // o xronos ston opoio to oxhma ftanei ston pelath
        public double waitingTime; // o xronos pou xreiazetai na perimenei to oxhma ston pelath prin ksekinisei to service
        public double pushForward; // o megistos xronos pou mporoume na push to arrivalTime tou oxhmatos, wste to service na ksekinisei sigoura entos parathurou stous epomenous
        public bool isRouted;
        public bool isDepot;

        public Node()
        {

        }

        public Node(Node n)
        {
            this.id = n.id;
            this.x = n.x;
            this.y = n.y;
            this.demand = n.demand;
            this.serviceTime = n.serviceTime;
            this.windowStart = n.windowStart;
            this.windowEnd = n.windowEnd;
            this.arrivalTime = n.arrivalTime;
            this.waitingTime = n.waitingTime;
            this.pushForward = n.pushForward;
            this.isRouted = n.isRouted;
            this.isDepot = n.isDepot;
        }

        public Node(int id, double x, double y, int d, double ws, double we, double st, bool isDepot)
        {
            this.id = id;
            this.x = x;
            this.y = y;
            this.demand = d;
            this.serviceTime = st;
            this.windowStart = ws;
            this.windowEnd = we;
            this.arrivalTime = 0;
            this.waitingTime = 0;
            this.pushForward = 0;
            this.isRouted = false;
            this.isDepot = isDepot;
        }
    }

}
