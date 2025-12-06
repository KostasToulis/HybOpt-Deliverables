// class Node
namespace SOP_Project
{
    public class Node
    {
        public int id;
        public double x;
        public double y;
        public int profit;
        public int set_id;
        public int pool_profit; // the profit that will be considered when the solution from pool is created

        public Node(int id, double x, double y, int set_id)
        {
            this.id = id;
            this.x = x;
            this.y = y;
            this.set_id = set_id;
            profit = 0;
            pool_profit = 0;
        }

        public Node DeepCopy() // generates a shallow copy of a Model object
        {
            return new Node(this.id, this.x, this.y, this.set_id);
        }

        override
        public string ToString()
        {
            return "id:" + id + " | (x,y):(" + x + "," + y + ") | profit " + profit + " | set_id:" + set_id;
        }
    }
}
