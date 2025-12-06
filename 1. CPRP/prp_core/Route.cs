using System;
using System.Collections.Generic;
using System.Data;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PRP
{
    public class Route
    {
        public List<Node> nodes;
        public List<int> quantDelivered;
        public List<int> loadTillMe;
        public List<double> costTillEnteringMe;

        public int load;
        public int realCapacity;
        public int effectiveCapacity;
        public double totalRoutingCost;
        public bool modified;

        public Route(int maxLoad)
        {
            nodes = new List<Node>();
            quantDelivered = new List<int>();
            realCapacity = maxLoad;
            effectiveCapacity = realCapacity;
            load = 0;
            modified = false;
            totalRoutingCost = 0.0;
            
            loadTillMe = new List<int>();
            costTillEnteringMe  = new List<double>();

            //Init loadTillMe
            for (int i = 0; i < 2; i++)
            {
                loadTillMe.Add(0);
                costTillEnteringMe.Add(0);
            }
                
        }

        public Route(Route r){
            //Copy basic attributes
            realCapacity = r.realCapacity;
            effectiveCapacity = r.effectiveCapacity;
            load = r.load;
            totalRoutingCost = r.totalRoutingCost;
            modified = r.modified;

            loadTillMe = new List<int>(r.loadTillMe);
            quantDelivered = new List<int>(r.quantDelivered);
            costTillEnteringMe = new List<double>(r.costTillEnteringMe);

            //Copy Nodes
            nodes = new List<Node>();
            for (int n = 0; n < r.nodes.Count; n++)
            {
                Node nNode = new Node(r.nodes[n]);
                
                //Explicitly fix the deliveryServices
                for (int i = 0; i < nNode.horizonDeliveryServices.Length; i++)
                {
                    nNode.horizonDeliveryServices[i] = new CustDelivery(nNode.deliveredQuantities[i], this);    
                }
                
                nodes.Add(nNode);
            }
        }

        public void initialize(Node depot)
        {
            reset();
            
            nodes.Add(depot);
            nodes.Add(depot);

            quantDelivered.Add(0);
            quantDelivered.Add(0);

            totalRoutingCost = 0;
            loadTillMe = new List<int>(nodes.Count);
            costTillEnteringMe = new List<double>(nodes.Count);


            for (int i = 0; i < 2; i++)
            {
                loadTillMe.Add(0);
                costTillEnteringMe.Add(0);
            }
        }
        
        public override string ToString()
        {
            return string.Join(",", nodes);

        }

        public void SetLoadAndCostLists(int p, PRP model)
        {
            double rtTillMe = load;
            loadTillMe.Clear();
            loadTillMe.Add(0);
            for (int k = 1; k < nodes.Count - 1; k++)
            {
                loadTillMe.Add(loadTillMe.Last() + nodes[k].deliveredQuantities[p]);
            }
            loadTillMe.Add(loadTillMe.Last());

            costTillEnteringMe.Clear();
            costTillEnteringMe.Add(0);
            for (int k = 1; k < nodes.Count; k++)
            {
                Node prev = nodes[k - 1];
                Node me = nodes[k];

                costTillEnteringMe.Add(costTillEnteringMe.Last() + model.distMatrix[prev.uid, me.uid]);
            }
        }

        public void calculateRoutingCost(PRP model)
        {
            totalRoutingCost = 0;
            for (int k = 1; k < nodes.Count; k++)
            {
                Node prev = nodes[k - 1];
                Node me = nodes[k];
                totalRoutingCost += model.distMatrix[prev.uid, me.uid];
            }
        }


        public void reset()
        {
            nodes.Clear();
            quantDelivered.Clear();
            loadTillMe.Clear();
            costTillEnteringMe.Clear();
            load = 0;
            totalRoutingCost = 0;
        }
    }
}
