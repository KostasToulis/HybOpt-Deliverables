using System;
using System.Collections.Generic;
using System.Text;

namespace F_CVRP.Operators
{
    public class ReplaceMove
    {
        public ReplaceMove()
        {
            Reset();
        }

        public int routeIndex { get; set; }
        public int nodeIndex { get; set; }
        public Node selectedNode { get; set; }
        public float moveCost { get; set; }
        public float? moveCostPenalized { get; set; }

        public void Reset()
        {
            routeIndex = -1;
            nodeIndex = -1;
            moveCost = 100000000;
            moveCostPenalized = 100000000;
        }

        public void StoreBestReplaceMove(int rtInd, int nodeInd, Node node, float moveCost, float? moveCostPenalized)
        {
            this.routeIndex = rtInd;
            this.nodeIndex = nodeInd;
            this.selectedNode = node;
            this.moveCost = moveCost;
            this.moveCostPenalized = moveCostPenalized;
        }

    }
}
