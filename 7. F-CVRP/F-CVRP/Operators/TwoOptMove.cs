using System;
using System.Collections.Generic;
using System.Text;

namespace F_CVRP.Operators
{
    public class TwoOptMove
    {
        public TwoOptMove()
        {
            Reset();
        }

        public int firstNodePosition { get; set; }
        public int firstRoutePosition { get; set; }
        public int secondNodePosition { get; set; }
        public int secondRoutePosition { get; set; }
        public float moveCost { get; set; }
        public float? moveCostPenalized { get; set; }

        public void Reset()
        {
            firstNodePosition = -1;
            firstRoutePosition = -1;
            secondNodePosition = -1;
            secondRoutePosition = -1;
            moveCost = 100000000;
            moveCostPenalized = 100000000;
        }

        public void StoreBestTwoOptMove(int rtInd1, int rtInd2, int nodeInd1, int nodeInd2, float moveCost, float? moveCostPenalized)
        {
            this.firstNodePosition = nodeInd1;
            this.firstRoutePosition = rtInd1;
            this.secondNodePosition = nodeInd2;
            this.secondRoutePosition = rtInd2;
            this.moveCost = moveCost;
            this.moveCostPenalized = moveCostPenalized;
        }

    }
}
