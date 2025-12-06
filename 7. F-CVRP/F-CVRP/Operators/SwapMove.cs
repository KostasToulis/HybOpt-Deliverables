using System;
using System.Collections.Generic;
using System.Text;

namespace F_CVRP.Operators
{
    public class SwapMove
    {
        public SwapMove()
        {
            Reset();
        }

        public int firstNodeIndex { get; set; }
        public int firstRouteIndex { get; set; }
        public int secondNodeIndex { get; set; }
        public int secondRouteIndex { get; set; }
        public float costChangeFirstRt { get; set; }
        public float costChangeSecondRt { get; set; }
        public float moveCost { get; set; }
        public float? moveCostPenalized { get; set; }

        public void Reset()
        {
            firstNodeIndex = -1;
            firstRouteIndex = -1;
            secondNodeIndex = -1;
            secondRouteIndex = -1;
            costChangeFirstRt = 100000000;
            costChangeSecondRt = 100000000;
            moveCost = 100000000;
            moveCostPenalized = 100000000;
        }


        public void StoreBestSwapMove(int firstRouteIndex, int secondRouteIndex, int firstNodeIndex, int secondNodeIndex, float moveCost, float costChangeFirstRoute, float costChangeSecondRoute, float? moveCostPenalized)
        {
            this.firstRouteIndex = firstRouteIndex;
            this.secondRouteIndex = secondRouteIndex;
            this.firstNodeIndex = firstNodeIndex;
            this.secondNodeIndex = secondNodeIndex;
            this.costChangeFirstRt = costChangeFirstRoute;
            this.costChangeSecondRt = costChangeSecondRoute;
            this.moveCost = moveCost;
            this.moveCostPenalized = moveCostPenalized;
        }
    }
}
