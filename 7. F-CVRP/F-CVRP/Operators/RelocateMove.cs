using System;
using System.Collections.Generic;
using System.Text;

namespace F_CVRP.Operators
{
    public class RelocationMove
    {
        public RelocationMove()
        {
            Reset();
        }

        public int originNodePosition { get; set; }
        public int originRoutePosition { get; set; }
        public int targetNodePosition { get; set; }
        public int targetRoutePosition { get; set; }
        public float costChangeOriginRt { get; set; }
        public float costChangeTargetRt { get; set; }
        public float moveCost { get; set; }
        public float? moveCostPenalized { get; set; }

        public void Reset()
        {
            originNodePosition = -1;
            originRoutePosition = -1;
            targetNodePosition = -1;
            targetRoutePosition = -1;
            costChangeOriginRt = 100000000;
            costChangeTargetRt = 100000000;
            moveCost = 100000000;
            moveCostPenalized = 100000000;
        }

        public void StoreBestRelocationMove(int originRouteIndex, int targetRouteIndex, int originNodeIndex, int targetNodeIndex, float moveCost, float originRtCostChange, float targetRtCostChange, float? moveCostPenalized)
        {
            this.originRoutePosition = originRouteIndex;
            this.originNodePosition = originNodeIndex;
            this.targetRoutePosition = targetRouteIndex;
            this.targetNodePosition = targetNodeIndex;
            this.costChangeOriginRt = originRtCostChange;
            this.costChangeTargetRt = targetRtCostChange;
            this.moveCost = moveCost;
            this.moveCostPenalized = moveCostPenalized;
        }
    }
}
