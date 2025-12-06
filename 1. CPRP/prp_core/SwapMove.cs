using System.Collections.Generic;

namespace PRP
{
    internal class SwapMove
    {
        public MOVES operatorType;
        public int day;

        public int firstRoutePosition;
        public int firstNodePosition;

        public int secondRoutePosition;
        public int secondNodePosition;

        public double firstRouteObjectiveChange;
        public double secondRouteObjectiveChange;

        public double totalObjectiveChange;
        public int[] arcsDeleted;
        public SwapMove()
        {
            operatorType = MOVES.SWAP;
            totalObjectiveChange = double.MaxValue;
            arcsDeleted = new int[10*3 + 1]; //Store at most 10 arcs
        }
    }
}