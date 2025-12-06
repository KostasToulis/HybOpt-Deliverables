using System.Collections.Generic;

namespace PRP
{
    internal class _2OptMove
    {
        public MOVES operatorType;
        public int day;

        public int firstRoutePosition;
        public int firstNodePosition;

        public int secondRoutePosition;
        public int secondNodePosition;

        public double firstRouteObjectiveChange;
        public double secondRouteObjectiveChange;
        public int[] arcsDeleted;
        public double totalObjectiveChange;

        public _2OptMove()
        {
            operatorType = MOVES.TWOOPT;
            totalObjectiveChange = double.MaxValue;
            arcsDeleted = new int[10*3 + 1];
        }
    }
}