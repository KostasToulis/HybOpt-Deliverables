using System;


namespace VrdpoProject
{
    public class Flip
    {
        double totalCost;
        double moveCost;
        int targetRoutePosition;
        int targetOptionPosition;
        int originRoutePosition;
        int originOptionPosition;
        int newOptionIndex;
        double costChangeOriginRt;
        double costChangeTargetRt;

        public Flip()
        {
            this.TotalCost = Math.Pow(10, 9);
            this.MoveCost =  Math.Pow(10, 9);
        }

        public void ReinitializeVariables()
        {
            totalCost = Math.Pow(10, 9);
            moveCost =  Math.Pow(10, 9);
            targetRoutePosition = -1;
            targetOptionPosition = -1;
            originRoutePosition = -1;
            originOptionPosition = -1;
            newOptionIndex = -1;
            costChangeOriginRt = -1;
            costChangeTargetRt = -1;
        }

        public bool IsValid()
        {
            return targetOptionPosition != -1;
        }

        public double TotalCost { get => totalCost; set => totalCost = value; }
        public double MoveCost { get => moveCost; set => moveCost = value; }
        public int TargetRoutePosition { get => targetRoutePosition; set => targetRoutePosition = value; }
        public int TargetOptionPosition { get => targetOptionPosition; set => targetOptionPosition = value; }
        public double CostChangeOriginRt { get => costChangeOriginRt; set => costChangeOriginRt = value; }
        public double CostChangeTargetRt { get => costChangeTargetRt; set => costChangeTargetRt = value; }
        public int OriginRoutePosition { get => originRoutePosition; set => originRoutePosition = value; }
        public int OriginOptionPosition { get => originOptionPosition; set => originOptionPosition = value; }
        public int NewOptionIndex { get => newOptionIndex; set => newOptionIndex = value; }
    }
}
