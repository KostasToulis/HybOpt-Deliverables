using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace VrdpoProject
{
    public class Swap
    {
        int positionOfFirstRoute;
        int positionOfSecondRoute;
        int positionOfFirstOption;
        int positionOfSecondOption;
        double costChangeFirstRt;
        double costChangeSecondRt;
        double moveCost;
        double totalCost;

        public Swap()
        {
            this.TotalCost = Math.Pow(10, 9);
            this.MoveCost = Math.Pow(10, 9);
        }

        public void ReinitializeVariables()
        {
            positionOfFirstRoute = -1;
            positionOfSecondRoute = -1;
            positionOfFirstOption = -1;
            positionOfSecondOption = -1;
            costChangeFirstRt = -1;
            costChangeSecondRt = -1;
            moveCost = Math.Pow(10, 9);
            totalCost = Math.Pow(10, 9);
        }

        public bool IsValid()
        {
            return positionOfFirstRoute != -1;
        }

        public int PositionOfFirstRoute { get => positionOfFirstRoute; set => positionOfFirstRoute = value; }
        public int PositionOfSecondRoute { get => positionOfSecondRoute; set => positionOfSecondRoute = value; }
        public int PositionOfFirstOption { get => positionOfFirstOption; set => positionOfFirstOption = value; }
        public int PositionOfSecondOption { get => positionOfSecondOption; set => positionOfSecondOption = value; }
        public double CostChangeFirstRt { get => costChangeFirstRt; set => costChangeFirstRt = value; }
        public double CostChangeSecondRt { get => costChangeSecondRt; set => costChangeSecondRt = value; }
        public double MoveCost { get => moveCost; set => moveCost = value; }
        public double TotalCost { get => totalCost; set => totalCost = value; }

    }
}
