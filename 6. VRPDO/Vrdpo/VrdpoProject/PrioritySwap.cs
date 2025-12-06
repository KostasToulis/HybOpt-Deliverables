using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace VrdpoProject
{
    public class PrioritySwap
    {
        int positionOfFirstRoute;
        int positionOfSecondRoute;
        int positionOfFirstOption;
        int priorityOfFirstOption;
        int positionOfSecondOption;
        int priorityOfSecondOption;
        double costChangeFirstRt;
        double costChangeSecondRt;
        double moveCost;
        double totalCost;
        Option altOption1;
        Option altOption2;
        bool moveRejected;

        bool timeWindowsError;
        bool moveApplied;

        public PrioritySwap()
        {
            this.MoveCost = Math.Pow(10, 9);
            this.TotalCost = Math.Pow(10, 9);
        }

        public void ReinitializeVariables()
        {
            positionOfFirstRoute = -1;
            positionOfSecondRoute = -1;
            positionOfFirstOption = -1;
            priorityOfFirstOption = -1;
            positionOfSecondOption = -1;
            priorityOfSecondOption = -1;
            costChangeFirstRt = -1;
            costChangeSecondRt = -1;
            totalCost = Math.Pow(10, 9);
            moveCost = Math.Pow(10, 9);
            timeWindowsError = false;
            moveRejected = false;
        }

        public bool IsValid()
        {
            return positionOfFirstRoute != -1;
        }

        public int PositionOfFirstRoute { get => positionOfFirstRoute; set => positionOfFirstRoute = value; }
        public int PositionOfSecondRoute { get => positionOfSecondRoute; set => positionOfSecondRoute = value; }
        public int PositionOfFirstOption { get => positionOfFirstOption; set => positionOfFirstOption = value; }
        public int PriorityOfFirstOption { get => priorityOfFirstOption; set => priorityOfFirstOption = value; }
        public int PositionOfSecondOption { get => positionOfSecondOption; set => positionOfSecondOption = value; }
        public int PriorityOfSecondOption { get => priorityOfSecondOption; set => priorityOfSecondOption = value; }
        public double CostChangeFirstRt { get => costChangeFirstRt; set => costChangeFirstRt = value; }
        public double CostChangeSecondRt { get => costChangeSecondRt; set => costChangeSecondRt = value; }
        public double MoveCost { get => moveCost; set => moveCost = value; }
        public double TotalCost { get => totalCost; set => totalCost = value; }
        public Option AltOption1 { get => altOption1; set => altOption1 = value; }
        public Option AltOption2 { get => altOption2; set => altOption2 = value; }
        public bool TimeWindowsError { get => timeWindowsError; set => timeWindowsError = value; }
        public bool MoveRejected { get => moveRejected; set => moveRejected = value; }
    }
}