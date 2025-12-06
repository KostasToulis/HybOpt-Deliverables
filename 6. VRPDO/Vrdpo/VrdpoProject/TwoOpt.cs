using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace VrdpoProject
{
    public  class TwoOpt
    {
        int positionOfFirstRoute;
        int positionOfSecondRoute;
        int positionOfFirstOption;
        int positionOfSecondOption;
        double[] ect1;
        double[] ect2;
        double[] lat1;
        double[] lat2;
        double moveCost = Math.Pow(10, 9);
        double totalCost = Math.Pow(10, 9);

        public TwoOpt()
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
            totalCost = Math.Pow(10, 9);
            moveCost = Math.Pow(10, 9);
        }
        public bool IsValid()
        {
            return positionOfFirstRoute != -1;
        }

        public int PositionOfFirstRoute { get => positionOfFirstRoute; set => positionOfFirstRoute = value; }
        public int PositionOfSecondRoute { get => positionOfSecondRoute; set => positionOfSecondRoute = value; }
        public int PositionOfFirstOption { get => positionOfFirstOption; set => positionOfFirstOption = value; }
        public int PositionOfSecondOption { get => positionOfSecondOption; set => positionOfSecondOption = value; }
        public double MoveCost { get => moveCost; set => moveCost = value; }
        public double TotalCost { get => totalCost; set => totalCost = value; }
        public double[] Ect1 { get => ect1; set => ect1 = value; }
        public double[] Ect2 { get => ect2; set => ect2 = value; }
        public double[] Lat1 { get => lat1; set => lat1 = value; }
        public double[] Lat2 { get => lat2; set => lat2 = value; }
    }
}
