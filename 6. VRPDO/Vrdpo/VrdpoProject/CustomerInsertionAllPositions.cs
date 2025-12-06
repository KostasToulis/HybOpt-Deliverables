using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace VrdpoProject
{

    internal class CustomerInsertionAllPositions
    {
        private Customer customer;
        private Route route;
        private int insertionPosition;
        private double cost;
        private double costPenalized;
        private double duration;
        private Option option;
        private Location location;
        private double ect;
        private double lat;

        public CustomerInsertionAllPositions(Customer customer, Route route, int insertionPosition,
            double cost, double costPenalized, double duration, Option option, Location location)
        {
            this.customer = customer;
            this.route = route;
            this.insertionPosition = insertionPosition;
            this.cost = cost;
            this.costPenalized = costPenalized;
            this.duration = duration;
            this.option = option;
            this.location = location;
        }

        public CustomerInsertionAllPositions()
        {
            this.customer = null;
            this.route = null;
            this.insertionPosition = -1000;
            this.cost = 1000000;
            this.costPenalized = 1000000;
            this.duration = 10000000;
            this.option = null;
        }

        public CustomerInsertionAllPositions(CustomerInsertionAllPositions original)
        {
            this.customer = original.customer;
            this.route = original.route;
            this.insertionPosition = original.insertionPosition;
            this.cost = original.cost;
            this.costPenalized = original.costPenalized;
            this.duration = original.duration;
            this.option = original.option;
            this.location = original.location;
        }

        public int InsertionPosition { get => insertionPosition; set => insertionPosition = value; }
        public double Cost { get => cost; set => cost = value; }
        public double CostPenalized { get => costPenalized; set => costPenalized = value; }
        public double Duration { get => duration; set => duration = value; }
        internal Customer Customer { get => customer; set => customer = value; }
        internal Route Route { get => route; set => route = value; }
        internal Option Option { get => option; set => option = value; }
        internal Location Location { get => location; set => location = value; }
        public double Ect { get => ect; set => ect = value; }
        public double Lat { get => lat; set => lat = value; }
    }
}