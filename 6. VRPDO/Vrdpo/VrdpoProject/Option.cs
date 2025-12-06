using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace VrdpoProject
{
    public class Option
    {
        private int id;
        private Location location;
        private Customer cust;
        private int prio;
        private int serviceTime;
        private int cost;
        private bool isServed;
        private int ready;
        private int due;

        public Option(int id, Location location, Customer cust, int prio, int serviceTime, int cost, int ready, int due)
        {
            this.id = id;
            this.location = location;
            this.cust = cust;
            this.prio = prio;
            this.serviceTime = (serviceTime * 10);//+ location.ServiceTime;
            this.cost = cost;
            isServed = false;
            this.ready = ready;
            this.due = due;
        }

        public Option Clone(Location loc)
        {
            // needs to add cloned customer seperately
            Location locationCopy = loc;
            var clone = (Option)this.MemberwiseClone();
            clone.Location = locationCopy;
            return clone;
        }

        public int Id { get => id; set => id = value; }
        public Location Location { get => location; set => location = value; }
        public Customer Cust { get => cust; set => cust = value; }
        public int Prio { get => prio; set => prio = value; }
        public int ServiceTime { get => serviceTime; set => serviceTime = value; }
        public int Cost { get => cost; set => cost = value; }
        public bool IsServed { get => isServed; set => isServed = value; }
        public int Ready { get => ready; set => ready = value; }
        public int Due { get => due; set => due = value; }
    }
}
