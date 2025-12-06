using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace VrdpoProject
{
    public class Location
    {
        private int id;
        private int xx;
        private int yy;
        private int maxCap;
        private int ready;
        private int due;
        private int type;
        private int serviceTime;
        private int cap = 0;
        //private int customer_id option_id
        public Location(int id, int xx, int yy, int maxCap, int ready, int due, int type, int serviceTime, int cap=0)
        {
            this.id = id;
            this.xx = xx;
            this.yy = yy;
            this.maxCap = maxCap;
            this.ready = ready;
            this.due = due;
            this.type = type;
            this.serviceTime = serviceTime;
            this.cap = cap;
        }
        public Location Clone()
        {
            return new Location(this.Id, this.Xx, this.Yy, this.maxCap, this.Ready, this.Due, this.Type, this.serviceTime, this.cap);
        }

        public int Id { get => id; set => id = value; }
        public int Xx { get => xx; set => xx = value; }
        public int Yy { get => yy; set => yy = value; }
        public int MaxCap { get => maxCap; set => maxCap = value; }
        public int Ready { get => ready; set => ready = value; }
        public int Due { get => due; set => due = value; }
        public int Type { get => type; set => type = value; }
        public int ServiceTime { get => serviceTime; set => serviceTime = value; }
        public int Cap { get => cap; set => cap = value; }
    }
}
