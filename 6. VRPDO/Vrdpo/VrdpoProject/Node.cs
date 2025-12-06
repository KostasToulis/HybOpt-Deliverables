using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace VrdpoProject
{
    internal class Node
    {
        private int id;
        private int location;
        private int xx;
        private int yy;
        private int dem;
        private int cap;
        private int ready;
        private int due;
        private int type;
        private int serviceTime;
        private bool isRouted;
        public Node(int id, int location, int xx, int yy, int dem, int cap, int serviceTime, bool isRouted, int ready, int due, int type)
        {
            this.id = id;
            this.location = location;
            this.xx = xx;
            this.yy = yy;
            this.dem = dem;
            this.cap = cap;
            this.serviceTime = serviceTime;
            this.isRouted = isRouted;
            this.ready = ready;
            this.due = due;
            this.type = type;
        }

        public int Id { get => id; set => id = value; }

        public int Location { get => location; set => location = value; }

        public int Xx { get => xx; set => xx = value; }

        public int Yy { get => yy; set => yy = value; }

        public int Dem { get => dem; set => dem = value; }

        public int Cap { get => cap; set => cap = value; }

        public int ServiceTime { get => serviceTime; set => serviceTime = value; }

        public bool IsRouted { get => isRouted; set => isRouted  = value; }
        public int Ready { get => ready; set => ready = value; }
        public int Due { get => due; set => due = value; }
        public int Type { get => type; set => type = value; }
    }
}
