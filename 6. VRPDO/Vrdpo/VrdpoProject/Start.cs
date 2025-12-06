using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace VrdpoProject
{
    internal class Start
    {
        static void Main(string[] args)
        {
            InstanceReader model = new(args[0]);
            Solver solver = new();
            solver.Instance = args[0];
            solver.Solve();
        }
    }
}
