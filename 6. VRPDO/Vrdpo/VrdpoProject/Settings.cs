using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace VrdpoProject
{
    public class Settings
    {
        public int restarts { get; set; }
        public int repetitions { get; set; }
        public bool verbal { get; set; }
        public double promisesRestartRatio { get; set; }
        public bool multiRestart { get; set; }
        public string schema { get; set; }
        public string type { get; set; }
    }
}