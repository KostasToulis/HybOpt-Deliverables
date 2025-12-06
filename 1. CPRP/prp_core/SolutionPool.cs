using System.Collections.Generic;

namespace PRP
{
    public class SolutionPool
    {
        public List<Solution> solutions = new List<Solution>();
        public Solution best;
        public Solution worst;
        private int _size;

        public SolutionPool(int size)
        {
            _size = size; //Pool size
        }
        

        public void addSolution(Solution sol)
        {        
            if (solutions.Count >= _size)
            {
                if (worst.totalObjective > sol.totalObjective)
                {
                    //Add solution and remove worst
                    solutions.Remove(worst);
                    Solution cpy = new Solution(sol, 0.0);
                    solutions.Add(cpy);
                }
            }
            else
            {
                Solution cpy = new Solution(sol, 0.0);
                solutions.Add(cpy);
            }
            
            //Recalculate Best and Worst solutions 
            calculateBest();
            calculateWorst();
            
        }

        public void removeSolution(Solution sol)
        {
            solutions.Remove(sol);
        }

        public void clear()
        {
            solutions.Clear();
        }

        public void calculateBest()
        {
            best = solutions[0];
            for (int i = 1; i < solutions.Count; i++)
            {
                if (solutions[i].totalObjective < best.totalObjective)
                    best = solutions[i];
            }
        }
        
        public void calculateWorst()
        {
            worst = solutions[0];
            for (int i = 1; i < solutions.Count; i++)
            {
                if (solutions[i].totalObjective > worst.totalObjective)
                    worst = solutions[i];
            }
        }
    }
}