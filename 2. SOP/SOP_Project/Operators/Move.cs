
namespace SOP_Project
{
    public abstract class Move // abstract class useful for generalizing operators and easily handling them by
                               // taking advantage of their common features 
    {
        public abstract bool IsMoveFound();
        public abstract void FindBestMove(Model m, Solution sol);
        public abstract void ApplyBestMove(Solution sol);
        public abstract Move ShallowCopy(); // copy the elements of an object into a new one
    
        public string GetMoveType()
        {
            return this.GetType().ToString().Replace("SOP_Project.", "");
        }
    }
}
