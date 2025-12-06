using MSOP.Fundamentals;

namespace MSOP.Operators
{
    public abstract class Move // abstract class useful for generalizing operators and easily handling them by
                               // taking advantage of their common features 
    {
        public abstract bool IsMoveFound();
        public abstract void FindBestMove(Solution sol);
        public abstract void ApplyBestMove(Solution sol);
        public abstract Move DeepCopy(); // copy the elements of an object into a new one

        public string GetMoveType()
        {
            return this.GetType().ToString().Replace("MSOP.Operators.", "");
        }
    }
}
