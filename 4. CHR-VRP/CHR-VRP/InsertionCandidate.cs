namespace CHRVRP
{
    public struct InsertionCandidate
    {
        public int routeInd;
        public int nodeInd;
        public int inRouteInd;
        public double distanceChange;
        public double cumDistChange;
        public double min;
        public double max;
        public double objectiveCriterion;

        public InsertionCandidate()
        {
            this.routeInd = -1;
            this.nodeInd = -1;
            this.inRouteInd = -1;
            this.distanceChange = 0;
            this.cumDistChange = 0;
            this.min = 0;
            this.max = 0;
            this.objectiveCriterion = 0;
        }

        public InsertionCandidate(int routeInd, int nodeInd, int inRouteInd, double distanceChange, double cumDistChange, double min, double max, double objectiveCriterion)
        {
            this.routeInd = routeInd;
            this.nodeInd = nodeInd;
            this.inRouteInd = inRouteInd;
            this.distanceChange = distanceChange;
            this.cumDistChange = cumDistChange;
            this.min = min;
            this.max = max;
            this.objectiveCriterion = objectiveCriterion;
        }

        public InsertionCandidate(InsertionCandidate ic)
        {
            this.routeInd = ic.routeInd;
            this.nodeInd = ic.nodeInd;
            this.inRouteInd = ic.inRouteInd;
            this.distanceChange = ic.distanceChange;
            this.cumDistChange = ic.cumDistChange;
            this.min = ic.min;
            this.max = ic.max;
            this.objectiveCriterion = ic.objectiveCriterion;
        }

        public override string ToString()
        {
            return string.Format("InsertionCandidate({0}, {1}, {2}, {3}, {4}, {5}, {6}, {7})",
                               this.objectiveCriterion, this.routeInd, this.nodeInd, this.inRouteInd, this.distanceChange, this.cumDistChange, this.min, this.max);
        }
    }

}