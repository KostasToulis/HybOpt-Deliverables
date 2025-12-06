namespace CCVRPTW {

    abstract class Move
    {
        public int originRouteIndex;
        public int targetRouteIndex;
        public int originNodeIndex;
        public int targetNodeIndex;
        public double cost;
        public double penalty;
        public double originRouteCostDiff;
        public double targetRouteCostDiff;
        
        public Move()
        {
            originNodeIndex = -1;
            originRouteIndex = -1;
            targetRouteIndex = -1;
            targetNodeIndex = -1;
            cost = Math.Pow(10, 9);
            originRouteCostDiff = 0;
            targetRouteCostDiff = 0;
        }
    };
    class Relocation : Move
    {
        public Relocation() : base() { }

        public void storeMove(int rtInd1, int rtInd2, int nodeInd1, int nodeInd2, double cst,
                              double frcd, double srcd, double pnlty)
        {
            originRouteIndex = rtInd1;
            targetRouteIndex = rtInd2;
            originNodeIndex = nodeInd1;
            targetNodeIndex = nodeInd2;
            cost = cst;
            originRouteCostDiff = frcd;
            targetRouteCostDiff = srcd;
            penalty = pnlty;
        }
    }
    class Swap : Move
    {
        public Swap() : base() { }

        public void storeMove(int rtInd1, int rtInd2, int nodeInd1, int nodeInd2, double cst,
                              double frcd, double srcd)
        {
            originRouteIndex = rtInd1;
            targetRouteIndex = rtInd2;
            originNodeIndex = nodeInd1;
            targetNodeIndex = nodeInd2;
            cost = cst;
            originRouteCostDiff = frcd;
            targetRouteCostDiff = srcd;
        }
    }

    class Replace : Move
    {
        public Node? n;
        public int priority;

        public Replace() : base() { }

        public void storeMove(Node node, int routeIndex, int pos, double cst, int prty)
        {
            n = node;
            targetRouteIndex = routeIndex;
            targetNodeIndex = pos;
            cost = cst;
            priority = prty;
            penalty = 0.0;
        }
    }

    class TwoOptMove : Move
    {
        public int originRouteLoad;
        public int targetRouteLoad;

        public TwoOptMove() : base()
        {
            penalty = 0;
            originRouteLoad = 0;
            targetRouteLoad = 0;
        }
        public void StoreMove(int rtInd1, int rtInd2, int nodeInd1, int nodeInd2, double cst,
                                        double pen, double frcd, double srcd, int frld, int srld)
        {
            originRouteIndex = rtInd1;
            targetRouteIndex = rtInd2;
            originNodeIndex = nodeInd1;
            targetNodeIndex = nodeInd2;
            cost = cst;
            originRouteCostDiff = frcd;
            targetRouteCostDiff = srcd;
            penalty = pen;
            originRouteLoad = frld;
            targetRouteLoad = srld;
        }
    }
    class Insertion : Move
    {
        public Node? customer;
        

        public Insertion() : base()
        {
            customer = null;
            penalty = 0;
        }

        public void StoreMove(Node n, int routeIndex, int insIndex, double cst)
        {
            customer = n;
            targetRouteIndex = routeIndex;
            targetNodeIndex = insIndex;
            cost = cst;
        }
    }
}
