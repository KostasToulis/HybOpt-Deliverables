namespace CCVRPTW {

    public interface ISegment
    {
        
    }

    public abstract class Segment
    {
        public Node? nodeStart;
        public Node? nodeEnd;
        public int load;
        public double travelTime;
        public double cumulativeDistances;
        public double distances;
        public int nonDepotNodes;

        public void From(Segment s)
        {
            nodeStart = s.nodeStart;
            nodeEnd = s.nodeEnd;
            load = s.load;
            travelTime = s.travelTime;
            cumulativeDistances = s.cumulativeDistances;
            distances = s.distances;
            nonDepotNodes = s.nonDepotNodes;
        }

        public static void CreateSingleSegment(Segment seg, Node n)
        {
            seg.nodeStart = n;
            seg.nodeEnd = n;
            seg.load = n.demand;
            seg.travelTime = n.serviceTime;
            seg.cumulativeDistances = 0;
            seg.distances = 0;
            seg.nonDepotNodes = n.isDepot ? 0 : 1;

            System.Diagnostics.Debug.Assert(seg.nodeStart != null);
        }

    }

    public class FeasibleSegment: Segment
    {
        public double timeWindowPenalty;
        public double firstNodeEarliestVisit;
        public double firstNodeLatestVisit;
        
        public const double timeWindowPenaltyWeight = 10000.0;

        public FeasibleSegment()
        {

        }

        public FeasibleSegment(FeasibleSegment seg)
        {
            From(seg);
        }

        public void From(FeasibleSegment seg)
        {
            timeWindowPenalty = seg.timeWindowPenalty;
            firstNodeEarliestVisit = seg.firstNodeEarliestVisit;
            firstNodeLatestVisit = seg.firstNodeLatestVisit;
            base.From(seg);
        }

        public static FeasibleSegment SingleSegment(Node n)
        {
            FeasibleSegment seg = new FeasibleSegment();
            CreateSingleSegment(seg, n);
            return seg;
        }

        public static void CreateSingleSegment(FeasibleSegment seg, Node n)
        {
            seg.firstNodeEarliestVisit = n.windowStart;
            seg.firstNodeLatestVisit = n.windowEnd;
            seg.timeWindowPenalty = 0;
            Segment.CreateSingleSegment(seg, n);
        }

        public static FeasibleSegment MergeSegments(FeasibleSegment seg1, FeasibleSegment seg2, double dxy, double twc, int capacity)
        {
            FeasibleSegment newSeg = new FeasibleSegment();
            MergeSegments(newSeg, seg1, seg2, dxy, twc, capacity);
            return newSeg;
        }

        public static void MergeSegments(FeasibleSegment newSeg, FeasibleSegment seg1, FeasibleSegment seg2, double dxy, double twc, int capacity)
        {
            //temps
            double Delta = seg1.travelTime - seg1.timeWindowPenalty + dxy;
            double D_WT = Math.Max(0, seg2.firstNodeEarliestVisit - Delta - seg1.firstNodeLatestVisit);
            double D_TW = Math.Max(0, seg1.firstNodeEarliestVisit + Delta - seg2.firstNodeLatestVisit);

            newSeg.nodeStart = seg1.nodeStart;
            newSeg.nodeEnd = seg2.nodeEnd;
            newSeg.load = seg1.load + seg2.load;
            newSeg.travelTime = seg1.travelTime + seg2.travelTime + dxy + D_WT;
            newSeg.timeWindowPenalty = seg1.timeWindowPenalty + seg2.timeWindowPenalty + D_TW;
            newSeg.firstNodeEarliestVisit = Math.Max(seg2.firstNodeEarliestVisit - Delta, seg1.firstNodeEarliestVisit) - D_WT;
            newSeg.firstNodeLatestVisit = Math.Min(seg2.firstNodeLatestVisit - Delta, seg1.firstNodeLatestVisit) + D_TW;
            newSeg.nonDepotNodes = seg1.nonDepotNodes + seg2.nonDepotNodes;


            if (seg2.nodeStart.isDepot)
            {
                newSeg.cumulativeDistances = seg1.cumulativeDistances;
                newSeg.distances = seg1.distances;
            }
            else
            {
                newSeg.cumulativeDistances = seg1.cumulativeDistances + seg2.cumulativeDistances + seg2.nonDepotNodes * (dxy + seg1.distances);
                newSeg.distances = seg1.distances + dxy + seg2.distances;
            }
        }


    }

    public class OldFeasibleSegment : Segment
    {
        public double earliestCompletionTime;
        public double latestStartingTime;
        public bool timeWindowFeasibility;
        public double travelServiceTime;
        public bool feasibility;
        
        
        public OldFeasibleSegment() { }

        public OldFeasibleSegment(OldFeasibleSegment seg)
        {
            From(seg);
        }

        public void From(OldFeasibleSegment seg)
        {
            earliestCompletionTime = seg.earliestCompletionTime;
            latestStartingTime = seg.latestStartingTime;
            timeWindowFeasibility = seg.timeWindowFeasibility;
            feasibility = seg.feasibility;
            base.From(seg);
        }

        public static void CreateSingleSegment(OldFeasibleSegment seg, Node n)
        {
            seg.earliestCompletionTime = n.windowStart + n.serviceTime;
            seg.latestStartingTime = n.windowEnd;
            seg.timeWindowFeasibility = true;
            seg.feasibility = true;
            Segment.CreateSingleSegment(seg, n);
        }

        public static OldFeasibleSegment SingleSegment(Node n)
        {
            OldFeasibleSegment seg = new OldFeasibleSegment();
            CreateSingleSegment(seg, n);
            return seg;
        }

        // dxy = distance(seg.nodeEnd, n)
        // twc = timeWindowCompatibility[seg.nodeEnd.id, n.id]
        public static OldFeasibleSegment ExpandRightWithCustomer(OldFeasibleSegment seg, Node n, double dxy, double twc, int capacity)
        {
            OldFeasibleSegment newSeg = new OldFeasibleSegment();
            ExpandRightWithCustomer(ref newSeg, seg, n, dxy, twc, capacity);
            return newSeg;
        }

        public static void ExpandRightWithCustomer(ref OldFeasibleSegment newSeg, OldFeasibleSegment seg, Node n, double dxy, double twc, int capacity)
        {
            newSeg.nodeStart = seg.nodeStart;
            newSeg.nodeEnd = n;
            newSeg.load = seg.load + n.demand;
            newSeg.travelServiceTime = seg.travelServiceTime + dxy + n.serviceTime;
            newSeg.travelTime = seg.travelTime + dxy;
            newSeg.earliestCompletionTime = Math.Max(seg.earliestCompletionTime + dxy + n.serviceTime, n.windowStart + n.serviceTime);
            newSeg.latestStartingTime = Math.Min(seg.latestStartingTime, n.windowEnd - dxy - seg.travelServiceTime);
            newSeg.timeWindowFeasibility = (seg.timeWindowFeasibility) && (seg.earliestCompletionTime + dxy <= n.windowEnd);
            newSeg.nonDepotNodes = (n.isDepot) ? seg.nonDepotNodes : seg.nonDepotNodes + 1;
            newSeg.feasibility = (newSeg.timeWindowFeasibility) && (newSeg.load < capacity);
            newSeg.cumulativeDistances = (n.isDepot) ? seg.cumulativeDistances : seg.cumulativeDistances + seg.travelTime + dxy;
            newSeg.distances = (n.isDepot) ? seg.distances : seg.distances + dxy;
        }

        // dxy = distance(n, seg.nodeStart)
        // twc = timeWindowCompatibility[n.id, seg.nodeStart.id]
        public static Segment ExpandLeftWithCustomer(OldFeasibleSegment seg, Node n, double dxy, double twc, int capacity)
        {
            OldFeasibleSegment newSeg = new OldFeasibleSegment();
            newSeg.nodeStart = n;
            newSeg.nodeEnd = seg.nodeEnd;
            newSeg.load = seg.load + n.demand;
            newSeg.travelServiceTime = n.serviceTime + dxy + seg.travelServiceTime;
            newSeg.travelTime = dxy + seg.travelTime;
            newSeg.earliestCompletionTime = Math.Max(n.windowStart + n.serviceTime + dxy + seg.travelServiceTime, seg.earliestCompletionTime);
            newSeg.latestStartingTime = Math.Min(n.windowEnd, seg.latestStartingTime - dxy - n.serviceTime);
            newSeg.timeWindowFeasibility = (seg.timeWindowFeasibility) && (n.windowStart + n.serviceTime + dxy <= seg.latestStartingTime);
            newSeg.nonDepotNodes = (n.isDepot) ? seg.nonDepotNodes : seg.nonDepotNodes + 1;
            newSeg.feasibility = (newSeg.timeWindowFeasibility) && (newSeg.load <= capacity);
            return newSeg;
        }

        // dxy = distance(seg1.nodeEnd, seg2.nodeStart)
        // twc = timeWindowCompatibility[seg1.nodeEnd.id, seg2.nodeStart.id]
        public static OldFeasibleSegment MergeSegments(OldFeasibleSegment seg1, OldFeasibleSegment seg2, double dxy, double twc, int capacity)
        {
            OldFeasibleSegment newSeg = new OldFeasibleSegment();
            MergeSegments(ref newSeg, seg1, seg2, dxy, twc, capacity);
            return newSeg;
        }

        public static void MergeSegments(ref OldFeasibleSegment newSeg, OldFeasibleSegment seg1, OldFeasibleSegment seg2, double dxy, double twc, int capacity)
        {
            //Assignments reordered so that no issues occur when seg1 == newSeg or seg2 == newSeg
            //TODO: CHECK THAT THIS SHIT IS CORRECT
            newSeg.nodeStart = seg1.nodeStart;
            newSeg.nodeEnd = seg2.nodeEnd;
            newSeg.load = seg1.load + seg2.load;
            newSeg.latestStartingTime = Math.Min(seg1.latestStartingTime, seg2.latestStartingTime - (dxy + seg1.travelServiceTime));
            newSeg.earliestCompletionTime = Math.Max(seg2.earliestCompletionTime, seg1.earliestCompletionTime + dxy + seg2.travelServiceTime);
            newSeg.timeWindowFeasibility = (seg1.timeWindowFeasibility) && (seg2.timeWindowFeasibility) && (seg1.earliestCompletionTime + dxy <= seg2.latestStartingTime);
            newSeg.travelServiceTime = seg1.travelServiceTime + dxy + seg2.travelServiceTime;
            newSeg.travelTime = seg1.travelTime + dxy + seg2.travelTime;
            newSeg.feasibility = (newSeg.timeWindowFeasibility) && (newSeg.load <= capacity);
            newSeg.nonDepotNodes = seg1.nonDepotNodes + seg2.nonDepotNodes;

            if (seg2.nodeStart.isDepot)
            {
                newSeg.cumulativeDistances = seg1.cumulativeDistances;
                newSeg.distances = seg1.distances;
            }
            else
            {
                newSeg.cumulativeDistances = seg1.cumulativeDistances + seg2.cumulativeDistances + seg2.nonDepotNodes * (dxy + seg1.travelTime);
                newSeg.distances = seg1.distances + dxy + seg2.distances;
            }
        }
    }
}
