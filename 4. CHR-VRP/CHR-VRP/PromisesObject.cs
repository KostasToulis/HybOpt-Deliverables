namespace CHRVRP;

public class PromisesObject
{
    public double[,] matrix;
    public List<(int, int)> pairs;

    public PromisesObject(int size)
    {
        matrix = new double[size, size];
        pairs = new List<(int, int)>();
        Reset();  // internal call to initialize the matrix to default values
    }

    /**
     * This method initializes all the matrix's values to infinity (in this case a very large arbitrary number, 10 to
     * the power of 9).
     */
    public void Reset()
    {
        //Console.WriteLine("Resetting Promises Matrix\n");
        var size = matrix.GetLength(0);
        for (var i = 0; i < size; i++)
        {
            for (var j = 0; j < size; j++)
            {
                matrix[i, j] = Math.Pow(10, 9);
            }
        }
    }

    /**
     * Checks whether the move violates a promise between arcs involved. It delegates the functionality to specific
     * methods, tailored to each move.
     */
    public bool CheckIfBroken(Solution solution, Move move)
    {
        pairs.Clear();
        if (move.GetType() == typeof(Swap))
        {
            return SwapPromisesBroken(solution, move);
        }
        
        if (move.GetType() == typeof(Relocation))
        {
            return RelocationPromisesBroken(solution, move);
        }
        
        if (move.GetType() == typeof(TwoOpt))
        {
            return TwoOptPromisesBroken(solution, move);
        }

        return true;  // default value
    }

    private bool TwoOptPromisesBroken(Solution solution, Move move)
    {
        if (move.originRouteIndex == -1)
        {
            return false;
        }
        // a list of tuples containing all the serial number pairs
        var newSolutionCost = move.cost;
        var route1 = solution.routes[move.originRouteIndex];
        var route2 = solution.routes[move.targetRouteIndex];

        // Get all necessary nodes for Swap move and add pairs to the list
        var nodeB = route1.sequence[move.originNodeIndex];
        var nodeE = route2.sequence[move.targetNodeIndex];

        if (route1 == route2)
        {
            pairs.Add((nodeB.serialNumber, nodeE.serialNumber));
            if (move.targetNodeIndex + 1 < route2.sequence.Count && move.originNodeIndex + 1 < route1.sequence.Count)
            {
                var nodeF = route2.sequence[move.targetNodeIndex + 1];
                var nodeC = route1.sequence[move.originNodeIndex + 1];
                pairs.Add((nodeF.serialNumber, nodeC.serialNumber));
            }
            return CheckPairs(newSolutionCost);
        }
        
        var nodeA = route1.sequence[move.originNodeIndex - 1];
        pairs.Add((nodeA.serialNumber, nodeB.serialNumber));
            
        var nodeD = route2.sequence[move.targetNodeIndex - 1];
        pairs.Add((nodeD.serialNumber, nodeE.serialNumber));
            
        if (move.targetNodeIndex + 1 < route2.sequence.Count)
        {
            var nodeF = route2.sequence[move.targetNodeIndex + 1];
            pairs.Add((nodeB.serialNumber, nodeF.serialNumber));
        }
        
        if (move.originNodeIndex + 1 < route1.sequence.Count)
        {
            var nodeC = route1.sequence[move.originNodeIndex + 1];
            pairs.Add((nodeE.serialNumber, nodeC.serialNumber));
        }

        return CheckPairs(newSolutionCost);    
    }


    /// Checks for broken promises for Relocation move.
    private bool RelocationPromisesBroken(Solution solution, Move move)
    {

        if (move.originRouteIndex == -1)
        {
            return false;
        }
        // a list of tuples containing all the serial number pairs
        var newSolutionCost = move.cost;
        var route1 = solution.routes[move.originRouteIndex];
        var route2 = solution.routes[move.targetRouteIndex];

        // Get all necessary nodes for Swap move and add pairs to the list
        var nodeA = route1.sequence[move.originNodeIndex - 1];
        var nodeB = route1.sequence[move.originNodeIndex];
        var nodeD = route2.sequence[move.targetNodeIndex - 1];

        pairs.Add((nodeD.serialNumber, nodeB.serialNumber));

        if (move.targetNodeIndex < route2.sequence.Count)
        {
            var nodeE = route2.sequence[move.targetNodeIndex];
            pairs.Add((nodeB.serialNumber, nodeE.serialNumber));
        }
        
        if (move.originNodeIndex + 1 < route1.sequence.Count)
        {
            var nodeC = route1.sequence[move.originNodeIndex + 1];
            pairs.Add((nodeA.serialNumber, nodeC.serialNumber));
        }

        return CheckPairs(newSolutionCost);
    }

    /// Checks for broken promises for Swap move.
    private bool SwapPromisesBroken(Solution solution, Move move)
    {
        if (move.originRouteIndex == -1)
        {
            return false;
        }
        // a list of tuples containing all the serial number pairs
        var newSolutionCost = move.cost;
        var route1 = solution.routes[move.originRouteIndex];
        var route2 = solution.routes[move.targetRouteIndex];

        // Get all necessary nodes for Swap move and add pairs to the list
        var nodeA = route1.sequence[move.originNodeIndex - 1];
        var nodeB = route1.sequence[move.originNodeIndex];
        var nodeD = route2.sequence[move.targetNodeIndex - 1];
        var nodeE = route2.sequence[move.targetNodeIndex];
        
        pairs.Add((nodeD.serialNumber, nodeB.serialNumber));
        
        if (move.targetNodeIndex + 1 < route2.sequence.Count)
        {
            var nodeF = route2.sequence[move.targetNodeIndex + 1];
            pairs.Add((nodeB.serialNumber, nodeF.serialNumber));
        }
        
        pairs.Add((nodeA.serialNumber, nodeE.serialNumber));
        
        if (move.originNodeIndex + 1 < route1.sequence.Count)
        {
            var nodeC = route1.sequence[move.originNodeIndex + 1];
            pairs.Add((nodeE.serialNumber, nodeC.serialNumber));
        }

        return CheckPairs(newSolutionCost);
    }

    private bool CheckPairs(double newSolutionCost)
    {
        // Iterate through the pairs and see if there is a broken promise
        foreach (var pair in pairs)
        {
            var currentPromise = matrix[pair.Item1, pair.Item2];
            if (currentPromise <= newSolutionCost)
            {
                // at least one pair breaks a promise
                return true;
            }
        }

        return false;  // if all pairs don't break promise, then return false
    }

    public void Update(Solution solution, Move move) 
    {
        pairs.Clear();
        if (move.GetType() == typeof(Swap))
        {
            SwapPromisesUpdate(solution, move);
        }

        if (move.GetType() == typeof(Relocation))
        {
            RelocationPromisesUpdate(solution, move);
        }
        
        if (move.GetType() == typeof(TwoOpt))
        {
            TwoOptPromisesUpdate(solution, move);
        }
    }

    private void TwoOptPromisesUpdate(Solution solution, Move move)
    {
        // a list of tuples containing all the serial number pairs
        var newSolutionCost = move.cost;
        var route1 = solution.routes[move.originRouteIndex];
        var route2 = solution.routes[move.targetRouteIndex];

        // Get all necessary nodes for Swap move and add pairs to the list
        var nodeB = route1.sequence[move.originNodeIndex];
        var nodeE = route2.sequence[move.targetNodeIndex];

        if (route1.id == route2.id)
        {
            pairs.Add((nodeB.serialNumber, nodeE.serialNumber));
            if (move.targetNodeIndex + 1 < route2.sequence.Count && move.originNodeIndex + 1 < route1.sequence.Count)
            {
                var nodeF = route2.sequence[move.targetNodeIndex + 1];
                var nodeC = route1.sequence[move.originNodeIndex + 1];
                pairs.Add((nodeF.serialNumber, nodeC.serialNumber));
            }
        }
        else
        {
            var nodeA = route1.sequence[move.originNodeIndex - 1];
            pairs.Add((nodeA.serialNumber, nodeB.serialNumber));
            
            var nodeD = route2.sequence[move.targetNodeIndex - 1];
            pairs.Add((nodeD.serialNumber, nodeE.serialNumber));
            
            if (move.targetNodeIndex + 1 < route2.sequence.Count)
            {
                var nodeF = route2.sequence[move.targetNodeIndex + 1];
                pairs.Add((nodeB.serialNumber, nodeF.serialNumber));
            }
        
            if (move.originNodeIndex + 1 < route1.sequence.Count)
            {
                var nodeC = route1.sequence[move.originNodeIndex + 1];
                pairs.Add((nodeE.serialNumber, nodeC.serialNumber));
            }
        }

        UpdatePairs(newSolutionCost);
    }

    private void SwapPromisesUpdate(Solution solution, Move move)
    {
        // a list of tuples containing all the serial number pairs
        var newSolutionCost = move.cost;
        var route1 = solution.routes[move.originRouteIndex];
        var route2 = solution.routes[move.targetRouteIndex];

        // Get all necessary nodes for Swap move and add pairs to the list
        var nodeA = route1.sequence[move.originNodeIndex - 1];
        var nodeB = route1.sequence[move.originNodeIndex];
        var nodeD = route2.sequence[move.targetNodeIndex - 1];
        var nodeE = route2.sequence[move.targetNodeIndex];

        pairs.Add((nodeD.serialNumber, nodeE.serialNumber));

        if (move.targetNodeIndex + 1 < route2.sequence.Count)
        {
            var nodeF = route2.sequence[move.targetNodeIndex + 1];
            pairs.Add((nodeE.serialNumber, nodeF.serialNumber));
        }

        pairs.Add((nodeA.serialNumber, nodeB.serialNumber));

        if (move.originNodeIndex + 1 < route1.sequence.Count)
        {
            var nodeC = route1.sequence[move.originNodeIndex + 1];
            pairs.Add((nodeB.serialNumber, nodeC.serialNumber));
        }

        UpdatePairs(newSolutionCost);
    }

    private void RelocationPromisesUpdate(Solution solution, Move move)
    {
        var newSolutionCost = move.cost;
        var route1 = solution.routes[move.originRouteIndex];
        var route2 = solution.routes[move.targetRouteIndex];

        // Get all necessary nodes for Swap move and add pairs to the list
        var nodeA = route1.sequence[move.originNodeIndex - 1];
        var nodeB = route1.sequence[move.originNodeIndex];
        var nodeD = route2.sequence[move.targetNodeIndex - 1];

        pairs.Add((nodeA.serialNumber, nodeB.serialNumber));

        if (move.targetNodeIndex < route2.sequence.Count)
        {
            var nodeE = route2.sequence[move.targetNodeIndex];
            pairs.Add((nodeD.serialNumber, nodeE.serialNumber));
        }
        
        if (move.originNodeIndex + 1 < route1.sequence.Count)
        {
            var nodeC = route1.sequence[move.originNodeIndex + 1];
            pairs.Add((nodeB.serialNumber, nodeC.serialNumber));
        }

        UpdatePairs(newSolutionCost);
    }

    private void UpdatePairs(double newSolutionCost)
    {
        // Iterate through the pairs and see if there is a broken promise
        foreach (var pair in pairs)
        {
            //Console.WriteLine($"Before: Matrix[{pair.Item1}, {pair.Item2}] = {matrix[pair.Item1, pair.Item2]}");
            matrix[pair.Item1, pair.Item2] = newSolutionCost;
            matrix[pair.Item2, pair.Item1] = newSolutionCost;
            //Console.WriteLine($"After: Matrix[{pair.Item1}, {pair.Item2}] = {matrix[pair.Item1, pair.Item2]}");
            // Console.WriteLine($"New promise = {matrix[pair.Item1, pair.Item2]}");
        }
    }
}