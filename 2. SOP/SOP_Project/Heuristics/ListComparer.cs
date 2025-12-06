using System;
using System.Collections.Generic;
using System.Text;

namespace SOP_Project
{
    class ListComparer : IEqualityComparer<List<int>>  // implementation for hashing and comparing List<int> type keys
                                                       // in dictionaries
    {
        public bool Equals(List<int> x, List<int> y)
        {
            return Pattern_To_String(x).Equals(Pattern_To_String(y));
        }

        public int GetHashCode(List<int> list)  // create new hashing according to the list elements
        {
            int hashcode = 0;
            foreach (int n in list)
            {
                hashcode ^= n;  // run the binary XOR operation between all list elements
            }
            return hashcode.GetHashCode();  // hash the final value
        }

        public string Pattern_To_String(List<int> pattern)
        {
            StringBuilder pattern_to_string = new StringBuilder(pattern[0]);
            for (int i = 1; i < pattern.Count; i++)
            {
                pattern_to_string.Append("," + pattern[i]);
            }
            return pattern_to_string.ToString();
        }

        public static bool OverlapsPattern(List<List<int>> patterns_in_list, List<int> pattern)  
            // checks whether the patterns already contained in a list overlap the pattern under examination.
            // This method makes use of syntactic matching methods to quickly find if the pattern is overlapped by any other. 
        {
            bool is_overlapped = false;
            foreach (List<int> pattern_in_list in patterns_in_list)
            {
                if (SyntacticMatching.RunBoyerMooreAlgorithm(pattern_in_list, pattern) >= 1)
                {
                    is_overlapped = true;
                    break;
                }
            }
            return is_overlapped;
        }
    }
}
