using System;
using System.Collections.Generic;
using System.Text;

namespace SOP_Project
{
    class SyntacticMatching
    {
        public static List<int> pool_as_set_seq;  // list containing all set ids in pool solutions in order of appearance
        public static int NO_OF_CHARS;  // size of the alphabet - declared in method Pool.Initialize()

        public static void PoolToSetSeq(List<Solution> sol_pool)  // create an integer array representation of the sol pool
        {
            pool_as_set_seq = new List<int>();
            foreach (Solution sol in sol_pool)
            {
                foreach (Set set in sol.route.sets_included)
                {
                    pool_as_set_seq.Add(set.id);
                }
            }

        }

        public static int GetMatching(List<int> pattern)  // implementing Boyer-Moore's algorithm for syntactic matching
                                                          // https://www.geeksforgeeks.org/boyer-moore-algorithm-for-pattern-searching/
                                                          // returns the number of times the given set_chain has been found in the pool
        {
            return RunBoyerMooreAlgorithm(pool_as_set_seq, pattern);
        }

        public static int RunBoyerMooreAlgorithm(List<int> text, List<int> pattern)
        {
            int m = pattern.Count;
            int n = text.Count;
            int[] badchar = new int[NO_OF_CHARS];

            if (m > n) { return 0; }

            // Initialize all occurrences as -1
            for (int i = 0; i < NO_OF_CHARS; i++)
                badchar[i] = -1;

            // Fill the actual value of last occurrence of a character
            for (int i = 0; i < m; i++)
                badchar[(int)pattern[i]] = i;


            int s = 0; // s is shift of the pattern with respect to text
            List<int> shifts = new List<int>();
            int occurences = 0;
            int last_occurence;
            while (s <= n - m)
            {
                int j = m - 1;

                while (j >= 0 && pattern[j] == text[s + j])
                    j--;

                if (j < 0)
                {
                    //Console.Write("Pattern occurs at shift " + s + " ");
                    //foreach (int i in pattern)
                    //{
                    //    Console.Write(i + "-");
                    //}
                    //Console.WriteLine();
                    shifts.Insert(0, s);
                    occurences++;

                    s += (s + m < n) ? m - badchar[text[s + m]] : 1;
                }
                else
                {
                    last_occurence = j - badchar[text[s + j]];
                    s += 1 > last_occurence ? 1 : last_occurence;
                }
            }

            // remove the pattern from the text to avoid unnecessary pattern matchings in next iterations
            //if (occurences >= 2)
            //{
            //    foreach (int shift in shifts)
            //    {
            //        RemovePattern(shift, m);
            //    }
            //}

            return occurences;
        }

        static void RemovePattern(int found_at, int pattern_length)  // remove the pattern from the sequence when found
                                                                     // to accelerate the search for next iterations
        {
            for (int i = found_at + pattern_length - 1; i >= found_at; i--)
            {
                pool_as_set_seq.RemoveAt(i);
            }
        }
    }
}
