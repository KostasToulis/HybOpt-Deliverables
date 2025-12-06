using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PRP
{
    public static class GlobalUtils
    {
        public static readonly double doublePrecision = 1e-6;
        public static bool suppress_output = false;

        public static bool IsEqual(double a, double b)
        {
            return IsEqual(a, b, doublePrecision);
        }

        public static bool IsEqual(double a, double b, double prec)
        {
            return Math.Abs(a - b) > prec ? false : true;
        }

        public static int Min(int a, int b, int c)
        {
            return Math.Min(Math.Min(a, b), c);
        }
        
        public static int Min(int a, int b, int c, int d)
        {
            return Math.Min(Min(a,b,c) ,d);
        }
        
        public static List<String> SeperateStringIntoSubstrings(char[] seperators,  string str)
        {
            string[] array = str.Split(seperators, 1000);
            List<String> data = new List<String>();
            for (int j = 0; j < array.Length; j++)
            {
                if (array[j] != "")
                    data.Add(array[j]);
            }
            return data;
        }
        
        public static List<String> SeperateStringIntoSubstrings(char seperator,  string str)
        {
            string[] array = str.Split(seperator, 100);
            List<String> data = new List<String>();
            for (int j = 0; j < array.Length; j++)
            {
                if (array[j] != "")
                    data.Add(array[j]);
            }
            return data;
        }

        public static void writeToConsole(object ob)
        {
            if (!suppress_output)
                Console.WriteLine(ob);
        }

        public static void writeToConsole(string text, params object[] objects)
        {
            if (!suppress_output)
                Console.WriteLine(text, objects);
        }

    }
}
