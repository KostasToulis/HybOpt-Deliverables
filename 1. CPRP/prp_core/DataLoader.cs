using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Linq.Expressions;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices.ComTypes;
using System.Text;
using System.Threading.Tasks;
using Gurobi;


namespace PRP
{
    public enum PRP_DATASET_VERSION
    {
        ADULYASAK_FMT = 0,
        ARCHETTI_FMT,
        BOUDIA_FMT
    };
    
    class DataLoader
    {
        public static List<Dictionary<string, int>> adul_instaces  = new List<Dictionary<string, int>> { new Dictionary<string, int> { 
 	{ "customers", 10 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 10 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 10 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 10 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 10 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 10 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 10 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 10 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 15 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 15 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 15 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 15 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 15 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 15 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 15 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 15 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 20 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 20 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 20 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 20 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 20 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 20 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 20 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 20 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 25 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 25 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 25 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 25 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 25 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 25 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 25 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 25 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 30 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 30 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 30 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 30 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 30 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 4 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 30 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 4 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 30 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 4 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 30 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 4 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 35 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 35 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 35 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 35 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 35 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 4 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 35 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 4 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 35 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 4 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 35 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 4 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 40 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 40 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 40 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 40 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 40 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 4 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 40 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 4 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 40 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 4 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 40 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 4 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 45 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 45 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 45 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 45 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 45 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 4 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 45 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 4 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 45 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 4 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 45 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 4 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 50 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 50 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 50 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 50 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 50 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 4 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 50 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 4 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 50 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 4 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 50 }, 
 	{ "periods", 3 }, 
 	{ "vehicles", 4 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 10 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 10 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 10 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 10 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 10 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 10 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 10 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 10 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 15 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 15 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 15 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 15 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 15 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 15 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 15 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 15 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 20 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 20 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 20 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 20 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 20 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 20 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 20 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 20 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 25 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 25 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 25 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 25 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 25 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 25 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 25 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 25 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 30 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 30 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 30 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 30 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 30 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 4 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 30 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 4 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 30 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 4 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 30 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 4 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 35 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 35 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 35 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 35 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 35 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 4 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 35 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 4 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 35 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 4 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 35 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 4 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 40 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 40 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 40 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 40 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 40 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 4 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 40 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 4 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 40 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 4 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 40 }, 
 	{ "periods", 6 }, 
 	{ "vehicles", 4 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 10 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 10 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 10 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 10 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 10 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 10 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 10 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 10 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 15 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 15 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 15 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 15 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 15 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 15 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 15 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 15 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 20 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 20 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 20 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 20 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 20 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 20 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 20 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 20 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 25 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 25 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 25 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 25 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 2 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 25 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 25 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 25 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 25 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 30 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 30 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 30 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 30 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 3 }, 
 	{ "index", 4 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 30 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 4 }, 
 	{ "index", 1 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 30 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 4 }, 
 	{ "index", 2 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 30 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 4 }, 
 	{ "index", 3 } }, 
 new Dictionary<string, int> { 
 	{ "customers", 30 }, 
 	{ "periods", 9 }, 
 	{ "vehicles", 4 }, 
 	{ "index", 4 } }, 
};
        
        
        public static DataInput LoadingInstanceOpener(string problemName, PRP_DATASET_VERSION version)
        {
            switch (version)
            {
	            case PRP_DATASET_VERSION.ADULYASAK_FMT:
					return parseAdulyasakInstance(problemName);
				case PRP_DATASET_VERSION.ARCHETTI_FMT:
					return parseArchettiInstance(problemName);
	            case PRP_DATASET_VERSION.BOUDIA_FMT:
			        return parseBoudiaInstance(problemName);
				default:
                {
                    GlobalUtils.writeToConsole("Data parser not implemented yet");
                    return null;
                }
			}
        }

        private static DataInput parseAdulyasakInstance(string problemName)
        {
            DataInput dataInput = new DataInput();
            dataInput.type = PRP_DATASET_VERSION.ADULYASAK_FMT;
            FileInfo src = new FileInfo(problemName);
            TextReader reader = src.OpenText();
            String str;
            char[] seperator = new char[2] { ' ', '\t' };
            List<String> data;
            //Skip first 8 lines
            for (int i=0;i<8;i++)
                reader.ReadLine();
            
            // (Instance Info)
            str = reader.ReadLine();
            data = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);
            int totalNodes = int.Parse(data.Last());
            dataInput.customerNum = totalNodes - 1;
            str = reader.ReadLine();
            data = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);
            dataInput.horizonDays = int.Parse(data.Last());
            str = reader.ReadLine();
            data = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);
            dataInput.availableVehicles = int.Parse(data.Last());
            str = reader.ReadLine();
            data = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);
            dataInput.dayVehicleCapacity = int.Parse(data.Last());
            dataInput.distanceCoeff = 1.0;
            
				
            //Parse Nodes
            
            //Depot first
            str = reader.ReadLine(); //Header
            str = reader.ReadLine();
            data = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);
            Depot depot = new Depot(dataInput.horizonDays);
            depot.uid = int.Parse(data[0]);
            depot.ID = (depot.uid+1).ToString();
            depot.x_coord = double.Parse(data[1]);
            depot.y_coord = double.Parse(data[2]);
            depot.startingInventory = int.Parse(data[3]);
            depot.unitHoldingCost = double.Parse(data[4]);
            depot.unitProductionCost = double.Parse(data[5]);
            depot.productionSetupCost = double.Parse(data[6]);
            depot.productionCapacity = (int) double.Parse(data[7]);
            depot.stockMaximumLevel = (int) double.Parse(data[8]);
            
            dataInput.nodes.Add(depot);

            //Customers/Retailers
            reader.ReadLine(); //Header
            dataInput.customers_zero_inv_cost = true;
            for (int i = 0; i < totalNodes - 1; i++ )
            {
                str = reader.ReadLine();
                data = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);

                Node cust = new Node(dataInput.horizonDays);

                cust.uid = i + 1;
                cust.ID = (cust.uid + 1).ToString();
                cust.x_coord = double.Parse((string)data[1]);
                cust.y_coord = double.Parse((string)data[2]);
                cust.startingInventory = (int) double.Parse(data[3]);
                cust.stockMaximumLevel = (int) double.Parse(data[4]);
                cust.stockMinimumLevel = (int) double.Parse(data[5]);
                int productRate = (int) double.Parse(data[6]);
                cust.productRate = new int[dataInput.horizonDays];
                cust.productRateSigned = new int[dataInput.horizonDays];
                for (int k = 0; k < dataInput.horizonDays; k++)
                {
	                cust.productRate[k] = productRate;
	                cust.productRateSigned[k] = -productRate;
                    depot.totalDemand += productRate;
                }
                cust.unitHoldingCost = double.Parse(data[7]);
                if (cust.unitHoldingCost > 0)
                    dataInput.customers_zero_inv_cost = false;
                cust.CalculateTotalDemand(); //Add up product rates to get the total demand of the customer
                dataInput.nodes.Add(cust);
            }


            reader.Close();

            return dataInput;
        }
        
        private static DataInput parseArchettiInstance(string problemName)
        {
            DataInput dataInput = new DataInput();
            dataInput.type = PRP_DATASET_VERSION.ARCHETTI_FMT;
            FileInfo src = new FileInfo(problemName);
            TextReader reader = src.OpenText();
            String str;
            char[] seperator = new char[2] { ' ', '\t' };
            List<String> data;
            //Skip first 8 lines
            for (int i=0;i<8;i++)
                reader.ReadLine();
            
            // (Instance Info)
            str = reader.ReadLine();
            data = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);
            int totalNodes = int.Parse(data.Last());
            dataInput.customerNum = totalNodes - 1;
            str = reader.ReadLine();
            data = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);
            dataInput.horizonDays = int.Parse(data.Last());
            str = reader.ReadLine();
            data = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);
            dataInput.dayVehicleCapacity = int.Parse(data.Last());
            if (dataInput.customerNum == 14)
				dataInput.availableVehicles = 1; 
            else
	            dataInput.availableVehicles = dataInput.customerNum;
            str = reader.ReadLine(); //Skip fixed transportation cost?!!!
			dataInput.distanceCoeff = 1.0;
            
			//Parse Nodes
            
            //Depot first
            str = reader.ReadLine(); //Header
            str = reader.ReadLine();
            data = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);
            Depot depot = new Depot(dataInput.horizonDays);
            depot.uid = int.Parse(data[0]);
            depot.ID = (depot.uid+1).ToString();
            depot.x_coord = double.Parse(data[1]);
            depot.y_coord = double.Parse(data[2]);
            depot.startingInventory = int.Parse(data[3]);
            depot.unitHoldingCost = double.Parse(data[4]);
            depot.unitProductionCost = double.Parse(data[5]);
            depot.productionSetupCost = double.Parse(data[6]);
            depot.productionCapacity = int.MaxValue;
            depot.stockMaximumLevel = int.MaxValue;
            
            dataInput.nodes.Add(depot);
            dataInput.customers_zero_inv_cost = true;

            //Customers/Retailers
            reader.ReadLine(); //Header
            for (int i = 0; i < totalNodes - 1; i++ )
            {
                str = reader.ReadLine();
                data = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);

                Node cust = new Node(dataInput.horizonDays);

                cust.uid = i + 1;
                cust.ID = (cust.uid + 1).ToString();
                cust.x_coord = double.Parse((string)data[1]);
                cust.y_coord = double.Parse((string)data[2]);
                cust.startingInventory = (int) double.Parse(data[3]);
                cust.stockMaximumLevel = (int) double.Parse(data[4]);
                cust.stockMinimumLevel = (int) double.Parse(data[5]);
                int productRate = (int) double.Parse(data[6]);
                cust.productRate = new int[dataInput.horizonDays];
                cust.productRateSigned = new int[dataInput.horizonDays];
                for (int k = 0; k < dataInput.horizonDays; k++)
                {
	                cust.productRate[k] = productRate;
	                cust.productRateSigned[k] = -productRate;
                    depot.totalDemand += productRate;
                }

                cust.unitHoldingCost = double.Parse(data[7]);
                if (cust.unitHoldingCost > 0)
                    dataInput.customers_zero_inv_cost = false;
                cust.CalculateTotalDemand(); //Add up product rates to get the total demand of the customer
                dataInput.nodes.Add(cust);

            }
            reader.Close();

            return dataInput;
        }
        
        private static DataInput parseBoudiaInstance(string problemName)
        {
            DataInput dataInput = new DataInput();
            dataInput.type = PRP_DATASET_VERSION.BOUDIA_FMT;
            FileInfo src;
            TextReader reader;
            String str;
            char[] seperator = new char[2] { ' ', '\t' };
            List<String> data;

            //Load parameters file
            if (!File.Exists(Path.Join(problemName, "parameters.txt")))
                throw new Exception("Missing Parameters file. Incorrect path maybe?");
            src = new FileInfo(Path.Join(problemName, "parameters.txt"));
            reader = src.OpenText();

            int productionCapacity,
	            storageCapacity,
	            setup_cost,
	            holding_cost,
	            vehNum,
	            vehCap,
	            custCap1_min,
	            custCap1_max,
	            custCap1,
	            custCap2_min,
	            custCap2_max,
	            custCap2,
	            custCap3_min,
	            custCap3_max,
	            custCap3;
            
            str = reader.ReadLine();
            data = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);
            productionCapacity = int.Parse(data.Last());
            str = reader.ReadLine();
            data = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);
            storageCapacity = int.Parse(data.Last());
            str = reader.ReadLine(); //SKip
            str = reader.ReadLine();
            data = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);
            setup_cost = int.Parse(data.Last());
            str = reader.ReadLine();
            data = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);
            holding_cost = int.Parse(data.Last());
            str = reader.ReadLine();//SKip
            str = reader.ReadLine();
            data = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);
            vehCap = int.Parse(data.Last());
            str = reader.ReadLine();
            data = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);
            vehNum = int.Parse(data.Last());
            str = reader.ReadLine();//SKip
            str = reader.ReadLine();//SKip
            str = reader.ReadLine();
            data = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);
            custCap1_min = int.Parse(data[1]); 
            custCap1_max = int.Parse(data[3]);
            custCap1 = int.Parse(data.Last());
            str = reader.ReadLine();
            data = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);
            custCap2_min = int.Parse(data[1]); 
            custCap2_max = int.Parse(data[3]);
            custCap2 = int.Parse(data.Last());
            str = reader.ReadLine();
            data = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);
            custCap3_min = int.Parse(data[1]); 
            custCap3_max = int.Parse(data[3]);
            custCap3 = int.Parse(data.Last());
			reader.Close();
            
			//Apply info
			dataInput.availableVehicles = vehNum;
			dataInput.dayVehicleCapacity = vehCap;
			

			//Load instances file
            src = new FileInfo(Path.Join(problemName, "instances.txt"));
            reader = src.OpenText();
            

            int totalCustomerNum;
            int totalNodes;
            int totalPeriodNum;
            int totalCostPerKm;
			

			str = reader.ReadLine();
			data = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);
            totalCustomerNum = int.Parse(data.Last());
            totalNodes = totalCustomerNum + 1;
            str = reader.ReadLine(); //Skip line
            str = reader.ReadLine(); 
            data = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);
            totalPeriodNum = int.Parse(data.Last());
            str = reader.ReadLine(); //Skip line
            str = reader.ReadLine();
            data = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);
            totalCostPerKm = int.Parse(data.Last());
            str = reader.ReadLine(); //Skip line
            
			
            //Set extra info
            dataInput.horizonDays = totalPeriodNum;
            dataInput.customerNum = totalCustomerNum;
            dataInput.distanceCoeff = totalCostPerKm;
            
            
            
            //Add depot
            Depot depot = new Depot(dataInput.horizonDays);
            depot.uid = 0;
            depot.ID = (depot.uid+1).ToString();
            depot.x_coord = 0.0;
            depot.y_coord = 0.0;
            depot.startingInventory = 0;
            depot.unitHoldingCost = holding_cost;
            depot.unitProductionCost = 0.0;
            depot.productionSetupCost = setup_cost;
            depot.productionCapacity = productionCapacity;
            depot.stockMaximumLevel = storageCapacity;
            
            dataInput.nodes.Add(depot);
            dataInput.customers_zero_inv_cost = true;

            //Parse Customer info
            for (int i = 0; i < totalCustomerNum; i++)
            {
	            str = reader.ReadLine();
	            data = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);
	            
	            Node cust = new Node(dataInput.horizonDays);

	            cust.uid = i + 1;
	            cust.ID = (cust.uid + 1).ToString();
	            cust.x_coord = double.Parse((string)data[2]);
	            cust.y_coord = double.Parse((string)data[3]);
	            cust.startingInventory = 0;
				cust.stockMinimumLevel = 0;
	            if (cust.uid >= custCap1_min && cust.uid <= custCap1_max)
		            cust.stockMaximumLevel = custCap1;
	            else if (cust.uid >= custCap2_min && cust.uid <= custCap2_max)
		            cust.stockMaximumLevel = custCap2;
	            else
		            cust.stockMaximumLevel = custCap3;

	            cust.productRate = new int[dataInput.horizonDays];
	            cust.productRateSigned = new int[dataInput.horizonDays];
	            for (int k = 0; k < dataInput.horizonDays; k++)
	            {
		            cust.productRate[k] = int.Parse((string) data[4 + k]);
		            cust.productRateSigned[k] = -cust.productRate[k];
                    depot.totalDemand += cust.productRate[k];
                }

                cust.unitHoldingCost = 0;
                if (cust.unitHoldingCost > 0)
                    dataInput.customers_zero_inv_cost = false;
                cust.CalculateTotalDemand(); //Add up product rates to get the total demand of the customer
				dataInput.nodes.Add(cust);
            }
			reader.Close();

			
			//Fix depot start inventory
			for (int i = 0; i < totalCustomerNum; i++)
				depot.startingInventory += dataInput.nodes[i + 1].productRate[0];
			
            return dataInput;
        }


        public static ProductionDataInput parseProductionSchedule(string problemName, int customerNum, string id, int vehicleNum, bool parse_routing)
        {
            ProductionDataInput dataInput = new ProductionDataInput();
			dataInput.ID = id;
			FileInfo src = new FileInfo(problemName);
            TextReader reader = src.OpenText();
            String str;
            char[] seperator = new char[2] { ' ', '\t' };
			List<String> data;
            //Fetch relaxed objective
            str = reader.ReadLine();
            data = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);
            dataInput.relaxedObjective = double.Parse(data.Last());
			
			//Skip first 6 lines
            for (int i=0; i<5; i++)
                reader.ReadLine();
            
            str = reader.ReadLine();
            data = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);
            dataInput.horizonDays = int.Parse(data.Last());
            
			//Initialize arrays
            dataInput.customerDeliveries = new int[dataInput.horizonDays, customerNum + 1];
            dataInput.customerRouteAssignment = new int[dataInput.horizonDays, customerNum + 1];
            dataInput.plantOpen = new bool[dataInput.horizonDays];
            dataInput.productionQuantities = new int[dataInput.horizonDays];
            
			//Skip first 4 lines
            for (int i=0;i<4;i++)
                reader.ReadLine();

            //Parse Production
            for (int i = 0; i < dataInput.horizonDays; i++)
            {
                reader.ReadLine(); //Skip first line
                str = reader.ReadLine();
                data = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);
                str = reader.ReadLine(); //Skip last empty line
                int q = (int) Math.Round(double.Parse(data.Last()));
                if (q > 0.0)
                    dataInput.plantOpen[i] = true;
                dataInput.productionQuantities[i] = q;
            }
            
            reader.ReadLine();
            //Parse Visits
            for (int i = 0; i < dataInput.horizonDays; i++)
            {
                reader.ReadLine(); //Skip first line
                str = reader.ReadLine();
                List<String> customerIdData = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);
                str = reader.ReadLine();
                data = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);
                str = reader.ReadLine(); //Skip last empty line

                for (int j = 0; j < customerIdData.Count; j++)
                {
                    int custId = int.Parse(customerIdData[j]) - 1; //Load ID
                    if (custId > 0)
                    {
                        int q = (int) Math.Round(double.Parse(data[j])); //Load quantity
                        dataInput.customerDeliveries[i, custId] = q;
                    }
                }
            }


            if (parse_routing)
            {
	            //Parse Route Info
	            for (int i = 0; i < dataInput.horizonDays; i++)
	            {
		            reader.ReadLine(); //Skip first line

		            for (int j = 0; j < vehicleNum; j++)
		            {
			            str = reader.ReadLine();
			            List<String> routeData = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);

			            //Save asignments

			            for (int k = 1; k < routeData.Count; k++)
			            {
				            int custId = int.Parse(routeData[k]) - 1; //Load ID
				            dataInput.customerRouteAssignment[i, custId] = j;
			            }
		            }

		            reader.ReadLine(); //Skip empty line
	            }
            }
            
			return dataInput;
        }
        
        
    }
}
