# Hybrid Optimization Methodologies for Transportation Logistics Management - HybOpt 
 

This document briefly outlines the different code projects developed in the context of project Hybrid Optimization Methodologies for Transportation Logistics Management – HybOpt. This research project (proposal number: 02562) is funded by the Hellenic Foundation for Research & Innovation (H.F.R.I.) through the “2nd Call for H.F.R.I. Research Projects to support Faculty Members and Researchers” program. 

 

## Project summary 

The HybOpt project aims to contribute to the field of logistics management by identifying challenging integrated vehicle routing problems with practical applications in freight transportation. This involves examining complex, real-life operational constraints and environmental considerations, moving beyond traditional literature variants. The project seeks to make significant theoretical and methodological contributions by designing a systemic methodology for decision-making in large-scale industrial logistics problems. Emphasis is placed on developing rigorous mathematical models and novel algorithmic frameworks within hybrid optimization schemes, exploring collaborations between mathematical programming and metaheuristic algorithms, as well as utilizing artificial intelligence methodologies for solving combinatorial optimization problems (COPs).  

 

## Repositories 

In this context, we have selected and tackled five complex integrated problems arising from the logistics and transportation industry. Our contribution can be found mainly in optimization methodologies but also in mathematical modelling for some of them. The following problems are modelled and solved: 

1. Cyclic Production Routing Problem (CPRP) 

2. Set Orienteering Problem (SOP) 

3. Set Team Orienteering Problem (STOP) 

4. Crowdsourced Humanitarian Relief Vehicle Routing Problem (CHR-VRP) 

5. Cumulative Capacitated Vehicle Routing Problem with Time-Windows (CCVRPTW) 

6. Vehicle Routing Problem with Delivery Options (VRPDO) 

7. Family Capacitated Vehicle Routing Problem (F-CVRP) 

 

## CPRP 

The repo solves Cyclic Production Routing Problem which is an extension of the well-known Production Routing Problem. A novel mathematical programming approach is provided along with exact and metaheuristic algorithms. The Gurobi solver is required.  

Starting point: Program.cs 

 

## SOP 

The repo solves the Set Orienteering Problem. It provides a powerful matheuristic algorithm that has improved the majority of best solutions known in literature for a well-known benchmark set. The Gurobi solver is required  

Starting point: Program.cs 

 

## STOP 

The repo solves the Set Team Orienteering Problem. It is a natural extension of the Set Orienteering Problem.  

Starting point: Program.cs 

 

## CHR-VRP 

The repo solves the Crowdsourced Humanitarian Relief Vehicle Routing Problem under 4 different objective using a powerful local search algorithm. The Gurobi solver is required.  

Starting point: Run.cs 

 

 

 

## CCVRPTW 

The repo solves the Cumulative Capacitated Vehicle Routing Problem with Time-Windows. The code has implemented routines for solving the problem using constraint programming solvers (CPLEX IBM CP Optimizer, OR Tools), a local search algorithm as well as a novel genetic algorithm. The provided solvers support 3 difference objectives: Total Distance, total cumulative distance and total cumulative time. 

Starting point: Run.cs 

 

## VRPDO 

The repo solves the Vehicle Routing Problem with Delivery Options. The code has implemented routines for solving the problem using a meta-heuristic algorithm based on local search, equipped with specialized neighborhood operators for the problem.  

Starting point: Start.cs 

 

## F-CVRP 

The repo solves the Family Capacitated Vehicle Routing Problem. The code utilizes a hybrid guided tabu search metaheuristic for solution refinement and two MIP model-based methods for solution construction. 

Starting point: Program.cs 

 

 
