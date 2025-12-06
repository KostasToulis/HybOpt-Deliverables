



1.) Traveltime and travel cost between locations are computed as 

(int)(ceil(10 * sqrt((double)(XCoord(i) - XCoord(j))*(XCoord(i) - XCoord(j)) + (YCoord(i) - YCoord(j))*(YCoord(i) - YCoord(j))) ));

2.) Time window of location l is then computed as:
Duedate + 5 if l is a single-delivery location and
Duedate + 2 if l is a multiple-delivery location

3.) Afterwards time windows and all service times are also multiplied with factor 10
(same factor as travel times and cost are multiplied with in 1.))