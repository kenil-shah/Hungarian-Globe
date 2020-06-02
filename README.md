# Hungarian_Globe_Solver

Implementation for hungarian globe solver using BFS, A* and RBFS search algorithms. (New branch)

![Alt Text](https://github.com/kenil-shah/Hungarian-Globe/blob/master/readme_images/Capture.PNG)

**There are 30 tiles in the  above given image. Goal of this project is to perform search algorithms in such a way that every tile is in it's desired location. Implemented BFS, AStar and RBFS to solve this problem**

## Installation

1) Clone this repository.
```
git clone https://github.com/kenil-shah/Hungarian-Globe.git
```

### Usage
```
python search.py ALG file_name

ALG : BFS/AStar/RBFS
file_name : Use ./PuzzleFiles to generate the state space for initial globe setting.

Command :- python3 search.py AStar PuzzleFiles/PathN-8.mb
```

### Heuristic Used
'''
	In this puzzle, the total number of tiles is 30 and every tile would have a current position and a goal position. Thus, we could find the number of steps taken by every tile to reach to its goal position. The heuristic in our puzzle could be the summation of number of steps taken by every tile to reach to its goal state. Moreover, we will  also divide the summation by 13. Now, in order to reach to an optimal path we have to design a heuristic that is admissible(tree-search) and consistent.

'''
![Alt Text](https://github.com/kenil-shah/Hungarian-Globe/blob/master/readme_images/heuristic.PNG)
