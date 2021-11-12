#include <bits/stdc++.h>

using namespace std;

// the explored states in the dfs and the bfs
unordered_set<string> explored;
// the maximum reached depth
int maxDepth = 0;
// the number of expanded states
int nodesExbanded = 0;

//the puzzle state
struct State{
  // the current state puzzle
  string puzzle;
  // the parent state
  State* parent;
  //the current cost
  int cost;
  //estimated cost
  int estimated;

  State(string _puzzle, State* _parent, int _cost){
    puzzle = _puzzle;
    parent = _parent;
    cost = _cost;
  }
  State(string _puzzle, State* _parent, int _cost, int _estimated){
    puzzle = _puzzle;
    parent = _parent;
    cost = _cost;
    estimated = _estimated;
  }

};

// a comparetor for the priority queue to get the state with the minimum estimated cost
struct Compare {
    bool operator()(State* a, State* b)
    {
        return a->estimated > b->estimated;
    }
};

// Get for each state its valid neighbours
vector<string> getNeighbours(string state){
  vector<string>neighbours;

  //get the place of the empty space tile
  int emptySpace;
  for(emptySpace=0;emptySpace<9;++emptySpace){
    if(state[emptySpace] == '0'){
      break;
    }
  }

  //check if the down position is valid
  if(emptySpace + 3 < 9){

    string newState=state;
    swap(newState[emptySpace],newState[emptySpace + 3]);
    //put the new state in the array
    neighbours.push_back(newState);

  }

  //check if the up position is valid
  if(emptySpace - 3 > 0){

    string newState=state;
    swap(newState[emptySpace],newState[emptySpace - 3]);
    neighbours.push_back(newState);

  }

  //check if the right position is valid
  if( (emptySpace + 1)/3 == emptySpace/3){

    string newState=state;
    swap(newState[emptySpace],newState[emptySpace + 1]);
    neighbours.push_back(newState);

  }

  //check if the left position is valid
  if(emptySpace && (emptySpace - 1)/3 == emptySpace/3){

    string newState=state;
    swap(newState[emptySpace],newState[emptySpace - 1]);
    neighbours.push_back(newState);

  }
  //return a vector of the valid neigbouring states
  return neighbours;
}

// check whether a state is the goal state or not
bool isGoal(string state){
  if (state.compare("012345678") == 0) return true;
  return false;
}

// printing the puzzle as 3*3
void printPuzzle(string puzzle){
  for(int i=0;i<9;++i){
    if(i%3==0)cout<<"\n";
    cout<<puzzle[i]<<" ";
  }
  cout<<"\n";
}

//The BFS (the unformed search)
State* BFS(string initial){
  explored.clear();
  nodesExbanded = 0;
  maxDepth = 0;
  State* root = new State(initial, NULL, 0);
  root->parent = root;
  queue<State*> q;
  q.push(root);
  while(q.size()){
    nodesExbanded++;
    State* node = q.front();
    q.pop();
    maxDepth=max(node->cost,maxDepth);
    //printPuzzle(node->puzzle);
    if (isGoal(node->puzzle)) return node;
    vector<string> neighbours = getNeighbours(node->puzzle);
    for(string neighbour : neighbours){
      if (explored.find(neighbour) == explored.end()){
        q.push(new State(neighbour, node, node->cost + 1));
        explored.insert(node->puzzle);
      }
    }
  }
  return NULL;
}

//The DFS (the unformed search)
State* DFS(string initial){
  explored.clear();
  maxDepth = 0;
  nodesExbanded = 0;
  State* root = new State(initial, NULL, 0);
  root->parent = root;
  stack<State*> s;
  s.push(root);
  while(s.size()){
    nodesExbanded++;
    State* node = s.top();
    s.pop();
    maxDepth=max(node->cost,maxDepth);
    //printPuzzle(node->puzzle);
    if (isGoal(node->puzzle)) return node;
    vector<string> neighbours = getNeighbours(node->puzzle);
    for(string neighbour : neighbours){
      if (explored.find(neighbour) == explored.end()){
        s.push(new State(neighbour, node, node->cost + 1));
        explored.insert(node->puzzle);
      }
    }
  }
  return NULL;
}

//the A* (the informed search) we are going to use Manhattan heuristic and Euclidean heuristic
unordered_map<string,int> explored_Astar;

State* AStar(string initial , int (*heuristic)(string)){
  explored_Astar.clear();
  maxDepth = 0;
  nodesExbanded = 0;
  State* root = new State(initial, NULL, 0);
  root->parent = root;
  priority_queue<State* ,vector<State*>,Compare> q;
  q.push(root);
  explored_Astar[root->puzzle]=0;
  while(q.size()){
    State* node = q.top();
    q.pop();
    maxDepth=max(node->cost,maxDepth);
    if (explored_Astar[node->puzzle] > node->cost) continue;
    nodesExbanded++;
    //printPuzzle(node->puzzle);
    if (isGoal(node->puzzle)) return node;
    vector<string> neighbours = getNeighbours(node->puzzle);
    for(string neighbour : neighbours){
      auto it = explored_Astar.find(neighbour);
      int cost=node->cost +1+ heuristic(neighbour);
      if (it == explored_Astar.end() || it->second > cost){
        q.push(new State(neighbour, node, node->cost +1, cost));
        explored_Astar[node->puzzle]=cost;
      }
    }
  }
  return NULL;
}

//Manhattan Distance Heuristic
//It is the sum of absolute values of differences in the goal’s x and y coordinates and
//the current cell’s x and y coordinates respectively,
//h = abs(current cell:x - goal:x) + abs(current cell:y - goal:y)
int manhattanHeuristic(string state){
  int cost=0;
  for(int i=0;i<9;++i){
    if (state[i] == '0') continue;
    int x=i/3 ,y=i%3;
    cost+=abs((state[i]-'0')/3 - x)+abs((state[i]-'0')%3 - y);
  }
  return cost;
}

// Euclidean Distance Heuristic
//It is the distance between the current cell and the goal cell using the distance formula
//h = sqrt((current cell:x - goal:x)^2 + (current cell.y - goal:y)^2)
int euclideanHeuristic(string state){
  double cost=0;
  for(int i=0;i<9;++i){
    double x=i/3 ,y=i%3;
    double currX=((state[i]-'0')/3 - x),currY=(state[i]-'0')%3 - y;
    cost+=sqrt(currX*currX+currY*currY);
  }
  return round(cost);
}

// To execute C++, please define "int main()"
int main() {
  //Testing
  srand (time(0));
  string input;
  //Getting the input
  cin >> input;
  State* out;
  //trying diffrent algorithms
  for(int operation = 1; operation <= 4; ++operation){
    auto first = chrono::high_resolution_clock::now();
    switch(operation){
      case 1:
        out = BFS(input);
        break;
      case 2:
        out = DFS(input);
        break;
      case 3:
        out = AStar(input, &manhattanHeuristic);
        break;
      case 4:
        out = AStar(input, &euclideanHeuristic);
        break;
    }
    auto second = chrono::high_resolution_clock::now();
    if (out == NULL){
      cout << "NOT SOLVABLE Time: " << chrono::duration_cast< chrono::duration<double> > (second-first).count() << '\n';
    } else {
      cout << "Cost: " << out->cost << " Max_Depth: " << maxDepth
           << " Time: " << chrono::duration_cast< chrono::duration<double> > (second-first).count()
           << " Number_of_nodes_exbanded: " << nodesExbanded << '\n';
      if (operation == 2) continue;
      vector<string> path;
      State* curr = out;
      do {
        path.push_back(curr->puzzle);
        curr = curr->parent;
      }while(curr->parent != curr);
      path.push_back(curr->puzzle);
      reverse(path.begin(), path.end());
      for(string puzzle : path){
        printPuzzle(puzzle);
      }
    }
  }
  return 0;
}
