#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <common/grid_utils.hpp>
#include <iostream>
using namespace std;


robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////
    robot_path_t path;
    path.utime = start.utime;
    path.path.push_back(start);    
    path.path_length = path.path.size();
    
    //Create goal node
    cell_t goalCell =  global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances);
    Node* goalNode = new Node(goalCell.x, goalCell.y);
    goalNode->g_cost = 1.0e16;
    goalNode->h_cost = 0.0;
    goalNode->parent = NULL;

    //Create start node
    cell_t startCell =  global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
    Node* startNode = new Node(startCell.x, startCell.y);
    startNode->g_cost = 0.0;
    startNode->h_cost = h_cost(startNode, goalNode);
    startNode->parent = NULL;
    
    // A valid start is in the grid
    if(distances.isCellInGrid(startCell.x, startCell.y))
    {
        if(distances(startCell.x, startCell.y) == 0){
            return path;
        }
    }
    else{
        return path;
    }

    //Create priority queue open list
    PriorityQueue openList;
    std::vector<Node*> closedList;
    std::vector<Node*> searchList;
    openList.push(startNode);
    searchList.push_back(startNode);

    bool found_path = false;

    cout<<"start node = "<<startNode->cell.x<<" "<<startNode->cell.y<<endl;
    cout<<"goal node = "<<goalNode->cell.x<<" "<<goalNode->cell.y<<endl;
    
    int i = 0;

    while(!openList.empty() && i < 10000){
        
        i++;
        
        //Pop open list
        Node* nextNode = openList.pop();
        delete_member(nextNode, searchList);
        
        //cout<<nextNode->cell.x<<" "<<nextNode->cell.y<<endl;
        //cout<<nextNode->g_cost<<" "<<nextNode->h_cost<<endl;
        
        //Push to closed list
        closedList.push_back(nextNode);
        
        //check if goal reached
        if(found_path){
            break;
        }
        
        //Expand the nextNode
        std::vector<Node*> children = expand_node(nextNode, distances, params);
        
        for(auto child: children)
        {
            //skip if it is an obstacle
            if(distances(child->cell.x, child->cell.y) > params.minDistanceToObstacle){
            //skip if it is in the closed list
                if(is_in_list(child, closedList)){
                    continue;
                }
                
                if(not is_in_list(child, searchList)){
                    child->g_cost = g_cost(nextNode, child, distances, params);
                    child->h_cost = h_cost(child, goalNode);
                    child->parent = nextNode;
                    searchList.push_back(child);
                    openList.push(child);
                    //cout<<"not in open"<<endl;
                }
                else{
                    Node* existingChild = get_member(child, searchList);
                    double g_costn = g_cost(nextNode, child, distances, params);
                    if(existingChild->g_cost > g_costn){
                        child->g_cost = g_cost(nextNode, child, distances, params);
                        child->h_cost = h_cost(child, goalNode);
                        child->parent = nextNode;
                        openList.push(child);
                        searchList.push_back(child);
                    }
                    //cout<<"in open"<<endl;
                }
            }
        }
        
        if(nextNode->cell.x == goalNode->cell.x && nextNode->cell.y == goalNode->cell.y){
            cout<<"Found Path !!!"<<endl;
            goalNode = nextNode;
            found_path = true;
        }
    }
    if(found_path){
        std::vector<Node*> node_path = extract_node_path(goalNode);
        std::vector<pose_xyt_t> pose_path = extract_pose_path(node_path, distances);
        //start is already included so i = 1
        for(int i = 1; i < pose_path.size(); i++){
            path.path.push_back(pose_path[i]); 
        }
        path.path_length = path.path.size();
        return path;
    }
    
    return path;
}

bool is_in_list(Node* node, std::vector<Node*> list)
{
    for(auto& item: list){
        if(node->cell.x == item->cell.x && node->cell.y == item->cell.y){
            return true;
        }
    }
    return false;
}

Node* get_member(Node* node, std::vector<Node*> list){
    for(auto& item: list){
        if(node->cell.x == item->cell.x && node->cell.y == item->cell.y){
            return node;
        }
    }
}

void delete_member(Node* node, std::vector<Node*> list){
    int idx;
    for(int i = 0; i < list.size(); i++){
        if(node->cell.x == list[i]->cell.x && node->cell.y == list[i]->cell.y){
            idx = i;
            break;
        }
    }
    list.erase(list.begin() + idx);
}

double h_cost(Node* from, Node* goal){
    //From lecture 12 slides
    int dx = std::abs(from->cell.x - goal->cell.x);
    int dy = std::abs(from->cell.y - goal->cell.y);
    //Using 4-way manhattan distance for the maze
    return (dx + dy);
    //return (dx + dy) + (1.414 - 2)*std::min(dx, dy);
}

double g_cost(Node* from, Node* to, const ObstacleDistanceGrid& distances, const SearchParams& params){
    double g_cost = 0;
    int delta_x = std::abs(from->cell.x - to->cell.x);
    int delta_y = std::abs(from->cell.y - to->cell.y);
    if(delta_x == 1 && delta_y == 1){
        g_cost += from->g_cost + 1.4;
    } else{
        g_cost += from->g_cost + 1.0;
    }
    //Incorporating obstacle distances in g_cost
    //Using comments in astar.hpp
    double obsDistance = distances(to->cell.x, to->cell.y);
    if(obsDistance > params.minDistanceToObstacle && obsDistance < params.maxDistanceWithCost){
        g_cost += std::pow(params.maxDistanceWithCost - obsDistance, params.distanceCostExponent);
    }
    return g_cost;
}
//find all children and return pointers to them
std::vector<Node*> expand_node(Node* node, const ObstacleDistanceGrid& distances, const SearchParams& params){
    std::vector<Node*> neighbors;
    /*
    const int xDeltas[8] = {1, 0, 0, -1, 1, 1, -1, -1};
    const int yDeltas[8] = {0, -1, 1, 0, 1, -1,  1, -1};
    for(int i = 0; i < 8; i++){
        int x = node->cell.x + xDeltas[i];
        int y = node->cell.y + yDeltas[i];
        if(distances.isCellInGrid(x, y)){
            Node* temp = new Node(x,y);
            neighbors.push_back(temp);
        }
    }
    */
   //make it 4-way for the maze
    const int xDeltas[4] = {1, 0, 0, -1};
    const int yDeltas[4] = {0, -1, 1, 0};
    for(int i = 0; i < 4; i++){
        int x = node->cell.x + xDeltas[i];
        int y = node->cell.y + yDeltas[i];
        if(distances.isCellInGrid(x, y)){
            Node* temp = new Node(x,y);
            neighbors.push_back(temp);
        }
    }
    return neighbors;
}

std::vector<Node*> extract_node_path(Node* node){
    std::vector<Node*> node_path_backwards;
    std::vector<Node*> node_path;
    while(node != NULL){
        node_path_backwards.push_back(node);
        node = node->parent;
    }
    //go from start to goal
    for(int i = node_path_backwards.size() - 1; i >= 0; i--){
        node_path.push_back(node_path_backwards[i]);
    }
    return node_path;
}


int discretize(int path_length, float interval_length, float x1, float x2, float y1, float y2, float theta1, float theta2, robot_path_t &path)
{
    int temp_path_length = path_length;
    pose_xyt_t nextPose;
    int diffx = round((x2 - x1)/interval_length);
    int diffy = round((y2 - y1)/interval_length);
    int num_intervals = std::max(fabs(diffx),fabs(diffy));
    
    float delx;
    if (diffx > 0) {delx = interval_length;}
    else if (diffx < 0) {delx = -interval_length;}
    else {delx = 0;}
    float dely;
    if (diffy > 0) {dely = interval_length;}
    else if (diffy < 0) {dely = -interval_length;}
    else {dely = 0;}

    for (int i = 0; i<num_intervals; i++)
    {
        nextPose.x = delx*(i+1) + x1;
        nextPose.y = dely*(i+1) + y1;
        if (i<num_intervals-1)
        {
            nextPose.theta = theta1;
        }
        else if (i == num_intervals-1)
        {
            nextPose.theta = theta2;
        }
        path.path[temp_path_length] = nextPose;
        temp_path_length++;
    }
    return temp_path_length;

}

//Convert Nodes to path
std::vector<pose_xyt_t> extract_pose_path(std::vector<Node*> nodePath, const ObstacleDistanceGrid& distances){
    std::vector<pose_xyt_t> pose_path;
    for(int i = 0; i < nodePath.size(); i++){
        Point<double> tempPoint;
        tempPoint.x = nodePath[i]->cell.x;
        tempPoint.y = nodePath[i]->cell.y;
        tempPoint = grid_position_to_global_position(tempPoint, distances);
        pose_xyt_t temp;
        temp.x = tempPoint.x;
        temp.y = tempPoint.y;
        if(i < nodePath.size() - 1){
            Point<double> nextPoint;
            nextPoint.x = nodePath[i + 1]->cell.x;
            nextPoint.y = nodePath[i + 1]->cell.y;
            nextPoint = grid_position_to_global_position(nextPoint, distances);
            float delta_x = nextPoint.x - temp.x;
            float delta_y = nextPoint.y - temp.y;
            temp.theta = std::atan2(delta_y, delta_x);
        }
        else{
            temp.theta = 0;
        }
        pose_path.push_back(temp);
    }

    //final discretized path
    float interval_length = 0.01;
    robot_path_t final_path;
    path.path.resize((0.05*pose_path.size() - 1)/interval_length);

    int path_length = 0;

    pose_xyt_t nextPose;
    nextPose.x = pose_path[0].x;
    nextPose.y = pose_path[0].y;
    nextPose.theta = pose_path.theta;
    final_path.path[path_length] = nextPose;
    path_length++;

    for(int i = 0; i < pose_path.size() - 1; i++){
        path_length = discretize(path_length, interval_length, pose_path[i].x,
                     pose_path[i+1].x, pose_path[i].y, pose_path[i+1].y, pose_path[i].theta, pose_path[i+1].theta, final_path);
    }
    final_path.path_length = final_path.path.size();

    return final_path;
}


