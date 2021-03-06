#include <planning/obstacle_distance_grid.hpp>
#include <slam/occupancy_grid.hpp>


ObstacleDistanceGrid::ObstacleDistanceGrid(void)
: width_(0)
, height_(0)
, metersPerCell_(0.05f)
, cellsPerMeter_(20.0f)
{
}

void ObstacleDistanceGrid::initializeDistances(const OccupancyGrid& map)
{
    int width = map.widthInCells();
    int height = map.heightInCells();
    bool flag = 0;
    for(int y = 0; y < height; y++) {
        for(int x = 0; x < width; x++) {
            if(map.logOdds(x,y) < 0){
                distance(x,y) = -1;
            }
            else {
                flag = 1;
                distance(x,y) = 0;
            }
        }
    }
    if(flag == 0){
        for(int y = 0; y < height; y++) {
            for(int x = 0; x < width; x++) {
               distance(x,y) = 100;
            }
        }
    }
        
}


void ObstacleDistanceGrid::setDistances(const OccupancyGrid& map)
{
    resetGrid(map);
    initializeDistances(map);

    std::priority_queue<DistanceNode> searchQueue;
    enqueue_obstacle_cells(*this, searchQueue);

    while(!searchQueue.empty()){
        DistanceNode nextNode = searchQueue.top();
        searchQueue.pop();
        expand_node(nextNode, *this, searchQueue);
    }

    ///////////// TODO: Implement an algorithm to mark the distance to the nearest obstacle for every cell in the map.
}


bool ObstacleDistanceGrid::isCellInGrid(int x, int y) const
{
    return (x >= 0) && (x < width_) && (y >= 0) && (y < height_);
}


void ObstacleDistanceGrid::resetGrid(const OccupancyGrid& map)
{
    // Ensure the same cell sizes for both grid
    metersPerCell_ = map.metersPerCell();
    cellsPerMeter_ = map.cellsPerMeter();
    globalOrigin_ = map.originInGlobalFrame();
    
    // If the grid is already the correct size, nothing needs to be done
    if((width_ == map.widthInCells()) && (height_ == map.heightInCells()))
    {
        return;
    }
    
    // Otherwise, resize the vector that is storing the data
    width_ = map.widthInCells();
    height_ = map.heightInCells();
    
    cells_.resize(width_ * height_);
}

void ObstacleDistanceGrid::enqueue_obstacle_cells(ObstacleDistanceGrid& grid, std::priority_queue<DistanceNode>& searchQueue){
    int width = grid.widthInCells();
    int height = grid.heightInCells();
    cell_t cell;

    for(cell.y = 0; cell.y < height; cell.y++){
        for(cell.x = 0; cell.x < width; cell.x++){
            if(distance(cell.x, cell.y) == 0){
                expand_node(DistanceNode(cell, 0.0), grid, searchQueue);
            }
        }
    }

}

void ObstacleDistanceGrid::expand_node(const DistanceNode& node, ObstacleDistanceGrid& grid, std::priority_queue<DistanceNode>& searchQueue){
    //For the side cells
    const int xDeltas_sd[4] = {1, 0, 0, -1};
    const int yDeltas_sd[4] = {0, -1, 1, 0};
    for(int i = 0; i < 4; i++){
        cell_t adjacentCell(node.cell.x + xDeltas_sd[i], node.cell.y + yDeltas_sd[i]);
        if(grid.isCellInGrid(adjacentCell.x, adjacentCell.y)){
            if(grid(adjacentCell.x, adjacentCell.y) == -1){
                DistanceNode adjacentNode(adjacentCell, node.distance + 1);
                grid(adjacentCell.x, adjacentCell.y) = adjacentNode.distance * grid.metersPerCell();
                searchQueue.push(adjacentNode);
            }
        }
    }
    /*
    //For the diagonal cells
    const int xDeltas_diag[4] = {1, 1, -1, -1};
    const int yDeltas_diag[4] = {1, -1,  1, -1};
    for(int i = 0; i < 4; i++){
        cell_t adjacentCell(node.cell.x + xDeltas_diag[i], node.cell.y + yDeltas_diag[i]);
        if(grid.isCellInGrid(adjacentCell.x, adjacentCell.y)){
            if(grid(adjacentCell.x, adjacentCell.y) == -1){
                DistanceNode adjacentNode(adjacentCell, node.distance + 1.4);
                grid(adjacentCell.x, adjacentCell.y) = adjacentNode.distance * grid.metersPerCell();
                searchQueue.push(adjacentNode);
            }
        }
    }*/
}
