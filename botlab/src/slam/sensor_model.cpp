#include <vector>
#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/grid_utils.hpp>
#include <slam/mapping.hpp>
#include <common/point.hpp>



SensorModel::SensorModel(void)
{
    ///////// TODO: Handle any initialization needed for your sensor model
}


double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map)
{
    ///////////// TODO: Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////
    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose);
    double scanScore = 0;

    for(auto& ray : movingScan){
        Point<double> endpoint(ray.origin.x + ray.range * std::cos(ray.theta),
                               ray.origin.y + ray.range * std::sin(ray.theta));
        auto rayEnd = global_position_to_grid_position(endpoint, map);
        
        Point<int> rayStart = global_position_to_grid_cell(ray.origin, map);
        Point<int> rayCell;

        rayCell.x = static_cast<int>((ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + rayStart.x);
        rayCell.y = static_cast<int>((ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + rayStart.y);
        
        if(map.logOdds(rayEnd.x, rayEnd.y) > 0.0){
            scanScore += map.logOdds(rayEnd.x, rayEnd.y);
            continue;
        }
        std::vector<Point<int>> listCells_b = get_cell_before(ray, map);
        std::vector<Point<int>> listCells_a = get_cell_after(ray, map);
        if(listCells_b.size() > 1){
            auto cellBefore = listCells_b[listCells_b.size() - 1];
            if(map.isCellInGrid(cellBefore.x, cellBefore.y) && map.logOdds(cellBefore.x, cellBefore.y) > 0.0){
                scanScore += (1/3) * map.logOdds(rayEnd.x, rayEnd.y);
            }
            //std::cout<<"cell start "<<rayStart.x<<" "<<rayStart.x<<std::endl;
            //std::cout<<"cell before "<<cellBefore.x<<" "<<cellBefore.y<<std::endl;
            //std::cout<<"cell end "<<rayCell.x<<" "<<rayCell.x<<std::endl;
        }
        
        if(listCells_a.size() == 2){
            auto cellAfter = listCells_a[1];
            if(map.isCellInGrid(cellAfter.x, cellAfter.y) && map.logOdds(cellAfter.x, cellAfter.y) > 0.0){
                scanScore += (1/3) * map.logOdds(rayEnd.x, rayEnd.y);
            }
            //std::cout<<"cell after"<<cellAfter.x<<" "<<cellAfter.y<<std::endl;
            //std::cout<<"cell "<<rayCell.x<<" "<<rayCell.x<<std::endl;
        }
    }
    return scanScore;
}

std::vector<Point<int>> SensorModel::get_cell_before(const adjusted_ray_t& ray, const OccupancyGrid& map){
    std::vector<Point<int>> listCells;
    
    //5.0f is the max laser distance
    if(ray.range <= 5.0f)
    {
        Point<int> rayStart = global_position_to_grid_cell(ray.origin, map);
        Point<int> rayCell;

        rayCell.x = static_cast<int>((ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + rayStart.x);
        rayCell.y = static_cast<int>((ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + rayStart.y);

        int dx = std::abs(rayCell.x - rayStart.x);
        int dy = std::abs(rayCell.y - rayStart.y);
        int sx = rayStart.x < rayCell.x ? 1 : -1;
        int sy = rayStart.y < rayCell.y ? 1 : -1;
        int err = dx - dy;
        int x = rayStart.x;
        int y = rayStart.y;

        while(x != rayCell.x || y != rayCell.y) 
        {
            Point<int> temp;
            temp.x = x;
            temp.y = y;
            listCells.push_back(temp);
            int e2 = 2 * err;
            if (e2 >= -dy) 
            {
                err -= dy;
                x += sx;
            }
            if (e2 <= dx) 
            {
                err += dx;
                y += sy;
            }
        }
    }
    return listCells;
}


std::vector<Point<int>> SensorModel::get_cell_after(const adjusted_ray_t& ray, const OccupancyGrid& map){
    std::vector<Point<int>> listCells;
    Point<int> rayStart = global_position_to_grid_cell(ray.origin, map);
    Point<int> rayCell;
    Point<int> extendedRayCell;

    rayCell.x = static_cast<int>((ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + rayStart.x);
    rayCell.y = static_cast<int>((ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + rayStart.y);

    //extended ray
    extendedRayCell.x = static_cast<int>((ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + rayCell.x);
    extendedRayCell.y = static_cast<int>((ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + rayCell.y);

    int dx = std::abs(extendedRayCell.x - rayCell.x);
    int dy = std::abs(extendedRayCell.y - rayCell.y);
    int sx = rayCell.x < extendedRayCell.x ? 1 : -1;
    int sy = rayCell.y < extendedRayCell.y ? 1 : -1;
    int err = dx - dy;
    int x = rayCell.x;
    int y = rayCell.y;

    while(x != extendedRayCell.x || y != extendedRayCell.y) 
    {
        Point<int> temp;
        temp.x = x;
        temp.y = y;
        listCells.push_back(temp);
        if(listCells.size() == 2){
            break;
        }
        int e2 = 2 * err;
        if (e2 >= -dy) 
        {
            err -= dy;
            x += sx;
        }
        if (e2 <= dx) 
        {
            err += dx;
            y += sy;
        }
    }
    return listCells;
}
