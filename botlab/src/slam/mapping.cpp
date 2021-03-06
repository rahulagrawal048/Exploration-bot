#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>


Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
{
}


void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////
    if(!initialized_)
    {
        previousPose_ = pose;
    }

    MovingLaserScan movingScan(scan, previousPose_, pose);

    for(auto& ray : movingScan)
    {
        scoreEndpoint(ray, map);
        scoreRay(ray, map);
    }

    initialized_ = true;
    previousPose_ = pose;

}

void Mapping::scoreEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    if(ray.range < kMaxLaserDistance_)
    {
        Point<int> rayStart = global_position_to_grid_cell(ray.origin, map);
        Point<int> rayCell;

        rayCell.x = static_cast<int>((ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + rayStart.x);
        rayCell.y = static_cast<int>((ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + rayStart.y);

        if(map.isCellInGrid(rayCell.x, rayCell.y))
        {
            increaseCellOdds(rayCell.x, rayCell.y, map);
        }
    }
}
void Mapping::scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    if(ray.range <= kMaxLaserDistance_)
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
            if(map.isCellInGrid(x, y))
            {
                decreaseCellOdds(x, y, map);
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
    }

}
void Mapping::decreaseCellOdds(int x, int y, OccupancyGrid& map)
{
    if (std::numeric_limits<CellOdds>::min() - map(x,y) < -kMissOdds_) 
    {
        map(x,y) -= kMissOdds_;
    } 
    else 
    {
        map(x,y) = std::numeric_limits<CellOdds>::min();
    }
}
void Mapping::increaseCellOdds(int x, int y, OccupancyGrid& map)
{
    if(std::numeric_limits<CellOdds>::max() - map(x,y) > kHitOdds_)
    {
        map(x,y) += kHitOdds_;
    }
    else
    {
        map(x,y) = std::numeric_limits<CellOdds>::max();
    }
}