/* 
MIT License

Copyright (c) 2023 Huang I Chun

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "base.hpp"
#include "utility.hpp"

MyPoint::MyPoint(int my_x, int my_y, int my_z) : x(my_x), y(my_y), z(my_z) {}

bool MyPointCompare::operator()(const MyPoint &lhs, const MyPoint &rhs) const
{
    // int num = (resolution - cube_length / 2) / cube_length + 1;
    int num = 100;
    return lhs.x + lhs.y * num + lhs.z * num * num < rhs.x + rhs.y * num + rhs.z * num * num;
}

int find_center(int x, int cube_length)
{
    int times = (x - cube_length / 2) / cube_length;
    if (times < 0)
    {
        return cube_length / 2;
    }
    else
    {
        int big_bound = cube_length / 2 + (times + 1) * cube_length;
        int small_bound = cube_length / 2 + times * cube_length;
        if (abs(big_bound - x) > abs(x - small_bound))
        {
            return small_bound;
        }
        else
        {
            return big_bound;
        }
    }
}

std::vector<int> sort_indexes(const std::vector<std::vector<double>> &v)
{

    // initialize original index locations
    std::vector<int> idx(v.size());
    iota(idx.begin(), idx.end(), 0);

    // sort indexes based on comparing values in v
    // using std::stable_sort instead of std::sort
    // to avoid unnecessary index re-orderings
    // when v contains elements of equal values
    std::stable_sort(idx.begin(), idx.end(),
                     [&v](int i1, int i2)
                     { 
                    int len1 = v[i1][0] * v[i1][0] + v[i1][1] * v[i1][1] + v[i1][2] * v[i1][2];
                    int len2 = v[i2][0] * v[i2][0] + v[i2][1] * v[i2][1] + v[i2][2] * v[i2][2];
                    return len1 > len2; });

    return idx;
}

bool from_prev_or_next(pcl::PointCloud<pcl::PointXYZRGB>::Ptr prev_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr next_cloud)
{
    std::vector<int> nearest_search_result_next_to_prev; // use nearest_search_result_next_to_prev[NEXT_FRAME_POINT_INDEX] = PREV_FRAME_POINT_INDEX
    std::vector<int> nearest_search_result_prev_to_next; // use nearest_search_result_prev_to_next[PREV_FRAME_POINT_INDEX] = NEXT_FRAME_POINT_INDEX

    for (int idx = 0; idx < next_cloud->points.size(); idx++)
    {
        nearest_search_result_next_to_prev.push_back(-1);
    }

    for (int idx = 0; idx < prev_cloud->points.size(); idx++)
    {
        nearest_search_result_prev_to_next.push_back(-1);
    }

    pcl::search::KdTree<pcl::PointXYZRGB> prev_kdtree;
    prev_kdtree.setInputCloud(prev_cloud);

    pcl::search::KdTree<pcl::PointXYZRGB> next_kdtree;
    next_kdtree.setInputCloud(next_cloud);

    // first we iterate through all next frame points to query nearest point; which means we find matching point-pairs first
    double avg_next_to_prev_dist = 0.0;
    for (int idx = 0; idx < next_cloud->points.size(); idx++)
    {
        std::vector<int> prev_pointIdxRadiusSearch;
        std::vector<float> prev_pointRadiusSquaredDistance;
        prev_kdtree.nearestKSearch(next_cloud->points[idx], 1, prev_pointIdxRadiusSearch, prev_pointRadiusSquaredDistance);
        nearest_search_result_next_to_prev[idx] = prev_pointIdxRadiusSearch[0];
        avg_next_to_prev_dist += prev_pointRadiusSquaredDistance[0];
    }

    avg_next_to_prev_dist = avg_next_to_prev_dist / next_cloud->points.size();

    double avg_prev_to_next_dist = 0.0;
    for (int idx = 0; idx < prev_cloud->points.size(); idx++)
    {
        std::vector<int> prev_pointIdxRadiusSearch;
        std::vector<float> prev_pointRadiusSquaredDistance;
        next_kdtree.nearestKSearch(prev_cloud->points[idx], 1, prev_pointIdxRadiusSearch, prev_pointRadiusSquaredDistance);
        nearest_search_result_prev_to_next[idx] = prev_pointIdxRadiusSearch[0];
        avg_prev_to_next_dist += prev_pointRadiusSquaredDistance[0];
    }
    avg_prev_to_next_dist /= next_cloud->points.size();
    if (avg_next_to_prev_dist > avg_prev_to_next_dist)
    {
        return true; // next search prev
    }
    else
    {
        return false; // prev search next
    }
}
