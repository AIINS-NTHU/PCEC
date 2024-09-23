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
#include "Stage2.hpp"

Stage2_Nearest_Neighbor::Stage2_Nearest_Neighbor(int my_mode) : mode(my_mode)
{
}

void Stage2_Nearest_Neighbor::run(Pipeline_Object &pipeline_obj)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds_prev_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds_next_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (mode == 1)
    {
        ds_prev_cloud = pipeline_obj.ds1_prev_cloud;
        ds_next_cloud = pipeline_obj.ds1_next_cloud;
    }
    else if (mode == 2)
    {
        ds_prev_cloud = pipeline_obj.ds2_prev_cloud;
        ds_next_cloud = pipeline_obj.ds2_next_cloud;
    }

    for (int idx = 0; idx < ds_next_cloud->points.size() + 2; idx++)
    {
        pipeline_obj.matching_table.push_back(-1);
    }

    pcl::search::KdTree<pcl::PointXYZRGB> prev_kdtree;
    prev_kdtree.setInputCloud(ds_prev_cloud);

    for (int idx = 0; idx < ds_next_cloud->points.size() + 2; idx++)
    {
        std::vector<int> prev_pointIdxRadiusSearch;
        std::vector<float> prev_pointRadiusSquaredDistance;
        prev_kdtree.nearestKSearch(ds_next_cloud->points[idx], 1, prev_pointIdxRadiusSearch, prev_pointRadiusSquaredDistance);
        pipeline_obj.matching_table[idx] = prev_pointIdxRadiusSearch[0];
    }

    if (_next != NULL)
    {
        _next->run(pipeline_obj);
    }
}

Stage2_CFDS::Stage2_CFDS(int my_mode) : mode(my_mode)
{
}

void Stage2_CFDS::run(Pipeline_Object &pipeline_obj)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds_prev_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds_next_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (mode == 1)
    {
        ds_prev_cloud = pipeline_obj.ds1_prev_cloud;
        ds_next_cloud = pipeline_obj.ds1_next_cloud;
    }
    else if (mode == 2)
    {
        ds_prev_cloud = pipeline_obj.ds2_prev_cloud;
        ds_next_cloud = pipeline_obj.ds2_next_cloud;
    }

    bool use_next_search_prev;
    int length = ds_next_cloud->points.size() > ds_prev_cloud->points.size() ? ds_next_cloud->points.size() : ds_prev_cloud->points.size();

    for (int idx = 0; idx < length + 2; idx++)
    {
        pipeline_obj.matching_table.push_back(-1);
    }

    int color_dist = 32;
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr prev_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr next_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr origin_prev_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr origin_next_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        origin_prev_cloud = ds_prev_cloud;
        origin_next_cloud = ds_next_cloud;

        use_next_search_prev = from_prev_or_next(origin_prev_cloud, origin_next_cloud);

        if (!use_next_search_prev)
        {
            // prev_cloud->points = next_cloud->points;
            // next_cloud->points = prev_cloud->points;
            prev_cloud = origin_next_cloud;
            next_cloud = origin_prev_cloud;
        }
        else
        {
            next_cloud = origin_next_cloud;
            prev_cloud = origin_prev_cloud;
        }

        // kmeans to categorize color
        std::vector<pcl::PointXYZRGB> kmeans_points;
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> kmeans_point_clouds;
        std::vector<pcl::search::KdTree<pcl::PointXYZRGB>> kmeans_kdtrees;

        for (int idx = 0; idx < prev_cloud->points.size(); idx++)
        {
            bool flag = true;
            for (int i = 0; i < kmeans_points.size(); i++)
            {
                if (abs(kmeans_points[i].r - prev_cloud->points[idx].r) < color_dist &&
                    abs(kmeans_points[i].g - prev_cloud->points[idx].g) < color_dist &&
                    abs(kmeans_points[i].b - prev_cloud->points[idx].b) < color_dist)
                {
                    flag = false;
                    break;
                }
            }

            if (flag)
            {
                kmeans_points.push_back(prev_cloud->points[idx]);
            }
        }

        for (int kmeans_round = 0; kmeans_round < 2; kmeans_round++)
        {

            kmeans_point_clouds.clear();
            for (int i = 0; i < kmeans_points.size(); i++)
            {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr kmeans_new_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                kmeans_point_clouds.push_back(kmeans_new_cloud);
            }

            for (int idx = 0; idx < prev_cloud->points.size(); idx++)
            {
                int matching_idx = 0;
                double color_diff = pow(prev_cloud->points[idx].r - kmeans_points[0].r, 2) +
                                    pow(prev_cloud->points[idx].g - kmeans_points[0].g, 2) +
                                    pow(prev_cloud->points[idx].b - kmeans_points[0].b, 2);
                for (int i = 1; i < kmeans_points.size(); i++)
                {
                    double new_color_diff = pow(prev_cloud->points[idx].r - kmeans_points[i].r, 2) +
                                            pow(prev_cloud->points[idx].g - kmeans_points[i].g, 2) +
                                            pow(prev_cloud->points[idx].b - kmeans_points[i].b, 2);
                    if (new_color_diff < color_diff)
                    {
                        color_diff = new_color_diff;
                        matching_idx = i;
                    }
                }
                kmeans_point_clouds[matching_idx]->push_back(prev_cloud->points[idx]);
            }

            kmeans_points.clear();
            for (int i = 0; i < kmeans_point_clouds.size(); i++)
            {
                if (kmeans_point_clouds[i]->points.size() == 0)
                    continue;

                double axis_x = 0, axis_y = 0, axis_z = 0;
                double red = 0, green = 0, blue = 0;

                for (int idx = 0; idx < kmeans_point_clouds[i]->points.size(); idx++)
                {
                    axis_x += kmeans_point_clouds[i]->points[idx].x;
                    axis_y += kmeans_point_clouds[i]->points[idx].y;
                    axis_z += kmeans_point_clouds[i]->points[idx].z;
                    red += kmeans_point_clouds[i]->points[idx].r;
                    green += kmeans_point_clouds[i]->points[idx].g;
                    blue += kmeans_point_clouds[i]->points[idx].b;
                }
                pcl::PointXYZRGB kmeans_point;
                kmeans_point.x = axis_x / kmeans_point_clouds[i]->points.size();
                kmeans_point.y = axis_y / kmeans_point_clouds[i]->points.size();
                kmeans_point.z = axis_z / kmeans_point_clouds[i]->points.size();
                kmeans_point.r = red / kmeans_point_clouds[i]->points.size();
                kmeans_point.g = green / kmeans_point_clouds[i]->points.size();
                kmeans_point.b = blue / kmeans_point_clouds[i]->points.size();
                kmeans_points.push_back(kmeans_point);
            }
        }

        for (int i = 0; i < kmeans_point_clouds.size(); i++)
        {
            pcl::search::KdTree<pcl::PointXYZRGB> kdtree;
            kdtree.setInputCloud(kmeans_point_clouds[i]);
            kmeans_kdtrees.push_back(kdtree);
        }

        // get max_x - min_x in a point cloud to get the voxel size for skelton downsample
        double max_x = -9999999, min_x = 9999999;
        double max_y = -9999999, min_y = 9999999;
        double max_z = -9999999, min_z = 9999999;

        for (int idx = 0; idx < next_cloud->points.size(); idx++)
        {
            if (max_x < next_cloud->points[idx].x)
            {
                max_x = next_cloud->points[idx].x;
            }
            if (max_y < next_cloud->points[idx].y)
            {
                max_y = next_cloud->points[idx].y;
            }
            if (max_z < next_cloud->points[idx].z)
            {
                max_z = next_cloud->points[idx].z;
            }

            if (min_x > next_cloud->points[idx].x)
            {
                min_x = next_cloud->points[idx].x;
            }
            if (min_y > next_cloud->points[idx].y)
            {
                min_y = next_cloud->points[idx].y;
            }
            if (min_z > next_cloud->points[idx].z)
            {
                min_z = next_cloud->points[idx].z;
            }
        }

        double voxel_length = max_x - min_x;
        if (max_y - min_y > voxel_length)
        {
            voxel_length = max_y - min_y;
        }
        if (max_z - min_z > voxel_length)
        {
            voxel_length = max_z - min_z;
        }
        voxel_length = voxel_length / 10;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr skelton_next_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::VoxelGrid<pcl::PointXYZRGB> sor2;
        sor2.setInputCloud(next_cloud);
        sor2.setLeafSize(voxel_length, voxel_length, voxel_length);
        sor2.filter(*skelton_next_cloud);

        // pcl::io::savePLYFileASCII("skelton_prev_cloud.ply", *skelton_prev_cloud);
        // pcl::io::savePLYFileASCII("skelton_next_cloud.ply", *skelton_next_cloud);
        // exit(0);

        // for each point in next_cloud, we will find the nn in skelton_next_cloud;'
        std::vector<int> next_cloud_nn_skelton_idx;
        for (int idx = 0; idx < next_cloud->points.size(); idx++)
        {
            next_cloud_nn_skelton_idx.push_back(-1);
        }
        pcl::search::KdTree<pcl::PointXYZRGB> skleton_next_kdtree;
        skleton_next_kdtree.setInputCloud(skelton_next_cloud);
        for (int idx = 0; idx < next_cloud->points.size(); idx++)
        {
            std::vector<int> nearest_idx;
            std::vector<float> nearest_dist;
            skleton_next_kdtree.nearestKSearch(next_cloud->points[idx], 1, nearest_idx, nearest_dist);
            next_cloud_nn_skelton_idx[idx] = nearest_idx[0];
        }

        // initialize a list with same length of skelton_next_cloud, store the max search distance of each point in skelton_next_cloud
        std::vector<double> next_skelton_search_radius;
        for (int idx = 0; idx < skelton_next_cloud->points.size(); idx++)
        {
            next_skelton_search_radius.push_back(-1);
        }

        // main process

        std::vector<int> match_time; // see if prev cloud index have been matched is_matched[PREV_CLOUD_IDX] = NEXT_CLOUD_IDX
        for (int idx = 0; idx < prev_cloud->points.size() + 2; idx++)
        {
            match_time.push_back(0);
        }

        pcl::search::KdTree<pcl::PointXYZRGB> prev_kdtree;
        prev_kdtree.setInputCloud(prev_cloud);

        pcl::search::KdTree<pcl::PointXYZRGB> next_kdtree;
        next_kdtree.setInputCloud(next_cloud);

        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        // first query nearest neighbor using kmeans result
        std::vector<int> next_nn_idx;
        std::vector<double> next_nn_distance;

        std::vector<pcl::PointXYZ> skelton_point_mv; // average motion vector of skelton point
        for (int i = 0; i < skelton_next_cloud->points.size(); i++)
        {
            pcl::PointXYZ temp;
            temp.x = 0;
            temp.y = 0;
            temp.z = 0;
            skelton_point_mv.push_back(temp);
        }

        for (int idx = 0; idx < next_cloud->points.size(); idx++)
        {
            int matching_idx = 0;
            double color_diff = pow(next_cloud->points[idx].r - kmeans_points[0].r, 2) +
                                pow(next_cloud->points[idx].g - kmeans_points[0].g, 2) +
                                pow(next_cloud->points[idx].b - kmeans_points[0].b, 2);
            for (int i = 1; i < kmeans_points.size(); i++)
            {
                double new_color_diff = pow(next_cloud->points[idx].r - kmeans_points[i].r, 2) +
                                        pow(next_cloud->points[idx].g - kmeans_points[i].g, 2) +
                                        pow(next_cloud->points[idx].b - kmeans_points[i].b, 2);
                if (new_color_diff < color_diff)
                {
                    color_diff = new_color_diff;
                    matching_idx = i;
                }
            }

            std::vector<int> nearest_idx;
            std::vector<float> nearest_dist;
            kmeans_kdtrees[matching_idx].nearestKSearch(next_cloud->points[idx], 1, nearest_idx, nearest_dist);

            // std::vector<int> nearest_idx;
            // std::vector<float> nearest_dist;
            // prev_kdtree.nearestKSearch(next_cloud->points[idx], 1, nearest_idx, nearest_dist);

            next_nn_idx.push_back(nearest_idx[0]);
            next_nn_distance.push_back(nearest_dist[0]);

            if (next_skelton_search_radius[next_cloud_nn_skelton_idx[idx]] < nearest_dist[0])
            {
                next_skelton_search_radius[next_cloud_nn_skelton_idx[idx]] = nearest_dist[0];
            }

            // skelton_point_mv[next_cloud_nn_skelton_idx[nearest_idx2[0]]].x += prev_cloud->points[nearest_idx2[0]].x - next_cloud->points[idx].x;
            // skelton_point_mv[next_cloud_nn_skelton_idx[nearest_idx2[0]]].y += prev_cloud->points[nearest_idx2[0]].y - next_cloud->points[idx].y;
            // skelton_point_mv[next_cloud_nn_skelton_idx[nearest_idx2[0]]].z += prev_cloud->points[nearest_idx2[0]].z - next_cloud->points[idx].z;
        }

        std::vector<double> nn_distance;

        nn_distance = next_nn_distance;
        // initialize original index locations
        std::vector<int> sort_idx(nn_distance.size());
        iota(sort_idx.begin(), sort_idx.end(), 0);

        // sort indexes based on comparing values in v
        // using std::stable_sort instead of std::sort
        // to avoid unnecessary index re-orderings
        // when v contains elements of equal values
        stable_sort(sort_idx.begin(), sort_idx.end(),
                    [&nn_distance](int i1, int i2)
                    { return nn_distance[i1] < nn_distance[i2]; });

        // find matching within a radius
        // std::vector<MyMatching> query_result;

        for (auto idx : sort_idx)
        {
            MyMatching temp;
            std::vector<int> prev_pointIdxRadiusSearch;
            std::vector<float> prev_pointRadiusSquaredDistance;

            // float search_radius = nn_distance[idx];
            float search_radius = next_skelton_search_radius[next_cloud_nn_skelton_idx[idx]];
            // magnification
            prev_kdtree.radiusSearchT(next_cloud->points[idx], double(magnification * search_radius + 1), prev_pointIdxRadiusSearch, prev_pointRadiusSquaredDistance);

            for (int i = 0; i < prev_pointIdxRadiusSearch.size(); i++)
            {
                if (abs(next_cloud->points[idx].r - prev_cloud->points[prev_pointIdxRadiusSearch[i]].r) < color_dist &&
                    abs(next_cloud->points[idx].g - prev_cloud->points[prev_pointIdxRadiusSearch[i]].g) < color_dist &&
                    abs(next_cloud->points[idx].b - prev_cloud->points[prev_pointIdxRadiusSearch[i]].b) < color_dist)
                {
                    temp.match_idx_list.push_back(prev_pointIdxRadiusSearch[i]);
                    temp.match_dist_list.push_back(prev_pointRadiusSquaredDistance[i]);
                }
            }
            // std::cout << temp.match_idx_list.size() << std::endl;
            if (temp.match_idx_list.size() == 0)
            {
                prev_kdtree.nearestKSearch(next_cloud->points[idx], 1, prev_pointIdxRadiusSearch, prev_pointRadiusSquaredDistance);
                temp.match_idx_list.push_back(prev_pointIdxRadiusSearch[0]);
                temp.match_dist_list.push_back(prev_pointRadiusSquaredDistance[0]);
            }

            double min_distance = -1;
            int matching_idx = -1;

            for (int i = 0; i < temp.match_idx_list.size(); i++)
            {
                double cur_color_dist = sqrt(pow(((int)prev_cloud->points[temp.match_idx_list[i]].r - next_cloud->points[idx].r), 2) +
                                             pow(((int)prev_cloud->points[temp.match_idx_list[i]].g - next_cloud->points[idx].g), 2) +
                                             pow(((int)prev_cloud->points[temp.match_idx_list[i]].b - next_cloud->points[idx].b), 2));
                double new_gamma = 0.00721688;

                double cur_total_dist = 0.000511936 * sqrt(temp.match_dist_list[i]) +
                                        0.000225527 * cur_color_dist +
                                        new_gamma * match_time[temp.match_idx_list[i]];
                // double cur_total_dist = 0.4 * sqrt(temp.match_dist_list[i]) / nn_distance[idx] + 0.2 * match_time[temp.match_idx_list[i]];

                if (min_distance > cur_total_dist || min_distance < 0)
                {
                    min_distance = cur_total_dist;
                    matching_idx = temp.match_idx_list[i];
                }
            }

            if (matching_idx != -1)
                match_time[matching_idx] += 1;

            pipeline_obj.matching_table[idx] = matching_idx;
        }

        // convert the matching list back to next match to prev
        std::vector<int> convert_list;
        if (!use_next_search_prev)
        {
            for (int idx = 0; idx < ds_next_cloud->points.size(); idx++)
            {
                convert_list.push_back(-1);
            }

            for (int i = 0; i < ds_next_cloud->points.size(); i++)
            {
                if (pipeline_obj.matching_table[i] < ds_next_cloud->points.size())
                {
                    convert_list[pipeline_obj.matching_table[i]] = i;
                }
            }
            // return 0;
            pipeline_obj.matching_table.clear();

            for (int idx = 0; idx < ds_next_cloud->points.size(); idx++)
            {
                // std::cout << convert_list[idx] << std::endl;
                if (convert_list[idx] < ds_prev_cloud->points.size())
                {
                    pipeline_obj.matching_table.push_back(convert_list[idx]);
                }
                else
                {
                    pipeline_obj.matching_table.push_back(-1);
                    // std::cout << "!!!!!";
                }
            }

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr has_matching_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            std::vector<int> has_matching_point_matching_list;
            for (int idx = 0; idx < pipeline_obj.matching_table.size(); idx++)
            {
                if (pipeline_obj.matching_table[idx] != -1)
                {
                    has_matching_pointcloud->push_back(ds_next_cloud->points[idx]);
                    has_matching_point_matching_list.push_back(pipeline_obj.matching_table[idx]);
                }
            }

            pcl::search::KdTree<pcl::PointXYZRGB> has_matching_kdtree;
            has_matching_kdtree.setInputCloud(has_matching_pointcloud);

            for (int idx = 0; idx < pipeline_obj.matching_table.size(); idx++)
            {
                if (pipeline_obj.matching_table[idx] == -1)
                {
                    std::vector<int> nearest_idx;
                    std::vector<float> nearest_dist;
                    has_matching_kdtree.nearestKSearch(ds_next_cloud->points[idx], 1, nearest_idx, nearest_dist);
                    pipeline_obj.matching_table[idx] = has_matching_point_matching_list[nearest_idx[0]];
                }
            }
        }
    }

    if (_next != NULL)
    {
        _next->run(pipeline_obj);
    }
}

Stage2_Query_Radius::Stage2_Query_Radius(int my_mode, double my_radius) : mode(my_mode), radius(my_radius)
{
}

void Stage2_Query_Radius::run(Pipeline_Object &pipeline_obj)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds_prev_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds_next_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (mode == 1)
    {
        ds_prev_cloud = pipeline_obj.ds1_prev_cloud;
        ds_next_cloud = pipeline_obj.ds1_next_cloud;
    }
    else if (mode == 2)
    {
        ds_prev_cloud = pipeline_obj.ds2_prev_cloud;
        ds_next_cloud = pipeline_obj.ds2_next_cloud;
    }

    double utility_alpha = 0.000511936;
    double utility_beta = 0.000225527;
    double utility_gamma = 0.00721688;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr prev_cloud_rescale(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr next_cloud_rescale(new pcl::PointCloud<pcl::PointXYZRGB>);

    prev_cloud_rescale->points = ds_prev_cloud->points;
    next_cloud_rescale->points = ds_next_cloud->points;

    for (int idx = 0; idx < prev_cloud_rescale->points.size(); idx++)
    {
        prev_cloud_rescale->points[idx].r = (int)(1.0 * (utility_beta / utility_alpha) * (int)prev_cloud_rescale->points[idx].r);
        prev_cloud_rescale->points[idx].g = (int)(1.0 * (utility_beta / utility_alpha) * (int)prev_cloud_rescale->points[idx].g);
        prev_cloud_rescale->points[idx].b = (int)(1.0 * (utility_beta / utility_alpha) * (int)prev_cloud_rescale->points[idx].b);
    }

    for (int idx = 0; idx < next_cloud_rescale->points.size(); idx++)
    {
        next_cloud_rescale->points[idx].r = (int)(1.0 * (utility_beta / utility_alpha) * (int)next_cloud_rescale->points[idx].r);
        next_cloud_rescale->points[idx].g = (int)(1.0 * (utility_beta / utility_alpha) * (int)next_cloud_rescale->points[idx].g);
        next_cloud_rescale->points[idx].b = (int)(1.0 * (utility_beta / utility_alpha) * (int)next_cloud_rescale->points[idx].b);
    }

    ds_prev_cloud = prev_cloud_rescale;
    ds_next_cloud = next_cloud_rescale;

    pcl::search::KdTree<pcl::PointXYZRGB> prev_kdtree;
    prev_kdtree.setInputCloud(ds_prev_cloud);

    std::vector<int> matched_times;
    for (int idx = 0; idx < ds_prev_cloud->points.size() + 2; idx++)
    {
        matched_times.push_back(0);
    }

    // will save downsample point cloud match to another downsample point cloud
    for (int idx = 0; idx < ds_next_cloud->points.size() + 2; idx++)
    {
        pipeline_obj.matching_table.push_back(-1);
    }

    for (int idx = 0; idx < ds_next_cloud->points.size() + 2; idx++)
    {
        std::vector<int> prev_pointIdxRadiusSearch;
        std::vector<float> prev_pointRadiusSquaredDistance;

        prev_kdtree.radiusSearchT(ds_next_cloud->points[idx], radius + 0.01f, prev_pointIdxRadiusSearch, prev_pointRadiusSquaredDistance);

        if (prev_pointIdxRadiusSearch.size() == 0)
        { // just use "searchPoint" to avoid access struct again
            pipeline_obj.matching_table[idx] = -1;
            continue;
        }

        double total_dist = -1.0;
        int match_point_index = prev_pointIdxRadiusSearch[0];
        pipeline_obj.matching_table[idx] = prev_pointIdxRadiusSearch[0];

        for (int i = 0; i < prev_pointIdxRadiusSearch.size(); i++)
        {
            // std::cout << (int)searchPoint.r << "   " << (int)(*prev_cloud)[prev_pointIdxRadiusSearch[i]].r << "  " << ((int)searchPoint.r - (int)(*prev_cloud)[prev_pointIdxRadiusSearch[i]].r) * ((int)searchPoint.r - (int)(*prev_cloud)[prev_pointIdxRadiusSearch[i]].r) << std::endl;
            // std::cout << (int)searchPoint.g << "   " << (int)(*prev_cloud)[prev_pointIdxRadiusSearch[i]].g << "  " << ((int)searchPoint.g - (int)(*prev_cloud)[prev_pointIdxRadiusSearch[i]].g) * ((int)searchPoint.g - (int)(*prev_cloud)[prev_pointIdxRadiusSearch[i]].g) << std::endl;
            // std::cout << (int)searchPoint.b << "   " << (int)(*prev_cloud)[prev_pointIdxRadiusSearch[i]].b << "  " << ((int)searchPoint.b - (int)(*prev_cloud)[prev_pointIdxRadiusSearch[i]].b) * ((int)searchPoint.b - (int)(*prev_cloud)[prev_pointIdxRadiusSearch[i]].b) << std::endl;
            double cur_color_dist = sqrt(((int)ds_next_cloud->points[idx].r - (int)(*ds_prev_cloud)[prev_pointIdxRadiusSearch[i]].r) * ((int)ds_next_cloud->points[idx].r - (int)(*ds_prev_cloud)[prev_pointIdxRadiusSearch[i]].r) +
                                         ((int)ds_next_cloud->points[idx].g - (int)(*ds_prev_cloud)[prev_pointIdxRadiusSearch[i]].g) * ((int)ds_next_cloud->points[idx].g - (int)(*ds_prev_cloud)[prev_pointIdxRadiusSearch[i]].g) +
                                         ((int)ds_next_cloud->points[idx].b - (int)(*ds_prev_cloud)[prev_pointIdxRadiusSearch[i]].b) * ((int)ds_next_cloud->points[idx].b - (int)(*ds_prev_cloud)[prev_pointIdxRadiusSearch[i]].b));
            double cur_total_dist = utility_alpha * sqrt(prev_pointRadiusSquaredDistance[i]) + utility_beta * cur_color_dist + utility_gamma * matched_times[prev_pointIdxRadiusSearch[i]];
            // utility_alpha * sqrt(prev_pointRadiusSquaredDistance[i]) + utility_beta * cur_color_dist + utility_gamma * matched_times[prev_pointIdxRadiusSearch[i]]

            // std::cout << cur_color_dist << std::endl
            //           << std::endl;

            if (total_dist < 0 || total_dist - cur_total_dist > 0.0001)
            {
                total_dist = cur_total_dist;
                pipeline_obj.matching_table[idx] = prev_pointIdxRadiusSearch[i];
            }
            else if (abs(total_dist - cur_total_dist) < 0.0001 && match_point_index > prev_pointIdxRadiusSearch[i])
            {
                pipeline_obj.matching_table[idx] = prev_pointIdxRadiusSearch[i];
            }
        }
        matched_times[pipeline_obj.matching_table[idx]] += 1;
    }

    if (_next != NULL)
    {
        _next->run(pipeline_obj);
    }
}
