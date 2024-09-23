// Prediction
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
#include "Stage4.hpp"

Stage4_Point_Based::Stage4_Point_Based(int my_mode) : mode(my_mode)
{
}

void Stage4_Point_Based::run(Pipeline_Object &pipeline_obj)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr prev_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr next_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (mode == 1)
    {
        prev_cloud = pipeline_obj.ds1_prev_cloud;
        next_cloud = pipeline_obj.ds1_next_cloud;
    }
    else if (mode == 2)
    {
        prev_cloud = pipeline_obj.ds2_prev_cloud;
        next_cloud = pipeline_obj.ds2_next_cloud;
    }

    double interpolate_a = pipeline_obj.temporal_index;
    double interpolate_b = 1 - pipeline_obj.temporal_index;

    for (int idx = 0; idx < next_cloud->points.size(); ++idx)
    {
        pcl::PointXYZRGB interpolate_point;
        interpolate_point.x = (int)next_cloud->points[idx].x + pipeline_obj.temporal_index * pipeline_obj.motion_estimation[idx][0];
        interpolate_point.y = (int)next_cloud->points[idx].y + pipeline_obj.temporal_index * pipeline_obj.motion_estimation[idx][1];
        interpolate_point.z = (int)next_cloud->points[idx].z + pipeline_obj.temporal_index * pipeline_obj.motion_estimation[idx][2];
        interpolate_point.r = (int)next_cloud->points[idx].r + pipeline_obj.temporal_index * pipeline_obj.motion_estimation[idx][3];
        interpolate_point.g = (int)next_cloud->points[idx].g + pipeline_obj.temporal_index * pipeline_obj.motion_estimation[idx][4];
        interpolate_point.b = (int)next_cloud->points[idx].b + pipeline_obj.temporal_index * pipeline_obj.motion_estimation[idx][5];
        pipeline_obj.predict_point_cloud->push_back(interpolate_point);
    }

    if (_next != NULL)
    {
        _next->run(pipeline_obj);
    }
}

Stage4_Cube_Based::Stage4_Cube_Based(int my_mode, int my_iterate_round) : mode(my_mode), iterate_round(my_iterate_round)
{
}

void Stage4_Cube_Based::run(Pipeline_Object &pipeline_obj)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr prev_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr next_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (mode == 1)
    {
        prev_cloud = pipeline_obj.ds1_prev_cloud;
        next_cloud = pipeline_obj.ds1_next_cloud;
    }
    else if (mode == 2)
    {
        prev_cloud = pipeline_obj.ds2_prev_cloud;
        next_cloud = pipeline_obj.ds2_next_cloud;
    }

    std::map<MyPoint, std::vector<int>, MyPointCompare> ds_block_list; // key: point xyz, value: query point index

    for (int x = pipeline_obj.cube_length / 2; x < pipeline_obj.resolution; x += pipeline_obj.cube_length)
    {
        for (int y = pipeline_obj.cube_length / 2; y < pipeline_obj.resolution; y += pipeline_obj.cube_length)
        {
            for (int z = pipeline_obj.cube_length / 2; z < pipeline_obj.resolution; z += pipeline_obj.cube_length)
            {
                ds_block_list[MyPoint(x, y, z)] = std::vector<int>();
            }
        }
    }

    for (int w = 0; w < next_cloud->points.size(); w++)
    {
        ds_block_list[MyPoint(find_center(next_cloud->points[w].x, pipeline_obj.cube_length), find_center(next_cloud->points[w].y, pipeline_obj.cube_length), find_center(next_cloud->points[w].z, pipeline_obj.cube_length))].push_back(w);
    }

    std::vector<MyPoint> non_empty_block_center; // save those center points which query points > 10

    for (int x = pipeline_obj.cube_length / 2; x < pipeline_obj.resolution; x += pipeline_obj.cube_length)
    {
        for (int y = pipeline_obj.cube_length / 2; y < pipeline_obj.resolution; y += pipeline_obj.cube_length)
        {
            for (int z = pipeline_obj.cube_length / 2; z < pipeline_obj.resolution; z += pipeline_obj.cube_length)
            {
                if (ds_block_list[MyPoint(x, y, z)].size() == 0)
                    continue;

                non_empty_block_center.push_back(MyPoint(x, y, z));
            }
        }
    }

    // block_list will use in prediction
    std::map<MyPoint, std::vector<int>, MyPointCompare> block_list; // key: point xyz, value: query point index

    for (int x = pipeline_obj.cube_length / 2; x < pipeline_obj.resolution; x += pipeline_obj.cube_length)
    {
        for (int y = pipeline_obj.cube_length / 2; y < pipeline_obj.resolution; y += pipeline_obj.cube_length)
        {
            for (int z = pipeline_obj.cube_length / 2; z < pipeline_obj.resolution; z += pipeline_obj.cube_length)
            {
                block_list[MyPoint(x, y, z)] = std::vector<int>();
            }
        }
    }

    for (int w = 0; w < next_cloud->points.size(); w++)
    {
        block_list[MyPoint(find_center(next_cloud->points[w].x, pipeline_obj.cube_length), find_center(next_cloud->points[w].y, pipeline_obj.cube_length), find_center(next_cloud->points[w].z, pipeline_obj.cube_length))].push_back(w);
    }

    int round = 0;
    std::vector<double> old_length_multiplier;
    for (int i = 0; i < non_empty_block_center.size(); i++)
    {
        old_length_multiplier.push_back(1.0);
    }

    for (; round < iterate_round; round++)
    {
        // check original(before motion) non empty block center point's adjacent situation
        std::vector<std::vector<int>> origin_adjacent_center;

        for (int i = 0; i < non_empty_block_center.size(); i++)
        {
            std::vector<int> temp;
            for (int j = 0; j < non_empty_block_center.size(); j++)
            {
                if ((non_empty_block_center[i].x + (pipeline_obj.cube_length + 1) > non_empty_block_center[j].x && non_empty_block_center[i].x - (pipeline_obj.cube_length + 1) < non_empty_block_center[j].x) &&
                    (non_empty_block_center[i].y + (pipeline_obj.cube_length + 1) > non_empty_block_center[j].y && non_empty_block_center[i].y - (pipeline_obj.cube_length + 1) < non_empty_block_center[j].y) &&
                    (non_empty_block_center[i].z + (pipeline_obj.cube_length + 1) > non_empty_block_center[j].z && non_empty_block_center[i].z - (pipeline_obj.cube_length + 1) < non_empty_block_center[j].z))
                {
                    temp.push_back(j);
                }
            }
            origin_adjacent_center.push_back(temp);
        }

        // check result(after motion) non empty block center point's adjacent situation
        std::vector<std::vector<int>> result_adjacent_center;
        for (int i = 0; i < non_empty_block_center.size(); i++)
        {
            std::vector<int> temp;
            double curr_x = non_empty_block_center[i].x + pipeline_obj.non_empty_block_vector[i][0];
            double curr_y = non_empty_block_center[i].y + pipeline_obj.non_empty_block_vector[i][1];
            double curr_z = non_empty_block_center[i].z + pipeline_obj.non_empty_block_vector[i][2];

            for (int j = 0; j < non_empty_block_center.size(); j++)
            {
                double ref_x = non_empty_block_center[j].x + pipeline_obj.non_empty_block_vector[j][0];
                double ref_y = non_empty_block_center[j].y + pipeline_obj.non_empty_block_vector[j][1];
                double ref_z = non_empty_block_center[j].z + pipeline_obj.non_empty_block_vector[j][2];
                // std::cout << "------------" << std::endl;
                // std::cout << curr_x << " " << curr_y << " " << curr_z << std::endl;
                // std::cout << ref_x << " " << ref_y << " " << ref_z << std::endl;
                if ((curr_x + (pipeline_obj.cube_length + 1) > ref_x && curr_x - (pipeline_obj.cube_length + 1) < ref_x) &&
                    (curr_y + (pipeline_obj.cube_length + 1) > ref_y && curr_y - (pipeline_obj.cube_length + 1) < ref_y) &&
                    (curr_z + (pipeline_obj.cube_length + 1) > ref_z && curr_z - (pipeline_obj.cube_length + 1) < ref_z))
                {
                    temp.push_back(j);
                }
            }
            result_adjacent_center.push_back(temp);
        }

        // get the index from large to small of the vector magnitude
        std::vector<int> sort_idx = sort_indexes(pipeline_obj.non_empty_block_vector);

        // compare before and after center point adjacent situation and get multiplier
        std::vector<double> length_multiplier(sort_idx.size());
        std::set<std::pair<int, int>> explored_pair;
        bool is_change = false;

        for (int i = 0; i < sort_idx.size(); i++)
        {
            std::vector<int> disappear_idx;
            // std::cout << 2 << std::endl;
            for (int j = 0; j < origin_adjacent_center[sort_idx[i]].size(); j++)
            {
                if (std::find(result_adjacent_center[sort_idx[i]].begin(), result_adjacent_center[sort_idx[i]].end(), origin_adjacent_center[sort_idx[i]][j]) == result_adjacent_center[sort_idx[i]].end())
                {
                    if (sort_idx[i] > origin_adjacent_center[sort_idx[i]][j] && explored_pair.find(std::make_pair(origin_adjacent_center[sort_idx[i]][j], sort_idx[i])) == explored_pair.end())
                    {
                        disappear_idx.push_back(origin_adjacent_center[sort_idx[i]][j]);
                    }
                    else if (explored_pair.find(std::make_pair(sort_idx[i], origin_adjacent_center[sort_idx[i]][j])) == explored_pair.end())
                    {
                        disappear_idx.push_back(origin_adjacent_center[sort_idx[i]][j]);
                    }
                }
            }
            if (disappear_idx.size() == 0)
            {
                length_multiplier[sort_idx[i]] = (old_length_multiplier[sort_idx[i]]);
                continue;
            }
            is_change = true;
            // std::cout << sort_idx[i] << " " << non_empty_block_center.size() << std::endl;

            double max_distance = abs((non_empty_block_center[sort_idx[i]].x + pipeline_obj.non_empty_block_vector[sort_idx[i]][0]) - (non_empty_block_center[disappear_idx[0]].x + pipeline_obj.non_empty_block_vector[disappear_idx[0]][0]));
            for (int j = 0; j < disappear_idx.size(); j++)
            {
                // std::cout << pipeline_obj.non_empty_block_vector[disappear_idx[j]].size() << " " << non_empty_block_center.size() << std::endl;
                if (max_distance < abs((non_empty_block_center[sort_idx[i]].x + pipeline_obj.non_empty_block_vector[sort_idx[i]][0]) - (non_empty_block_center[disappear_idx[j]].x + pipeline_obj.non_empty_block_vector[disappear_idx[j]][0])))
                    max_distance = abs((non_empty_block_center[sort_idx[i]].x + pipeline_obj.non_empty_block_vector[sort_idx[i]][0]) - (non_empty_block_center[disappear_idx[j]].x + pipeline_obj.non_empty_block_vector[disappear_idx[j]][0]));
                if (max_distance < abs((non_empty_block_center[sort_idx[i]].y + pipeline_obj.non_empty_block_vector[sort_idx[i]][1]) - (non_empty_block_center[disappear_idx[j]].y + pipeline_obj.non_empty_block_vector[disappear_idx[j]][1])))
                    max_distance = abs((non_empty_block_center[sort_idx[i]].y + pipeline_obj.non_empty_block_vector[sort_idx[i]][1]) - (non_empty_block_center[disappear_idx[j]].y + pipeline_obj.non_empty_block_vector[disappear_idx[j]][1]));
                if (max_distance < abs((non_empty_block_center[sort_idx[i]].z + pipeline_obj.non_empty_block_vector[sort_idx[i]][2]) - (non_empty_block_center[disappear_idx[j]].z + pipeline_obj.non_empty_block_vector[disappear_idx[j]][2])))
                    max_distance = abs((non_empty_block_center[sort_idx[i]].z + pipeline_obj.non_empty_block_vector[sort_idx[i]][2]) - (non_empty_block_center[disappear_idx[j]].z + pipeline_obj.non_empty_block_vector[disappear_idx[j]][2]));
            }
            // std::cout << 4 << std::endl;
            // std::cout << max_distance / pipeline_obj.cube_length << " " << old_length_multiplier[sort_idx[i]] << std::endl;
            if ((max_distance / pipeline_obj.cube_length) > old_length_multiplier[sort_idx[i]])
            {
                length_multiplier[sort_idx[i]] = (max_distance / pipeline_obj.cube_length);
            }
            else
            {
                length_multiplier[sort_idx[i]] = (old_length_multiplier[sort_idx[i]]);
            }

            for (int j = 0; j < disappear_idx.size(); j++)
            {
                if (sort_idx[i] > disappear_idx[j])
                {
                    explored_pair.insert(std::make_pair(disappear_idx[j], sort_idx[i]));
                }
                else
                {
                    explored_pair.insert(std::make_pair(sort_idx[i], disappear_idx[j]));
                }
            }
        }

        // check if length_multiplier is same as last turn

        bool multiplier_same = true;
        for (int i = 0; i < length_multiplier.size(); i++)
        {
            if (abs(old_length_multiplier[i] - length_multiplier[i]) > 0.0001)
            {
                multiplier_same = false;
                break;
            }
        }
        if (multiplier_same && round != 0)
        {
            break;
        }
        else
        {
            old_length_multiplier.clear();
            for (int i = 0; i < length_multiplier.size(); i++)
            {
                old_length_multiplier.push_back(length_multiplier[i]);
            }
        }

        // do cheb again
        pipeline_obj.predict_point_cloud->points.clear();

        for (int i = 0; i < non_empty_block_center.size(); i++)
        {
            // std::cout << block_list[MyPoint(x, y, z)].size() << std::endl;
            if (block_list[non_empty_block_center[i]].size() == 0)
                continue;
            double denominator = 0;
            double vector_x = 0.0, vector_y = 0.0, vector_z = 0.0, vector_r = 0.0, vector_g = 0.0, vector_b = 0.0;
            for (int idx = 0; idx < block_list[non_empty_block_center[i]].size(); idx++)
            {
                std::vector<int> prev_pointIdxRadiusSearch;
                std::vector<float> prev_pointRadiusSquaredDistance;

                prev_pointIdxRadiusSearch.push_back(pipeline_obj.matching_table[block_list[non_empty_block_center[i]][idx]]);

                double distance_reciprocal = 1.0 / (sqrt(pow(next_cloud->points[block_list[non_empty_block_center[i]][idx]].x - prev_cloud->points[prev_pointIdxRadiusSearch[0]].x, 2) +
                                                         pow(next_cloud->points[block_list[non_empty_block_center[i]][idx]].y - prev_cloud->points[prev_pointIdxRadiusSearch[0]].y, 2) +
                                                         pow(next_cloud->points[block_list[non_empty_block_center[i]][idx]].z - prev_cloud->points[prev_pointIdxRadiusSearch[0]].z, 2)) +
                                                    pipeline_obj.cube_length / 2);
                distance_reciprocal = 1;
                denominator += distance_reciprocal;

                vector_x += distance_reciprocal * (next_cloud->points[block_list[non_empty_block_center[i]][idx]].x - (pipeline_obj.temporal_index * next_cloud->points[block_list[non_empty_block_center[i]][idx]].x + (1 - pipeline_obj.temporal_index) * prev_cloud->points[prev_pointIdxRadiusSearch[0]].x));
                vector_y += distance_reciprocal * (next_cloud->points[block_list[non_empty_block_center[i]][idx]].y - (pipeline_obj.temporal_index * next_cloud->points[block_list[non_empty_block_center[i]][idx]].y + (1 - pipeline_obj.temporal_index) * prev_cloud->points[prev_pointIdxRadiusSearch[0]].y));
                vector_z += distance_reciprocal * (next_cloud->points[block_list[non_empty_block_center[i]][idx]].z - (pipeline_obj.temporal_index * next_cloud->points[block_list[non_empty_block_center[i]][idx]].z + (1 - pipeline_obj.temporal_index) * prev_cloud->points[prev_pointIdxRadiusSearch[0]].z));
            }

            vector_x = vector_x / block_list[non_empty_block_center[i]].size();
            vector_y = vector_y / block_list[non_empty_block_center[i]].size();
            vector_z = vector_z / block_list[non_empty_block_center[i]].size();
            vector_r = vector_r / block_list[non_empty_block_center[i]].size();
            vector_g = vector_g / block_list[non_empty_block_center[i]].size();
            vector_b = vector_b / block_list[non_empty_block_center[i]].size();

            for (int idx = 0; idx < block_list[non_empty_block_center[i]].size(); idx++)
            {
                pcl::PointXYZRGB interpolate_point;
                interpolate_point.x = (int)next_cloud->points[block_list[non_empty_block_center[i]][idx]].x - vector_x;
                interpolate_point.y = (int)next_cloud->points[block_list[non_empty_block_center[i]][idx]].y - vector_y;
                interpolate_point.z = (int)next_cloud->points[block_list[non_empty_block_center[i]][idx]].z - vector_z;
                interpolate_point.r = (int)next_cloud->points[block_list[non_empty_block_center[i]][idx]].r - vector_r;
                interpolate_point.g = (int)next_cloud->points[block_list[non_empty_block_center[i]][idx]].g - vector_g;
                interpolate_point.b = (int)next_cloud->points[block_list[non_empty_block_center[i]][idx]].b - vector_b;
                pipeline_obj.predict_point_cloud->push_back(interpolate_point);
            }

            std::vector<double> temp;
            temp.push_back(vector_x);
            temp.push_back(vector_y);
            temp.push_back(vector_z);
            temp.push_back(vector_r);
            temp.push_back(vector_g);
            temp.push_back(vector_b);

            pipeline_obj.non_empty_block_vector.push_back(temp);
        }

        // std::cout << output_filename.substr(0, output_filename.size() - 4) + "_" + std::string(number) + ".ply" << std::endl;
        // pcl::io::savePLYFileASCII(output_filename.substr(0, output_filename.size() - 4) + "_" + std::string(number) + ".ply", *pipeline_obj.predict_point_cloud);
        if (round == iterate_round - 1)
        {
            break;
        }

        if (!is_change)
        {
            break;
        }
    }

    if (_next != NULL)
    {
        _next->run(pipeline_obj);
    }
}

Stage4_Neighboring_Based::Stage4_Neighboring_Based(int my_mode) : mode(my_mode)
{
}

void Stage4_Neighboring_Based::run(Pipeline_Object &pipeline_obj)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr prev_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr next_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (mode == 1)
    {
        prev_cloud = pipeline_obj.ds1_prev_cloud;
        next_cloud = pipeline_obj.ds1_next_cloud;
    }
    else if (mode == 2)
    {
        prev_cloud = pipeline_obj.ds2_prev_cloud;
        next_cloud = pipeline_obj.ds2_next_cloud;
    }

    std::map<MyPoint, std::vector<int>, MyPointCompare> ds_block_list; // key: point xyz, value: query point index

    for (int x = pipeline_obj.cube_length / 2; x < pipeline_obj.resolution; x += pipeline_obj.cube_length)
    {
        for (int y = pipeline_obj.cube_length / 2; y < pipeline_obj.resolution; y += pipeline_obj.cube_length)
        {
            for (int z = pipeline_obj.cube_length / 2; z < pipeline_obj.resolution; z += pipeline_obj.cube_length)
            {
                ds_block_list[MyPoint(x, y, z)] = std::vector<int>();
            }
        }
    }

    for (int w = 0; w < next_cloud->points.size(); w++)
    {
        ds_block_list[MyPoint(find_center(next_cloud->points[w].x, pipeline_obj.cube_length), find_center(next_cloud->points[w].y, pipeline_obj.cube_length), find_center(next_cloud->points[w].z, pipeline_obj.cube_length))].push_back(w);
    }

    std::vector<MyPoint> non_empty_block_center; // save those center points which query points > 10

    for (int x = pipeline_obj.cube_length / 2; x < pipeline_obj.resolution; x += pipeline_obj.cube_length)
    {
        for (int y = pipeline_obj.cube_length / 2; y < pipeline_obj.resolution; y += pipeline_obj.cube_length)
        {
            for (int z = pipeline_obj.cube_length / 2; z < pipeline_obj.resolution; z += pipeline_obj.cube_length)
            {
                if (ds_block_list[MyPoint(x, y, z)].size() == 0)
                    continue;

                non_empty_block_center.push_back(MyPoint(x, y, z));
            }
        }
    }

    // block_list will use in prediction
    std::map<MyPoint, std::vector<int>, MyPointCompare> block_list; // key: point xyz, value: query point index

    for (int x = pipeline_obj.cube_length / 2; x < pipeline_obj.resolution; x += pipeline_obj.cube_length)
    {
        for (int y = pipeline_obj.cube_length / 2; y < pipeline_obj.resolution; y += pipeline_obj.cube_length)
        {
            for (int z = pipeline_obj.cube_length / 2; z < pipeline_obj.resolution; z += pipeline_obj.cube_length)
            {
                block_list[MyPoint(x, y, z)] = std::vector<int>();
            }
        }
    }

    for (int w = 0; w < next_cloud->points.size(); w++)
    {
        block_list[MyPoint(find_center(next_cloud->points[w].x, pipeline_obj.cube_length), find_center(next_cloud->points[w].y, pipeline_obj.cube_length), find_center(next_cloud->points[w].z, pipeline_obj.cube_length))].push_back(w);
    }

    std::vector<std::vector<int>> origin_adjacent_center;
    for (int i = 0; i < non_empty_block_center.size(); i++)
    {
        std::vector<int> temp;

        // first push the block itself into the vector
        temp.push_back(i);
        for (int j = 0; j < non_empty_block_center.size(); j++)
        {
            if ((non_empty_block_center[i].x + (pipeline_obj.cube_length + 1) > non_empty_block_center[j].x && non_empty_block_center[i].x - (pipeline_obj.cube_length + 1) < non_empty_block_center[j].x) &&
                (non_empty_block_center[i].y + (pipeline_obj.cube_length + 1) > non_empty_block_center[j].y && non_empty_block_center[i].y - (pipeline_obj.cube_length + 1) < non_empty_block_center[j].y) &&
                (non_empty_block_center[i].z + (pipeline_obj.cube_length + 1) > non_empty_block_center[j].z && non_empty_block_center[i].z - (pipeline_obj.cube_length + 1) < non_empty_block_center[j].z))
            {
                temp.push_back(j);
            }
        }
        origin_adjacent_center.push_back(temp);
    }

    for (int i = 0; i < non_empty_block_center.size(); i++)
    {

        int x = non_empty_block_center[i].x;
        int y = non_empty_block_center[i].y;
        int z = non_empty_block_center[i].z;

        for (int idx = 0; idx < block_list[MyPoint(x, y, z)].size(); idx++)
        {
            // below two should be same size, the final motion should be "sum of each element of two vector muitiply together" and "divide by sum of coe"
            std::vector<double> coe_list;                 // size at most 27
            std::vector<std::vector<double>> motion_list; // size at most 27

            double center_x = 0;
            double center_y = 0;
            double center_z = 0;

            for (int j = 0; j < origin_adjacent_center[i].size(); j++)
            {
                // calculate coe
                double volumn;

                volumn = abs((non_empty_block_center[origin_adjacent_center[i][j]].x - next_cloud->points[block_list[MyPoint(x, y, z)][idx]].x) *
                             (non_empty_block_center[origin_adjacent_center[i][j]].y - next_cloud->points[block_list[MyPoint(x, y, z)][idx]].y) *
                             (non_empty_block_center[origin_adjacent_center[i][j]].z - next_cloud->points[block_list[MyPoint(x, y, z)][idx]].z));

                if (volumn != 0)
                {
                    coe_list.push_back(1.0 / volumn);
                }
                else
                {
                    coe_list.push_back(999999.0);
                }

                motion_list.push_back(pipeline_obj.non_empty_block_vector[origin_adjacent_center[i][j]]);
            }

            // calcuate sum of coe_list
            double sum = 0;
            for (int j = 0; j < coe_list.size(); j++)
            {
                sum += coe_list[j];
            }

            pcl::PointXYZRGB interpolate_point;
            double molecular_x = 0;
            for (int j = 0; j < coe_list.size(); j++)
            {
                molecular_x += coe_list[j] * motion_list[j][0];
            }

            double molecular_y = 0;
            for (int j = 0; j < coe_list.size(); j++)
            {
                molecular_y += coe_list[j] * motion_list[j][1];
            }

            double molecular_z = 0;
            for (int j = 0; j < coe_list.size(); j++)
            {
                molecular_z += coe_list[j] * motion_list[j][2];
            }

            interpolate_point.x = (int)next_cloud->points[block_list[MyPoint(x, y, z)][idx]].x - molecular_x / sum;
            interpolate_point.y = (int)next_cloud->points[block_list[MyPoint(x, y, z)][idx]].y - molecular_y / sum;
            interpolate_point.z = (int)next_cloud->points[block_list[MyPoint(x, y, z)][idx]].z - molecular_z / sum;
            interpolate_point.r = (int)next_cloud->points[block_list[MyPoint(x, y, z)][idx]].r - pipeline_obj.non_empty_block_vector[i][3];
            interpolate_point.g = (int)next_cloud->points[block_list[MyPoint(x, y, z)][idx]].g - pipeline_obj.non_empty_block_vector[i][4];
            interpolate_point.b = (int)next_cloud->points[block_list[MyPoint(x, y, z)][idx]].b - pipeline_obj.non_empty_block_vector[i][5];
            if (interpolate_point.x == interpolate_point.x || interpolate_point.y == interpolate_point.y || interpolate_point.z == interpolate_point.z)
            { // test if nan
                pipeline_obj.predict_point_cloud->push_back(interpolate_point);
            }
            else
            {
                interpolate_point.x = (int)next_cloud->points[block_list[MyPoint(x, y, z)][idx]].x;
                interpolate_point.y = (int)next_cloud->points[block_list[MyPoint(x, y, z)][idx]].y;
                interpolate_point.z = (int)next_cloud->points[block_list[MyPoint(x, y, z)][idx]].z;
                pipeline_obj.predict_point_cloud->push_back(interpolate_point);
            }
        }
    }

    if (_next != NULL)
    {
        _next->run(pipeline_obj);
    }
}
