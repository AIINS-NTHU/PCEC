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

// Motion Estimation
#include "base.hpp"
#include "utility.hpp"
#include "Stage3.hpp"

Stage3_Cube_Estimation::Stage3_Cube_Estimation(int my_mode, int my_cube_length) : mode(my_mode), cube_length(my_cube_length)
{
}

void Stage3_Cube_Estimation::run(Pipeline_Object &pipeline_obj)
{

    pipeline_obj.cube_length = cube_length;

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

    std::map<MyPoint, std::vector<int>, MyPointCompare> ds_block_list; // key: point xyz, value: query point index

    for (int x = cube_length / 2; x < pipeline_obj.resolution; x += cube_length)
    {
        for (int y = cube_length / 2; y < pipeline_obj.resolution; y += cube_length)
        {
            for (int z = cube_length / 2; z < pipeline_obj.resolution; z += cube_length)
            {
                ds_block_list[MyPoint(x, y, z)] = std::vector<int>();
            }
        }
    }

    for (int w = 0; w < ds_next_cloud->points.size(); w++)
    {
        ds_block_list[MyPoint(find_center(ds_next_cloud->points[w].x, cube_length), find_center(ds_next_cloud->points[w].y, cube_length), find_center(ds_next_cloud->points[w].z, cube_length))].push_back(w);
    }

    std::vector<MyPoint> non_empty_block_center; // save those center points which query points > 10

    for (int x = cube_length / 2; x < pipeline_obj.resolution; x += cube_length)
    {
        for (int y = cube_length / 2; y < pipeline_obj.resolution; y += cube_length)
        {
            for (int z = cube_length / 2; z < pipeline_obj.resolution; z += cube_length)
            {
                if (ds_block_list[MyPoint(x, y, z)].size() == 0)
                    continue;

                non_empty_block_center.push_back(MyPoint(x, y, z));

                double vector_x = 0.0, vector_y = 0.0, vector_z = 0.0, vector_r = 0.0, vector_g = 0.0, vector_b = 0.0;
                double correct_matching_num = 0.0;
                for (int idx = 0; idx < ds_block_list[MyPoint(x, y, z)].size(); idx++)
                {
                    if (pipeline_obj.matching_table[ds_block_list[MyPoint(x, y, z)][idx]] != -1)
                    {
                        correct_matching_num += 1;
                        vector_x += (ds_next_cloud->points[ds_block_list[MyPoint(x, y, z)][idx]].x - (pipeline_obj.temporal_index * ds_next_cloud->points[ds_block_list[MyPoint(x, y, z)][idx]].x + (1 - pipeline_obj.temporal_index) * ds_prev_cloud->points[pipeline_obj.matching_table[ds_block_list[MyPoint(x, y, z)][idx]]].x));
                        vector_y += (ds_next_cloud->points[ds_block_list[MyPoint(x, y, z)][idx]].y - (pipeline_obj.temporal_index * ds_next_cloud->points[ds_block_list[MyPoint(x, y, z)][idx]].y + (1 - pipeline_obj.temporal_index) * ds_prev_cloud->points[pipeline_obj.matching_table[ds_block_list[MyPoint(x, y, z)][idx]]].y));
                        vector_z += (ds_next_cloud->points[ds_block_list[MyPoint(x, y, z)][idx]].z - (pipeline_obj.temporal_index * ds_next_cloud->points[ds_block_list[MyPoint(x, y, z)][idx]].z + (1 - pipeline_obj.temporal_index) * ds_prev_cloud->points[pipeline_obj.matching_table[ds_block_list[MyPoint(x, y, z)][idx]]].z));
                    }
                }

                vector_x = vector_x / correct_matching_num;
                vector_y = vector_y / correct_matching_num;
                vector_z = vector_z / correct_matching_num;
                vector_r = 0;
                vector_g = 0;
                vector_b = 0;

                std::vector<double> temp;
                temp.push_back(vector_x);
                temp.push_back(vector_y);
                temp.push_back(vector_z);
                temp.push_back(vector_r);
                temp.push_back(vector_g);
                temp.push_back(vector_b);

                pipeline_obj.non_empty_block_vector.push_back(temp);
            }
        }
    }

    if (_next != NULL)
    {
        _next->run(pipeline_obj);
    }
}

Stage3_Point_Estimation::Stage3_Point_Estimation(int my_mode) : mode(my_mode)
{
}

void Stage3_Point_Estimation::run(Pipeline_Object &pipeline_obj)
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

    // pcl::io::savePLYFileASCII("prev.ply", *ds_prev_cloud);
    // pcl::io::savePLYFileASCII("next.ply", *ds_next_cloud);

    for (int idx = 0; idx < ds_next_cloud->points.size(); ++idx)
    {
        std::vector<double> motion_vector;
        if (pipeline_obj.matching_table[idx] != -1)
        {
            motion_vector.push_back(-ds_next_cloud->points[idx].x + ds_prev_cloud->points[pipeline_obj.matching_table[idx]].x);
            motion_vector.push_back(-ds_next_cloud->points[idx].y + ds_prev_cloud->points[pipeline_obj.matching_table[idx]].y);
            motion_vector.push_back(-ds_next_cloud->points[idx].z + ds_prev_cloud->points[pipeline_obj.matching_table[idx]].z);
            motion_vector.push_back(-ds_next_cloud->points[idx].r + ds_prev_cloud->points[pipeline_obj.matching_table[idx]].r);
            motion_vector.push_back(-ds_next_cloud->points[idx].g + ds_prev_cloud->points[pipeline_obj.matching_table[idx]].g);
            motion_vector.push_back(-ds_next_cloud->points[idx].b + ds_prev_cloud->points[pipeline_obj.matching_table[idx]].b);
        }
        else
        {
            motion_vector.push_back(0);
            motion_vector.push_back(0);
            motion_vector.push_back(0);
            motion_vector.push_back(0);
            motion_vector.push_back(0);
            motion_vector.push_back(0);
        }
        pipeline_obj.motion_estimation.push_back(motion_vector);
    }

    if (_next != NULL)
    {
        _next->run(pipeline_obj);
    }
}
