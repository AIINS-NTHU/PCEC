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

#ifndef BASE_HPP
#define BASE_HPP
#include <Eigen/Core>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>

class Pipeline_Object
{
public:
    double temporal_index; // should be 0 - 1
    double resolution;     // height of the point cloud

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr readfile_prev_cloud; // input origin point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr readfile_next_cloud; // input origin point cloud

    // Output variable of each stage
    // Stage 1 Preprocessing: After pre-process point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds1_prev_cloud; // using downsample1
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds1_next_cloud; // using downsample1
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds2_prev_cloud; // using downsample2
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds2_next_cloud; // using downsample2

    // Stage 2 Matching: Obtain matching table
    std::vector<int> matching_table; // use nearest_search_result[ds_NEXT_FRAME_POINT_INDEX] = ds_PREV_FRAME_POINT_INDEX

    // Stage 3 Motion Estimation: Motion estimation per point/center
    std::vector<std::vector<double>> motion_estimation;
    int cube_length;
    std::vector<std::vector<double>> non_empty_block_vector; // each element have six number, which is the vector of x y z r g b

    // Stage 4 Prediction:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr predict_point_cloud;

    // Stage 5 Post-Processing:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud; // output point cloud

    Pipeline_Object(std::string prev_filename, std::string next_filename, double t, double res) : temporal_index(t),
                                                                                                  resolution(res),
                                                                                                  readfile_prev_cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
                                                                                                  readfile_next_cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
                                                                                                  ds1_prev_cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
                                                                                                  ds1_next_cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
                                                                                                  ds2_prev_cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
                                                                                                  ds2_next_cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
                                                                                                  predict_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
                                                                                                  new_cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
    {
        if (temporal_index < 0.5)
        {
            std::string tmp;
            tmp = prev_filename;
            prev_filename = next_filename;
            next_filename = tmp;
            temporal_index = 1 - temporal_index;
        }

        if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(prev_filename, *readfile_prev_cloud) == -1 || pcl::io::loadPLYFile<pcl::PointXYZRGB>(next_filename, *readfile_next_cloud) == -1) //* load the file
        {
            PCL_ERROR("Couldn't read file ply or pcd file \n");
            return;
        }
    };

    ~Pipeline_Object(){};
};

class Stage
{
private:
public:
    Stage *_next;

    Stage()
    {
        _next = NULL;
    }

    virtual void run(Pipeline_Object &pipeline_obj) = 0;

    void connect(Stage *nextstage)
    {
        _next = nextstage;
    }
};
#endif