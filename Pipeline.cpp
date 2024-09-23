#include <iostream>
#include <stdio.h>
#include <stddef.h>
#include <vector>
#include <algorithm>
#include <string>
#include <ctime>
#include <bits/stdc++.h>
#include <math.h>
#include <chrono>
#include <map>
#include <numeric>
#include <set>
#include <utility>
#include <stdlib.h>

#include <Eigen/Core>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
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

#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>

// #include "utility.hpp"
#include "base.hpp"
#include "Stage1.hpp"
#include "Stage2.hpp"
#include "Stage3.hpp"
#include "Stage4.hpp"
#include "Stage5.hpp"
#include "config.hpp"

void pipeline(Stage *stages[], Pipeline_Object &pipeline_obj)
{
    int i = 0;
    for (i = 0; i < 4; i++)
    {
        stages[i]->connect(stages[i + 1]);
    }
    stages[i]->connect(NULL);
    stages[0]->run(pipeline_obj);
}

int main(int argc, char **argv)
{
    prev_filename = argv[1];
    next_filename = argv[2];
    output_filename = argv[3];
    temporal_index = atof(argv[4]) * 1.0;

    Pipeline_Object my_obj(prev_filename, next_filename, temporal_index, resolution);

    // my_stage1, my_stage2, my_stage3, my_stage4, my_stage5
    Stage *stages[] = {my_stage1, my_stage2, my_stage3, my_stage4, my_stage5};

    pipeline(stages, my_obj);

    // Pipeline5 example
    // Stage1_Downsample my_stage1(1.9, 15);
    // my_stage1.run(my_obj);

    return 0;
}
