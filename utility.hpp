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

#ifndef UTILITY_HPP
#define UTILITY_HPP

class MyPoint
{
public:
    int x;
    int y;
    int z;
    MyPoint(int my_x, int my_y, int my_z);
};

struct MyPointCompare
{
    bool operator()(const MyPoint &lhs, const MyPoint &rhs) const;
};

int find_center(int x, int cube_length);

std::vector<int> sort_indexes(const std::vector<std::vector<double>> &v);

class MyMatching
{
    // 宣告 public 成員
public:
    std::vector<int> match_idx_list;
    std::vector<float> match_dist_list;
};

bool from_prev_or_next(pcl::PointCloud<pcl::PointXYZRGB>::Ptr prev_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr next_cloud);
#endif