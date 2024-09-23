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

// Post-Processing

#ifndef STAGE5_HPP
#define STAGE5_HPP

class Stage5_None : public Stage
{
private:
    std::string output_filename;

public:
    Stage5_None(std::string my_output_filename);
    void run(Pipeline_Object &pipeline_obj);
};

// class Stage5_PSR
// {
// private:
// public:
//     void run(const std::string output_filename,
//              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &predict_point_cloud,
//              pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud);
// };
#endif