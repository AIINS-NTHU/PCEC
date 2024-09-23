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

// Pipeline_Object parameters
std::string prev_filename = "longdress_vox10_ai_dec_1051.ply";
std::string next_filename = "longdress_vox10_ai_dec_1055.ply";
std::string output_filename = "output.ply";
double temporal_index = 0.5;
double resolution = 1024;

// My combination of stages

// pipeline F
// Stage *my_stage1 = new Stage1_None();
// Stage *my_stage2 = new Stage2_Query_Radius(1, 2);
// Stage *my_stage3 = new Stage3_Point_Estimation(1);
// Stage *my_stage4 = new Stage4_Point_Based(1);
// Stage *my_stage5 = new Stage5_None("output.ply");

// pipeline Q
Stage *my_stage1 = new Stage1_Downsample(1.0, 15.0);
Stage *my_stage2 = new Stage2_CFDS(2);
Stage *my_stage3 = new Stage3_Cube_Estimation(2, 128);
Stage *my_stage4 = new Stage4_Neighboring_Based(1);
Stage *my_stage5 = new Stage5_None(output_filename);

// pipeline B
// Stage *my_stage1 = new Stage1_None();
// Stage *my_stage2 = new Stage2_Nearest_Neighbor(1);
// Stage *my_stage3 = new Stage3_Cube_Estimation(1, 128);
// Stage *my_stage4 = new Stage4_Cube_Based(1, 1);
// Stage *my_stage5 = new Stage5_None("output.ply");
