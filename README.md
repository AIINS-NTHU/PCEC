<div align="center">
    <h1>
        <strong>Composing Error Concealment Pipelines for Dynamic 3D Point Cloud Streaming</strong>
    </h1>
</div>

<div align="center">
    <a href="https://github.com/Huang-I-Chun">I-Chun Huang</a><sup>1</sup>,
    <a href="https://yuang-ian.github.io">Yuang Shi</a><sup>2</sup>,
    <a href="https://syjintw.github.io/">Yuan-Chun Sun</a><sup>1</sup>,
    <a href="https://www.comp.nus.edu.sg/~ooiwt/">Wei Tsang Ooi</a><sup>2</sup>,
    <a href="https://people.cs.nycu.edu.tw/~chuang/">Chun-Ying Huang</a><sup>3</sup>,
    <a href="https://nmsl.cs.nthu.edu.tw/cheng-hsin-hsu/">Cheng-Hsin Hsu</a><sup>1</sup>,
    <p>
        <sup>1</sup>National Tsing Hua University,
        <sup>2</sup>National University of Singapore,
        <sup>3</sup>National Yang Ming Chiao Tung University
    </p>
</div>


<div align="center">
    <a href="" target='_blank'>
        <img src="https://img.shields.io/badge/Paper-%F0%9F%93%83-blue">
    </a>
    <a href="https://hits.seeyoufarm.com"><img src="https://hits.seeyoufarm.com/api/count/incr/badge.svg?url=https%3A%2F%2Fgithub.com%2FAIINS-NTHU%2FPCEC&count_bg=%2379C83D&title_bg=%23555555&icon=&icon_color=%23E7E7E7&title=Visitors&edge_flat=false"/></a>
</div>

## About

<p align="center">
  <a href="">
    <img src="/fig/teaser.png" alt="teaser" width="80%">
  </a>
</p>

This work proposes the very first error concealment pipeline framework for dynamic 3D point cloud streaming, which is designed to be flexible and extensible, allowing users to customize the pipeline in different usage scenarios.  

## Outline
- [About](#about)
- [Outline](#outline)
- [Overview](#overview)
- [Implementation](#implementation)
  - [Requirements](#requirements)
  - [Build](#build)
  - [Usage](#usage)
  - [Pipeline Example](#pipeline-example)
  - [Create Your Own Algorithm](#create-your-own-algorithm)
  - [Other Tips](#other-tips)
- [License](#license)
- [Citation](#citation)


## Overview

The proposed error concealment framework comprises five stages: 
1. **Pre-Processing**. Downsample point clouds to reduce the computational overhead of the following stages.
2. **Matching**. Compute the matching table from two anchor (reference) frames. 
3. **Motion Estimation**. Calculate the resulting motion vectors from two anchor (reference) frames.
4. **Prediction**. Generate error-concealed frame in good visual quality using the anchor frames and motion vectors.
5. **Post-Processing**. Improve spatial smoothness by repairing the cracks and artifacts produced by the previous stages.

Multiple algorithms are proposed for each stage, with careful consideration of different usage scenarios. Moreover, based on the quantitative evaluation and analysis, we concatenate the proposed algorithms in different stages to form four representative error concealment pipelines for diverse usage scenarios:
1. **Fast (F)** for minimum time complexity.
2. **Balance (B)** for a good tradeoff between quality and time complexity.
3. **Quality (Q)** for the highest quality.
4. **Quality+ (Q+)** is built upon Q, but uses the more expensive Poisson Surface Reconstruction (PSR) for surface refinement.


| Pipeline | Stg. 1. Pre-Proc. | Stg. 2. Matching | Stg. 3. M. Est. | Stg. 4. Pred. | Stg. 5. Post-Proc. |
| :---: | :---: | :---: | :---: | :---: | :---: |
| Fast (F) | - | QR | PM | PP | - |
| Balance (B) | - | NN | CM | CP | - |
| Quality (Q) | Downs. $\left(l_1, 1\right)$ | AQR | CM | NP | - |
| Quality+ $(\mathrm{Q}+$ ) | Downs. $\left(l_1, l_2\right)$ | AQR | CM | NP | Ups. + PSR |


This repository contains the official implementations of the proposed algorithms for each stage and representative pipelines. The code is written in C++, and the pipeline can be easily customized to meet the requirements of different applications.

## Implementation

### Requirements

Testing Environment Version:
- Ubuntu 20.04
- [Cmake](https://cmake.org/install/)=3.23.2
- [Point Cloud Library (PCL)](https://pointclouds.org/downloads/)=1.10.0

Example install command:
```bach
sudo apt update
sudo apt install cmake g++ make
sudo apt install libpcl-dev
```

### Build

Compile executable file `Pipeline` under a new folder `build`
```bach
git clone git@github.com:AIINS-NTHU/PCEC.git
cd PCEC
mkdir build
cd build
cmake ..
make
```

### Usage

Run following command:
```bash
python client.py -prev <PREVIOUS_FRAME_PATH> -next <NEXT_FRAME_PATH> -out <OUTPUT_PATH> -pos <RELATIVE_POSITION>
``` 

Example command:
```bash
python client.py -prev example/longdress_1051.ply -next example/longdress_1055.ply -out example/output.ply -pos 0.5
```
`-prev example/longdress_1051.ply` previous point cloud file we provide

`-next example/longdress_1055.ply` next point cloud file we provide

`-out example/output.ply` output point cloud file path

`-pos 0.5` means the output file is right at the middle of previous and next frames

<!-- The output.ply will look like:

<p align="center">
  <a href="">
    <img src="fig/output_illustration.png" alt="teaser" width="20%">
  </a>
</p> -->

### Pipeline Example

In `config.hpp`, we've provided F, B, Q pipeline combinations for your convenience. The current pipeline in use is labeled as "Q". 

To switch to a different pipeline combination, simply comment or uncomment the relevant lines based on your requirements. Remember that every time you make changes to either the .hpp or .cpp files, you'll need to rebuild the project.

### Create Your Own Algorithm

To create custom methods, please consult the `base.hpp` file. Within `base.hpp`, we've established a virtual class called Stage, which all methods must inherit from. Additionally, in the `Pipeline_Object` definition, we've clearly specified the output variables for each stage. It's essential to ensure that any custom stage algorithms you create align with the input and output requirements of each respective stage. 

After implement your own stage algorithms, modify the pipeline reference in `config.hpp`, and rebuild the project.


### Other Tips

Given that various point cloud datasets may employ different coordinate systems and units of measurement, we recommend scaling your dataset to align with the [8iVFB dataset](http://plenodb.jpeg.org/pc/8ilabs/) for consistency.


## License

Distributed under the MIT License. See `LICENSE` for more information.


## Citation

If you find our work useful in your research, please consider citing:

```bibtex
@article{huang2024composing,
  title={Composing Error Concealment Pipelines for Dynamic 3D Point Cloud Streaming},
  author={Huang, I-Chun and Shi, Yuang and Sun, Yuan-Chun and Ooi, Wei Tsang and Huang, Chun-Ying and Hsu, Cheng-Hsin},
  journal={Placeholder, Under Review},
  year={2024}
}
```

This work stands on the shoulders of our previous works. If you find our previous works useful in your research, please consider citing them:

```bibtex
@inproceedings{sun2023dynamic,
  title={A Dynamic 3D Point Cloud Dataset for Immersive Applications},
  author={Sun, Yuan-Chun and Huang, I-Chun and Shi, Yuang and Ooi, Wei Tsang and Huang, Chun-Ying and Hsu, Cheng-Hsin},
  booktitle={Proceedings of the 14th Conference on ACM Multimedia Systems},
  pages={376--383},
  year={2023}
}

@inproceedings{hung2022error,
  title={Error concealment of dynamic 3d point cloud streaming},
  author={Hung, Tzu-Kuan and Huang, I-Chun and Cox, Samuel Rhys and Ooi, Wei Tsang and Hsu, Cheng-Hsin},
  booktitle={Proceedings of the 30th ACM International Conference on Multimedia},
  pages={3134--3142},
  year={2022}
}

@inproceedings{wu2021dynamic,
  title={Dynamic 3D point cloud streaming: Distortion and concealment},
  author={Wu, Cheng-Hao and Li, Xiner and Rajesh, Rahul and Ooi, Wei Tsang and Hsu, Cheng-Hsin},
  booktitle={Proceedings of the 31st ACM Workshop on Network and Operating Systems Support for Digital Audio and Video},
  pages={98--105},
  year={2021}
}
```