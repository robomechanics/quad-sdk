[![CircleCI](https://circleci.com/gh/robomechanics/quad-sdk/tree/main.svg?style=shield)](https://circleci.com/gh/robomechanics/quad-sdk/tree/main)
![Example image](doc/quad_sdk_promo.png)

![Alt text](doc/dock1.jpg?raw=true)
![Alt text](doc/dock2.jpg?raw=true)
![Alt text](doc/dock4.jpg?raw=true)

## Ylo2 robot github link : https://github.com/elpimous/ylo-2


## Overview

Quad-SDK is an open source, ROS-based full stack software framework for agile quadrupedal locomotion. The design of Quad-SDK is focused on the vertical integration of planning, control, estimation, communication, and development tools which enable agile quadrupedal locomotion in simulation and hardware with minimal user changes for multiple platforms. The modular software architecture allows researchers to experiment with their own implementations of different components while leveraging the existing framework. Quad-SDK also offers Gazebo simulation support and a suite of visualization and data-processing tools for rapid development. Refer to the [paper] for high-level details of the framework.

**Keywords:** Legged Robotics, Quadrupeds, Planning, Control, Leaping, ROS

### License

The source code is released under a [MIT License](LICENSE).

**Authors: Joe Norby, Yanhao Yang, Ardalan Tajbakhsh, Jiming Ren, Justin K. Yim, Alexandra Stutt, Qishun Yu, Nikolai Flowers, and Aaron M. Johnson<br />
Affiliation: [The Robomechanics Lab at Carnegie Mellon University](https://www.cmu.edu/me/robomechanicslab/)<br />
Maintainer: Ardalan Tajbakhsh, atajbakh@andrew.cmu.edu**

The packages in Quad-SDK have been tested under [ROS] Melodic on Ubuntu 18.04.
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

### Publications

If you use this work in an academic context, please cite the following publications as relevant:

* Repository: J. Norby, Y. Yang, A. Tajbakhsh, J. Ren, J. K. Yim, A. Stutt, Q. Yu, N. Flowers, and A. M. Johnson. Quad-
SDK: Full stack software framework for agile quadrupedal locomotion. In ICRA Workshop on
Legged Robots, May 2022. ([paper])

        @inproceedings{abs:norby-quad-sdk-2022,
          author        = {Joseph Norby and Yanhao Yang and Ardalan Tajbakhsh and Jiming Ren and Justin K. Yim and Alexandra Stutt and Qishun Yu and Nikolai Flowers and Aaron M. Johnson},
          title         = {Quad-{SDK}: Full Stack Software Framework for Agile Quadrupedal Locomotion},
          booktitle     = {ICRA Workshop on Legged Robots},
          year          = {2022},
          month         = {May},
          type          = {workshop abstract},
          url_Info      = {https://leggedrobots.org/index.html},
          url_PDF       = {http://www.andrew.cmu.edu/user/amj1/papers/Quad_SDK_ICRA_Abstract.pdf},
          keywords      = {Control,Planning,Leaping}
        }
        
* Global Planner: J. Norby and A. M. Johnson, “Fast global motion planning for dynamic legged robots,” in 2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2020, pp. 3829–3836. ([paper](https://www.andrew.cmu.edu/user/amj1/papers/IROS2020_Fast_Global_Motion_Planning.pdf))

        @inproceedings{Norby2020,
	  	title={Fast global motion planning for dynamic legged robots},
	  	author={Norby, Joseph and Johnson, Aaron M},
	  	booktitle={2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
	  	pages={3829--3836},
	  	year={2020},
	  	organization={IEEE}
		}



## Installation

Refer to the [Quad-SDK Wiki](https://github.com/robomechanics/quad-sdk/wiki/1.-Getting-Started-with-Quad-SDK) for installation, dependency, and unit testing information. Currently Quad-SDK requires ROS Melodic on Ubuntu 18.04. All other dependencies are installed with the included setup script.
[paper]: https://www.andrew.cmu.edu/user/amj1/papers/Quad_SDK_ICRA_Abstract.pdf
