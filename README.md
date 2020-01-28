# Sytém pro detekci prostředí mobilního robotu
Systém pro sběr dat o prostředí a jeho detekci za účelem rozpoznání umožňující změnu chování mobilního robotu.

    Tato práce byla podpořena Ministerstvem školství mládeže a tělovýchovy České republiky, projekt číslo LTARF18017.

# Environment detection system
Environment detection system for mobile robot. The goal is to collect data to recognize the environment and adapt mobile robot behaviour.

Tested on Ubuntu 18.04 LTS with ROS Melodic


## System concept
<img width=600px src="https://raw.githubusercontent.com/neduchal/env_detection/master/imgs/eds_concept.png" />

## Example map visualization
<img width=600px src="https://raw.githubusercontent.com/neduchal/env_detection/master/imgs/map_example.png" />

## Mobile robot used for testing
<img width=600px src="https://raw.githubusercontent.com/neduchal/env_detection/master/imgs/thumper_cropped.png" />


## Dependencies for version 1.0.0

### Grid map library
no significant changes in the fork yet, better to use original ANYbotics version

    https://github.com/neduchal/grid_map or https://github.com/ANYbotics/grid_map

dependencies of grid map library

    sudo apt-get install ros-melodic-octomapros-melodic-octomap-msgs ros-melodic-costmap-2d

## Subscribed topics

* **`/map`** ([nav_mags/OccupancyGrid])

Map of the environment generated by Simultaneous Localization And Mapping (SLAM) algorithm

## Tfs

* The transformation between **`/map`** frame and **`/base_link`** frame have to be set in order to determine pose in the map.

## Example usage

There is an example launch file used for testing multiple sensors. 

    roslaunch env_detection_core test.launch


# Citation

To cite this project in your research:

```
    @article{neduchal2019environment,
      title={Environment detection system for localization and mapping purposes},
      author={Neduchal, P and Bure{\v{s}}, L and {\v{Z}}elezn{\`y}, M},
      journal={IFAC-PapersOnLine},
      volume={52},
      number={27},
      pages={323--328},
      year={2019},
      publisher={Elsevier}
    }
```
# Acknowledgement

    This work was supported by the Ministry of Education of the Czech Republic, project No. LTARF18017.
