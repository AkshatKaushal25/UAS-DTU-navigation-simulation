# 🛠️ Setup Instructions

### Create a workspace and add an src folder to it

``` mkdir nav2_ws/src && cd src/```

### Clone the repo

```git clone https://github.com/AkshatKaushal25/UAS-DTU-navigation-simulation.git . ```

### head back to root directory

```cd ..```

### Install Dependencies 

```rosdep install -y --from-paths ./src --ignore-src```

### Build the directory 

```colcon build --symlink-install```
#Contribution
