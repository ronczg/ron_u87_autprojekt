#   Turtle House Package
## ROS 2 C++ Package

A package célja a TurtleSim szimulátor használatával egy ház kirajzolása, amely egy négyzet alap és egy háromszög tetőből áll. A node a geometry_msgs/Twist típusú üzeneteket publikálja a teknős mozgásának vezérléséhez. A csomag ROS 2 Humble alatt készült.

## Packages and Build
It is assumed that the workspace is ~/ros2_ws/.

### Clone the Package

``` r
cd ~/ros2_ws/src
```
``` r
git clone https://github.com/ronczg/ron_u87_autprojekt
```

### Build the Package
``` r
cd ~/ros2_ws
```
``` r
colcon build --packages-select turtle_house --symlink-install
```
### Sourcing the Workspace
Before running any ROS 2 commands, don't forget to source the workspace:
``` r
source ~/ros2_ws/install/setup.bash
```
## Running the Node
To launch the TurtleSim and draw the house, use the following commands:

### Start TurtleSim:

``` r
ros2 run turtlesim turtlesim_node
```
### Run the Draw House Node:
``` r
ros2 run turtle_house draw_house
```

## Graph

A TurtleSim és a Draw House Node kapcsolatának vizualizálása Mermaid diagrammal:

```mermaid
graph TD;
    A[Turtlesim Node] -->|/turtle1/cmd_vel| B[Draw House Node];

