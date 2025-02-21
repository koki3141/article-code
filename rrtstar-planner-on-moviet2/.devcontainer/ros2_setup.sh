source /opt/ros/humble/setup.bash
export FRI_CLIENT_VERSION=1.15
mkdir -p src
vcs import src < .devcontainer/repos.yaml
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install
source install/setup.bash
