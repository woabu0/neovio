#!/bin/bash

echo "========================================="
echo "Neovio Autonomous Robot - Setup Check"
echo "========================================="
echo ""

# Check ROS installation
echo "Checking ROS installation..."
if [ -z "$ROS_DISTRO" ]; then
    echo "  ❌ ROS is not sourced. Run: source /opt/ros/noetic/setup.bash"
else
    echo "  ✅ ROS $ROS_DISTRO is installed"
fi

# Check required ROS packages
echo ""
echo "Checking ROS packages..."
packages=("gmapping" "navigation" "move_base" "rviz" "tf")
for pkg in "${packages[@]}"; do
    if rospack find $pkg > /dev/null 2>&1; then
        echo "  ✅ $pkg is installed"
    else
        echo "  ❌ $pkg is missing. Install with: sudo apt install ros-$ROS_DISTRO-$pkg"
    fi
done

# Check Python dependencies
echo ""
echo "Checking Python dependencies..."
if python3 -c "import gpiozero" 2>/dev/null; then
    echo "  ✅ gpiozero is installed"
else
    echo "  ❌ gpiozero is missing. Install with: pip3 install gpiozero"
fi

# Check USB permissions
echo ""
echo "Checking USB permissions..."
if groups | grep -q dialout; then
    echo "  ✅ User is in dialout group"
else
    echo "  ❌ User is not in dialout group. Run: sudo usermod -a -G dialout $USER"
fi

# Check LiDAR device
echo ""
echo "Checking LiDAR device..."
if ls /dev/ttyUSB* 2>/dev/null; then
    echo "  ✅ LiDAR device found:"
    ls -l /dev/ttyUSB* 2>/dev/null | awk '{print "    " $0}'
else
    echo "  ⚠️  No /dev/ttyUSB* devices found. Connect your LiDAR."
fi

# Check workspace
echo ""
echo "Checking workspace..."
if [ -f "src/neovio_autonomous_car/package.xml" ]; then
    echo "  ✅ Workspace structure is correct"
else
    echo "  ❌ Workspace structure is incorrect"
fi

if [ -d "devel" ]; then
    echo "  ✅ Workspace has been built"
else
    echo "  ⚠️  Workspace not built. Run: catkin_make"
fi

# Check launch files
echo ""
echo "Checking launch files..."
launch_files=("slam.launch" "navigation.launch" "complete_system.launch")
for launch in "${launch_files[@]}"; do
    if [ -f "src/neovio_autonomous_car/launch/$launch" ]; then
        echo "  ✅ $launch exists"
    else
        echo "  ❌ $launch is missing"
    fi
done

echo ""
echo "========================================="
echo "Setup check complete!"
echo "========================================="

