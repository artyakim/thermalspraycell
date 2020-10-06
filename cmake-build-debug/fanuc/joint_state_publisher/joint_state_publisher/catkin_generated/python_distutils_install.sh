#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/artemyakimchuk/Documents/thermalspray_cell/catkin_ws/src/fanuc/joint_state_publisher/joint_state_publisher"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/usr/local/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/usr/local/lib/python2.7/dist-packages:/home/artemyakimchuk/Documents/thermalspray_cell/catkin_ws/src/cmake-build-debug/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/artemyakimchuk/Documents/thermalspray_cell/catkin_ws/src/cmake-build-debug" \
    "/usr/bin/python2" \
    "/home/artemyakimchuk/Documents/thermalspray_cell/catkin_ws/src/fanuc/joint_state_publisher/joint_state_publisher/setup.py" \
    egg_info --egg-base /home/artemyakimchuk/Documents/thermalspray_cell/catkin_ws/src/cmake-build-debug/fanuc/joint_state_publisher/joint_state_publisher \
    build --build-base "/home/artemyakimchuk/Documents/thermalspray_cell/catkin_ws/src/cmake-build-debug/fanuc/joint_state_publisher/joint_state_publisher" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/usr/local" --install-scripts="/usr/local/bin"
