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

echo_and_run cd "/home/stitch0001/catkin_WS/src/reach_ros_node"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/stitch0001/catkin_WS/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/stitch0001/catkin_WS/install/lib/python2.7/dist-packages:/home/stitch0001/catkin_WS/src/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/stitch0001/catkin_WS/src" \
    "/usr/bin/python2" \
    "/home/stitch0001/catkin_WS/src/reach_ros_node/setup.py" \
     \
    build --build-base "/home/stitch0001/catkin_WS/src/reach_ros_node" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/stitch0001/catkin_WS/install" --install-scripts="/home/stitch0001/catkin_WS/install/bin"
