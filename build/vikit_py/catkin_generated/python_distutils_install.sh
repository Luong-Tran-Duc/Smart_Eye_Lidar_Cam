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

echo_and_run cd "/home/aa-04/Smart_Eye_Lidar_Cam/src/rpg_vikit/vikit_py"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/aa-04/Smart_Eye_Lidar_Cam/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/aa-04/Smart_Eye_Lidar_Cam/install/lib/python3/dist-packages:/home/aa-04/Smart_Eye_Lidar_Cam/build/vikit_py/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/aa-04/Smart_Eye_Lidar_Cam/build/vikit_py" \
    "/usr/bin/python3" \
    "/home/aa-04/Smart_Eye_Lidar_Cam/src/rpg_vikit/vikit_py/setup.py" \
     \
    build --build-base "/home/aa-04/Smart_Eye_Lidar_Cam/build/vikit_py" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/aa-04/Smart_Eye_Lidar_Cam/install" --install-scripts="/home/aa-04/Smart_Eye_Lidar_Cam/install/bin"
