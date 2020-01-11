#! /bin/bash
docker build -t ascend/msp_fc_driver -f Dockerfile.dev .
docker run -it --name "msp_fc_driver" --rm \
    --device /dev/ttyACM0 \
    -e DISPLAY=$DISPLAY -u root -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v "$PWD:/root/catkin_ws/src/msp_fc_driver:ro" \
    --net host \
    ascend/msp_fc_driver tmux
