# ROS2 image transport for Foxglove CompressedVideo messages

This plugin provides a ROS2 image transport for encoding messages in Foxglove's ``CompressedVideo`` message format, using the FFMpeg library.
These messages can be recorded in a rosbag for processing with e.g. Foxglove Studio.

There is also an unsupported decoder provided for testing purposes. Use at your own peril.

## Supported systems

Continuous integration is tested under Ubuntu with the following ROS2 distros:

 [![Build Status](https://build.ros2.org/buildStatus/icon?job=Hdev__foxglove_compressed_video_transport__ubuntu_jammy_amd64&subject=Humble)](https://build.ros2.org/job/Hdev__foxglove_compressed_video_transport__ubuntu_jammy_amd64/)
 [![Build Status](https://build.ros2.org/buildStatus/icon?job=Idev__foxglove_compressed_video_transport__ubuntu_jammy_amd64&subject=Iron)](https://build.ros2.org/job/Idev__foxglove_compressed_video_transport__ubuntu_jammy_amd64/)
 [![Build Status](https://build.ros2.org/buildStatus/icon?job=Jdev__foxglove_compressed_video_transport__ubuntu_noble_amd64&subject=Jazzy)](https://build.ros2.org/job/Jdev__foxglove_compressed_video_transport__ubuntu_noble_amd64/)
[![Build Status](https://build.ros2.org/buildStatus/icon?job=Rdev__foxglove_compressed_video_transport__ubuntu_noble_amd64&subject=Rolling)](https://build.ros2.org/job/Rdev__foxglove_compressed_video_transport__ubuntu_noble_amd64/)


## Installation

### From packages

```bash
sudo apt-get install ros-${ROS_DISTRO}-foxglove-compressed-video-transport
```

### From source

Set the following shell variables:
```bash
repo=foxglove_compressed_video_transport
url=https://github.com/ros-misc-utilities/${repo}.git
```
and follow the [instructions here](https://github.com/ros-misc-utilities/.github/blob/master/docs/build_ros_repository.md)

Make sure to source your workspace's ``install/setup.bash`` afterwards.
If all goes well you should see the transport show up:

```
ros2 run image_transport list_transports
```

should give output (among other transport plugins):

```text
"image_transport/foxglove"
 - Provided by package: foxglove_compressed_video_transport
 - Publisher: 
      This plugin encodes frames into foxglove compressed video packets
    
 - Subscriber: 
      This plugin decodes frames from foxglove compressed video packets
```

Remember to install the plugin on both hosts, the one that is encoding and
the one that is decoding (viewing).

## Parameters

### Publisher (camera driver)

Here is a list of the available encoding parameters:

- ``encoding``: the libav (ffmpeg) encoder being used. The default is ``libx264``, which is on-CPU unaccelerated encoding.
  Depending on your hardware, your encoding options may include the hardware accelerated ``h264_nvenc`` or ``h264_vaapi``.
  You can list all available encoders with ``ffmpeg --codecs``. In the h264 row, look for ``(encoders)``.
- ``preset``: default is empty (""). Valid values can be for instance ``slow``, ``ll`` (low latency) etc.
   To find out what presets are available, run e.g.
   ``fmpeg -hide_banner -f lavfi -i nullsrc -c:v libx264 -preset help -f mp4 - 2>&1``
- ``profile``: For instance ``baseline``, ``main``. See [the ffmpeg website](https://trac.ffmpeg.org/wiki/Encode/H.264).
- ``tune``: See [the ffmpeg website](https://trac.ffmpeg.org/wiki/Encode/H.264). The default is empty("").
- ``gop_size``: The number of frames between keyframes. For foxglove, this *must be set to 1*.
   The larger this number the more latency you will have, but also the more efficient the transmission becomes.
- ``bit_rate``: The max bit rate [in bits/s] that the encoding will target. Default is ``8242880`.
- ``delay``: Not sure what it does, but doesn't help with delay. Default is empty ("").
- ``pixel_format``: Forces a different pixel format for internal conversions. Experimental, don't use.
- ``qmax``: Max quantization rate. Defaults to 10. See [ffmpeg documentation](https://www.ffmpeg.org/ffmpeg-codecs.html).
   The larger this number, the worse the image looks, and the more efficient the encoding.
- ``measure_performance``: For performance debugging (developers only). Defaults to false.
- ``performance_interval``: How many frames to wait between logging performance data.

The parameters are under the ``foxglove`` variable block. If you launch
your publisher node (camera driver), you can give it a parameter list on the way like so:
```
            parameters=[
                params_path,
                {
                    '.image_raw.foxglove.encoding': 'h264_vaapi',  # 'libx264'
                    '.image_raw.foxglove.profile': 'main',
                    '.image_raw.floxglove.preset': 'll',
                },
            ],
```
See the example launch file for a V4L USB camera

### Subscriber (viewer)

The subscriber plugin is used for debugging purposes and is not supported.

### Setting encoding parameters when launching camera driver

The ``launch`` directory contains an example launch file ``cam.launch.py`` that demonstrates
how to set encoding profile and preset for e.g. a usb camera.



## License

This software is issued under the Apache License Version 2.0.
