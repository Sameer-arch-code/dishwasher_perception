#!/usr/bin/env python3

# DISCLAIMER
# DO NOT start this node on the robot without headphones

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    audio_play = Node(
        package="audio_play",
        executable="audio_play_node",
        name="audio_test_node",
        remappings=[
            ("audio", "/audio_capture/audio"),
            ("audio_info", "/audio_capture/audio_info"),
        ],
        parameters=[
            {
                "device": "hw:1,0",
                "dst": "alsasink",
            },
        ],
    )
    # use this command to check if your sound works inside docker (replace hw:3,0 with your device check with arecord -l)
    # gst-launch-1.0 audiotestsrc ! audioconvert ! audioresample ! alsasink device=hw:3,0

    # if you hear a beep sound everythingk is fine

    return LaunchDescription([audio_play])
