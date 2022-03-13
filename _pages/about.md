---
permalink: /about/
title: "About this project"
excerpt: "This thesis project focuses on developing a multi-robot system that can cooperatively perform tasks such as transporting objects."
date: 2022-03-13T14:46:59+01:00
layouts_gallery:
  - url: /assets/images/one-arm-moveit-gazebo.png
    image_path: /assets/images/one-arm-moveit-gazebo.png
    alt: "splash layout example"
  - url: /assets/images/two-arm-moveit-gazebo.png
    image_path: /assets/images/two-arm-moveit-gazebo.png
    alt: "single layout with comments and related posts"
  - url: /assets/images/four-arm-moveit-gazebo.png
    image_path: /assets/images/four-arm-moveit-gazebo.png
    alt: "archive layout example"
toc: true
---

{% include gallery id="layouts_gallery" caption="Simulations in Gazebo for one, two and four UR10s working simultaneously." %}

This thesis project focuses on developing a multi-robot system that can cooperatively perform tasks such as transporting objects. There is not much documentation on how to develop a system where multiple robots can be controlled simultaneously in the environment of ROS, which is widely used in research and prototyped for testing before going into production.

Two solutions have been designed, developed, implemented and experimentally evaluated: the first is the ROS package called MoveIt!, where the work is mainly focused on the configuration to allow the simultaneous control of different cobots; the second is the creation or use of a third-party planner that sends the commands directly to the controllers in charge of executing the movements of the cobots, and each of them has a built-in gripper that allows them to perform different tasks.

The Leap Motion device is also integrated into the system. It is capable of detecting, tracking and recognizing the user's hand gestures and serves as an interface for the simultaneous control of up to two cobots, enabling the manipulation of objects.

[Read more...](https://deposita.unizar.es/record/66296?ln=es)