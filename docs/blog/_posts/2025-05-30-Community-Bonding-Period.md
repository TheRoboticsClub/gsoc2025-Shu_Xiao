---
layout: post
title: Community Bonding Period
---

# Community Bonding Period
> *The period of time between when accepted students are announced and the time these students are expected to start coding. This time is an excellent one to introduce students to the community, get them on the right mailing lists, working with their mentors on their timeline for the summer, etc.* [1](https://developers.google.com/open-source/gsoc/resources/glossary#community_bonding_period)

## Hello from Shu
Hi everyone! I’m Shu Xiao, currently pursuing my Master’s in Robotics and Advanced Construction at IAAC, Barcelona. I'm thrilled to be participating in **Google Summer of Code 2025** with [JdeRobot](https://github.com/JdeRobot), where I’ll be working on migrating and improving the *Machine Vision* exercise from ROS1 + MoveIt1 to a modern ROS2 + MoveIt2 stack using Docker.

This first stage has been about settling in, understanding the infrastructure, and exploring the foundational tools and structure of Robotics Academy.

---

## What did I do in this period?

During the Community Bonding phase, I focused on:

- Getting familiar with the **Robotics Academy ecosystem**, especially how exercises are structured and run inside Docker containers.
- Testing out the classic **Basic Vacuum Cleaner** exercise, which introduced me to their simulation framework, robot API, and how coverage algorithms are implemented.
- Communicating with mentors and understanding expectations, tools, and collaborative flow.

A highlight was studying the Vacuum Cleaner logic, including motion planning strategies like:

- <strong>Random Angle Generation</strong><br>
<div class="video_container">
<iframe src="https://www.youtube.com/embed/7a1UGHvFG_o" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen class="video"></iframe>
</div>

- <strong>Dash Movement</strong><br>
<div class="video_container">
<iframe src="https://www.youtube.com/embed/1fuIjOV0E8U" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen class="video"></iframe>
</div>

Result:
![img](/gsoc2025-Shu_Xiao/assets/img/blogs/dash.png)

- <strong>Spiral Movement</strong><br>
<div class="video_container">
<iframe src="https://www.youtube.com/embed/QCSzOZ23W50" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen class="video"></iframe>
</div>

Result:
![img](/gsoc2025-Shu_Xiao/assets/img/blogs/spiral.png)

---

## Next Steps

Heading into the coding period, I’ll be shifting my attention to the Machine Vision exercise:

- Set up and verify the existing ROS1 + MoveIt1 version
- Rebuild the stack in ROS2 + MoveIt2
- Redesign a student-friendly Python API and GUI
- Start building a modularized, Dockerized environment for deployment
