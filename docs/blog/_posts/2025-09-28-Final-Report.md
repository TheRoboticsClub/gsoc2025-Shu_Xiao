---
layout: post
title: Final Report
---

# GSoC 2025 Final Report – Shu Xiao

## 1. Personal Information
- **Name**: Shu Xiao  
- **Email**: shu.xiao@students.iaac.net  
- **GitHub**: [https://github.com/Shu980101](https://github.com/Shu980101)  
- **Country & Time Zone**: Spain (CET, UTC+1)  

---

## 2. Title of the Project
**Migration and Enhancement of 'Machine Vision' Exercise to ROS2 + MoveIt2 in Robotics Academy**

<strong>Final Machine Vision Exercise Demo</strong><br>
<div class="video_container">
<iframe src="https://www.youtube.com/embed/s74suaPilt8" title="Final Machine Vision Exercise Demo" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen class="video"></iframe>
</div>

---

## 3. Brief
This project focused on migrating the **Machine Vision** exercise from the old Robotics Academy (ROS1 + MoveIt1) to the new **ROS2 + MoveIt2** ecosystem.  

The work included:  
- Porting the simulation environment and robot description to ROS2 standards  
- Updating the perception pipeline (color/shape filters, PCL integration)  
- Developing a modernized **Python API and HAL** for student interaction  
- Creating a **web-based exercise** integrated with Robotics Academy  
- Writing comprehensive **documentation** for maintainers and students  

The new version delivers a robust, modular, and student-friendly exercise aligned with current ROS2 best practices.

---

## 4. Benefits to the Community
This project makes Robotics Academy more **accessible and relevant** for today’s learners:  

- 🎯 Provides hands-on experience with **ROS2 + MoveIt2** in a realistic simulation  
- 🛠️ Offers a **clean Python API and HAL** for easier learning and experimentation  
- 👀 Adds a **web interface** for direct interaction, making exercises easier to run and test  
- 📝 Includes clear documentation to help students, mentors, and maintainers  
- 🔄 Improves long-term maintainability by aligning with **ROS2 standards**  

---

## 5. Deliverables
- ✅ **Full migration** of the Machine Vision exercise to ROS2 + MoveIt2  
- ✅ **Dockerized setup** for consistent deployment in Robotics Academy  
- ✅ **Redesigned Python API + HAL** for motion and perception control  
- ✅ **Stable perception pipeline** with modular PCL filters (color + shape)  
- ✅ **Pick-and-place workflow** driven by perception (not hardcoded positions)  
- ✅ **Web-based exercise integration** with Robotics Academy GUI  
- ✅ **Documentation**: setup guide, API reference, student instructions  
- ✅ **Final submission** merged and demoed successfully  

---

## 6. Timeline and Work Summary

### **Community Bonding**
- Explored Robotics Academy ecosystem  
- Tested existing exercises (e.g., Vacuum Cleaner)  
- Synced with mentors and set up development environment  

### **Phase 1 – Core Migration**
- **Weeks 1–3**: Debugged legacy code, wrote new pick-and-place algorithm, stabilized Gazebo world, improved path planning  
- **Week 4**: Planned ROS2 migration, studied MoveIt2 architecture  
- **Weeks 5–6**: Adopted IFRA-Cranfield framework, migrated Gazebo world to ROS2  
- **Week 7**: Built dual-camera setup, added objects, published point clouds  
- **Week 8**: Fully migrated environment to ROS2 + MoveIt2  

### **Phase 2 – Perception & HAL**
- **Week 9–10**: Migrated and stabilized PCL filter server, created PERCEPTION API  
- **Week 11**: Tested HAL API, implemented hardcoded pick-and-place workflow  
- **Week 12**: Fixed gripper driver, integrated perception outputs into pick-and-place  

### **Phase 3 – Planning & Integration**
- **Week 13**: Restructured launch files with MoveItConfigsBuilder for planning scene  
- **Week 14**: Tested planning scene (functional but included objects incorrectly)  
- **Week 15**: Removed planning scene (blocking progress), explored RA structure  
- **Week 16**: Final integration with Robotics Academy, developed web-based algorithm, prepared documentation and submission  

---

## 7. Technical Details
- **Languages**: Python, C++  
- **Frameworks**: ROS2 Humble, MoveIt2, Gazebo, RViz2  
- **Tools**: Docker, GitHub, OpenCV, PCL  
- **Libraries**: IFRA-Cranfield ros2_SimRealRobotControl, MoveItConfigsBuilder  
- **System**: Ubuntu 22.04  

---

## 8. Results
- Successfully migrated and enhanced the Machine Vision exercise  
- Delivered a **functional perception-to-action pipeline** in ROS2 + MoveIt2  
- Enabled **dynamic object detection** for manipulation 
- Integrated the exercise into Robotics Academy with a **web-based GUI**  
- Produced complete **documentation** for future users and maintainers  

---

## 9. Future Work
While the project is complete, possible future improvements include:  
- Reintroducing a refined **planning scene** with correct object handling  
- Expanding perception pipeline with **deep learning-based detection**  
- Adding **real-robot integration** for students with hardware access  
- Improving GUI with **real-time visualization** of planning and execution  

---

## 10. Acknowledgements
Special thanks to:  
- My mentors Diego Martín, Pankhuri Vanjani and Javier I from the Robotics Academy team for their guidance and feedback  
- The IFRA-Cranfield group for their modular ROS2 framework that helped accelerate migration  
- The wider ROS and MoveIt2 community for excellent documentation and tutorials  

---

📍 *Posted from Barcelona, Spain*  
🧠 *Project: Migration and Enhancement of Machine Vision Exercise to ROS2 + MoveIt2*  
🔗 *GitHub Repository*: [https://github.com/TheRoboticsClub/gsoc2025-Shu_Xiao](https://github.com/TheRoboticsClub/gsoc2025-Shu_Xiao)  
