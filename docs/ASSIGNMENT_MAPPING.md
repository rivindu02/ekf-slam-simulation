# Assignment requirement mapping

Source: MiniProject_FilterImplementation2.pdf.

| Assignment item | Delivered evidence |
|---|---|
| Select KF, EKF or PF | EKF, implemented explicitly in src/ekf_slam.py |
| Specific state-estimation application | 2D landmark SLAM for a differential-drive robot |
| Preferred implementation platform | Python / NumPy; optional Gazebo Classic / ROS 2 Humble |
| Mathematical derivations | Report Section 3: motion, measurement Jacobians, state augmentation, prediction, Joseph correction |
| Code explanation and snippets | Report Sections 4 and Appendix A; commented source |
| Demonstrate effectiveness through simulation | 120 trial results, odometry comparison, map and trajectory errors |
| Report structure | Introduction, methodology, implementation, results, discussion and conclusion |
| Visualizations | Four plot sets, GIF, presentation; Gazebo world |
| Explore parameters/scenarios | Higher sensor/odometry noise, dropout, no reuse, underestimated covariance |
| Correctness and understanding | Eleven automated tests, replay validation, diagnostics and viva notes |
| Unique combination | EKF + landmark SLAM; distinct from the provided localization selections |
| Deadline | Not stated in supplied PDF; student must use the course's actual deadline |

Personal cover details and final course registration/acceptance are not inferable from the supplied files. Fill those in before submission. The deliverable does not assert lecturer approval or fabricate student identity.
