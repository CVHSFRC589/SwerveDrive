// package frc.robot;

// import java.util.ArrayList;
// import java.util.List;

// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.Trajectory.State;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.WaitCommand;

// class Auton {
//     public String name = "";
//     public List<PathPlannerTrajectory> traj = new ArrayList<PathPlannerTrajectory>();
//     public Command commandB = new WaitCommand(0);
//     public Command commandR = new WaitCommand(0);
//     public Trajectory fulltrajB = new PathPlannerTrajectory();
//     public Trajectory fulltrajR = new PathPlannerTrajectory();
//     public boolean DoPath = true;

//     public Auton(String name) {
//             this.name = name;
//     }

//     public Auton build(String hname, PathConstraints constraint, PathConstraints... constraints) {
//             this.traj = PathPlanner.loadPathGroup(hname, false, constraint, constraints);
//             this.commandB = this.commandB.andThen(RobotContainer.m_autoBuilder.fullAuto(this.traj));
//             this.commandR = this.commandR.andThen(RobotContainer.m_autoBuilder.fullAuto(flipList(this.traj)));
//             return this;
//     }
    
//     private List<PathPlannerTrajectory> flipList(List<PathPlannerTrajectory> list) {
//           List<PathPlannerTrajectory> end = new ArrayList<PathPlannerTrajectory>(list.size());
//           for (PathPlannerTrajectory traj : list){
//             List<State> states = new ArrayList<State>(traj.getStates().size());
//             for (int i = 0; i < traj.getStates().size(); ++i) {
//               PathPlannerState current = (PathPlannerState)traj.getStates().get(i);
//               PathPlannerState b = new PathPlannerState();
//               b.accelerationMetersPerSecondSq = current.accelerationMetersPerSecondSq;
//               b.angularVelocityRadPerSec = current.angularVelocityRadPerSec;
//               b.curvatureRadPerMeter = current.curvatureRadPerMeter;
//               b.holonomicAngularVelocityRadPerSec = current.holonomicAngularVelocityRadPerSec;
//               b.holonomicRotation = current.holonomicRotation.times(-1).plus(new Rotation2d(Math.PI));
//               b.poseMeters = new Pose2d(new Translation2d((current.poseMeters.getTranslation().getX()) * -1 + 16.5, current.poseMeters.getTranslation().getY()), new Rotation2d(current.poseMeters.getRotation().getRadians() * -1 + Math.PI));
//               b.timeSeconds = current.timeSeconds;
//               b.velocityMetersPerSecond = current.velocityMetersPerSecond;

//           states.add(b);  

//         }
//         PathPlannerTrajectory p = new PathPlannerTrajectory (states, traj.getMarkers(), traj.getStartStopEvent(), traj.getEndStopEvent(), traj.fromGUI);
//         end.add(p);
//       }
//       return end;
//         }

// }