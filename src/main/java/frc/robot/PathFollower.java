package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;

public class PathFollower {
    DifferentialDriveOdometry odometry;
    PathPlannerTrajectory path;
    RamseteController ramsete;
    DifferentialDriveKinematics kinematics;
    Timer timer;
    private double xTol = 0.03;
    private double yTol = 0.03;
    private double angTol = 3;
    private double maxVel = 0.8;
    private double maxAcc = 0.4;
    private boolean atEndpoint = false;

  public PathFollower() {}

  // Loads the path. Should be called immediately before the user would like the robot to begin tracking the path.
  public void loadPath(String pathName, double currentYaw, double currentRightPos, double currentLeftPos) {
    timer = new Timer(); 
    timer.start();
    path = PathPlanner.loadPath(pathName, new PathConstraints(maxVel, maxAcc));
    PathPlannerState startingState = path.getInitialState();
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(currentYaw), currentLeftPos, currentRightPos, startingState.poseMeters);
    ramsete = new RamseteController(2, 0.7);
    kinematics = new DifferentialDriveKinematics(Drivetrain.trackWidth);
  }

  // Generates wheel speeds to track the path. Should be called each period until the endpoint is reached.
  public DifferentialDriveWheelSpeeds getWheelSpeeds(double currentYaw, double currentRightPos, double currentLeftPos) {
    PathPlannerState currentGoal = (PathPlannerState) path.sample(timer.get());
    odometry.update(Rotation2d.fromDegrees(currentYaw), currentRightPos, currentLeftPos);
    ChassisSpeeds chassisSpeeds = ramsete.calculate(odometry.getPoseMeters(), currentGoal);
    PathPlannerState endState = path.getEndState();
    atEndpoint = 
      Math.abs(odometry.getPoseMeters().getRotation().getDegrees() - endState.poseMeters.getRotation().getDegrees()) < angTol 
      && Math.abs(odometry.getPoseMeters().getX() - endState.poseMeters.getX()) < xTol 
      && Math.abs(odometry.getPoseMeters().getY() - endState.poseMeters.getY()) < yTol;
    return kinematics.toWheelSpeeds(chassisSpeeds);
  }
  
  // Tells whether the robot has reached the endpoint, within the defined tolerance.
  public boolean atEndpoint() {
    return atEndpoint;
  }

  public void setXTol(double desiredXTol) {
    xTol = desiredXTol;
  }

  public void setYTol(double desiredYTol) {
    yTol = desiredYTol;
  }

  public void setAngTol(double desiredAngTol) {
    angTol = desiredAngTol;
  }

  public void setMaxVel(double desiredMaxVel) {
    maxVel = desiredMaxVel;
  }

  public void setMaxAcc(double desiredMaxAcc) {
    maxAcc = desiredMaxAcc;
  }
}