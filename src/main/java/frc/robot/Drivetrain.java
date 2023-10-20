package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain {
  // Drive motors
  private final WPI_TalonFX rightFront = new WPI_TalonFX(4);
  private final WPI_TalonFX rightBack = new WPI_TalonFX(3);
  private final WPI_TalonFX leftFront = new WPI_TalonFX(2);
  private final WPI_TalonFX leftBack = new WPI_TalonFX(1);

  // Motor controller groups for teleop
  private final MotorControllerGroup leftMotors = new MotorControllerGroup(leftFront, leftBack);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(rightFront, rightBack);
  private final DifferentialDrive tank = new DifferentialDrive(leftMotors, rightMotors);
  
  // Gyroscope for yaw and pitch
  private final AHRS gyro = new AHRS(); // NavX2 gyro
  
  // Drivetrain mechanical constants
  private static final double trackWidth = 0.55;
  private static final double falconTicksPerRev = 2048.0;
  private static final double wheelCirc = 6*0.0254*Math.PI;
  private static final double gearRatio = 10.71;
  private static final double correctionFactor = 1.0;
  private static final double ticksPerMeter = falconTicksPerRev*gearRatio/(wheelCirc*correctionFactor);
  
  // Teleop controller mapping
  private static final double maxTurnPower = 0.55;
  private static final double xAccLimit = 3.0;
  private static final double AngAccLimit = 3.0/maxTurnPower;
  private static final double driveControllerDeadband = 0.05;
  private final SlewRateLimiter xAccLimiter = new SlewRateLimiter(xAccLimit);
  private final SlewRateLimiter angAccLimiter = new SlewRateLimiter(AngAccLimit);
 
  // Path following
  private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0), 0, 0);
  private ArrayList<PathPlannerTrajectory> paths = new ArrayList<PathPlannerTrajectory>();
  private RamseteController ramsete;
  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(trackWidth);
  private final Timer timer = new Timer();

  public Drivetrain() {
    initializeDriveMotor(rightFront);
    initializeDriveMotor(rightBack);
    initializeDriveMotor(leftFront);
    initializeDriveMotor(leftBack);
    rightBack.setInverted(true);
    rightFront.setInverted(true);
    gyro.calibrate();
    Timer.delay(2.0); // calibration delay for gyro
    gyro.zeroYaw();
  }

  // Manual control of the drivetrain based on raw controller inputs. One of the driving methods should be called each period.
  public final void drive(double drivePower, double turnPower) {
    double commandedDrivePower = xAccLimiter.calculate(MathUtil.applyDeadband(drivePower, driveControllerDeadband));
    double commandedTurnPower = angAccLimiter.calculate(MathUtil.applyDeadband(turnPower, driveControllerDeadband)*maxTurnPower);
    tank.arcadeDrive(commandedDrivePower, commandedTurnPower);
  }
  
  // Autonomous control of the drivetrain based on calculated wheel speeds. One of the driving methods should be called each period.
  private final void setWheelSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds) {
    rightFront.set(ControlMode.Velocity, wheelSpeeds.rightMetersPerSecond/10*ticksPerMeter);
    leftFront.set(ControlMode.Velocity, wheelSpeeds.leftMetersPerSecond/10*ticksPerMeter);
    rightBack.set(ControlMode.Follower, 4);
    leftBack.set(ControlMode.Follower, 2);
    tank.feed();
  }

  public final void updateOdometry() {
    odometry.update(Rotation2d.fromDegrees(getYaw()), getLeftPos(), getRightPos());
  }

  public final double getRobotX() {
    return odometry.getPoseMeters().getX();
  }

  public final double getRobotY() {
    return odometry.getPoseMeters().getY();
  }

  public final double getLeftPos() {
    return (leftFront.getSelectedSensorPosition() + leftBack.getSelectedSensorPosition())/(2*ticksPerMeter);
  }

  public final double getRightPos() {
    return (rightFront.getSelectedSensorPosition() + rightBack.getSelectedSensorPosition())/(2*ticksPerMeter);
  }

  public final double getYaw() {
    return -gyro.getYaw();
  }

  public final double getPitch() {
    return gyro.getPitch();
  }

  // Loads the path. All paths should be loaded during robotInit() since this call is computationally expensive. Each path is stored and refered to by the provided index.
  // pathName: The name of the path in Path Planner
  // maxPathVel: Maximum robot velocity while following this path. Units: meters per second
  // maxPathAcc: Maximum robot acceleration while following this path. Units: meters per second^2
  // pathReversal: Whether the path should be followed in forwards or reverse. 
  public final void loadPath(String pathName, double maxPathVel, double maxPathAcc, boolean pathReversal) {
    paths.add(PathPlanner.loadPath(pathName, new PathConstraints(maxPathVel, maxPathAcc), pathReversal));
  }

  // Should be called once exactly 1 period prior to the start of calls to followPath() each time a new path is followed. 
  public final void resetPathController() {
    timer.restart();
    ramsete = new RamseteController(2, 0.7);
  }

  // Resets the robot's odometry to the start point of the path loaded into loadPath()
  public final void resetOdometryToPathStart(int pathIndex) {
    odometry.resetPosition(Rotation2d.fromDegrees(getYaw()), getLeftPos(), getRightPos(), paths.get(pathIndex).getInitialState().poseMeters);

  }

  // Resets the robot's odometry pose to the desired value. Units: meters and degrees. 
  public final void resetOdometry(double xPos, double yPos, double angPos) {
    odometry.resetPosition(Rotation2d.fromDegrees(getYaw()), getLeftPos(), getRightPos(), new Pose2d(xPos, yPos, Rotation2d.fromDegrees(angPos)));
  }

  // Tracks the path. Should be called each period until the endpoint is reached.
  public final void followPath(int pathIndex) {
    PathPlannerState currentGoal = (PathPlannerState) paths.get(pathIndex).sample(timer.get());
    setWheelSpeeds(kinematics.toWheelSpeeds(ramsete.calculate(odometry.getPoseMeters(), currentGoal)));
    SmartDashboard.putNumber("goalX", currentGoal.poseMeters.getX());
    SmartDashboard.putNumber("goalY", currentGoal.poseMeters.getY());
    SmartDashboard.putNumber("goalAng", currentGoal.poseMeters.getRotation().getDegrees());
  }

  // Tells whether the robot has reached the endpoint of the path, within the specified tolerance.
  // pathIndex: Which path to check, pathXTol and pathYTol: the allowable difference in position in meters, pathAngTol: the allowable difference in angle in degrees
  public final boolean atEndpoint(int pathIndex, double pathXTol, double pathYTol, double pathAngTol) {
    PathPlannerState endState = paths.get(pathIndex).getEndState();
    return Math.abs(getYaw() - endState.poseMeters.getRotation().getDegrees()) < pathAngTol 
    && Math.abs(getRobotX() - endState.poseMeters.getX()) < pathXTol 
    && Math.abs(getRobotY() - endState.poseMeters.getY()) < pathYTol;
  }

  private final void initializeDriveMotor(WPI_TalonFX motor) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.supplyCurrLimit.enable = true;
    config.supplyCurrLimit.triggerThresholdCurrent = 40; // max current in amps (40 was original)
    config.supplyCurrLimit.triggerThresholdTime = 0.5; // max time exceeding max current in seconds
    config.supplyCurrLimit.currentLimit = 40; // max current after exceeding threshold 

    // Sets the PID controller parameters
    double kI_drive = 0.0003;
    config.slot0.kP = 0.04;
    config.slot0.kI = kI_drive;
    config.slot0.kD = 1.0;
    config.slot0.kF = 0.0447;
    config.slot0.maxIntegralAccumulator = 0.8*1023.0/kI_drive;

    while (motor.configAllSettings(config, 30).value != 0) {}
    while (motor.setSelectedSensorPosition(0, 0, 30).value != 0) {}
    motor.setNeutralMode(NeutralMode.Brake);
  }
}