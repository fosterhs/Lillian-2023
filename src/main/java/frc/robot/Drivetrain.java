package frc.robot;

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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain {
  private final WPI_TalonFX rightFront = new WPI_TalonFX(4); // front right drive motor
  private final WPI_TalonFX rightBack = new WPI_TalonFX(3); // back right drive motor
  private final WPI_TalonFX leftFront = new WPI_TalonFX(2); // front left drive motor
  private final WPI_TalonFX leftBack = new WPI_TalonFX(1); // back left drive motor
  private final MotorControllerGroup leftMotors = new MotorControllerGroup(leftFront, leftBack);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(rightFront, rightBack);
  private final DifferentialDrive tank = new DifferentialDrive(leftMotors, rightMotors);
  private final AHRS gyro = new AHRS(); // NavX2 gyro
  private static final double trackWidth = 0.55;
  private static final double falconTicksPerRev = 2048.0;
  private static final double wheelCirc = 6*0.0254*Math.PI;
  private static final double gearRatio = 10.71;
  private static final double ticksPerMeter = falconTicksPerRev*gearRatio/wheelCirc;
  private static final double maxTurnPower = 0.55;
  private static final double xAccLimit = 3.0;
  private static final double AngAccLimit = 3.0/maxTurnPower;
  private static final double driveControllerDeadband = 0.05;
  private final SlewRateLimiter xAccLimiter = new SlewRateLimiter(xAccLimit);
  private final SlewRateLimiter angAccLimiter = new SlewRateLimiter(AngAccLimit);
  private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0), 0, 0);
  private PathPlannerTrajectory path;
  private RamseteController ramsete;
  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(trackWidth);
  private final Timer timer = new Timer();
  private double pathXTol = 0.03;
  private double pathYTol = 0.03;
  private double pathAngTol = 3.0;
  private double maxPathVel = 0.8;
  private double maxPathAcc = 0.4;
  private boolean pathReversal = false;

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
  public final void driveManual(double drivePower, double turnPower) {
    double commandedDrivePower = xAccLimiter.calculate(MathUtil.applyDeadband(drivePower, driveControllerDeadband));
    double commandedTurnPower = angAccLimiter.calculate(MathUtil.applyDeadband(turnPower, driveControllerDeadband)*maxTurnPower);
    tank.arcadeDrive(commandedDrivePower, commandedTurnPower);
    updateOdometry();
  }
  
  // Autonomous control of the drivetrain based on calculated wheel speeds. One of the driving methods should be called each period.
  private final void setWheelSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds) {
    rightFront.set(ControlMode.Velocity, wheelSpeeds.rightMetersPerSecond/10*ticksPerMeter);
    leftFront.set(ControlMode.Velocity, wheelSpeeds.leftMetersPerSecond/10*ticksPerMeter);
    rightBack.set(ControlMode.Follower, 4);
    leftBack.set(ControlMode.Follower, 2);
    tank.feed();
    updateOdometry();
  }

  private final void updateOdometry() {
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

  // Loads the path. Should be called immediately before the user would like the robot to begin tracking the path. Assumes the robot is starting at the field position indicated at the start of the path.
  public final void loadPath(String pathName) {
    timer.restart();
    path = PathPlanner.loadPath(pathName, new PathConstraints(maxPathVel, maxPathAcc), pathReversal);
    PathPlannerState startingState = path.getInitialState();
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getYaw()), getLeftPos(), getRightPos(), startingState.poseMeters);
    ramsete = new RamseteController(2, 0.7);
  }

  // Tracks the path. Should be called each period until the endpoint is reached.
  public final void followPath() {
    PathPlannerState currentGoal = (PathPlannerState) path.sample(timer.get());
    setWheelSpeeds(kinematics.toWheelSpeeds(ramsete.calculate(odometry.getPoseMeters(), currentGoal)));
    SmartDashboard.putNumber("goalX", currentGoal.poseMeters.getX());
    SmartDashboard.putNumber("goalY", currentGoal.poseMeters.getY());
    SmartDashboard.putNumber("goalAng", currentGoal.poseMeters.getRotation().getDegrees());
  }

  // Tells whether the robot has reached the endpoint, within the defined tolerance.
  public final boolean atEndpoint() {
    PathPlannerState endState = path.getEndState();
    return Math.abs(getYaw() - endState.poseMeters.getRotation().getDegrees()) < pathAngTol 
    && Math.abs(getRobotX() - endState.poseMeters.getX()) < pathXTol 
    && Math.abs(getRobotY() - endState.poseMeters.getY()) < pathYTol;
  }
  
  // Path parameters should be adjusted prior to calling loadPath()
  public final void setPathXTol(double desiredXTol) {
    pathXTol = desiredXTol;
  }

  public final void setPathYTol(double desiredYTol) {
    pathYTol = desiredYTol;
  }

  public final void setPathAngTol(double desiredAngTol) {
    pathAngTol = desiredAngTol;
  }

  public final void setMaxPathVel(double desiredMaxVel) {
    maxPathVel = desiredMaxVel;
  }

  public final void setPathReversal(boolean desiredReversal) {
    pathReversal = desiredReversal;
  }

  public final void setMaxPathAcc(double desiredMaxAcc) {
    maxPathAcc = desiredMaxAcc;
  }

  private final void initializeDriveMotor(WPI_TalonFX motor) {
    motor.configFactoryDefault();

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.supplyCurrLimit.enable = true;
    config.supplyCurrLimit.triggerThresholdCurrent = 40; // max current in amps (40 was original)
    config.supplyCurrLimit.triggerThresholdTime = 0.1; // max time exceeding max current in seconds
    config.supplyCurrLimit.currentLimit = 40; // max current after exceeding threshold 
    motor.configAllSettings(config);

    motor.setSelectedSensorPosition(0); // sets encoder to 0
    motor.setNeutralMode(NeutralMode.Brake);

    double kI_drive = 0.0003;
    motor.config_kP(0, 0.04);
    motor.config_kI(0, kI_drive);
    motor.config_kD(0, 1);
    motor.config_kF(0, 0.0447);
    motor.configMaxIntegralAccumulator(0, 0.8*1023/kI_drive);
  }
}