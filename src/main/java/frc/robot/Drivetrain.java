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

public class Drivetrain {
  private WPI_TalonFX rightFront = new WPI_TalonFX(4); // front right drive motor
  private WPI_TalonFX rightBack = new WPI_TalonFX(3); // back right drive motor
  private WPI_TalonFX leftFront = new WPI_TalonFX(2); // front left drive motor
  private WPI_TalonFX leftBack = new WPI_TalonFX(1); // back left drive motor
  private MotorControllerGroup motorGroupLeft = new MotorControllerGroup(leftFront, leftBack);
  private MotorControllerGroup motorGroupRight = new MotorControllerGroup(rightFront, rightBack);
  private DifferentialDrive tank = new DifferentialDrive(motorGroupLeft, motorGroupRight);
  private AHRS gyro = new AHRS(); // NavX2 gyro
  private static final double trackWidth = 0.55;
  private static final double falconTicksPerRev = 2048.0;
  private static final double wheelCirc = 6*0.0254*Math.PI;
  private static final double gearRatio = 10.71;
  private static final double ticksPerMeter = falconTicksPerRev*gearRatio/wheelCirc;
  private static final double maxAngVelPower = 0.55;
  private static final double xAccLimit = 3.0;
  private static final double AngAccLimit = 3.0/maxAngVelPower;
  private static final double driveControllerDeadband = 0.05;
  private SlewRateLimiter xAccLimiter = new SlewRateLimiter(xAccLimit);
  private SlewRateLimiter angAccLimiter = new SlewRateLimiter(AngAccLimit);
  private DifferentialDriveOdometry odometry;
  private PathPlannerTrajectory path;
  private RamseteController ramsete;
  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(trackWidth);
  private Timer timer;
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
  public void driveManual(double xPower, double angPower) {
    double xVel = xAccLimiter.calculate(MathUtil.applyDeadband(xPower, driveControllerDeadband));
    double angVel = angAccLimiter.calculate(MathUtil.applyDeadband(angPower, driveControllerDeadband)*maxAngVelPower);
    tank.arcadeDrive(xVel, angVel);
  }
  
  // Autonomous control of the drivetrain based on calculated wheel speeds. One of the driving methods should be called each period.
  private void setWheelSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds) {
    rightFront.set(ControlMode.Velocity, wheelSpeeds.rightMetersPerSecond/10*ticksPerMeter);
    leftFront.set(ControlMode.Velocity, wheelSpeeds.rightMetersPerSecond/10*ticksPerMeter);
    rightBack.set(ControlMode.Follower, 4);
    leftBack.set(ControlMode.Follower, 2);
    tank.feed();
  }

  public double getLeftPos() {
    return (leftFront.getSelectedSensorPosition() + leftBack.getSelectedSensorPosition())/(2*ticksPerMeter);
  }

  public double getRightPos() {
    return (rightFront.getSelectedSensorPosition() + rightBack.getSelectedSensorPosition())/(2*ticksPerMeter);
  }

  public double getYaw() {
    return -gyro.getYaw();
  }

  public double getPitch() {
    return gyro.getPitch();
  }

  // Loads the path. Should be called immediately before the user would like the robot to begin tracking the path. Assumes the robot is starting at the field position indicated at the start of the path.
  public void loadPath(String pathName) {
    timer = new Timer(); 
    timer.start();
    path = PathPlanner.loadPath(pathName, new PathConstraints(maxPathVel, maxPathAcc), pathReversal);
    PathPlannerState startingState = path.getInitialState();
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getYaw()), getLeftPos(), getRightPos(), startingState.poseMeters);
    ramsete = new RamseteController(2, 0.7);
  }

  // Tracks the path. Should be called each period until the endpoint is reached.
  public void followPath() {
    PathPlannerState currentGoal = (PathPlannerState) path.sample(timer.get());
    odometry.update(Rotation2d.fromDegrees(getYaw()), getLeftPos(), getRightPos());
    setWheelSpeeds(kinematics.toWheelSpeeds(ramsete.calculate(odometry.getPoseMeters(), currentGoal)));
  }

  // Tells whether the robot has reached the endpoint, within the defined tolerance.
  public boolean atEndpoint() {
    PathPlannerState endState = path.getEndState();
    return Math.abs(odometry.getPoseMeters().getRotation().getDegrees() - endState.poseMeters.getRotation().getDegrees()) < pathAngTol 
    && Math.abs(odometry.getPoseMeters().getX() - endState.poseMeters.getX()) < pathXTol 
    && Math.abs(odometry.getPoseMeters().getY() - endState.poseMeters.getY()) < pathYTol;
  }
  
  // Path parameters should be adjusted prior to calling loadPath()
  public void setPathXTol(double desiredXTol) {
    pathXTol = desiredXTol;
  }

  public void setPathYTol(double desiredYTol) {
    pathYTol = desiredYTol;
  }

  public void setPathAngTol(double desiredAngTol) {
    pathAngTol = desiredAngTol;
  }

  public void setMaxPathVel(double desiredMaxVel) {
    maxPathVel = desiredMaxVel;
  }

  public void setPathReversal(boolean desiredReversal) {
    pathReversal = desiredReversal;
  }

  public void setMaxPathAcc(double desiredMaxAcc) {
    maxPathAcc = desiredMaxAcc;
  }

  private void initializeDriveMotor(WPI_TalonFX motor) {
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