package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Robot extends TimedRobot {
  PIDController pid = new PIDController(1, 0.25, 0.5);
  WPI_TalonFX left = new WPI_TalonFX(2); // left drive motor
  WPI_TalonFX right = new WPI_TalonFX(0); // right drive motor
  WPI_TalonFX belt = new WPI_TalonFX(1); // belt motor
  WPI_TalonFX intakeInternal = new WPI_TalonFX(3); // internal intake motor
  WPI_TalonFX intakeExternal = new WPI_TalonFX(4); // external intake motor
  DifferentialDrive drive = new DifferentialDrive(left, right);
  XboxController controller = new XboxController(0);
  ADIS16448_IMU gyro = new ADIS16448_IMU(); // RoboRIO-mounted gyroscope
  Timer timer = new Timer(); 
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(0), 0,0);
  double encoderTicksPerMeter = 2048*10.71/(0.0254*6*Math.PI); // theoretical 45812 ticks per meter
  double trackWidth = 0.69474;  // value obtained from SysID
  // controller inputs
  double leftStickY;
  double leftStickX;
  double rightStickY;
  double rightStickX;
  double leftTrigger;
  double rightTrigger;
  // motor encoder values
  double positionLeft;
  double positionRight;
  double positionAverage = (positionLeft+positionRight)/2;
  double positionBelt;
  double positionInternalIntake;
  double positionExternalIntake;
  double time; // match time
  // odometry calculated robot position
  Pose2d robotPosition;
  double robotX;
  double robotY;
  double angle; // gyro angle
  double distance = 1;
  double error = distance-positionAverage;
  double xSpeed;//declaration

  @Override
  public void robotInit() {
    initializeMotors(); // starts and configures the motors
    timer.start(); // starts the timer at 0s.
    gyro.calibrate(); // sets the gyro angle to 0 based on the current robot position 
    updateVariables(); // updates and publishes variables to shuffleboard
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    timer.reset(); // sets the timer to 0
    left.setNeutralMode(NeutralMode.Brake);
    right.setNeutralMode(NeutralMode.Brake);
    updateVariables();
  }

  @Override
  public void autonomousPeriodic() {
    updateVariables();
    drive.arcadeDrive(pid.calculate((positionLeft+positionRight), distance), 0);
  }

  @Override
  public void teleopInit() {
    timer.reset(); // sets the timer to 0
    left.setNeutralMode(NeutralMode.Brake);
    right.setNeutralMode(NeutralMode.Brake);
    updateVariables();
  }

  @Override
  public void teleopPeriodic() {
    updateVariables();
    
    // sets motor speeds based on controller inputs
    drive.arcadeDrive(-leftStickY, -rightStickX, true);
    belt.set(-rightTrigger);
    intakeExternal.set(leftTrigger);
    intakeInternal.set(leftTrigger);
  }

  @Override
  public void disabledInit() {
    left.setNeutralMode(NeutralMode.Coast);
    right.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  public void initializeMotors() {
    motorConfig(left);
    //PID coefficients
    left.config_kF(0, 0.046, 30);
    left.config_kP(0, 0.05, 30);
    left.config_kI(0, 0.001, 30);
    left.config_kD(0, 3, 30);
    left.setNeutralMode(NeutralMode.Coast); 

    motorConfig(right);
    right.config_kF(0, 0.046, 30);
    right.config_kP(0, 0.05, 30);
    right.config_kI(0, 0.001, 30);
    right.config_kD(0, 3, 30);
    right.setInverted(true);
    right.setNeutralMode(NeutralMode.Coast); 

    motorConfig(belt);
    intakeInternal.config_kF(0, 0, 30);
    intakeInternal.config_kP(0, 1, 30);
    intakeInternal.config_kI(0, 0.005, 30);
    intakeInternal.config_kD(0, 10, 30);
    belt.configMotionCruiseVelocity(20000, 30);
    belt.configMotionAcceleration(6000, 30);
    belt.setNeutralMode(NeutralMode.Brake); 

    motorConfig(intakeInternal);
    intakeInternal.config_kF(0, 0, 30);
    intakeInternal.config_kP(0, 1, 30);
    intakeInternal.config_kI(0, 0.005, 30);
    intakeInternal.config_kD(0, 10, 30);
    intakeInternal.configMotionCruiseVelocity(20000, 30);
    intakeInternal.configMotionAcceleration(6000, 30);
    intakeInternal.setNeutralMode(NeutralMode.Brake);

    motorConfig(intakeExternal);
    intakeExternal.config_kF(0, 0, 30);
    intakeExternal.config_kP(0, 1, 30);
    intakeExternal.config_kI(0, 0.005, 30);
    intakeExternal.config_kD(0, 10, 30);
    intakeExternal.configMotionCruiseVelocity(20000, 30);
    intakeExternal.configMotionAcceleration(6000, 30);
    intakeExternal.setNeutralMode(NeutralMode.Brake);
  }

  // runs manufacturer recommended startup commands for Falcon 500 motors. Should be run at startup for all motors.
  public void motorConfig(WPI_TalonFX motor) {
    motor.configFactoryDefault();
    motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0 , 30);
    motor.configNeutralDeadband(0.001, 30);
    motor.setSensorPhase(false);
    motor.setInverted(false);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);
    motor.configNominalOutputForward(0, 30);
    motor.configNominalOutputReverse(0, 30);
    motor.configPeakOutputForward(1, 30);
    motor.configPeakOutputReverse(-1, 30);
    motor.selectProfileSlot(0, 0);
    motor.configOpenloopRamp(1.5);
    motor.setSelectedSensorPosition(0, 0, 30);
  }
  
  // initializes variables and publishes values on dashboard
  public void updateVariables() {
    // updates variables
    leftStickY = controller.getLeftY();
    leftStickX = controller.getLeftX();
    rightStickY = controller.getRightY();
    rightStickX = controller.getRightX();
    leftTrigger = controller.getLeftTriggerAxis();
    rightTrigger = controller.getRightTriggerAxis();
    positionLeft = left.getSelectedSensorPosition(0)/encoderTicksPerMeter;
    positionRight = right.getSelectedSensorPosition(0)/encoderTicksPerMeter;
    positionBelt = belt.getSelectedSensorPosition(0);
    positionInternalIntake = intakeInternal.getSelectedSensorPosition(0);
    positionExternalIntake = intakeExternal.getSelectedSensorPosition(0);
    time = timer.get();
    angle = -gyro.getGyroAngleZ();
    odometry.update(new Rotation2d(angle*Math.PI/180), positionLeft, positionRight);
    robotPosition = odometry.getPoseMeters();
    robotX = robotPosition.getX();
    robotY = robotPosition.getY();
    xSpeed += 1;// update
    
    // publishes updated variables to the dashboard
    SmartDashboard.putNumber("leftStickY", leftStickY);
    SmartDashboard.putNumber("leftStickX", leftStickX);
    SmartDashboard.putNumber("rightStickY", rightStickY);
    SmartDashboard.putNumber("rightStickX", rightStickX);
    SmartDashboard.putNumber("rightTrigger", rightTrigger);
    SmartDashboard.putNumber("leftTrigger", leftTrigger);
    SmartDashboard.putNumber("Encoder (Left)", positionLeft);
    SmartDashboard.putNumber("Encoder (Right)", positionRight);
    SmartDashboard.putNumber("Encoder (Belt)", positionBelt);
    SmartDashboard.putNumber("Encoder (External Intake)", positionExternalIntake);
    SmartDashboard.putNumber("Encoder (Internal Intake)", positionInternalIntake);
    SmartDashboard.putNumber("Clock",  time);
    SmartDashboard.putNumber("Angle", angle);
    SmartDashboard.putNumber("RobotX",  robotX);
    SmartDashboard.putNumber("RobotY", robotY);
    SmartDashboard.putNumber("xSpeed", xSpeed);//publish
  }
}