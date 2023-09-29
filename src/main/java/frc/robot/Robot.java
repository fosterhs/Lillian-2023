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
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  WPI_TalonFX rightFront = new WPI_TalonFX(4); // front right drive motor
  WPI_TalonFX rightBack = new WPI_TalonFX(3); // back right drive motor
  WPI_TalonFX leftFront = new WPI_TalonFX(2); // front left drive motor
  WPI_TalonFX leftBack = new WPI_TalonFX(1); // back left drive motor
  WPI_TalonFX topArm = new WPI_TalonFX(0); // top arm motor
  CANSparkMax bottomArm = new CANSparkMax(1, MotorType.kBrushed); // bottom arm motor
  MotorControllerGroup motorGroupLeft = new MotorControllerGroup(leftFront, leftBack);
  MotorControllerGroup motorGroupRight = new MotorControllerGroup(rightFront, rightBack);
  DifferentialDrive drive = new DifferentialDrive(motorGroupLeft, motorGroupRight);
  XboxController driveController = new XboxController(0);
  XboxController armController = new XboxController(1);
  SparkMaxAbsoluteEncoder bottomArmEncoder = bottomArm.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
  AHRS gyro = new AHRS(); // NavX2 gyro
  Timer timer = new Timer(); 
  double encoderTicksPerMeter = 2048*10.71/(0.0254*6*Math.PI); // theoretical 45812 ticks per meter traveled
  double encoderTicksPerRev = 2048*12*64/16; // theoretical 98304 ticks per revolution of top arm
  double stage2StartTime;
  PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);
  double totalCurrent = 0;
  double topArmCurrent = 0;

  // Pneumatics Variables
  Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  boolean solenoidToggle = false;

  // Arm Control Variables
  double armCoarseAdjustRate = 0.008;
  double bottomArmTolerance = 0.005;
  double topArmTolerance = 0.005;
  double armDeadband = 0.06;
  double topArmSetpoint = 0;
  double bottomArmSetpoint = 0;
  boolean armAtSetpoint = true;
  boolean bottomArmAtSetpoint = true;
  boolean topArmAtSetpoint = true;
  double minJoystickTopArmResponse = 0.1;
  double maxTopArmOutput = 1;
  double bottomArmAngle;
  double topArmAngle;
  double clawX;
  double clawZ;
  boolean proximitySensorStatus;
  double clawOpenTime;
  boolean clawAutoClosed = false;
  double clawAutoCloseTime;
  
  // Drive Variables
  double minJoystickDriveResponse = 0.24;

  // Controller Variables
  double d_leftStickY;
  double d_leftStickX;
  double d_rightStickY;
  double d_rightStickX;
  double d_leftTrigger;
  double d_rightTrigger;
  double a_leftStickY;
  double a_leftStickX;
  double a_rightStickY;
  double a_rightStickX;
  double a_leftTrigger;
  double a_rightTrigger;

  // Motor Variables
  double positionLeftBack;
  double positionLeftFront;
  double positionRightBack;
  double positionRightFront;
  double positionTopArm;
  double positionBottomArm;

  double angle;
  double time;
  double yaw;
  double pitch;

  // Auto Variables
  int autoStage = 1;
  PIDController pidSpeed = new PIDController(1, 0.2, 0.15);
  int settleInterations = 0;

  // Digital Inputs
  DigitalInput input0 = new DigitalInput(0);
  DigitalInput input1 = new DigitalInput(1);
  boolean sensor0;
  boolean sensor1;

  double autoClawOpenStageTime;

  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(0), 0, 0);
  PathPlannerTrajectory path = PathPlanner.loadPath("Test Path", new PathConstraints(0.8, 0.4));
  RamseteController ramsete = new RamseteController(2, 0.7);
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.55);
  double robotX = 0;
  double robotY = 0;

  @Override
  public void robotInit() {
    initializeMotors();
    gyro.calibrate();
    Timer.delay(2); // calibration delay for gyro
    gyro.zeroYaw();
    timer.start();
    updateVariables();
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    timer.reset();
    compressor.enableDigital();
  }

  @Override
  public void autonomousPeriodic() {
    updateVariables();

    // Trajectory following
    PathPlannerState currentGoal = (PathPlannerState) path.sample(timer.get());
    ChassisSpeeds chassisSpeeds = ramsete.calculate(odometry.getPoseMeters(), currentGoal);
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
    double leftSpeed = wheelSpeeds.leftMetersPerSecond/10*encoderTicksPerMeter;
    double rightSpeed = wheelSpeeds.rightMetersPerSecond/10*encoderTicksPerMeter;
    leftBack.set(ControlMode.Velocity, leftSpeed);
    leftFront.set(ControlMode.Velocity, leftSpeed);
    rightBack.set(ControlMode.Velocity, rightSpeed);
    rightFront.set(ControlMode.Velocity, rightSpeed);
    drive.feed();
  }

  @Override
  public void teleopInit() {
    timer.reset();
    compressor.enableDigital();
  }

  @Override
  public void teleopPeriodic() {
    updateVariables();

    // Proximity Sensor Code
    if (!clawAutoClosed && proximitySensorStatus && (a_leftTrigger > 0.25)) {
      closeClaw();
      clawAutoClosed = true;
      clawAutoCloseTime = time;
    } 
    if (!proximitySensorStatus && (a_leftTrigger > 0.25) && ((time - clawAutoCloseTime) > 1.0)) {
      openClaw();
      clawAutoClosed = false;
    }

    // Drive Code: leftStickY controls speed, rightStickX controls rotation.
    double translation = 0;
    if (Math.abs(d_leftStickY) > 0.04) {
      if (d_leftStickY > 0) {
        translation = -minJoystickDriveResponse-(1-minJoystickDriveResponse)*d_leftStickY*d_leftStickY;
      } else {
        translation = minJoystickDriveResponse+(1-minJoystickDriveResponse)*d_leftStickY*d_leftStickY;
      }
   }
   double rotation = 0;
   if (Math.abs(d_rightStickX) > 0.04) {
     if (d_rightStickX > 0) {
       rotation = -minJoystickDriveResponse-(0.5-minJoystickDriveResponse)*d_rightStickX*d_rightStickX;
     } else {
      rotation = minJoystickDriveResponse+(0.5-minJoystickDriveResponse)*d_rightStickX*d_rightStickX;
     }
   }
    drive.arcadeDrive(translation, rotation);

    // Left bumper: close the claw
    if (armController.getLeftBumperPressed()) {
      closeClaw();
    }
    // Right bumper: open the claw
    if (armController.getRightBumperPressed()) {
      openClaw();
      clawAutoClosed = false;
    }
    
    // Arm actuation code
    // Hands back user control to arm operator after setpoint malfunction
    if (armController.getPOV() == 360) {
      topArmSetpoint = positionTopArm;
      armAtSetpoint = true;
    }
    // Drive/Carry
    if (armController.getAButtonPressed()) {
      bottomArmSetpoint = 0;
      topArmSetpoint = -0.068;
      armAtSetpoint = false;
    }
    // Front Floor Pickup/Carry
    if (armController.getBButtonPressed()) {
      bottomArmSetpoint = 0.057;
      topArmSetpoint = -0.255;
      armAtSetpoint = false;
    } 
    // Double Substation Pickup
    if (armController.getXButtonPressed()) {
      bottomArmSetpoint = 0.095;
      topArmSetpoint = -0.073;
      armAtSetpoint = false;
    }
    // Cube Scoring (Middle)
    if (armController.getYButtonPressed()) {
      bottomArmSetpoint = 0.136;
      topArmSetpoint = -0.111;
      armAtSetpoint = false;
    }
    // Cube Scoring (Top)
    if (armController.getPOV() == 0) {
      bottomArmSetpoint = 0.245;
      topArmSetpoint = 0.043;
      armAtSetpoint = false;
    } 
    // Cone Scoring (Top)
    if (a_rightTrigger > 0.25) {
      bottomArmSetpoint = 0.256;
      topArmSetpoint = 0.110;
      armAtSetpoint = false;
    }
    // Neutral/Starting Position
    if (armController.getStartButtonPressed()) {
      bottomArmSetpoint = 0.01;
      topArmSetpoint = 0;
      armAtSetpoint = false;
    }
    // Cone Scoring (Middle)
    if (armController.getBackButtonPressed()) {
      bottomArmSetpoint = 0.158;
      topArmSetpoint = -0.040;
      armAtSetpoint = false;
    }

    if (!armAtSetpoint) {
      moveArmToSetpoint();
    } else {
      moveArmManual();
    }
  }

  @Override
  public void disabledInit() {
    timer.reset();
    compressor.disable();
  }

  @Override
  public void disabledPeriodic() {
    updateVariables();

    // Resets all timer/encoder/arm variables to 0 before the match.
    if (driveController.getStartButtonPressed()) {
      leftBack.setSelectedSensorPosition(0);
      leftFront.setSelectedSensorPosition(0);
      rightBack.setSelectedSensorPosition(0);
      rightFront.setSelectedSensorPosition(0);
      topArm.setSelectedSensorPosition(0);
      topArmSetpoint = positionTopArm;
      bottomArmSetpoint = positionBottomArm;
      armAtSetpoint = true;
      pidSpeed.reset();
      autoStage = 1;
      settleInterations = 0;
      timer.reset();
      updateVariables();
    }
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  public void moveArmToSetpoint() {
    bottomArmAtSetpoint = false;
    topArmAtSetpoint = false;
    
    if ((Math.abs(positionBottomArm - bottomArmSetpoint)) < bottomArmTolerance) {
      bottomArmAtSetpoint = true;
      bottomArm.set(0);
    }
    else if (positionBottomArm < bottomArmSetpoint) {
      bottomArm.set(1);
    } 
    else if (positionBottomArm > bottomArmSetpoint) {
      bottomArm.set(-1);
    }

    if ((Math.abs(positionTopArm - topArmSetpoint)) < topArmTolerance) {
      topArmAtSetpoint = true;
    }
    if (!topArmAtSetpoint) {
      topArm.set(ControlMode.MotionMagic, topArmSetpoint*encoderTicksPerRev);
    }

    if (topArmAtSetpoint && bottomArmAtSetpoint) {
      armAtSetpoint = true;
    }
  }

  public void moveArmManual() {
    if (Math.abs(a_leftStickY) > armDeadband && (positionBottomArm > 0.05 || a_leftStickY < 0)) {
      bottomArm.set(-a_leftStickY);
    } else {
      bottomArm.set(0);
    }

    if (Math.abs(a_rightStickY) > armDeadband) {
      if (a_rightStickY > 0) {
        topArmSetpoint = topArmSetpoint - (minJoystickTopArmResponse + (1-minJoystickTopArmResponse)*a_rightStickY*a_rightStickY)*armCoarseAdjustRate;
      } else if (a_rightStickY < 0) {
        topArmSetpoint = topArmSetpoint + (minJoystickTopArmResponse + (1-minJoystickTopArmResponse)*a_rightStickY*a_rightStickY)*armCoarseAdjustRate;
      }
    }
    if (topArmSetpoint > 0.6) {
      topArmSetpoint = 0.6;
    } 
    if (topArmSetpoint < -0.3) {
      topArmSetpoint = -0.3;
    }
    topArm.set(ControlMode.MotionMagic, encoderTicksPerRev*topArmSetpoint);
  }

  public void openClaw() {
    solenoid.set(DoubleSolenoid.Value.kForward);
    solenoidToggle = true;
  }

  public void closeClaw() {
    solenoid.set(DoubleSolenoid.Value.kReverse);
    solenoidToggle = false;
  }

  public void initializeCTREMotor(WPI_TalonFX motor) {
    motor.configFactoryDefault();
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.supplyCurrLimit.enable = true;
    config.supplyCurrLimit.triggerThresholdCurrent = 10; // max current in amps (40 was original)
    config.supplyCurrLimit.triggerThresholdTime = 0.1; // max time exceeding max current in seconds
    config.supplyCurrLimit.currentLimit = 10; // max current after exceeding threshold 
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

  public void initializeMotors() {
    initializeCTREMotor(topArm);
    initializeCTREMotor(leftBack);
    initializeCTREMotor(leftFront);
    initializeCTREMotor(rightBack);
    initializeCTREMotor(rightFront);
    
    // inverts right drive motors
    rightBack.setInverted(true);
    rightFront.setInverted(true);
    
    // sets motion magic parameters for topArm motor
    topArm.config_kP(0, 1);
    topArm.config_kI(0, 0.005);
    topArm.config_kD(0, 10);
    topArm.configMotionAcceleration(24000);
    topArm.configMotionCruiseVelocity(70000);
    topArm.configMaxIntegralAccumulator(0, 0.8);
    topArm.config_IntegralZone(0, 30000);
    topArm.configAllowableClosedloopError(0, 100);
    topArm.configClosedLoopPeakOutput(0, maxTopArmOutput);
    topArm.configPeakOutputForward(maxTopArmOutput);
    topArm.configPeakOutputReverse(-maxTopArmOutput);

    bottomArm.restoreFactoryDefaults(); // resets bottomArm to default
    bottomArm.setSmartCurrentLimit(8); // sets current limit for bottomArm in amps
    bottomArm.setInverted(true);
    bottomArm.setIdleMode(IdleMode.kBrake);
    bottomArmEncoder.setZeroOffset(0.224);
  }
  
  // updates all program variables. should be called at the begining of every loop.
  public void updateVariables() {
    positionBottomArm = bottomArmEncoder.getPosition(); // 0-1 represents 1 full revolution of the bottom arm. Starts at 0.
    if (positionBottomArm > 0.5) {
      positionBottomArm = positionBottomArm - 1;
    }
    sensor0 = input0.get();
    sensor1 = input1.get();
    if (!sensor0 || !sensor1) {
      proximitySensorStatus = true;
    } else {
      proximitySensorStatus = false;
    }
    positionTopArm = topArm.getSelectedSensorPosition()/encoderTicksPerRev; // 0-1 represents 1 full revolution of the top arm. Starts at 0.
    positionLeftBack = leftBack.getSelectedSensorPosition()/encoderTicksPerMeter; // wheel distance in meters. Starts at 0.
    positionLeftFront = leftFront.getSelectedSensorPosition()/encoderTicksPerMeter; // wheel distance in meters. Starts at 0. 
    positionRightBack = rightBack.getSelectedSensorPosition()/encoderTicksPerMeter; // wheel distance in meters. Starts at 0.
    positionRightFront = rightFront.getSelectedSensorPosition()/encoderTicksPerMeter; // wheel distance in meters. Starts at 0.
    yaw = gyro.getYaw(); // Starts at 0. The turning angle.
    pitch = gyro.getPitch(); // Starts at 0. The wheelie angle. 
    time = timer.get(); // Time in seconds. Resets to 0 after the robot is enabled/disabled.
    d_leftStickY = driveController.getLeftY();
    d_leftStickX = driveController.getLeftX();
    d_rightStickY = driveController.getRightY();
    d_rightStickX = driveController.getRightX();
    d_leftTrigger = driveController.getLeftTriggerAxis();
    d_rightTrigger = driveController.getRightTriggerAxis();
    a_leftStickY = armController.getLeftY();
    a_leftStickX = armController.getLeftX();
    a_rightStickY = armController.getRightY();
    a_rightStickX = armController.getRightX();
    a_leftTrigger = armController.getLeftTriggerAxis();
    a_rightTrigger = armController.getRightTriggerAxis();
    totalCurrent = pdp.getTotalCurrent();
    topArmCurrent = pdp.getCurrent(12);
    bottomArmAngle = positionBottomArm*360+10;
    topArmAngle = 90 - bottomArmAngle + 10 + positionTopArm*360;
    clawX = -0.72*Math.cos(bottomArmAngle*Math.PI/180) + 0.92*Math.cos(topArmAngle*Math.PI/180);
    clawZ = 0.72*Math.sin(bottomArmAngle*Math.PI/180) + 0.92*Math.sin(topArmAngle*Math.PI/180);

    odometry.update(Rotation2d.fromDegrees(-yaw), (positionLeftBack+positionLeftFront)/2, (positionRightBack+positionRightFront)/2);
    robotX = odometry.getPoseMeters().getX();
    robotY = odometry.getPoseMeters().getY();

    SmartDashboard.putNumber("robotX", robotX);
    SmartDashboard.putNumber("robotY", robotY);
    SmartDashboard.putBoolean("Sensor", proximitySensorStatus);
    SmartDashboard.putNumber("Bottom Arm Angle", bottomArmAngle);
    SmartDashboard.putNumber("Top Arm Angle", topArmAngle);
    SmartDashboard.putNumber("Claw X", clawX);
    SmartDashboard.putNumber("Claw Y", clawZ);
    SmartDashboard.putNumber("Total I", totalCurrent);
    SmartDashboard.putNumber("Arm I", topArmCurrent);
    SmartDashboard.putNumber("Bottom Arm Position", positionBottomArm);
    SmartDashboard.putNumber("Top Arm Position", positionTopArm);
    SmartDashboard.putNumber("Left Back Position", positionLeftBack);
    SmartDashboard.putNumber("Left Front Position", positionLeftFront);
    SmartDashboard.putNumber("Right Back Position", positionRightBack);
    SmartDashboard.putNumber("Right Front Position", positionRightFront);
    SmartDashboard.putNumber("Yaw", yaw);
    SmartDashboard.putNumber("Pitch", pitch);
    SmartDashboard.putNumber("Time", time);
    SmartDashboard.putNumber("d_leftStickY", d_leftStickY);
    SmartDashboard.putNumber("d_leftStickX", d_leftStickX);
    SmartDashboard.putNumber("d_rightStickY", d_rightStickY);
    SmartDashboard.putNumber("d_rightStickX", d_rightStickX);
    SmartDashboard.putNumber("d_leftTrigger", d_leftTrigger);
    SmartDashboard.putNumber("d_rightTrigger", d_rightTrigger);
    SmartDashboard.putNumber("a_leftStickY", a_leftStickY);
    SmartDashboard.putNumber("a_leftStickX", a_leftStickX);
    SmartDashboard.putNumber("a_rightStickY", a_rightStickY);
    SmartDashboard.putNumber("a_rightStickX", a_rightStickX);
    SmartDashboard.putNumber("a_leftTrigger", a_leftTrigger);
    SmartDashboard.putNumber("a_rightTrigger", a_rightTrigger);
    SmartDashboard.putBoolean("Solenoid", solenoidToggle);
    SmartDashboard.putBoolean("At Setpoint", armAtSetpoint);
    SmartDashboard.putBoolean("Top At Setpoint", topArmAtSetpoint);
    SmartDashboard.putBoolean("Bottom At Setpoint", bottomArmAtSetpoint);
    SmartDashboard.putNumber("Angle", angle);
    SmartDashboard.putNumber("Stage", autoStage);
    SmartDashboard.putNumber("Top Arm Setpoint", topArmSetpoint);
    SmartDashboard.putBoolean("Proximity Sensor 0", sensor0);
    SmartDashboard.putBoolean("Proximity Sensor 1", sensor1);
  }
}