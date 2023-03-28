package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

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
  RelativeEncoder bottomArmEncoder = bottomArm.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 2048); //bottom arm encoder
  AHRS gyro = new AHRS(); // NavX2 gyro
  Timer timer = new Timer(); 
  double encoderTicksPerMeter = 2048*10.71/(0.0254*6*Math.PI); // theoretical 45812 ticks per meter traveled
  double encoderTicksPerRev = 2048*12*64/16; // theoretical 98304 ticks per revolution of top arm
  double stage2StartTime;

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
  double minJoystickTopArmResponse = 0.2;
  
  // Drive Variables
  double minJoystickDriveResponse = 0.25;

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
  PIDController pidSpeed = new PIDController(1, 0.2, 0.2);
  int settleInterations = 0;

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
    
    // Auto Arm First Setpoint
    bottomArmSetpoint = 0;
    topArmSetpoint = 0.141;
    armAtSetpoint = false;
  }

  @Override
  public void autonomousPeriodic() {
    updateVariables();
    if (autoStage == 1) { // Moves the arm to the 1st auto setpoint
      closeClaw();
      drive.arcadeDrive(0, 0);
      if (!armAtSetpoint) {
        moveArmToSetpoint();
      }
      if (armAtSetpoint) {
        autoStage++;
        stage2StartTime = timer.get();
      }  
    } else if (autoStage == 2) { // Delays 2 seconds, then opens the claw
      drive.arcadeDrive(0, 0);
      if ((timer.get() - stage2StartTime) > 2.0) {
        openClaw();
        autoStage++;
      }
    } else if (autoStage == 3) { // Moves forward by a set distance using PID control
      double averagePosition = (positionLeftBack + positionLeftFront + positionRightBack + positionRightFront)/4;
      double translation = pidSpeed.calculate(averagePosition, 3.2);
      drive.arcadeDrive(translation, 0);
      if (averagePosition > 3.2) {
        autoStage++;
        // Auto Arm Second Setpoint
        bottomArmSetpoint = 0;
        topArmSetpoint = -0.246;
        armAtSetpoint = false;
      }
    } else if (autoStage == 4) { // Moves the arm to the 2nd Auto Setpoint
      drive.arcadeDrive(0, 0);
      if (!armAtSetpoint) {
        moveArmToSetpoint();
      }
      if (armAtSetpoint) {
        autoStage++;
      }
    } else if (autoStage == 5) {
        closeClaw();
    } else {
      drive.arcadeDrive(0, 0);
    }
  }

  @Override
  public void teleopInit() {
    timer.reset();
    compressor.enableDigital();
  }

  @Override
  public void teleopPeriodic() {
    updateVariables();

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
    }
    
    // Arm actuation code

    // Neutral/Starting Position
    if (armController.getStartButtonPressed()) {
      bottomArmSetpoint = 0;
      topArmSetpoint = 0;
      armAtSetpoint = false;
    }
    // Drive/Carry
    if (armController.getAButtonPressed()) {
      bottomArmSetpoint = 0;
      topArmSetpoint = 0;
      armAtSetpoint = false;
    }
    // Front Floor Pickup/Carry
    if (armController.getBButtonPressed()) {
      bottomArmSetpoint = 0;
      topArmSetpoint = 0.198;
      armAtSetpoint = false;
    } 
    // Double Substation Pickup
    if (armController.getXButtonPressed()) {
      bottomArmSetpoint = 0;
      topArmSetpoint = -0.251;
      armAtSetpoint = false;
    }
    // Cube Scoring (Middle)
    if (armController.getYButtonPressed()) {
      bottomArmSetpoint = 0;
      topArmSetpoint = 0.141;
      armAtSetpoint = false;
    }
    // Cone Scoring (Middle)
    if (armController.getBackButtonPressed()) {
      bottomArmSetpoint = 0.15;
      topArmSetpoint = 0;
      armAtSetpoint = false;
    }
    // Cube Scoring (Top)
    if (a_leftTrigger > 0.25) {
      bottomArmSetpoint = 0;
      topArmSetpoint = 0.238;
      armAtSetpoint = false;
    }
    // Cone Scoring (Top)
    if (a_rightTrigger > 0.25) {
      bottomArmSetpoint = 0.119;
      topArmSetpoint = 0;
      armAtSetpoint = false;
    }

    if (!armAtSetpoint) {
      moveArmToSetpoint();
    }
    if (armAtSetpoint) {
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
      bottomArmEncoder.setPosition(0);
      topArmSetpoint = positionTopArm;
      bottomArmSetpoint = positionBottomArm;
      armAtSetpoint = true;
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
    bottomArmAtSetpoint = true;
    topArmAtSetpoint = false;
    
    if (!bottomArmAtSetpoint) {
      if (positionBottomArm < bottomArmSetpoint) {
        bottomArm.set(1);
      } 
      else if (positionBottomArm > bottomArmSetpoint) {
        bottomArm.set(-1);
      }
      if ((Math.abs(positionBottomArm - bottomArmSetpoint)) < bottomArmTolerance) {
        bottomArmAtSetpoint = true;
        bottomArm.set(0);
      }
    }

    if (!topArmAtSetpoint) {
      topArm.set(ControlMode.MotionMagic, topArmSetpoint*encoderTicksPerRev);
    }
    if ((Math.abs(positionTopArm - topArmSetpoint)) < topArmTolerance) {
      topArmAtSetpoint = true;
    }

    if (topArmAtSetpoint && bottomArmAtSetpoint) {
      armAtSetpoint = true;
    }
  }

  public void moveArmManual() {
    if (Math.abs(a_leftStickY) > armDeadband) {
      bottomArm.set(a_leftStickY);
    } else {
      bottomArm.set(0);
    }
    if (Math.abs(a_rightStickY) > armDeadband) {
      if (a_rightStickY > 0) {
        topArmSetpoint = topArmSetpoint - (minJoystickTopArmResponse + (1-minJoystickTopArmResponse)*a_rightStickY*a_rightStickY)*armCoarseAdjustRate;
      } else {
        topArmSetpoint = topArmSetpoint + (minJoystickTopArmResponse + (1-minJoystickTopArmResponse)*a_rightStickY*a_rightStickY)*armCoarseAdjustRate;
      }
    } 
    topArm.set(ControlMode.MotionMagic, encoderTicksPerRev*topArmSetpoint);
  }

  public void openClaw() {
    solenoid.set(DoubleSolenoid.Value.kReverse);
    solenoidToggle = true;
  }

  public void closeClaw() {
    solenoid.set(DoubleSolenoid.Value.kForward);
    solenoidToggle = false;
  }

  public void brakeMotors() {
    topArm.setNeutralMode(NeutralMode.Brake);
    leftBack.setNeutralMode(NeutralMode.Brake);
    leftFront.setNeutralMode(NeutralMode.Brake);
    rightBack.setNeutralMode(NeutralMode.Brake);
    rightFront.setNeutralMode(NeutralMode.Brake);
    bottomArm.setIdleMode(IdleMode.kBrake);
  }

  public void initializeCTREMotor(WPI_TalonFX motor) {
    motor.configFactoryDefault();
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.supplyCurrLimit.enable = true;
    config.supplyCurrLimit.triggerThresholdCurrent = 40; // max current in amps
    config.supplyCurrLimit.triggerThresholdTime = 0.1; // max time exceeding max current in seconds
    config.supplyCurrLimit.currentLimit = 40; // max current after exceeding threshold 
    motor.configAllSettings(config);
    motor.setSelectedSensorPosition(0); // sets encoder to 0
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
    topArm.configMotionAcceleration(16000);
    topArm.configMotionCruiseVelocity(50000);
    topArm.configMaxIntegralAccumulator(0, 0.2);
    topArm.config_IntegralZone(0, 20000);
    topArm.configAllowableClosedloopError(0, 100);

    bottomArm.restoreFactoryDefaults(); // resets bottomArm to default
    bottomArm.setSmartCurrentLimit(8); // sets current limit for bottomArm in amps
    brakeMotors();
    bottomArm.setInverted(true);
  }
  
  // updates all program variables. should be called at the begining of every loop.
  public void updateVariables() {
    positionBottomArm = bottomArmEncoder.getPosition()/2; // 0-1 represents 1 full revolution of the bottom arm. Starts at 0.
    if (positionBottomArm > 140) { // Resolves encoder negative overrun issue
      positionBottomArm = positionBottomArm - 274.6583251953125;
    }
    //positionBottomArm = bottomArmEncoderAbs.getPosition();
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
  }
}