package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
  double encoderTicksPerRev = 2048*12*64/18; // theoretical 87381 ticks per revolution of top arm
  boolean coast = true;

  // Pneumatics Variables
  Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  boolean compressorToggle = false;
  boolean solenoidToggle = false;

  // Arm Control Variables
  double armCoarseAdjustRate = 0.008;
  double armFineAdjustRange = 0.04;
  double bottomArmTolerance = 0.005;
  double topArmTolerance = 0.005;
  double armDeadband = 0.05;
  double topArmSetpoint = 0;
  double bottomArmSetpoint = 0;
  boolean armAtSetpoint = true;
  boolean bottomArmAtSetpoint = true;
  boolean topArmAtSetpoint = true;
  
  // Drive Variables
  SlewRateLimiter slewSpeedController = new SlewRateLimiter(0.60);
  SlewRateLimiter slewRotationController = new SlewRateLimiter(60.0);
  double maxRotationSpeed = 0.40;
  double minJoystickResponse = 0.24;

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
  int autoStage = 1;
  PIDController pidSpeed = new PIDController(1, 0.2, 1);
  PIDController pidRotate = new PIDController(0.04, 0, 0);

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
    brakeMotors();
    compressor.enableDigital();
    
    // Auto Arm Variables
    bottomArmSetpoint = 0.15;
    topArmSetpoint = 0.1;
    armAtSetpoint = false;
    bottomArmAtSetpoint = false;
    topArmAtSetpoint = false;
  }

  @Override
  public void autonomousPeriodic() {
    updateVariables();
    if (autoStage == 1) {
      solenoid.set(DoubleSolenoid.Value.kForward);
      drive.feed();
      if (!armAtSetpoint) {
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
          autoStage++;
        }
      }     
    } else if (autoStage == 2) {
      solenoid.set(DoubleSolenoid.Value.kReverse);
      autoStage++;
      drive.feed();
    } else if (autoStage == 3) {
      // Robot moves 2 meters forward
      double averagePosition = (positionLeftBack + positionLeftFront + positionRightBack + positionRightFront)/4;
      double translation = pidSpeed.calculate(averagePosition, 2);
      drive.arcadeDrive(translation, 0);
      if (averagePosition > 2) {
        autoStage++;
      }
    } else {
      drive.feed();
    }
  }

  @Override
  public void teleopInit() {
    timer.reset();
    brakeMotors();
    compressor.enableDigital();
     // sets motors to brake when 0 commands are given
  }

  @Override
  public void teleopPeriodic() {
    updateVariables();
    // Pneumatics Code
    // Right Bumper will toggle the solenoid
    if (armController.getRightBumperPressed()) {
      solenoidToggle = !solenoidToggle;
    }
    if (solenoidToggle) {
      solenoid.set(DoubleSolenoid.Value.kForward);
    } else {
      solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    // sets motor speeds based on controller inputs with slew rate and max speed protections.
    // left stick Y controls speed, right stick X controls rotation.
    double translation = 0;
    if (Math.abs(d_leftStickY) > 0.02) {
      if (d_leftStickY > 0) {
        translation = -minJoystickResponse-(1-minJoystickResponse)*d_leftStickY*d_leftStickY;
      } else {
        translation = minJoystickResponse+(1-minJoystickResponse)*d_leftStickY*d_leftStickY;
      }
   }
   double rotation = 0;
   if (Math.abs(d_rightStickX) > 0.02) {
     if (d_rightStickX > 0) {
       rotation = -minJoystickResponse-(0.6-minJoystickResponse)*d_rightStickX*d_rightStickX;
     } else {
      rotation = minJoystickResponse+(0.6-minJoystickResponse)*d_rightStickX*d_rightStickX;
     }
   }
    drive.arcadeDrive(translation, rotation);
    
    // Arm actuation code
    if (armController.getAButtonPressed()) {
      bottomArmSetpoint = 0;
      topArmSetpoint = -0.1;
      armAtSetpoint = false;
      bottomArmAtSetpoint = false;
      topArmAtSetpoint = false;
    }
    if (armController.getBButtonPressed()) {
      bottomArmSetpoint = 0.05;
      topArmSetpoint = -0.05;
      armAtSetpoint = false;
      bottomArmAtSetpoint = false;
      topArmAtSetpoint = false;
    }
    if (armController.getXButtonPressed()) {
      bottomArmSetpoint = 0.1;
      topArmSetpoint = 0.05;
      armAtSetpoint = false;
      bottomArmAtSetpoint = false;
      topArmAtSetpoint = false;
    }
    if (armController.getYButtonPressed()) {
      bottomArmSetpoint = 0.15;
      topArmSetpoint = 0.1;
      armAtSetpoint = false;
      bottomArmAtSetpoint = false;
      topArmAtSetpoint = false;
    }
    if (armController.getLeftBumperPressed()) {
      topArmSetpoint = positionTopArm;
      armAtSetpoint = true;
    }

    if (!armAtSetpoint) {
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

    if (armAtSetpoint) {
      if (Math.abs(a_rightTrigger-a_leftTrigger) > armDeadband) {
        bottomArm.set(a_rightTrigger-a_leftTrigger);
      } else {
        bottomArm.set(0);
      }
      if (Math.abs(a_rightStickY) > armDeadband) {
        topArmSetpoint = topArmSetpoint - a_rightStickY*armCoarseAdjustRate;
      } 
      topArm.set(ControlMode.MotionMagic, encoderTicksPerRev*(topArmSetpoint - a_leftStickY*armFineAdjustRange));
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
    if (!coast && time > 2) { // Sets the motors to coast after 2 seconds
      coastMotors();
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

  public void coastMotors() {
    topArm.setNeutralMode(NeutralMode.Coast);
    leftBack.setNeutralMode(NeutralMode.Coast);
    leftFront.setNeutralMode(NeutralMode.Coast);
    rightBack.setNeutralMode(NeutralMode.Coast);
    rightFront.setNeutralMode(NeutralMode.Coast);
    bottomArm.setIdleMode(IdleMode.kBrake);
    coast = true;
  }

  public void brakeMotors() {
    topArm.setNeutralMode(NeutralMode.Brake);
    leftBack.setNeutralMode(NeutralMode.Brake);
    leftFront.setNeutralMode(NeutralMode.Brake);
    rightBack.setNeutralMode(NeutralMode.Brake);
    rightFront.setNeutralMode(NeutralMode.Brake);
    bottomArm.setIdleMode(IdleMode.kBrake);
    coast = false;
  }

  public void initializeCTREMotor(WPI_TalonFX motor) {
    motor.configFactoryDefault();
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.supplyCurrLimit.enable = true;
    config.supplyCurrLimit.triggerThresholdCurrent = 40; // max current in amps
    config.supplyCurrLimit.triggerThresholdTime = 0; // max time exceeding max current in seconds
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

    topArm.configPeakOutputForward(0.25); // limits topArm to 25% power in forwards direction
    topArm.configPeakOutputReverse(-0.25); // limits topArm to 25% power in backwards direction
    topArm.configClosedLoopPeakOutput(0, 0.25); // limits topArm to 25% power during Motion Magic and other closed loop control
    
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
    bottomArm.setSmartCurrentLimit(20); // sets current limit for bottomArm in amps
    coastMotors(); // sets all motors to coast
    bottomArm.setInverted(true);
  }
  
  // updates all program variables. should be called at the begining of every loop.
  public void updateVariables() {
    positionBottomArm = bottomArmEncoder.getPosition()/2; // 0-1 represents 1 full revolution of the bottom arm. Starts at 0.
    if (positionBottomArm > 140) { // Resolves encoder negative overrun issue
      positionBottomArm = positionBottomArm - 274.6583251953125;
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
    SmartDashboard.putBoolean("Compressor", compressorToggle);
    SmartDashboard.putBoolean("Solenoid", solenoidToggle);
    SmartDashboard.putBoolean("Coast", coast);
    SmartDashboard.putBoolean("At Setpoint", armAtSetpoint);
    SmartDashboard.putBoolean("Top At Setpoint", topArmAtSetpoint);
    SmartDashboard.putBoolean("Bottom At Setpoint", bottomArmAtSetpoint);
    SmartDashboard.putNumber("Angle", angle);
    SmartDashboard.putNumber("Stage", autoStage);
    SmartDashboard.putNumber("Top Arm Setpoint", topArmSetpoint);
  }
}