package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
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
  public static final double trackWidth = 0.55;
  private static final double falconTicksPerRev = 2048;
  private static final double wheelCirc = 6*0.0254*Math.PI;
  private static final double gearRatio = 10.71;
  private static final double ticksPerMeter = falconTicksPerRev*gearRatio/wheelCirc;
  private static final double maxAngVelPower = 0.55;
  private static final double xAccLimit = 3;
  private static final double AngAccLimit = 3/maxAngVelPower;
  private static final double driveControllerDeadband = 0.05;
  private SlewRateLimiter xAccLimiter = new SlewRateLimiter(xAccLimit);
  private SlewRateLimiter angAccLimiter = new SlewRateLimiter(AngAccLimit);
  private PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);
  
  public Drivetrain() {
    initializeDriveMotor(rightFront);
    initializeDriveMotor(rightBack);
    initializeDriveMotor(leftFront);
    initializeDriveMotor(leftBack);
    rightBack.setInverted(true);
    rightFront.setInverted(true);
    gyro.calibrate();
    Timer.delay(2); // calibration delay for gyro
    gyro.zeroYaw();
  }

  // Manual control of the drivetrain based on raw controller inputs. One of the driving methods should be called each period.
  public void driveManual(double xPower, double angPower) {
    double xVel = xAccLimiter.calculate(MathUtil.applyDeadband(xPower, driveControllerDeadband));
    double angVel = angAccLimiter.calculate(MathUtil.applyDeadband(angPower, driveControllerDeadband)*maxAngVelPower);
    tank.arcadeDrive(xVel, angVel);
  }
  
  // Autonomous control of the drivetrain based on calculated wheel speeds. One of the driving methods should be called each period.
  public void setWheelSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds) {
    double rightVel = wheelSpeeds.rightMetersPerSecond;
    double leftVel = wheelSpeeds.leftMetersPerSecond;
    double rightVelCTREUnits = rightVel/10*ticksPerMeter;
    double leftVelCTREUnits = leftVel/10*ticksPerMeter;
    rightFront.set(ControlMode.Velocity, rightVelCTREUnits);
    leftFront.set(ControlMode.Velocity, leftVelCTREUnits);
    rightBack.set(ControlMode.Follower, 4);
    leftBack.set(ControlMode.Follower, 2);
    tank.feed();
  }

  public double getLeftPos() {
    return (leftFront.getSelectedSensorPosition() + leftBack.getSelectedSensorPosition())/2;
  }

  public double getRightPos() {
    return (rightFront.getSelectedSensorPosition() + rightBack.getSelectedSensorPosition())/2;
  }

  public double getYaw() {
    return -gyro.getYaw();
  }

  public double getPitch() {
    return gyro.getPitch();
  }

  public double getTotalCurrent() {
    return pdp.getTotalCurrent();
  }

  private void initializeDriveMotor(WPI_TalonFX motor) {
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
}