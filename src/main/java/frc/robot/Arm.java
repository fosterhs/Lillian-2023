package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;

public class Arm {
  private WPI_TalonFX topArm = new WPI_TalonFX(0); // top arm motor
  private CANSparkMax bottomArm = new CANSparkMax(1, MotorType.kBrushed); // bottom arm motor
  private SparkMaxAbsoluteEncoder bottomArmEncoder = bottomArm.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
  private static final double planetaryRatio = 12.0;
  private static final double chainRatio = 64.0/16;
  private static final double falconTicksPerRev = 2048.0;
  private static final double ticksPerRev = falconTicksPerRev*planetaryRatio*chainRatio;
  private static final double armControllerDeadband = 0.05;
  private static final double bottomLowLimit = 10.0/360; 
  private static final double topManualAdjustRate = 144.0/(360*50); // maximum 144 degrees per second
  private static final double minTopArmResponse = 0.1;
  private static final double topLowLimit = -0.3;
  private static final double topHighLimit = 0.6;
  private static final double bottomTol = 1.8/360;
  private static final double topTol = 1.8/360;
  private boolean bottomAtSetpoint = true;
  private boolean topAtSetpoint = true;
  private boolean atSetpoint = true;
  private double bottomSetpoint = 0.0;
  private double topSetpoint = 0.0;

  public Arm() {
    initializeArmMotors();
    bottomArmEncoder.setZeroOffset(0.224);
  }

  // Should be called prior to calling moveToSetpoint() to define the setpoint.
  public void setSetpoint(double desiredBottomSetpoint, double desiredTopSetpoint) {
    bottomSetpoint = desiredBottomSetpoint;
    topSetpoint = desiredTopSetpoint;
    bottomAtSetpoint = false;
    topAtSetpoint = false;
    atSetpoint = false;
  }
  
  // Moves the arm to the setpoint autonomously.
  public void moveToSetpoint() {
    if (Math.abs(getPosBottom() - bottomSetpoint) < bottomTol) {
      bottomAtSetpoint = true;
      bottomArm.set(0);
    } else if (getPosBottom() < bottomSetpoint) {
      bottomArm.set(1);
    } else if (getPosBottom() > bottomSetpoint) {
        bottomArm.set(-1);
    }
    
    if (Math.abs(getPosTop() - topSetpoint) < topTol) {
      topAtSetpoint = true;
    }
    topArm.set(ControlMode.MotionMagic, topSetpoint*ticksPerRev);
  
    atSetpoint = bottomAtSetpoint && topAtSetpoint;
  }
  
  // Manual control of the arm based on raw controller inputs.
  public void moveManual(double powerBottom, double powerTop) {
    if (getPosBottom() > bottomLowLimit || powerBottom > 0) {
      bottomArm.set(MathUtil.applyDeadband(powerBottom, armControllerDeadband));
    } else {
      bottomArm.set(0);
    }
    
    if (Math.abs(powerTop) > armControllerDeadband) {
      if (powerTop > 0) {
        topSetpoint = topSetpoint + (minTopArmResponse + (1-minTopArmResponse)*Math.pow(powerTop, 2))*topManualAdjustRate;
      } else {
        topSetpoint = topSetpoint - (minTopArmResponse + (1-minTopArmResponse)*Math.pow(powerTop, 2))*topManualAdjustRate;
      }
    }
    if (topSetpoint > topHighLimit) {
      topSetpoint = topHighLimit;
    }
    if (topSetpoint < topLowLimit) {
      topSetpoint = topLowLimit;
    }
    topArm.set(ControlMode.MotionMagic, topSetpoint*ticksPerRev);
  }

  // Hands back manual control in the case of an arm collision where the setpoint cannot be achieved.
  public void reset() {
    bottomAtSetpoint = true;
    topAtSetpoint = true;
    atSetpoint = true;
    bottomArm.set(0);
    topArm.set(ControlMode.MotionMagic, getPosTop());
  }

  public double getPosBottom() {
    double posBottom = bottomArmEncoder.getPosition();
    if (posBottom > 0.5) {
        posBottom = posBottom - 1;
    }
    return posBottom;
  }

  public double getPosTop() {
    return topArm.getSelectedSensorPosition()/ticksPerRev;
  }

  public boolean atSetpoint() {
    return atSetpoint;
  }

  public double getClawX() {
    double bottomAng = getPosBottom()*360+10;
    double topAng = 90-bottomAng+10+getPosTop()*360;
    return -0.72*Math.cos(bottomAng*Math.PI/180) + 0.92*Math.cos(topAng*Math.PI/180);
  }

  public double getClawZ() {
    double bottomAng = getPosBottom()*360+10;
    double topAng = 90-bottomAng+10+getPosTop()*360;
    return 0.72*Math.sin(bottomAng*Math.PI/180) + 0.92*Math.sin(topAng*Math.PI/180);
  }

  private void initializeArmMotors() {
    topArm.configFactoryDefault();

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.supplyCurrLimit.enable = true;
    config.supplyCurrLimit.triggerThresholdCurrent = 10; // max current in amps (40 was original)
    config.supplyCurrLimit.triggerThresholdTime = 0.1; // max time exceeding max current in seconds
    config.supplyCurrLimit.currentLimit = 10; // max current after exceeding threshold 
    topArm.configAllSettings(config);

    topArm.setSelectedSensorPosition(0); // sets encoder to 0
    topArm.setNeutralMode(NeutralMode.Brake);

    double kI_topArm = 0.005;
    topArm.config_kP(0, 1);
    topArm.config_kI(0, kI_topArm);
    topArm.config_kD(0, 10);
    topArm.configMotionAcceleration(24000);
    topArm.configMotionCruiseVelocity(70000);
    topArm.configMaxIntegralAccumulator(0, 0.8*1023/kI_topArm);
    topArm.config_IntegralZone(0, 30000);
    topArm.configAllowableClosedloopError(0, 100);

    bottomArm.restoreFactoryDefaults(); // resets bottomArm to default
    bottomArm.setSmartCurrentLimit(8); // sets current limit for bottomArm in amps
    bottomArm.setInverted(true);
    bottomArm.setIdleMode(IdleMode.kBrake);
  }
}