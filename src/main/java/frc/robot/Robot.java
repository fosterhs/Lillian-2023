package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends TimedRobot {
  private final WPI_TalonFX m_rightMotor = new WPI_TalonFX(1); // Note - Creates a new motor, because the motor exists physically not digitally yet.
  private final WPI_TalonFX m_leftMotor = new WPI_TalonFX(5); // Note - Creates another new motor
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor); // Note - Creates an instance of differential drive that takes in inputs of the left and right motors. */
  private final XboxController m_driverController = new XboxController(0); // Note - Creates an Xbox Controller object/instance
  // private ShuffleboardTab joystickValues = Shuffleboard.getTab("Joystick Values");
  /* private GenericEntry leftYValue = joystickValues.add("Left Y Value", 0)
  .getEntry(); Note - Creates an entry variable that both creates values and sends them, and receives their value updates. Use only if putNumber doesn't work.*/ 
  private final Timer elapsedTime = new Timer();

  @Override
  public void robotInit() {
    m_rightMotor.setInverted(true); // Note - Inverts the motors on one side because of the orientation physically. If not inverted, then it would turn.
    CameraServer.startAutomaticCapture();
    /* Shuffleboard.selectTab("Joystick Values"); 
    Shuffleboard.getTab("Joystick Values")
    .add("Joystick X Value", 0); */       
  }

  @Override
  public void robotPeriodic() {
    /* Note - Arcade Drive function that takes in the controllers left Y axis values and right X values to move the robot. The controllers sticks will return a value between 1 
    and -1, which when inputted into the function, will give the motors speed/movement. Motor movement is simple. It's 0% power at default, meaning it has 0% speed, but if you
    give the motor 20% power, it has 20% speed. Similarlly if you take the 1 and -1 values the sticks give off and assign them to the motors, you have motor speed. Although 1 wouuld
    act as 100%. This function, which is built into the library, has shortened this down for us so we only have to call one function. Because we gave the variable m_robotDrive the motor inputs, we just call the function without having to reference the motors. */
    m_robotDrive.arcadeDrive(-m_driverController.getLeftY(), -m_driverController.getRightX()); // Note - Takes in the controller's left Y axis & right x axis values to move the robot. LeftY moves back & forth, RightX turns.
    /* Shuffleboard.getTab("Joystick Values")
    .add("Joystick X Value", m_driverController.getLeftX()); */
    SmartDashboard.putNumber("Left Joystick's Y Axis Value:" , m_driverController.getLeftY());
    SmartDashboard.putNumber("Right Joystick's X Axis Value:", m_driverController.getRightX());
    /* if (m_driverController.getLeftY() > 0 && m_driverController.getLeftY() < 0) {
      Shuffleboard.getTab("Joystick Values")
      .add("Left Y Value", m_driverController.getLeftY());
    } Note - This line is meant to check if the y stick is above or below 0, or if it's moving up or down. If so, then it will access the Joystick Values tab and update the y position
    with the current y axis calue with the .getLeftY method. It's placed here in case the line above this one doesn't update. */
    // double yPosition = leftYValue.getDouble(1.0); Note - Playing around generic entries, use only if putNumber doesn't work as planned.
  }

  @Override
  public void autonomousInit() {
    m_robotDrive.arcadeDrive(0, 0);
    elapsedTime.start();
    elapsedTime.reset();
  }

  @Override
  public void autonomousPeriodic() {
    double matchTime = elapsedTime.get();
    if (matchTime < 4) {
      m_leftMotor.set(ControlMode.PercentOutput, 0.1);
      m_rightMotor.set(ControlMode.PercentOutput, -0.1);
    } else if (matchTime > 4) {
      m_leftMotor.set(ControlMode.PercentOutput, 0);
      m_rightMotor.set(ControlMode.PercentOutput, 0);
    }
  }

  @Override
  public void teleopInit() {
    m_robotDrive.arcadeDrive(0, 0);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

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
}
