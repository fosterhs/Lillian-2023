package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final WPI_VictorSPX m_rightMotor = new WPI_VictorSPX(0); // Note - Creates a new motor, because the motor exists physically not digitally yet.
  private final WPI_VictorSPX m_leftMotor = new WPI_VictorSPX(1); // Note - Creates another new motor
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor); // Note - Creates an instance of differential drive that takes in inputs of the left and right motors. */
  private final XboxController m_driverController = new XboxController(0); // Note - Creates an Xbox Controller object/instance
  ShuffleboardTab joystickValues = Shuffleboard.getTab("Joystick Values");

  @Override
  public void robotInit() {
    m_rightMotor.setInverted(true); // Note - Inverts the motors on one side because of the orientation physically. If not inverted, then it would turn.
    CameraServer.startAutomaticCapture();        
  }

  @Override
  public void robotPeriodic() {
    /* Note - Arcade Drive function that takes in the controllers left Y axis values and right X values to move the robot. The controllers sticks will return a value between 1 
    and -1, which when inputted into the function, will give the motors speed/movement. Motor movement is simple. It's 0% power at default, meaning it has 0% speed, but if you
    give the motor 20% power, it has 20% speed. Similarlly if you take the 1 and -1 values the sticks give off and assign them to the motors, you have motor speed. Although 1 wouuld
    act as 100%. This function, which is built into the library, has shortened this down for us so we only have to call one function. Because we gave the variable m_robotDrive the motor inputs, we just call the function without having to reference the motors. */
    m_robotDrive.arcadeDrive(-m_driverController.getLeftY(), -m_driverController.getRightX()); // Note - Takes in the controller's left Y axis & right x axis values to move the robot. LeftY moves back & forth, RightX turns.
    SmartDashboard.putNumber("Joystick X value" , m_driverController.getLeftY());
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

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
