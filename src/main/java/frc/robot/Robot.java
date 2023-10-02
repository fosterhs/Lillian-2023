package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  Drivetrain drivetrain = new Drivetrain();
  Arm arm = new Arm();
  Claw claw = new Claw();
  PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);
  Timer timer = new Timer();
  XboxController armCont = new XboxController(0);
  Joystick stick = new Joystick(1);
  int autoStage = 1;

  public void autonomousInit() {
    claw.enableCompressor();
    claw.close();
    arm.setSetpoint(0.245, 0.043); // Cube Score (Top)
  }

  public void autonomousPeriodic() {
    if (autoStage == 1) { // Moves arm to Cube Score (Top)
      drivetrain.driveManual(0, 0);
      arm.moveToSetpoint();
      if (arm.atSetpoint()) {
        autoStage++;
        timer.start();
      }
    }
    if (autoStage == 2) { // Opens claw after 0.7s. Delays another 0.3s before advancing to the next stage.
      drivetrain.driveManual(0, 0);
      if (timer.get() > 0.7) { // Delays 0.7s
        claw.open();
      }
      if (timer.get() > 1.0) { // Delays another 0.3s
        drivetrain.setPathReversal(true);
        drivetrain.loadPath("Auto Path");
        arm.setSetpoint(0.057, -0.255); // Front Floor Pickup
        autoStage++;
      }
    }
    if (autoStage == 3) { // Follows the PathPlanner trajectory to the game piece. Moves arm to Front Floor Pickup.
      arm.moveToSetpoint();
      drivetrain.followPath();
      if (arm.atSetpoint() && drivetrain.atEndpoint()) {
        autoStage++;
      }
    }
    if (autoStage == 4) { // Stops.
      drivetrain.driveManual(0, 0);
    }
  }

  public void teleopInit() {
    claw.enableCompressor();
  }

  public void teleopPeriodic() {
    drivetrain.driveManual(-stick.getY(), -stick.getZ());

    if (armCont.getLeftTriggerAxis() > 0.2 && claw.getSensor() && !claw.getClosed()) { // Claw Auto Close
      claw.close();
      timer.reset();
    }
    if (armCont.getLeftTriggerAxis() > 0.2 && !claw.getSensor() && timer.get() > 1.0) { // Claw Auto Open
      claw.open();
    }
    if (armCont.getLeftBumperPressed()) {
      claw.close();
    }
    if (armCont.getRightBumperPressed()) {
      claw.open();
    }

    if (armCont.getPOV() == 0) { // Hands over manual control in the case of an arm collision that prevents the arm from reaching the setpoint.
      arm.reset();
    }
    if (armCont.getAButtonPressed()) { // Drive/Carry
      arm.setSetpoint(0, -0.068);
    }
    if (armCont.getBButtonPressed()) { // Front Floor Pickup
      arm.setSetpoint(0.057, -0.255);
    }
    if (armCont.getXButtonPressed()) { // Substation Pickup
      arm.setSetpoint(0.095, -0.073);
    }
    if (armCont.getYButtonPressed()) { // Cube Score (Middle)
      arm.setSetpoint(0.136, -0.111);
    }
    if (armCont.getPOV() == 180) { // Cube Score (Top)
      arm.setSetpoint(0.245, 0.043);
    }
    if (armCont.getRightTriggerAxis() > 0.25) { // Cone Score (Top)
      arm.setSetpoint(0.256, 0.110);
    }
    if (armCont.getStartButtonPressed()) { // Starting Position
      arm.setSetpoint(0, 0);
    }
    if (armCont.getBackButtonPressed()) { // Cone Score (Middle)
      arm.setSetpoint(0.158, -0.040);
    }
    if (!arm.atSetpoint()) {
      arm.moveToSetpoint();
    } else {
      arm.moveManual(-armCont.getLeftY(), -armCont.getRightY());
    }
  }

  public void robotPeriodic() {
    updateDash();
  }

  public void updateDash() {
    SmartDashboard.putBoolean("Claw Sensor", claw.getSensor());
    SmartDashboard.putBoolean("Claw Closed", claw.getClosed());
    SmartDashboard.putBoolean("Arm at Setpoint", arm.atSetpoint());
    SmartDashboard.putNumber("posBottomArm", arm.getPosBottom());
    SmartDashboard.putNumber("posTopArm", arm.getPosTop());
    SmartDashboard.putNumber("ClawX", arm.getClawX());
    SmartDashboard.putNumber("ClawZ", arm.getClawZ());
    SmartDashboard.putNumber("Total Current", pdp.getTotalCurrent());
    SmartDashboard.putNumber("Drivetrain leftPos", drivetrain.getLeftPos());
    SmartDashboard.putNumber("Drivetrain rightPos", drivetrain.getRightPos());
    SmartDashboard.putNumber("Yaw", drivetrain.getYaw());
    SmartDashboard.putNumber("Pitch", drivetrain.getPitch()); 
    SmartDashboard.putNumber("Auto Stage", autoStage); 
  }
}