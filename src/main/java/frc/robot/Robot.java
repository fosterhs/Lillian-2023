package frc.robot;

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
  XboxController driveController = new XboxController(0);
  XboxController armController = new XboxController(1);
  int autoStage = 1;

  public void autonomousInit() {
    claw.enableCompressor();
    claw.close();
    arm.setSetpoint(0.245, 0.043); // Cube Score (Top)
  }

  public void autonomousPeriodic() {
    drivetrain.updateOdometry();
    if (autoStage == 1) { // Moves arm to Cube Score (Top)
      drivetrain.drive(0, 0);
      arm.moveToSetpoint();
      if (arm.atSetpoint()) {
        autoStage++;
        timer.start();
      }
    }
    if (autoStage == 2) { // Opens claw after 0.7s. Delays another 0.3s before advancing to the next stage.
      drivetrain.drive(0, 0);
      if (timer.get() > 0.7) { // Delays 0.7s
        claw.open();
      }
      if (timer.get() > 1.0) { // Delays another 0.3s
        drivetrain.resetPathController();
        drivetrain.resetOdometryToPathStart(0);
        arm.setSetpoint(0.057, -0.255); // Front Floor Pickup
        autoStage++;
      }
    }
    if (autoStage == 3) { // Follows the PathPlanner trajectory to the game piece. Moves arm to Front Floor Pickup.
      arm.moveToSetpoint();
      drivetrain.followPath(0);
      if (arm.atSetpoint() && drivetrain.atEndpoint(0, 0.03, 0.03, 1)) {
        autoStage++;
      }
    }
    if (autoStage == 4) { // Stops.
      drivetrain.drive(0, 0);
    }
  }

  public void teleopInit() {
    claw.enableCompressor();
  }

  public void teleopPeriodic() {
    drivetrain.updateOdometry();
    drivetrain.drive(-driveController.getLeftY(),-driveController.getRightX());

    if (armController.getLeftTriggerAxis() > 0.2 && claw.getSensor() && !claw.getClosed()) { // Claw Auto Close
      claw.close();
      timer.reset();
    }
    if (armController.getLeftTriggerAxis() > 0.2 && !claw.getSensor() && timer.get() > 1.0) { // Claw Auto Open
      claw.open();
    }
    if (armController.getLeftBumperPressed()) {
      claw.close();
    }
    if (armController.getRightBumperPressed()) {
      claw.open();
    }

    if (armController.getBackButtonPressed()) { // Hands over manual control in the case of an arm collision that prevents the arm from reaching the setpoint.
      arm.reset();
    }
    if (armController.getStartButtonPressed()) { // Starting Position
      arm.setSetpoint(0, 0);
    }
    if (armController.getAButtonPressed()) { // Drive/Carry
      arm.setSetpoint(0, -0.068);
    }
    if (armController.getBButtonPressed()) { // Front Floor Pickup
      arm.setSetpoint(0.057, -0.255);
    }
    if (armController.getXButtonPressed()) { // Substation Pickup
      arm.setSetpoint(0.095, -0.073);
    }
    if (armController.getPOV() == 180) { // Cube Score (Middle)
      arm.setSetpoint(0.136, -0.111);
    }
    if (armController.getPOV() == 90) { // Cube Score (Top)
      arm.setSetpoint(0.245, 0.043);
    }
    if (armController.getPOV() == 0) { // Cone Score (Top)
      arm.setSetpoint(0.256, 0.110);
    }
    if (armController.getPOV() == 270) { // Cone Score (Middle)
      arm.setSetpoint(0.158, -0.040);
    }
    if (!arm.atSetpoint()) {
      arm.moveToSetpoint();
    } else {
      arm.moveManual(-armController.getLeftY(), -armController.getRightY());
    }
  }

  public void robotInit() {
    drivetrain.loadPath("Auto Path", 2.0, 1.0, true);
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
    SmartDashboard.putNumber("RobotX", drivetrain.getRobotX());
    SmartDashboard.putNumber("RobotY", drivetrain.getRobotY());
    SmartDashboard.putNumber("Yaw", drivetrain.getYaw());
    SmartDashboard.putNumber("Pitch", drivetrain.getPitch()); 
    SmartDashboard.putNumber("Auto Stage", autoStage); 
  }
}