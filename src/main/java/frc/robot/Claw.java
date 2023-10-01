package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Claw {
  private Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  private DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  private DigitalInput clawInput0 = new DigitalInput(0);
  private DigitalInput clawInput1 = new DigitalInput(1);
  private boolean clawClosed = true;

  public Claw() {}

  public void enableCompressor() {
    compressor.enableDigital();
  }

  public boolean getSensor() {
    boolean clawSensor0 = !clawInput0.get();
    boolean clawSensor1 = !clawInput1.get();
    return clawSensor0 || clawSensor1;
  }

  public void open() {
    solenoid.set(DoubleSolenoid.Value.kForward);
    clawClosed = false;
  }
  
  public void close() {
    solenoid.set(DoubleSolenoid.Value.kReverse);
    clawClosed = true;
  }
  
  public boolean getClosed() {
    return clawClosed;
  }
}