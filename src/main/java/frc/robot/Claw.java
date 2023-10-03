package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Claw {
  private final Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  private final DigitalInput clawInput0 = new DigitalInput(0);
  private final DigitalInput clawInput1 = new DigitalInput(1);
  private boolean clawClosed = true;

  public Claw() {}

  public final void enableCompressor() {
    compressor.enableDigital();
  }

  public final boolean getSensor() {
    boolean clawSensor0 = !clawInput0.get();
    boolean clawSensor1 = !clawInput1.get();
    return clawSensor0 || clawSensor1;
  }

  public final void open() {
    solenoid.set(DoubleSolenoid.Value.kForward);
    clawClosed = false;
  }
  
  public final void close() {
    solenoid.set(DoubleSolenoid.Value.kReverse);
    clawClosed = true;
  }
  
  public final boolean getClosed() {
    return clawClosed;
  }
}