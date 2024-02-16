package frc.robot.subsystems.Sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class InfraRed implements frc.robot.subsystems.Sensors.DigitalInput {

  private DigitalInput IR;

  private boolean inverted;

  public InfraRed(int port, boolean invert) {
    IR = new DigitalInput(port);
    inverted = invert;
  }

  @Override
  public boolean getBoolean() {

    return inverted ? !IR.get() : IR.get();
  }

  @Override
  public int getBinary() {
    if (getBoolean()) {
      return 1;
    } else {
      return 0;
    }
  }
}
