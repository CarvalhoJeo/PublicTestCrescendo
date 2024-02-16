package frc.robot.Joysticks;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorController implements IOperatorController {
  private static OperatorController mInstance = null;

  public static OperatorController getInstance() {
    if (mInstance == null) {
      mInstance = new OperatorController();
    }

    return mInstance;
  }

  CommandXboxController controller;

  private OperatorController() {
    controller = new CommandXboxController(1);
  }

  @Override
  public Trigger getIntake() {

    return controller.a();
  }

  @Override
  public Trigger getSecured() {
    return controller.y();
  }

  @Override
  public Trigger getPrepareAmp() {
    return controller.leftBumper();
  }

  @Override
  public Trigger getPrepareSpeaker() {
    return controller.rightBumper();
  }

  @Override
  public Trigger getShoot() {
    return controller.leftTrigger(0.2);
  }
}
