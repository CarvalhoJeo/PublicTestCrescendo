package frc.robot.Joysticks;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IOperatorController {
  Trigger getIntake();

  Trigger getSecured();

  Trigger getPrepareAmp();

  Trigger getPrepareSpeaker();

  Trigger getShoot();
}
