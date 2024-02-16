package frc.robot.subsystems.Motors;

public interface Motor {
  final int maximumRetries = 5;

  void factoryDefault();

  void clearStickyFaults();

  void configurePIDF(double P, double I, double D, double F, double Izone);

  void configurePIDF(double P, double I, double D, double F);

  void configurePIDWrapping(double minInput, double maxInput);

  void setMotorBrake(boolean isBrakeMode);

  void setInverted(boolean inverted);

  void setInvertedEncoder(boolean inverted);

  void burnFlash();

  void set(double percentOutput);

  void setPositionReference(double position);

  void setPositionReferenceArbFF(double position, double feedforward);

  void setVelocityReference(double velocity, double feedforward);

  double getVoltage();

  void setVoltage(double voltage);

  double getAppliedOutput();

  double getVelocity();

  double getPosition();

  void setPositionFactor(double factor);

  void setVelocityFactor(double factor);

  void setPosition(double position);

  void setVoltageCompensation(double nominalVoltage);

  void setCurrentLimit(int currentLimit);

  void setLoopRampRate(double rampRate);

  Object getMotor();

}
