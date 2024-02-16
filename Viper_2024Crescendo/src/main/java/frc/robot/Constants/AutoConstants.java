// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import com.pathplanner.lib.util.PIDConstants;

public final class AutoConstants {
  public static final PIDConstants TranslationPID = new PIDConstants(6, 0, 0);
  public static final PIDConstants angleAutoPID = new PIDConstants(5.5, 0, 0);
  public static final double MAX_ACCELERATION_AUTO = 4;
}
