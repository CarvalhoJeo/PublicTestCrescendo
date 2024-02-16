package frc.Java_Is_UnderControl.Util;

public class Util {

  private Util() {
  }

  public static Boolean inRange(double x, double minX, double maxX) {
    return x > minX && x < maxX;
  }
}
