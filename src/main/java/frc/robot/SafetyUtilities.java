
package frc.robot;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.elevator2;
import frc.robot.subsystems.armlift;
import frc.robot.Constants.Safety;

public final class SafetyUtilities{

public static boolean isInterfereIntakeUp (double x, double y) {
    // x = intake angle, deg
    // y = lift position, in
    // true if arm and elevator positions are safe if intake is up
    return y > Safety.m1 * x + Safety.b1; 
}

public static boolean isInterfereIntakeDown (double x, double y) {
    // x = intake angle, deg
    // y = lift position, in
    // true if arm and elevator positions are safe while intake is down
    return (y > Safety.m2 * x + Safety.b2) || (y > Safety.m3 * x + Safety.b3);
}

}