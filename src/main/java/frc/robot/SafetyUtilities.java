
package frc.robot;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.elevator2;
import frc.robot.subsystems.armlift;
import frc.robot.Constants.Safety;
import frc.robot.Constants.ElevatorConstants;

public final class SafetyUtilities{

public static boolean isL1 (double x, double y){
    // x = intake angle, deg
    // y = lift position, in
    // true if arm and elevator positions are above line 1
    // line 1 is the interference boundary with intake up
    // line 1 is y = m1*x + b1
    return y > Safety.m1 * x + Safety.b1; 
}

public static boolean isL2 (double x, double y){
    // x = intake angle, deg
    // y = lift position, in
    // true if arm and elevator positions are above line 2
    // line 2 is the interference boundary with intake down, left boundary
    // line 2 is y = m2*x + b2
    return y > Safety.m2 * x + Safety.b2; 
}

public static boolean isL3 (double x, double y){
    // x = intake angle, deg
    // y = lift position, in
    // true if arm and elevator positions are above line 3
    // line 3 is the interference boundary with intake down, right boundary
    // line 3 is y = m3*x + b3
    return y > Safety.m3 * x + Safety.b3; 
}

public static boolean isL4 (double y){
    // y = lift position, in
    // true if elevator position is above kLSU, the lowest safe arm movement position with intake down
    return y > ElevatorConstants.kLSD;
}

public static boolean isInterfereIntakeUp (double x, double y) {
    // x = intake angle, deg
    // y = lift position, in
    // true if arm and elevator positions are interfering if intake is up
    return !isL1(x,y);
}

public static boolean isInterfereIntakeDown (double x, double y) {
    // x = intake angle, deg
    // y = lift position, in
    // true if arm and elevator positions are interfering while intake is down
    return !isL2(x,y) & !isL3(x,y);
}

}