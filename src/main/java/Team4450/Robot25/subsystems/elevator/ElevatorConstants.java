package Team4450.Robot25.subsystems.elevator;

public class ElevatorConstants {
    public static final int ELEVATOR_LEFT = 11; //CAN ID for the Elevator Left Motor
    public static final int ELEVATOR_RIGHT = 12; //CAN ID for the Elevator Right Motor

    // ELEVATOR_WINCH_FACTOR is a conversion factor from motor rotations to meters of height change.
    // It is multiplied by the native rotations of the motor shaft to get the height change in the MAXSpline shaft since startup or the last encoder reset.
    // MATH EXPLANATION (2025):
    // Gear Reduction of Gearbox: 38:9 (38 rotations of the motor shaft rotates the spool 9 times).
    // To solve for the winch factor, you need the ratio of winch rotations to motor rotations.
    // So, 38 motor rotations / 9 winch rotations, and you need to take the reciprocal to get the winch factor.
    // The ratio is (1.0 / (38.0 / 9.0)) spool rotations for every turn of the shaft.
    // Multiply by 2π for radians traveled/angular displacement and by the spool radius in meters to get linear displacement.
    // The spool radius is 0.875 inches, which is 0.022225 meters (source: looked it up).
    // The factor is negative, likely because the gears swap rotation direction, but this is not a significant issue.
    public static final double  ELEVATOR_WINCH_FACTOR = (-1.0 / (38.0 / 9.0)) * (2 * Math.PI) * 0.022225; //Changed to 2025 Value!

}
