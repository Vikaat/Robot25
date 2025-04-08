package Team4450.Robot25.subsystems.coralManipulator;

public class CoralManipulatorConstants {
    public static final int CORAL_MANIPULATOR = 9; // CAN ID for the Coral Manipulator motor
    public static final int CORAL_PIVOT = 0; // PORT Assingment for the Coral Pivot solenoid

    public static enum CoralManipulatorState {
        IDLE,
        INTAKE,
        OUTTAKE,
        PIVOT_UP,
        PIVOT_DOWN
    }
}
