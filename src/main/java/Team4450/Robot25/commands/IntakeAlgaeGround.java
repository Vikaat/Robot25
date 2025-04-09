package Team4450.Robot25.commands;

import Team4450.Lib.Util;
import Team4450.Robot25.subsystems.elevatedManipulator.ElevatedManipulator;
import Team4450.Robot25.subsystems.elevatedManipulator.ElevatedManipulator.PresetPosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeAlgaeGround extends Command {
    private final ElevatedManipulator elevatedManipulator;

    private static enum State{INTAKE_ALGAE, FEEDING, HOLDING, STOP};
    private State state = State.INTAKE_ALGAE;

    double startTime;
    public IntakeAlgaeGround(ElevatedManipulator elevatedManipulator){
        this.elevatedManipulator = elevatedManipulator;
        addRequirements(elevatedManipulator);
    }

    public void initialize(){
        state = State.INTAKE_ALGAE;
        elevatedManipulator.executeSetPosition(PresetPosition.ALGAE_GROUND_INTAKE); //Moves the algae manipulator and elevator to the intake position and extends the ground intake
        SmartDashboard.putString("Algae Ground Intake Status", state.name());
    }

    public void execute(){
        switch(state){
            case INTAKE_ALGAE:
                elevatedManipulator.algaeManipulator.startIntaking();
                elevatedManipulator.algaeGroundIntake.startRollers();
                if(elevatedManipulator.algaeManipulator.getCurrent() > 75.0){
                    state = State.FEEDING;
                }
                break;

            case FEEDING:
                elevatedManipulator.algaeGroundIntake.feedAlgae();
                elevatedManipulator.algaeManipulator.holdAlgae();
                elevatedManipulator.algaeManipulator.pivotUp();
                if(elevatedManipulator.algaeManipulator.hasAlgae() == true){
                    state = State.HOLDING;
                }
                break;
            case HOLDING:
                if(elevatedManipulator.elevator.getElevatorHeight() > 0.35){
                    elevatedManipulator.algaeManipulator.holdAlgae();
                    elevatedManipulator.algaeManipulator.pivotDown();
                    elevatedManipulator.algaeGroundIntake.stopRollers();
                    elevatedManipulator.algaeGroundIntake.retractIn();
                    state = State.STOP;
                }
                break;

            case STOP: 
                SmartDashboard.putString("status", "Algae Ground Intake Sequenced");               
                break;
        }
    }

    public boolean isFinished(){
        return state == State.STOP;
    }   

    public void end(boolean interrupted){
        Util.consoleLog("interrupted=%b", interrupted);
        elevatedManipulator.algaeManipulator.holdAlgae();    
        elevatedManipulator.algaeGroundIntake.stopRollers();
    }
}