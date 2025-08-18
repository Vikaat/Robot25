
package Team4450.Robot25;

import static Team4450.Robot25.Constants.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import Team4450.Robot25.commands.DriveCommand;
import Team4450.Robot25.commands.ExtendClimber;
import Team4450.Robot25.commands.IntakeCoral;
import Team4450.Robot25.commands.OuttakeCoral;
import Team4450.Robot25.commands.OuttakeProcessor;
import Team4450.Robot25.commands.IntakeAlgaeGround;
import Team4450.Robot25.commands.Preset;
import Team4450.Robot25.commands.RemoveAlgae;
import Team4450.Robot25.commands.RetractClimber;
import Team4450.Robot25.commands.OuttakeAlgae;


import Team4450.Robot25.subsystems.AlgaeManipulator;
import Team4450.Robot25.subsystems.AlgaeGroundIntake;
import Team4450.Robot25.subsystems.Candle;
import Team4450.Robot25.subsystems.CoralManipulator;
import Team4450.Robot25.subsystems.DriveBase;
import Team4450.Robot25.subsystems.ShuffleBoard;
import Team4450.Robot25.subsystems.ElevatedManipulator.PresetPosition;
import Team4450.Robot25.subsystems.ElevatedManipulator;
import Team4450.Robot25.subsystems.Elevator;
import Team4450.Robot25.subsystems.Climber;

import Team4450.Lib.MonitorPDP;
import Team4450.Lib.NavX;
import Team4450.Lib.Util;
import Team4450.Lib.CameraFeed;
import Team4450.Lib.XboxController;
import Team4450.Lib.MonitorCompressorPH;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer 
{
	// Subsystems.

	public static ShuffleBoard			shuffleBoard;
	public static DriveBase 			  driveBase;
	private Candle        				candle = null;

	// Subsystem Default Commands.

    // Persistent Commands.

	// Some notes about Commands.
	// When a Command is created with the New operator, its constructor is called. When the
	// command is added to the Scheduler to be run, its initialize method is called. Then on
	// each scheduler run, as long as the command is still scheduled, its execute method is
	// called followed by isFinished. If isFinished it false, the command remains in the
	// scheduler list and on next run, execute is called followed by isFinished. If isFinished
	// returns true, the end method is called and the command is removed from the scheduler list.
	// Now if you create another instance with new, you get the constructor again. But if you 
	// are re-scheduling an existing command instance (like the ones above), you do not get the
	// constructor called, but you do get initialize called again and then on to execute & etc.
	// So this means you have to be careful about command initialization activities as a persistent
	// command in effect has two lifetimes (or scopes): Class global and each new time the command
	// is scheduled. Note the FIRST doc on the scheduler process is not accurate as of 2020.
	
	// GamePads. 2 Game Pads use RobotLib XboxController wrapper class for some extra features.
	// Note that button responsiveness may be slowed as the schedulers command list gets longer 
	// or commands get longer as buttons are processed once per scheduler run.
	
	private XboxController			driverController =  new XboxController(DRIVER_PAD);
	public static XboxController	utilityController = new XboxController(UTILITY_PAD);

	private PowerDistribution		pdp = new PowerDistribution(REV_PDB, PowerDistribution.ModuleType.kRev);

	// Navigation board.
	public static NavX			navx;

	private static SendableChooser<Command>	autoChooser;
	
	private static String 					autonomousCommandName = "none";

	/**
	 * The container for the robot. Contains subsystems, Opertor Interface devices, and commands.
	 */
	public RobotContainer() throws Exception
	{
		Util.consoleLog();

		// Get information about the match environment from the Field Control System.
      
		getMatchInformation();

		// Read properties file from RoboRio "disk". If we fail to open the file,
		// log the exception but continue and default to competition robot.
      
		try {
			robotProperties = Util.readProperties();
		} catch (Exception e) { Util.logException(e);}

        // ? Why does it matter
		// Is this the competition or clone robot?
   		
		if (robotProperties == null || robotProperties.getProperty("RobotId").equals("comp"))
			isComp = true;
		else
			isClone = true;
 		
		// Create NavX object here since must done before CameraFeed is created (don't remember why).
        // Navx calibrates at power on and must complete before robot moves. Takes ~1 second for 2nd
        // generation Navx ~15 seconds for classic Navx. 
        //
        // !!!!! THIS IS A TERRIBLE ASSUMPTION WHY NOT JUST FORCE A WAIT !!!!!!
        //
        // We assume there will be enough time between
        // power on and our first movement because normally things don't happen that fast

		// Warning: The navx instance is shared with the swerve drive code. Resetting or otherwise
		// manipulating the navx (as opposed to just reading data) may crash the swerve drive code.

		navx = NavX.getInstance();

		// Add navx as a Sendable. Updates the dashboard heading indicator automatically.
 		
		SmartDashboard.putData("Gyro2", navx);

		// Invert driving joy sticks Y axis so + values mean forward.
		// Invert driving joy sticks X axis so + values mean right.
	  
		driverController.invertY(true);
		driverController.invertX(true);		

		// Create subsystems prior to button mapping.

		shuffleBoard = new ShuffleBoard();
		driveBase = new DriveBase();

		// Create any persistent commands.

		// Set any subsystem Default commands.

		// This sets up the photonVision subsystem to constantly update the robotDrive odometry
	    // with AprilTags (if it sees them). (As well as vision simulator)


		// Set the default drive command. This command will be scheduled automatically to run
		// every teleop period and so use the gamepad joy sticks to drive the robot. 

		// We pass the GetY() functions on the Joysticks as a DoubleSuppier. The point of this 
		// is removing the direct connection between the Drive and XboxController classes. We
		// are in effect passing functions into the Drive command so it can read the values
		// later when the Drive command is executing under the Scheduler. Drive command code does
		// not have to know anything about the JoySticks (or any other source) but can still read
		// them. We can pass the DoubleSupplier two ways. First is with () -> lambda expression
		// which wraps the getLeftY() function in a DoubleSupplier instance. Second is using the
		// controller class convenience method getRightYDS() which returns getRightY() as a 
		// DoubleSupplier. We show both ways here as an example.

		// The joystick controls for driving:
		// Left stick Y axis -> forward and backwards movement (throttle)
		// Left stick X axis -> left and right movement (strafe)
		// Right stick X axis -> rotation
		// Note: X and Y axis on stick is opposite X and Y axis on the WheelSpeeds object
		// and the odometry pose2d classes.
		// Wheelspeeds +X axis is down the field away from alliance wall. +Y axis is left
		// when standing at alliance wall looking down the field.
		// This is handled here by swapping the inputs. Note that first axis parameter below
		// is the X wheelspeeds input and the second is Y wheelspeeds input.

		// Note that field oriented driving does the movements in relation to the field. So
		// throttle is always down the field and back and strafe is always left right from
		// the down the field axis, no matter which way the robot is pointing. Robot oriented
		// driving movemments are in relation to the direction the robot is currently pointing.

		// Note that the controller instance is passed to the drive command for use in displaying
		// debugging information on Shuffleboard. It is not required for the driving function.

		driveBase.setDefaultCommand(new DriveCommand(driveBase,
		 							() -> driverController.getLeftY(),
									driverController.getLeftXDS(), 
									driverController.getRightXDS(),
									driverController));
		
		// Start a thread that will wait 30 seconds then disable the missing
		// joystick warning. This is long enough for when the warning is valid
		// but will stop flooding the console log when we are legitimately
		// running without both joysticks plugged in.

		new Thread(() -> {
			try {
				Timer.delay(30);    
	  
				DriverStation.silenceJoystickConnectionWarning(true);
			} catch (Exception e) { }
		  }).start();

		// Log info about NavX.
	  
		navx.dumpValuesToNetworkTables();
 		
		if (navx.isConnected())
			Util.consoleLog("NavX connected version=%s", navx.getAHRS().getFirmwareVersion());
		else
		{
			Exception e = new Exception("NavX is NOT connected!");
			Util.logException(e);
		}
        
        // Configure autonomous routines and send to dashboard.
		
		setAutoChoices();

		// Configure the button bindings.
		
        configureButtonBindings();
        
        // Load any trajectory files in a separate thread on first scheduler run.
        // We do this because trajectory loads can take up to 10 seconds to load so we want this
        // being done while we are getting started up. Hopefully will complete before we are ready to
        // use the trajectory.
		
		// NotifierCommand loadTrajectory = new NotifierCommand(this::loadTestTrajectory, 0);
        // loadTrajectory.setRunWhenDisabled(true);
        // CommandScheduler.getInstance().schedule(loadTrajectory);
		
		// //testTrajectory = loadTrajectoryFile("Slalom-1.wpilib.json");
		
		// loadTrajectory = new NotifierCommand(this::loadPPTestTrajectory, 0);
        // loadTrajectory.setRunWhenDisabled(true);
        // CommandScheduler.getInstance().schedule(loadTrajectory);

		//PathPlannerTrajectory ppTestTrajectory = loadPPTrajectoryFile("richard");

		Util.consoleLog(functionMarker);
	}

	/**
	 * Use this method to define your button->command mappings.
     * 
     * These buttons are for real robot driver station with 3 sticks and launchpad.
	 * The launchpad makes the colored buttons look like a joystick.
	 */
	private void configureButtonBindings() 
	{
		Util.consoleLog();
	  
		// ------- Driver pad buttons -------------
		
		// For simple functions, instead of creating commands, we can call convenience functions on
		// the target subsystem from an InstantCommand. It can be tricky deciding what functions
		// should be an aspect of the subsystem and what functions should be in Commands...

		// POV buttons do same as alternate driving mode but without any lateral
		// movement and increments of 45deg.
		// new Trigger(()-> driverController.getPOV() != -1)
		// 	.onTrue(new PointToYaw(()->PointToYaw.yawFromPOV(driverController.getPOV()), driveBase, false))

		// vibrate between 30 and 25 sec left in match.
		new Trigger(() -> Timer.getMatchTime() < 30 && Timer.getMatchTime() > 25).whileTrue(new StartEndCommand(
			() -> {
				driverController.setRumble(RumbleType.kBothRumble, 0.5);
				utilityController.setRumble(RumbleType.kBothRumble, 0.5);},
			() -> {
				driverController.setRumble(RumbleType.kBothRumble, 0);
				utilityController.setRumble(RumbleType.kBothRumble, 0);
		}));

		// toggle slow-mode
		new Trigger(() -> driverController.getLeftBumperButton())
			.onTrue(new InstantCommand(driveBase::enableSlowMode))
			.onFalse(new InstantCommand(driveBase::disableSlowMode));

		// reset field orientation (direction).
		new Trigger(() -> driverController.getStartButton())
			.onTrue(new InstantCommand(driveBase::zeroGyro));

		// toggle field-oriented driving mode.
		new Trigger(() -> driverController.getAButton())
			.onTrue(new InstantCommand(driveBase::toggleFieldRelative));

		//Holding Right D-Pad button sets X pattern to stop movement.
		new Trigger(() -> driverController.getPOV() == 90)
				.onTrue(new RunCommand(() -> driveBase.setX(), driveBase));

		// -------- Utility pad buttons ----------

		//Use Preset Command for the following:

        // Example
		// // Moves the coral manipulator/elevator to the L1 Branch scoring position
		// new Trigger(() -> utilityController.getXButton())
		// .onTrue(new Preset(elevatedManipulator, PresetPosition.CORAL_SCORING_L1_NEW));

	}
	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 * Determines which auto command from the selection made by the operator on the
	 * DS drop down list of commands.
	 * @return The Command to run in autonomous.
	 */
	public Command getAutonomousCommand() {
		Command				autoCommand;

		autoCommand = autoChooser.getSelected();

		if (autoCommand == null) 
		{
			autonomousCommandName = "none";

			return autoCommand;
		}

		autonomousCommandName = autoCommand.getName();

		Util.consoleLog("auto name=%s", autonomousCommandName);

		return autoCommand;
  	}

	public static String getAutonomousCommandName()
	{
		return autonomousCommandName;
	}
  
    // Configure SendableChooser (drop down list on dashboard) with auto program choices and
	// send them to SmartDashboard/ShuffleBoard.
	
	private void setAutoChoices()
	{
	 	Util.consoleLog();
		
		// Register commands called from PathPlanner Autos.

        // Example
		// NamedCommands.registerCommand("Intake Coral", new IntakeCoral(elevatedManipulator));

		// Create a chooser with the PathPlanner Autos located in the PP
		// folders.

	    autoChooser = AutoBuilder.buildAutoChooser();
		
    	SmartDashboard.putData("Auto Program", autoChooser);
	}

	/**
	 *  Get and log information about the current match from the FMS or DS.
	 */
	public void getMatchInformation()
	{
		alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
  	  	location = DriverStation.getLocation().orElse(0);
  	  	eventName = DriverStation.getEventName();
	  	matchNumber = DriverStation.getMatchNumber();
	  	gameMessage = DriverStation.getGameSpecificMessage();
    
	  	Util.consoleLog("Alliance=%s, Location=%d, FMS=%b event=%s match=%d msg=%s", 
    		  		   alliance.name(), location, DriverStation.isFMSAttached(), eventName, matchNumber, 
    		  		   gameMessage);
	}
		
	public void fixPathPlannerGyro() {
		driveBase.fixPathPlannerGyro();
	}
}
