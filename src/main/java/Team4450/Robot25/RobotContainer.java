// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package Team4450.Robot25;

import Team4450.Lib.MonitorCompressorPH;
import Team4450.Lib.MonitorPDP;
import Team4450.Lib.Util;


import static Team4450.Robot25.Constants.*;

import Team4450.Robot25.commands.Preset;
import Team4450.Robot25.commands.DriveCommands;
import Team4450.Robot25.commands.IntakeCoral;
import Team4450.Robot25.commands.OuttakeCoral;
import Team4450.Robot25.commands.RemoveAlgae;
import Team4450.Robot25.commands.OuttakeAlgae;
import Team4450.Robot25.commands.OuttakeProcessor;
import Team4450.Robot25.commands.IntakeAlgaeGround;

import Team4450.Robot25.subsystems.algaeGroundIntake.AlgaeGroundIntake;
import Team4450.Robot25.subsystems.algaeManipulator.AlgaeManipulator;
import Team4450.Robot25.subsystems.coralManipulator.CoralManipulator;
import Team4450.Robot25.subsystems.drive.Drive;
import Team4450.Robot25.subsystems.elevator.Elevator;
import Team4450.Robot25.subsystems.drive.GyroIO;
import Team4450.Robot25.subsystems.drive.GyroIONavX;
import Team4450.Robot25.subsystems.drive.ModuleIO;
import Team4450.Robot25.subsystems.drive.ModuleIOSim;
import Team4450.Robot25.subsystems.drive.ModuleIOSpark;
import Team4450.Robot25.subsystems.elevatedManipulator.ElevatedManipulator;
import Team4450.Robot25.subsystems.elevatedManipulator.ElevatedManipulator.PresetPosition;
import Team4450.Robot25.subsystems.vision.Vision;
import Team4450.Robot25.subsystems.vision.VisionIO;
import Team4450.Robot25.subsystems.vision.VisionIOPhotonVision;
import Team4450.Robot25.subsystems.vision.VisionIOPhotonVisionSim;
import Team4450.Robot25.subsystems.vision.VisionConstants;
import Team4450.Robot25.subsystems.climber.Climber;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public static Drive drive;
  public static Vision vision;
  public static Climber climber;
  public static CoralManipulator coralManipulator;
  public static AlgaeManipulator algaeManipulator;
  public static AlgaeGroundIntake algaeGroundIntake;
  public static Elevator elevator;
  public static ElevatedManipulator elevatedManipulator;

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController utilityController = new CommandXboxController(1);

  // Dashboard inputs
  private LoggedDashboardChooser<Command> autoChooser;

  private PowerDistribution		pdp = new PowerDistribution(REV_PDB, PowerDistribution.ModuleType.kRev);
  private Compressor				pcm = new Compressor(PneumaticsModuleType.REVPH);
//   private final MonitorPDP     		monitorPDPThread;
  private MonitorCompressorPH	monitorCompressorThread;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() throws Exception {
    
   // Read properties file from RoboRio "disk". If we fail to open the file,
		// log the exception but continue and default to competition robot.
      
		try {
			robotProperties = Util.readProperties();
		} catch (Exception e) { Util.logException(e);}

		// Is this the competition or clone robot?
   		
		if (robotProperties == null || robotProperties.getProperty("RobotId").equals("comp"))
			isComp = true;
		else
			isClone = true;
 		
		// Set compressor enabled switch on dashboard from properties file.
		// Later code will read that setting from the dashboard and turn 
		// compressor on or off in response to dashboard setting.
 		
		boolean compressorEnabled = true;	// Default if no property.

		if (robotProperties != null) 
			compressorEnabled = Boolean.parseBoolean(robotProperties.getProperty("CompressorEnabledByDefault"));
		
		SmartDashboard.putBoolean("CompressorEnabled", compressorEnabled);

		// Reset PDB & PCM sticky faults.
    
		resetFaults();

        monitorCompressorThread = MonitorCompressorPH.getInstance(pcm);
   		monitorCompressorThread.setDelay(1.0);
   		monitorCompressorThread.SetLowPressureAlarm(50);
   		monitorCompressorThread.start();
   		
   		// monitorPDPThread = MonitorPDP.getInstance(pdp);
   		// monitorPDPThread.start();

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(VisionConstants.camera0Name, VisionConstants.robotToCamera0),
                new VisionIOPhotonVision(VisionConstants.camera1Name, VisionConstants.robotToCamera1));
        
        climber = 
            new Climber();

        coralManipulator = 
            new CoralManipulator();

        algaeManipulator =
            new AlgaeManipulator();

        algaeGroundIntake =
            new AlgaeGroundIntake();
        elevator = 
            new Elevator(drive);
        
        elevatedManipulator =
            new ElevatedManipulator(coralManipulator, algaeManipulator, algaeGroundIntake, elevator);
;
        
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose));

        climber =
            new Climber();

        coralManipulator = 
            new CoralManipulator();
        
        algaeManipulator =
            new AlgaeManipulator();

        algaeGroundIntake =
            new AlgaeGroundIntake();

        elevator =
            new Elevator(drive);
        
        elevatedManipulator =
            new ElevatedManipulator(coralManipulator, algaeManipulator, algaeGroundIntake, elevator);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        
        vision = 
            new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        climber =
            new Climber();

        coralManipulator = 
            new CoralManipulator();
        
        algaeManipulator =
            new AlgaeManipulator();
        
        algaeGroundIntake =
            new AlgaeGroundIntake();

        elevator =
            new Elevator(drive);
        
        elevatedManipulator =
            new ElevatedManipulator(coralManipulator, algaeManipulator, algaeGroundIntake, elevator);
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    
    new Thread(() -> {
			try {
				Timer.delay(30);    
	  
				DriverStation.silenceJoystickConnectionWarning(true);
			} catch (Exception e) { }
		  }).start();
    
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    elevator.setDefaultCommand(
        Commands.run(() -> elevator.move(-utilityController.getLeftY() * 0.5), elevator));

    setAutoChoices();

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    if (Timer.getMatchTime() < 30 && Timer.getMatchTime() > 25) {
        new StartEndCommand(
            () -> {
                driverController.setRumble(RumbleType.kBothRumble, 0.5);
                utilityController.setRumble(RumbleType.kBothRumble, 0.5);
            },
            () -> {
                driverController.setRumble(RumbleType.kBothRumble, 0);
                utilityController.setRumble(RumbleType.kBothRumble, 0);
            }).schedule();
    }

    // Lock to 0° when A button is held
    driverController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    driverController
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    
    driverController
        .leftBumper()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.enableSlowMode())
                .ignoringDisable(true))
        .onFalse(
            Commands.runOnce(
                    () ->
                        drive.disableSlowMode())
                .ignoringDisable(true));

    driverController
        .povDown()
        .onTrue(
            Commands.runOnce(
                    () ->
                        climber.extendPiston())
                .ignoringDisable(true));
    
    driverController
        .povUp()
        .onTrue(
            Commands.runOnce(
                    () ->
                        climber.retractPiston())
                .ignoringDisable(true));

    utilityController
        .x()
        .onTrue(
            new Preset(elevatedManipulator, PresetPosition.CORAL_SCORING_L1_NEW)
                .ignoringDisable(false));

    utilityController
        .a()
        .onTrue(
            new Preset(elevatedManipulator, PresetPosition.CORAL_SCORING_L2)
                .ignoringDisable(false));

    utilityController
        .b()
        .onTrue(
            new Preset(elevatedManipulator, PresetPosition.CORAL_SCORING_L3)
                .ignoringDisable(false));
        
    utilityController
        .y()
        .onTrue(
            new Preset(elevatedManipulator, PresetPosition.CORAL_SCORING_L4)
                .ignoringDisable(false));
    
    utilityController
        .back()
        .onTrue(
            new Preset(elevatedManipulator, PresetPosition.RESET)
                .ignoringDisable(false));
    
    utilityController
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        elevator.resetEncoders())
                .ignoringDisable(false));

    utilityController
        .leftTrigger()
        .whileTrue(
            new IntakeCoral(elevatedManipulator)
            .ignoringDisable(false))
        .onFalse(
            Commands.runOnce(
                    ()  ->
                        coralManipulator.stop())
            );

    utilityController
        .rightTrigger()
        .onTrue(
            new OuttakeCoral(elevatedManipulator)
            .ignoringDisable(false));
    
    utilityController
        .leftBumper()
        .onTrue(
            new RemoveAlgae(elevatedManipulator)
                .ignoringDisable(false));
    
    utilityController
        .rightBumper()
        .onTrue(
            new OuttakeAlgae(elevatedManipulator)
                .ignoringDisable(false)
        );
    
    utilityController
        .povUp()
        .onTrue(
            new Preset(elevatedManipulator, PresetPosition.ALGAE_REMOVE_L3)
                .ignoringDisable(false));
    
    utilityController
        .povDown()
        .onTrue(
            new Preset(elevatedManipulator, PresetPosition.ALGAE_REMOVE_L2)
                .ignoringDisable(false));
    
    utilityController
        .povLeft()
        .onTrue(
            new Preset(elevatedManipulator, PresetPosition.ALGAE_PROCESSOR_SCORING)
                .ignoringDisable(false));
    
    utilityController
        .povRight()
        .onTrue(
            new Preset(elevatedManipulator, PresetPosition.ALGAE_NET_SCORING)
                .ignoringDisable(false));
      }

    public void resetFaults()
	{
		// This code turns on/off the automatic compressor management if requested by DS. Putting this
		// here is a convenience since this function is called at each mode change.
		if (SmartDashboard.getBoolean("CompressorEnabled", true)) 
			pcm.enableDigital();
		else
			pcm.disable();
		
		pdp.clearStickyFaults();
		//pcm.clearAllStickyFaults(); // Add back if we use a CTRE pcm.
		
		// if (monitorPDPThread != null) monitorPDPThread.reset();
    }



	private void setAutoChoices()
	{
	 	Util.consoleLog();
		
		// Register commands called from PathPlanner Autos.

		NamedCommands.registerCommand("Intake Coral", new IntakeCoral(elevatedManipulator));
		NamedCommands.registerCommand("Outtake Coral", new OuttakeCoral(elevatedManipulator));
		NamedCommands.registerCommand("Remove Algae", new RemoveAlgae(elevatedManipulator));
		NamedCommands.registerCommand("Outtake Algae", new OuttakeAlgae(elevatedManipulator));
		NamedCommands.registerCommand("Outtake Processor", new OuttakeProcessor(elevatedManipulator));
		NamedCommands.registerCommand("Raise to L1", new Preset(elevatedManipulator, PresetPosition.CORAL_SCORING_L1_NEW));
		NamedCommands.registerCommand("Raise to L2", new Preset(elevatedManipulator, PresetPosition.CORAL_SCORING_L2));
		NamedCommands.registerCommand("Raise to L3", new Preset(elevatedManipulator, PresetPosition.CORAL_SCORING_L3));
		NamedCommands.registerCommand("Raise to L4", new Preset(elevatedManipulator, PresetPosition.CORAL_SCORING_L4));
		NamedCommands.registerCommand("Remove Algae L2", new Preset(elevatedManipulator, PresetPosition.ALGAE_REMOVE_L2));
		NamedCommands.registerCommand("Remove Algae L3", new Preset(elevatedManipulator, PresetPosition.ALGAE_REMOVE_L3)); 
		NamedCommands.registerCommand("Algae Net Scoring", new Preset(elevatedManipulator, PresetPosition.ALGAE_NET_SCORING)); 
		NamedCommands.registerCommand("Algae Processor Scoring", new Preset(elevatedManipulator, PresetPosition.ALGAE_PROCESSOR_SCORING));
		NamedCommands.registerCommand("Intake Algae Ground", new IntakeAlgaeGround(elevatedManipulator));
		NamedCommands.registerCommand("Reset Elevator", new Preset(elevatedManipulator, PresetPosition.RESET));
		NamedCommands.registerCommand("Algae Pivot Up", new InstantCommand(() -> algaeManipulator.pivotUp()));
		NamedCommands.registerCommand("Climb", new Preset(elevatedManipulator, PresetPosition.CLIMB));
		// Create a chooser with the PathPlanner Autos located in the PP
		// folders.

        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
		
        SmartDashboard.putData("Auto Program", autoChooser.getSendableChooser());
	}
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public String getAutonomousCommandName()
	{
        return getAutonomousCommand().getName();
	}
}
