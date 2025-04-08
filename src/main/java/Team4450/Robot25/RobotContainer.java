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

import Team4450.Robot25.commands.DriveCommands;
import Team4450.Robot25.commands.IntakeCoral;
import Team4450.Robot25.commands.OuttakeCoral;
import Team4450.Robot25.commands.RemoveAlgae;

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

import Team4450.Robot25.Constants;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Compressor;

import static Team4450.Robot25.Constants.REV_PDB;

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
  public static CoralManipulator coralManipulator;
  public static AlgaeManipulator algaeManipulator;
  public static AlgaeGroundIntake algaeGroundIntake;
  public static Elevator elevator;
  public static ElevatedManipulator elevatedManipulator;

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController utilityController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private PowerDistribution		pdp = new PowerDistribution(REV_PDB, PowerDistribution.ModuleType.kRev);
  private Compressor				pcm = new Compressor(PneumaticsModuleType.REVPH);
  private final MonitorPDP     		monitorPDPThread;
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
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    elevator.setDefaultCommand(
        Commands.run(() -> elevator.move(-utilityController.getLeftY()), elevator));

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
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    

    utilityController
        .x()
        .onTrue(
            Commands.runOnce(
                    () ->
                        elevatedManipulator.executeSetPosition(PresetPosition.CORAL_SCORING_L1))
                .ignoringDisable(false));

    utilityController
        .a()
        .onTrue(
            Commands.runOnce(
                    () ->
                        elevatedManipulator.executeSetPosition(PresetPosition.CORAL_SCORING_L2))
                .ignoringDisable(false));

    utilityController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        elevatedManipulator.executeSetPosition(PresetPosition.CORAL_SCORING_L3))
                .ignoringDisable(false));
        
    utilityController
        .y()
        .onTrue(
            Commands.runOnce(
                    () ->
                        elevatedManipulator.executeSetPosition(PresetPosition.CORAL_SCORING_L4))
                .ignoringDisable(false));
    
    utilityController
        .back()
        .onTrue(
            Commands.runOnce(
                    () ->
                        elevatedManipulator.executeSetPosition(PresetPosition.RESET))
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
            .ignoringDisable(false));

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
		
		if (monitorPDPThread != null) monitorPDPThread.reset();
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
