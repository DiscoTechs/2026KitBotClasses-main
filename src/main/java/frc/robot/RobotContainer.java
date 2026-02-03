package frc.robot;

import frc.robot.commands.AutoDrive;
import frc.robot.commands.Intake;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;

import static frc.robot.Constants.FuelConstants.INTAKING_INTAKE_VOLTAGE;
import static frc.robot.Constants.FuelConstants.SPIN_UP_SECONDS;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {





  // ================= SUBSYSTEMS =================
  private final CANDriveSubsystem canDriveSubsystem =
      new CANDriveSubsystem();
  private final CANFuelSubsystem canFuelSubsystem = new CANFuelSubsystem();

  // ================= CONTROLLERS =================
 //driver
  private final XboxController driverController =
      new XboxController(0);
//operator
  private final CommandXboxController operatorController = new CommandXboxController(1); //from docs




  // ================= TRIGGERS =================
  // Right trigger acts like a button when pressed > 20%
  private final Trigger rightTrigger =
      new Trigger(() -> driverController.getRightTriggerAxis() > 0.2);

  // ================= CONSTRUCTOR =================
  public RobotContainer() {
    configureDefaultCommands();
    configureBindings();
  }

  // ================= DEFAULT DRIVE =================
  private void configureDefaultCommands() {
    canDriveSubsystem.setDefaultCommand(
        canDriveSubsystem.run(() ->
            canDriveSubsystem.driveArcade(
                -driverController.getLeftY(),
                driverController.getRightX()
            )
        )
    );

    canFuelSubsystem.setDefaultCommand(new Intake(canFuelSubsystem));
  }

  // ================= BUTTON BINDINGS =================
  private void configureBindings() {

    operatorController.leftBumper()
      .whileTrue(canFuelSubsystem.runEnd(() -> canFuelSubsystem.setIntakeLauncherRoller(-INTAKING_INTAKE_VOLTAGE), () -> canFuelSubsystem.stop())); //from docs

    //operatorController.rightBumper()`.whileTrue(canFuelSubsystem.spinUpCommand().withTmeout(SPIN_UP_SECONDS).andThen(canFuelSubsystem.LaunchCommand()).finallyDo(() -> fuelSubsystem.stop));  //from docs

    // Hold RIGHT TRIGGER to use Limelight auto-centering + forward drive
    rightTrigger.whileTrue(
        canDriveSubsystem.run(() ->
        
            canDriveSubsystem.limelightDrive()
        )
    );
  }

  // ================= AUTONOMOUS =================
  public Command getAutonomousCommand() {
    return null;
  }
}
