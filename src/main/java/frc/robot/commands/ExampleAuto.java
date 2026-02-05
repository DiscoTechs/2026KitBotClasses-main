// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.CANDriveSubsystem;
// import frc.robot.subsystems.CANFuelSubsystem;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class ExampleAuto extends SequentialCommandGroup {
//   /** Creates a new ExampleAuto. */
//   public ExampleAuto(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
//     // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());
//     addCommands(
//     // Drive backwards for .25 seconds. The driveArcadeAuto command factory
//     // intentionally creates a command which does not end which allows us to control
//     // the timing using the withTimeout decorator
//             new AutoDrive(driveSubsystem,-0.5,  0.0).withTimeout(1),
//     // Spin up the launcher for 1 second and then launch balls for 9 seconds, for a
//     // total of 10 seconds
//             new Launch(ballSubsystem).withTimeout(5));
    

  // }
// }



package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;

import static frc.robot.Constants.FuelConstants.*;

public class ExampleAuto extends SequentialCommandGroup {

  public ExampleAuto(
      CANDriveSubsystem driveSubsystem,
      CANFuelSubsystem fuelSubsystem
  ) {



       addCommands(

    // Drive BACKWARD and intake at the same time for 5 seconds
    new ParallelCommandGroup(
        new AutoDrive(driveSubsystem, 0.5, 0.0),
        new Intake(fuelSubsystem)
    ).withTimeout(2.5),

    // Stop everything
    fuelSubsystem.runOnce(fuelSubsystem::stop),
    driveSubsystem.runOnce(() -> driveSubsystem.driveArcade(0, 0)),

    // Spin up, then outtake for 5 seconds
    new SpinUp(fuelSubsystem)
        .withTimeout(SPIN_UP_SECONDS)
        .andThen(new Launch(fuelSubsystem).withTimeout(5))
        .finallyDo(fuelSubsystem::stop)
)
    ;
  }
}


