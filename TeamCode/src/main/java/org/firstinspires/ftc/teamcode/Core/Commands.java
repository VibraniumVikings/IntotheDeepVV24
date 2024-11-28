package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

import java.util.function.BooleanSupplier;

public class Commands {
    // Builtins
    public static Command sleep(long ms) { return new WaitCommand(ms); }
    public static Command sleepUntil(BooleanSupplier condition) { return new WaitUntilCommand(condition); }
    public static Command instant(Runnable toRun, Subsystem... requirements) { return new InstantCommand(toRun, requirements); }

    // Pedro Pathing
    public static FollowPathCommand followPath(Follower follower, PathChain path) { return new FollowPathCommand(follower, path); }
    public static FollowPathCommand followPath(Follower follower, Path path) { return new FollowPathCommand(follower, path); }
    public static FollowPathFast fastPath(Follower follower, Path path) { return new FollowPathFast(follower, path); }
    public static FollowPathFast fastPath(Follower follower, PathChain path) { return new FollowPathFast(follower, path); }

    // Intake Commands ---------------------------------------------------------------------------------------------------

    // Extend into various states
    public static InstantCommand RotateClaw45Degrees(AutonomousRobot robot) { return new InstantCommand(robot::clawTo45Degrees); }
    public static InstantCommand RotateClaw45DegreesCCW(AutonomousRobot robot) { return new InstantCommand(robot::clawToNeg45Degrees); }
    public static Command ExtendIntakeToGripSample(AutonomousRobot robot) { return new ExtendIntakeToGripSample(robot); }
    public static Command ExtendIntakeToGripSpecimen(AutonomousRobot robot) { return new ExtendIntakeToGripSpecimen(robot); }
    public static Command RotateClaw(AutonomousRobot robot, double degrees) { return new RotateClaw(robot, degrees); }

    // Sample & Specimen Grab Function
    public static Command GrabGameObjectWithIntake(AutonomousRobot robot) { return new GrabGameObjectWithIntake(robot); }
    public static Command Hold(AutonomousRobot robot) { return new Hold(robot); }
    public static Command Release(AutonomousRobot robot) { return new Release(robot); }

    // Retract the intake and end when a transfer is possible, doesn't actually transfer
    public static Command RetractIntakeForTransfer(AutonomousRobot robot) { return new RetractIntakeForTransfer(robot); }

    // Outtake Commands ---------------------------------------------------------------------------------------------------

    // First stage outtake commands
    public static Command RaiseSlidesForSampleDump(AutonomousRobot robot)  { return new RaiseSlidesForSampleDump(robot); }
    public static Command RaiseSlidesForSpecimenDump(AutonomousRobot robot) { return new RaiseSlidesForSpecimenDump(robot); }

    // Clip / dump commands, clip automatically lets go when reached to be idiot proof and not damage slides
    public static Command DumpSample(AutonomousRobot robot) { return new DumpSample(robot); }
    public static Command ClipSpecimen(AutonomousRobot robot) { return new ClipSpecimen(robot); }
}
