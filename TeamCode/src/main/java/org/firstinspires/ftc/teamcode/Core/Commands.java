package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Core.vvHardwareITDPedro;
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

    // Pickup Commands ---------------------------------------------------------------------------------------------------

    // Extend into various states
    public static InstantCommand closeClaw(vvHardwareITDPedro robot) { return new InstantCommand(robot::closeClaw); }
    public static InstantCommand openClaw(vvHardwareITDPedro robot) { return new InstantCommand(robot::openClaw); }

    // Sample & Specimen Grab Function
    public static Command pickSample(vvHardwareITDPedro robot) { return new pickSample(robot); }
    public static Command Hold(vvHardwareITDPedro robot) { return new Hold(robot); }
    public static Command Release(vvHardwareITDPedro robot) { return new Release(robot); }

    // Carry and collapse
    public static Command collapse(vvHardwareITDPedro robot) { return new collapse(robot); }

    // Outtake Commands ---------------------------------------------------------------------------------------------------

    // Basket commands
    public static Command rearBasket(vvHardwareITDPedro robot)  { return new rearBasket(robot); }
    public static Command highChamber(vvHardwareITDPedro robot) { return new highChamber(robot); }

    // Clip / dump commands, clip automatically lets go when reached
    public static Command wallPick(AvvHardwareITDPedro robot) { return new wallPick(robot); }
    public static Command clipSpecimen(vvHardwareITDPedro robot) { return new clipSpecimen(robot); }
}
