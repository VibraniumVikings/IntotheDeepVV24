package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

public class FollowPathFast extends CommandBase {
    private final Follower follower;
    private final PathChain path;
    private boolean holdEnd = false;
    private double maxSpeed = 1;

    public FollowPathFast(Follower follower, PathChain path) {
        this.follower = follower;
        this.path = path;
    }

    public FollowPathFast(Follower follower, Path path) {
        this(follower, new PathChain(path));
    }

    /**
     * Decides whether or not to make the robot maintain its position once the path ends.
     *
     * @param holdEnd If the robot should maintain its ending position
     * @return This command for compatibility in command groups
     */
    public FollowPathFast setHoldEnd(boolean holdEnd) {
        this.holdEnd = holdEnd;
        return this;
    }

    /**
     * Sets the follower's maximum speed
     * @param speed Between 0 and 1
     * @return This command for compatibility in command groups
     */
    public FollowPathFast setSpeed(double speed) {
        this.maxSpeed = speed;
        return this;
    }

    @Override
    public void initialize() {
        follower.setMaxPower(this.maxSpeed);
        follower.followPath(path, holdEnd);
    }

    @Override
    public boolean isFinished() {
        if ( follower.getCurrentPathNumber() == this.path.size() - 1 && Math.abs(follower.headingError) < 0.05 ) {
            return follower.getCurrentTValue() > 0.95;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        follower.setMaxPower(1);
    }
}