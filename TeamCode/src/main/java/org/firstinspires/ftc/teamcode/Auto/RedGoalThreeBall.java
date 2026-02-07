package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class RedGoalThreeBall extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        //START POSITION_END POSITION
        //DRIVE > MOVEMENT STATE
        //SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        DRIVE_STARTPOS_SHOOTPOS1,
        SHOOT_PRELOAD1,
        SHOOT_PRELOAD2,
        DRIVE_SHOOTPOS1_BALLPOS,
        DRIVE_BALLPOS_PICKPOS,
        DRIVE_PICKPOS_SHOOTPOS2,
        DRIVE_SHOOTPOS2_ENDPOS
    }

    PathState pathState;

    private final Pose startPose = new Pose(123.15492957746481,122.70422535211266, Math.toRadians(216));
    private final Pose shoot1Pose = new Pose(96.33802816901408,96.16901408450704, Math.toRadians(41));
    private final Pose ballPose = new Pose(96.19718309859155,83.94366197183099, Math.toRadians(0));
    private final Pose pickPose = new Pose(129.07042253521126,83.47887323943662, Math.toRadians(0));
    private final Pose shoot2Pose = new Pose(96.3943661971831,96.3380281690141, Math.toRadians(41));
    private final Pose endPose = new Pose(120.05633802816902,71.98591549295774, Math.toRadians(180));

    private PathChain driveStartPosShootPos1, driveShootPos1BallPos, driveBallPosPickPos, drivePickPosShootPos2, driveShootPos2EndPos;

    public void buildPaths() {
        driveStartPosShootPos1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shoot1Pose))
                .setLinearHeadingInterpolation(startPose.getHeading(),shoot1Pose.getHeading())
                .build();
        driveShootPos1BallPos = follower.pathBuilder()
                .addPath(new BezierLine(shoot1Pose, ballPose))
                .setLinearHeadingInterpolation(shoot1Pose.getHeading(), ballPose.getHeading())
                .build();
        driveBallPosPickPos = follower.pathBuilder()
                .addPath(new BezierLine(ballPose, pickPose))
                .setLinearHeadingInterpolation(ballPose.getHeading(), pickPose.getHeading())
                .build();
        drivePickPosShootPos2 = follower.pathBuilder()
                .addPath(new BezierLine(pickPose, shoot2Pose))
                .setLinearHeadingInterpolation(pickPose.getHeading(), shoot2Pose.getHeading())
                .build();
        driveShootPos2EndPos = follower.pathBuilder()
                .addPath(new BezierLine(shoot2Pose, endPose))
                .setLinearHeadingInterpolation(shoot2Pose.getHeading(), endPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch(pathState) {
            case DRIVE_STARTPOS_SHOOTPOS1:
                follower.followPath(driveStartPosShootPos1, true);
                setPathState(PathState.SHOOT_PRELOAD1);
                break;
            case SHOOT_PRELOAD1:
                if (!follower.isBusy()) {
                    //flywheel logic
                    telemetry.addLine("Done Path 1");
                }
                setPathState(PathState.DRIVE_SHOOTPOS1_BALLPOS);
                break;
            case DRIVE_SHOOTPOS1_BALLPOS:
                follower.followPath(driveShootPos1BallPos, true);
                setPathState(PathState.DRIVE_BALLPOS_PICKPOS);
                break;
            case DRIVE_BALLPOS_PICKPOS:
                follower.followPath(driveBallPosPickPos, true);
                setPathState(PathState.DRIVE_PICKPOS_SHOOTPOS2);
                break;
            case DRIVE_PICKPOS_SHOOTPOS2:
                follower.followPath(drivePickPosShootPos2, true);
                setPathState(PathState.SHOOT_PRELOAD2);
                break;
            case SHOOT_PRELOAD2:
                if (!follower.isBusy()) {
                    //flywheel logic
                    telemetry.addLine("Done Path 2");
                }
                setPathState(PathState.DRIVE_SHOOTPOS2_ENDPOS);
                break;
            case DRIVE_SHOOTPOS2_ENDPOS:
                follower.followPath(driveShootPos2EndPos, true);
                break;
            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SHOOTPOS1;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setPose(startPose);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
    }
}
