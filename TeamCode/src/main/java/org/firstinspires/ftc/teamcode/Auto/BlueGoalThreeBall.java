package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class BlueGoalThreeBall extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;
// hello. andrea's dad SUCKS!!!!!!!!!!!
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

    private final Pose startPose = new Pose(20.61971830985918,122.02816901408448, Math.toRadians(325));
    private final Pose shoot1Pose = new Pose(47.66197183098591,95.71830985915494, Math.toRadians(138));
    private final Pose ballPose = new Pose(47.74647887323944,84.3943661971831, Math.toRadians(180));
    private final Pose pickPose = new Pose(15.098591549295772,84.05633802816904, Math.toRadians(180));
    private final Pose shoot2Pose = new Pose(47.94366197183098,95.88732394366201, Math.toRadians(138));
    private final Pose endPose = new Pose(24.056338028169026,71.7605633802817, Math.toRadians(0));

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
