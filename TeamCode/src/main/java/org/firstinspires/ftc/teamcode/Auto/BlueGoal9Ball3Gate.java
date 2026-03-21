package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class BlueGoal9Ball3Gate extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private FlywheelLogicNotSorted shooter = new FlywheelLogicNotSorted();
    private IntakeLogicNotSorted intake = new IntakeLogicNotSorted();

    private boolean shotsTriggered = false;
    private boolean intakeTriggered = false;

    public enum PathState {
        //START POSITION_END POSITION
        //DRIVE > MOVEMENT STATE
        //SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        DRIVE_STARTPOS_SHOOT1POS,
        SHOOT_PRELOAD1,
        DRIVE_SHOOT1POS_BALL1POS,
        DRIVE_BALL1POS_PICK1POS,
        DRIVE_PICK1POS_GATEPOS,
        DRIVE_GATEPOS_SHOOT2POS,
        SHOOT_PRELOAD2,
        DRIVE_SHOOT2POS_BALL2POS,
        DRIVE_BALL2POS_PICK2POS,
        DRIVE_PICK2POS_BACK_UP,
        DRIVE_BACK_UP_SHOOT3POS,
        SHOOT_PRELOAD3,
        DRIVE_SHOOT3POS_ENDPOS
    }

    PathState pathState;

    private final Pose startPose = new Pose(20.24998, 121.927644166881, Math.toRadians(143.5));
    private final Pose shoot1Pose = new Pose(47.84192, 95.8023690459329, Math.toRadians(136.5));
    private final Pose ball1Pose = new Pose(45.98681, 84, Math.toRadians(180));
    private final Pose pick1Pose = new Pose(28.00656, 84, Math.toRadians(180));
    private final Pose gatePose = new Pose(21.00656, 77, Math.toRadians(180));
    private final Pose shoot2Pose = new Pose(47.84192, 95.8023690459329, Math.toRadians(138.5));
    private final Pose ball2Pose = new Pose(44.96708, 60.26978997878086, Math.toRadians(180));
    private final Pose pick2Pose = new Pose(14.18419, 60.20268438143005, Math.toRadians(180));
    private final Pose backUp = new Pose(25.18419, 60.20268438143005, Math.toRadians(180));
    private final Pose shoot3Pose = new Pose(47.84192, 95.8023690459329, Math.toRadians(138.5));
    private final Pose endPose = new Pose(28.33551, 64.63820390196331, Math.toRadians(180));




    private PathChain driveStartPosShoot1Pos, driveShoot1PosBall1Pos, driveBall1PosPick1Pos, drivePick1PosGatePos, driveGatePosShoot2Pos, driveShoot2PosBall2Pos, driveBall2PosPick2Pos, drivePick2PosBackUp, driveBackUpShoot3Pos, driveShoot3PosEndPos;

    public void buildPaths() {
        driveStartPosShoot1Pos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shoot1Pose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shoot1Pose.getHeading())
                .build();
        driveShoot1PosBall1Pos = follower.pathBuilder()
                .addPath(new BezierLine(shoot1Pose, ball1Pose))
                .setLinearHeadingInterpolation(shoot1Pose.getHeading(), ball1Pose.getHeading())
                .build();
        driveBall1PosPick1Pos = follower.pathBuilder()
                .addPath(new BezierLine(ball1Pose, pick1Pose))
                .setLinearHeadingInterpolation(ball1Pose.getHeading(), pick1Pose.getHeading())
                .build();
        drivePick1PosGatePos = follower.pathBuilder()
                .addPath(new BezierLine(pick1Pose, gatePose))
                .setLinearHeadingInterpolation(pick1Pose.getHeading(), gatePose.getHeading())
                .build();
        driveGatePosShoot2Pos = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, shoot2Pose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), shoot2Pose.getHeading())
                .build();
        driveShoot2PosBall2Pos = follower.pathBuilder()
                .addPath(new BezierLine(shoot2Pose, ball2Pose))
                .setLinearHeadingInterpolation(shoot2Pose.getHeading(), ball2Pose.getHeading())
                .build();
        driveBall2PosPick2Pos = follower.pathBuilder()
                .addPath(new BezierLine(ball2Pose, pick2Pose))
                .setLinearHeadingInterpolation(ball2Pose.getHeading(), pick2Pose.getHeading())
                .build();
        drivePick2PosBackUp = follower.pathBuilder()
                .addPath(new BezierLine(pick2Pose, backUp))
                .setLinearHeadingInterpolation(pick2Pose.getHeading(), backUp.getHeading())
                .build();
        driveBackUpShoot3Pos = follower.pathBuilder()
                .addPath(new BezierLine(backUp, shoot3Pose))
                .setLinearHeadingInterpolation(backUp.getHeading(), shoot3Pose.getHeading())
                .build();
        driveShoot3PosEndPos = follower.pathBuilder()
                .addPath(new BezierLine(shoot3Pose, endPose))
                .setLinearHeadingInterpolation(shoot3Pose.getHeading(), endPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch(pathState) {
            case DRIVE_STARTPOS_SHOOT1POS:

                shooter.spinUp(true);
                follower.followPath(driveStartPosShoot1Pos, true);
                setPathState(PathState.SHOOT_PRELOAD1);
                break;
            case SHOOT_PRELOAD1:
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        shooter.fireShots(3);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.isBusy()) {
                        shotsTriggered = false;
                        setPathState(PathState.DRIVE_SHOOT1POS_BALL1POS);
                    }
                }
                break;
            case DRIVE_SHOOT1POS_BALL1POS:
                if(!follower.isBusy()){
                    follower.followPath(driveShoot1PosBall1Pos, true);
                    setPathState(PathState.DRIVE_BALL1POS_PICK1POS);
                }

                break;
            case DRIVE_BALL1POS_PICK1POS:
                intake.intakeBalls(3);
                if(!follower.isBusy()){
                    follower.followPath(driveBall1PosPick1Pos, true);
                    setPathState(PathState.DRIVE_PICK1POS_GATEPOS);
                }
                break;
            case DRIVE_PICK1POS_GATEPOS:
                if(!follower.isBusy()){
                    follower.followPath(drivePick1PosGatePos, true);
                    setPathState(PathState.DRIVE_GATEPOS_SHOOT2POS);
                }

                break;
            case DRIVE_GATEPOS_SHOOT2POS:
                if(!follower.isBusy()){
                    follower.followPath(driveGatePosShoot2Pos, true);
                    setPathState(PathState.SHOOT_PRELOAD2);
                }

                break;
            case SHOOT_PRELOAD2:
                if (!follower.isBusy() && !intake.isStillGoing()) {
                    if (!shotsTriggered) {
                        shooter.fireShots(3);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.isBusy()) {

                        setPathState(PathState.DRIVE_SHOOT2POS_BALL2POS);
                    }
                }
                break;
            case DRIVE_SHOOT2POS_BALL2POS:
                if(!follower.isBusy()){
                    follower.followPath(driveShoot2PosBall2Pos, true);
                    setPathState(PathState.DRIVE_BALL2POS_PICK2POS);
                }

                break;
            case DRIVE_BALL2POS_PICK2POS:
                intake.intakeBalls(3);
                if(!follower.isBusy()){
                    follower.followPath(driveBall2PosPick2Pos, true);
                    setPathState(PathState.DRIVE_PICK2POS_BACK_UP);
                }
                break;
            case DRIVE_PICK2POS_BACK_UP:
                if(!follower.isBusy()){
                    follower.followPath(drivePick2PosBackUp, true);
                    setPathState(PathState.DRIVE_BACK_UP_SHOOT3POS);
                }
                break;
            case DRIVE_BACK_UP_SHOOT3POS:
                if(!follower.isBusy()){
                    follower.followPath(driveBackUpShoot3Pos, true);
                    setPathState(PathState.SHOOT_PRELOAD3);
                }
                break;
            case SHOOT_PRELOAD3:
                if (!follower.isBusy()&& !intake.isStillGoing()) {
                    if (!shotsTriggered) {
                        shooter.fireShots(3);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.isBusy()) {
                        setPathState(PathState.DRIVE_SHOOT3POS_ENDPOS);
                    }
                }
                break;
            case DRIVE_SHOOT3POS_ENDPOS:
                if(!follower.isBusy()){
                    follower.followPath(driveShoot3PosEndPos, true);
                    telemetry.addLine("Done Autonomous");
                }

                break;
            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();

        shotsTriggered = false;
        intakeTriggered = false;
    }

    @Override
    public void init(){
        pathState = PathState.DRIVE_STARTPOS_SHOOT1POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);


        shooter.init(hardwareMap);
        intake.init(hardwareMap);
        buildPaths();
        follower.setPose(startPose);

    }

    @Override
    public void init_loop() {
        shooter.init(hardwareMap);
    }






    @Override
    public void start() {
        opModeTimer.resetTimer();

        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        shooter.update();
        intake.update();
        statePathUpdate();
        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());


    }
}
