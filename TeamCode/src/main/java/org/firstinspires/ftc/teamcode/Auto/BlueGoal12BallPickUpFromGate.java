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
public class BlueGoal12BallPickUpFromGate extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer, intakeTimer;

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
        DRIVE_PICK1POS_BACKUP,
        DRIVE_BACKUP_SHOOT2POS,
        SHOOT_PRELOAD2,
        DRIVE_SHOOT2POS_GATELINEUPPOS,
        DRIVE_GATELINEUPPOS_GATEPOS,
        DRIVE_GATEPOS_RAMPPOS,
        DRIVE_RAMPPOS_BACKUP2,
        DRIVE_BACKUP2_SHOOT3POS,
        SHOOT_PRELOAD3,
        DRIVE_SHOOT3POS_GATELINEUPPOS2,
        DRIVE_GATELINEUPPOS2_GATEPOS2,
        DRIVE_GATEPOS2_RAMPPOS2,
        DRIVE_RAMPPOS2_BACKUP3,
        DRIVE_BACKUP3_SHOOT4POS,
        SHOOT_PRELOAD4,
        DRIVE_SHOOT4POS_BALL4POS,
        DRIVE_BALL4POS_PICK4POS,
        DRIVE_PICK4POS_SHOOT5POS,
        SHOOT_PRELOAD5,
        DRIVE_SHOOT5POS_ENDPOS

    }

    PathState pathState;

    private final Pose startPose = new Pose(20.24998, 121.927644166881, Math.toRadians(143.5));
    private final Pose shoot1Pose = new Pose(47.84192, 95.8023690459329, Math.toRadians(136.5));
    private final Pose ball1Pose = new Pose(44.96708, 60.26978997878086, Math.toRadians(180));
    private final Pose pick1Pose = new Pose(14.18419, 60.20268438143005, Math.toRadians(180));
    private final Pose backUp = new Pose(39, 60.20268438143005, Math.toRadians(180));
    private final Pose shoot2Pose = new Pose(47.84192, 95.8023690459329, Math.toRadians(137.5));
    private final Pose gatePose = new Pose(20.5, 64.63820390196331, Math.toRadians(180));
    private final Pose gateLineUpPose = new Pose(39, 64.63820390196331, Math.toRadians(180));
    private final Pose rampPose = new Pose(13.51,57.6, Math.toRadians(135));
    private final Pose backUp2 = new Pose(44,57.6, Math.toRadians(137.5));
    private final Pose shoot3Pose = new Pose(47.84192, 95.8023690459329, Math.toRadians(137.5));
    private final Pose gatePose2 = new Pose(21.00656, 64.63820390196331, Math.toRadians(180));
    private final Pose gateLineUpPose2 = new Pose(39, 64.63820390196331, Math.toRadians(180));
    private final Pose rampPose2 = new Pose(13.51, 57.6, Math.toRadians(135));
    private final Pose backUp3 = new Pose(44, 57.6, Math.toRadians(180));
    private final Pose shoot4Pose = new Pose(47.84192, 95.8023690459329, Math.toRadians(137.5));
    private final Pose ball4Pose = new Pose(45.98681, 84, Math.toRadians(180));
    private final Pose pick4Pose = new Pose(26.00656, 84, Math.toRadians(180));
    private final Pose shoot5Pose = new Pose(47.84192, 95.8023690459329, Math.toRadians(137.5));
    private final Pose endPose = new Pose(28.33551, 64.63820390196331, Math.toRadians(180));




    private PathChain driveStartPosShoot1Pos, driveShoot1PosBall1Pos, driveBall1PosPick1Pos, drivePick1PosBackUp, driveBackUpShoot2Pos, driveShoot2PosGateLineUpPos, driveGateLineUpPosGatePos, driveGatePosRampPos, driveRampPosBackUp2, driveBackUp2Shoot3Pos, driveShoot3PosGateLineUpPos2, driveGateLineUpPos2GatePos2, driveGatePos2RampPos2, driveRampPos2BackUp3, driveBackUp3Shoot4Pos, driveShoot4PosBall4Pos, driveBall4PosPick4Pos, drivePick4PosShoot5Pos, driveShoot5PosEndPos;

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
        drivePick1PosBackUp = follower.pathBuilder()
                .addPath(new BezierLine(pick1Pose, backUp))
                .setLinearHeadingInterpolation(pick1Pose.getHeading(), backUp.getHeading())
                .build();
        driveBackUpShoot2Pos = follower.pathBuilder()
                .addPath(new BezierLine(backUp, shoot2Pose))
                .setLinearHeadingInterpolation(backUp.getHeading(), shoot2Pose.getHeading())
                .build();
        driveShoot2PosGateLineUpPos = follower.pathBuilder()
                .addPath(new BezierLine(shoot2Pose, gateLineUpPose))
                .setLinearHeadingInterpolation(shoot2Pose.getHeading(), gateLineUpPose.getHeading())
                .build();
        driveGateLineUpPosGatePos = follower.pathBuilder()
                .addPath(new BezierLine(gateLineUpPose, gatePose))
                .setLinearHeadingInterpolation(gateLineUpPose.getHeading(), gatePose.getHeading())
                .build();
        driveGatePosRampPos = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, rampPose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), rampPose.getHeading())
                .build();
        driveRampPosBackUp2 = follower.pathBuilder()
                .addPath(new BezierLine(rampPose, backUp2))
                .setLinearHeadingInterpolation(rampPose.getHeading(), backUp2.getHeading())
                .build();
        driveBackUp2Shoot3Pos = follower.pathBuilder()
                .addPath(new BezierLine(backUp2, shoot3Pose))
                .setLinearHeadingInterpolation(backUp2.getHeading(), shoot3Pose.getHeading())
                .build();
        driveShoot3PosGateLineUpPos2 = follower.pathBuilder()
                .addPath(new BezierLine(shoot3Pose, gateLineUpPose2))
                .setLinearHeadingInterpolation(shoot3Pose.getHeading(), gateLineUpPose2.getHeading())
                .build();
        driveGateLineUpPos2GatePos2 = follower.pathBuilder()
                .addPath(new BezierLine(gateLineUpPose2, gatePose2))
                .setLinearHeadingInterpolation(gateLineUpPose2.getHeading(), gatePose2.getHeading())
                .build();
        driveGatePos2RampPos2 = follower.pathBuilder()
                .addPath(new BezierLine(gatePose2, rampPose2))
                .setLinearHeadingInterpolation(gatePose2.getHeading(), rampPose2.getHeading())
                .build();
        driveRampPos2BackUp3 = follower.pathBuilder()
                .addPath(new BezierLine(rampPose2, backUp3))
                .setLinearHeadingInterpolation(rampPose2.getHeading(), backUp3.getHeading())
                .build();
        driveBackUp3Shoot4Pos = follower.pathBuilder()
                .addPath(new BezierLine(backUp3, shoot4Pose))
                .setLinearHeadingInterpolation(backUp3.getHeading(), shoot4Pose.getHeading())
                .build();
        driveShoot4PosBall4Pos = follower.pathBuilder()
                .addPath(new BezierLine(shoot4Pose, ball4Pose))
                .setLinearHeadingInterpolation(shoot4Pose.getHeading(), ball4Pose.getHeading())
                .build();
        driveBall4PosPick4Pos = follower.pathBuilder()
                .addPath(new BezierLine(ball4Pose, pick4Pose))
                .setLinearHeadingInterpolation(ball4Pose.getHeading(), pick4Pose.getHeading())
                .build();
        drivePick4PosShoot5Pos = follower.pathBuilder()
                .addPath(new BezierLine(pick4Pose, shoot5Pose))
                .setLinearHeadingInterpolation(pick4Pose.getHeading(), shoot5Pose.getHeading())
                .build();
        driveShoot5PosEndPos = follower.pathBuilder()
                .addPath(new BezierLine(shoot5Pose, endPose))
                .setLinearHeadingInterpolation(shoot5Pose.getHeading(), endPose.getHeading())
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
                    setPathState(PathState.DRIVE_PICK1POS_BACKUP);
                }
                break;
            case DRIVE_PICK1POS_BACKUP:
                if(!follower.isBusy()){
                    follower.followPath(drivePick1PosBackUp, true);
                    setPathState(PathState.DRIVE_BACKUP_SHOOT2POS);
                }

                break;
            case DRIVE_BACKUP_SHOOT2POS:
                if(!follower.isBusy()){
                    follower.followPath(driveBackUpShoot2Pos, true);
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

                        setPathState(PathState.DRIVE_SHOOT2POS_GATELINEUPPOS);
                    }
                }
                break;
            case DRIVE_SHOOT2POS_GATELINEUPPOS:
                if(!follower.isBusy()){
                    follower.followPath(driveShoot2PosGateLineUpPos, true);
                    setPathState(PathState.DRIVE_GATELINEUPPOS_GATEPOS);
                }

                break;
            case DRIVE_GATELINEUPPOS_GATEPOS:
                if(!follower.isBusy()){
                    follower.followPath(driveGateLineUpPosGatePos, true);
                    intakeTimer.resetTimer();
                    setPathState(PathState.DRIVE_GATEPOS_RAMPPOS);
                }

                break;
            case DRIVE_GATEPOS_RAMPPOS:
                intake.intakeBalls(3);
                if(!follower.isBusy()){
                    follower.followPath(driveGatePosRampPos, true);
                    if(intakeTimer.getElapsedTimeSeconds() > 3.5){
                        setPathState(PathState.DRIVE_RAMPPOS_BACKUP2);
                    }
                }
                break;
            case DRIVE_RAMPPOS_BACKUP2:
                if(!follower.isBusy()){
                    follower.followPath(driveRampPosBackUp2, true);
                    setPathState(PathState.DRIVE_BACKUP2_SHOOT3POS);
                }
                break;
            case DRIVE_BACKUP2_SHOOT3POS:
                if(!follower.isBusy()){
                    follower.followPath(driveBackUp2Shoot3Pos, true);
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
                        setPathState(PathState.DRIVE_SHOOT4POS_BALL4POS);
                    }
                }
                break;
            case DRIVE_SHOOT3POS_GATELINEUPPOS2:
                if(!follower.isBusy()){
                    follower.followPath(driveShoot3PosGateLineUpPos2, true);
                    setPathState(PathState.DRIVE_GATELINEUPPOS2_GATEPOS2);
                }
                break;
            case DRIVE_GATELINEUPPOS2_GATEPOS2:
                if(!follower.isBusy()){
                    follower.followPath(driveGateLineUpPos2GatePos2, true);
                    intakeTimer.resetTimer();
                    setPathState(PathState.DRIVE_GATEPOS2_RAMPPOS2);
                }
                break;
            case DRIVE_GATEPOS2_RAMPPOS2:
                intake.intakeBalls(3);
                if(!follower.isBusy()){
                    follower.followPath(driveGatePos2RampPos2, true);
                    if(intakeTimer.getElapsedTimeSeconds() > 2){
                        setPathState(PathState.DRIVE_RAMPPOS2_BACKUP3);
                    }
                }

                break;
            case DRIVE_RAMPPOS2_BACKUP3:
                if(!follower.isBusy()){
                    follower.followPath(driveRampPos2BackUp3, true);
                    setPathState(PathState.DRIVE_BACKUP3_SHOOT4POS);
                }
                break;
            case DRIVE_BACKUP3_SHOOT4POS:
                if(!follower.isBusy()){
                    follower.followPath(driveBackUp3Shoot4Pos, true);
                    setPathState(PathState.SHOOT_PRELOAD4);
                }
                break;
            case SHOOT_PRELOAD4:
                if (!follower.isBusy()&& !intake.isStillGoing()) {
                    if (!shotsTriggered) {
                        shooter.fireShots(3);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.isBusy()) {
                        setPathState(PathState.DRIVE_SHOOT4POS_BALL4POS);
                    }
                }
                break;
            case DRIVE_SHOOT4POS_BALL4POS:
                if(!follower.isBusy()){
                    follower.followPath(driveShoot4PosBall4Pos, true);
                    setPathState(PathState.DRIVE_BALL4POS_PICK4POS);
                }

                break;
            case DRIVE_BALL4POS_PICK4POS:
                intake.intakeBalls(3);
                if(!follower.isBusy()){
                    follower.followPath(driveBall4PosPick4Pos, true);
                    setPathState(PathState.DRIVE_PICK4POS_SHOOT5POS);
                }
                break;
            case DRIVE_PICK4POS_SHOOT5POS:
                if(!follower.isBusy()){
                    follower.followPath(drivePick4PosShoot5Pos, true);
                    setPathState(PathState.SHOOT_PRELOAD5);
                }

                break;
            case SHOOT_PRELOAD5:
                if (!follower.isBusy()&& !intake.isStillGoing()) {
                    if (!shotsTriggered) {
                        shooter.fireShots(3);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.isBusy()) {
                        setPathState(PathState.DRIVE_SHOOT5POS_ENDPOS);
                    }
                }
                break;
            case DRIVE_SHOOT5POS_ENDPOS:
                if(!follower.isBusy()){
                    follower.followPath(driveShoot5PosEndPos, true);
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
        intakeTimer = new Timer();
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

