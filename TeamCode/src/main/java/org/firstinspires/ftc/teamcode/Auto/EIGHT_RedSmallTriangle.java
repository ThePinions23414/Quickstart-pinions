
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
public class EIGHT_RedSmallTriangle extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private FlywheelLogicSmallTriangle shooter = new FlywheelLogicSmallTriangle();
    private IntakeLogicSmallTriangle intake = new IntakeLogicSmallTriangle();

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
        DRIVE_PICK1POS_SHOOT2POS,
        SHOOT_PRELOAD2,
        DRIVE_SHOOT2POS_BALL2POS,
        DRIVE_BALL2POS_BALL2POS2,
        DRIVE_BALL2POS2_BALL2POS3,
        DRIVE_BALL2POS3_SHOOT3POS,




        SHOOT_PRELOAD3,
        DRIVE_SHOOT3POS_ENDPOS
    }

    PathState pathState;

    private final Pose startPose = new Pose(80.35, 8.177777777777768, Math.toRadians(90));
    private final Pose shoot1Pose = new Pose(84.5, 12.177777777777763, Math.toRadians(64));
    private final Pose ball1Pose = new Pose(99.5, 35.37777777777776, Math.toRadians(0));
    private final Pose pick1Pose = new Pose(135.73, 35.688888888888876, Math.toRadians(0));
    private final Pose shoot2Pose = new Pose(84.5, 12.177777777777763, Math.toRadians(64));
    private final Pose ball2Pose = new Pose(130.73, 16.488888888888887, Math.toRadians(345));
    private final Pose ball2Pose2 = new Pose(122, 16.333333333333334, Math.toRadians(345));
    private final Pose ball2Pose3 = new Pose(131, 11.955555555555568, Math.toRadians(340));
    private final Pose shoot3Pose = new Pose(84.5, 12.177777777777763, Math.toRadians(64));
    private final Pose endPose = new Pose(98.6, 24.37153723529663, Math.toRadians(90));



    private PathChain driveStartPosShoot1Pos, driveShoot1PosBall1Pos, driveBall1PosPick1Pos, drivePick1PosShoot2Pos, driveShoot2PosBall2Pos, ball2Pos1Ball2Pos2, ball2Pos2Ball2Pos3, ball2PosShoot3Pos, driveShoot3PosEndPos;

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
        drivePick1PosShoot2Pos = follower.pathBuilder()
                .addPath(new BezierLine(pick1Pose, shoot2Pose))
                .setLinearHeadingInterpolation(pick1Pose.getHeading(), shoot2Pose.getHeading())
                .build();
        driveShoot2PosBall2Pos = follower.pathBuilder()
                .addPath(new BezierLine(shoot2Pose, ball2Pose))
                .setLinearHeadingInterpolation(shoot2Pose.getHeading(), ball2Pose.getHeading())
                .build();
        ball2Pos1Ball2Pos2 = follower.pathBuilder()
                .addPath(new BezierLine(ball2Pose, ball2Pose2))
                .setLinearHeadingInterpolation(ball2Pose.getHeading(), ball2Pose2.getHeading())
                .build();
        ball2Pos2Ball2Pos3 = follower.pathBuilder()
                .addPath(new BezierLine(ball2Pose2, ball2Pose3))
                .setLinearHeadingInterpolation(ball2Pose2.getHeading(), ball2Pose3.getHeading())
                .build();
        ball2PosShoot3Pos = follower.pathBuilder()
                .addPath(new BezierLine(ball2Pose3, shoot3Pose))
                .setLinearHeadingInterpolation(ball2Pose3.getHeading(), shoot3Pose.getHeading())
                .build();

        driveShoot3PosEndPos = follower.pathBuilder()
                .addPath(new BezierLine(shoot3Pose, endPose))
                .setLinearHeadingInterpolation(shoot3Pose.getHeading(), endPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch(pathState) {
            case DRIVE_STARTPOS_SHOOT1POS:
                shooter.getID();
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
                    setPathState(PathState.DRIVE_PICK1POS_SHOOT2POS);
                }
                break;
            case DRIVE_PICK1POS_SHOOT2POS:
                if(!follower.isBusy()){
                    follower.followPath(drivePick1PosShoot2Pos, true);
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
                if(pathTimer.getElapsedTimeSeconds() < 1.5) {
                    intake.intakeBalls(3);

                    if (!follower.isBusy()) {
                        follower.followPath(driveShoot2PosBall2Pos, true);
                        setPathState(PathState.DRIVE_BALL2POS_BALL2POS2);
                    }

                }else{
                    setPathState(PathState.DRIVE_BALL2POS_BALL2POS2);
                }
                break;
            case DRIVE_BALL2POS_BALL2POS2:

                if(pathTimer.getElapsedTimeSeconds() < 1){
                    if(!follower.isBusy()){
                        follower.followPath(ball2Pos1Ball2Pos2, true);
                        setPathState(PathState.DRIVE_BALL2POS2_BALL2POS3);
                    }
                }else{
                    setPathState(PathState.DRIVE_BALL2POS2_BALL2POS3);
                }

                break;

            case DRIVE_BALL2POS2_BALL2POS3:

                if(pathTimer.getElapsedTimeSeconds() < 1){
                    if(!follower.isBusy()){
                        follower.followPath(ball2Pos2Ball2Pos3, true);
                        setPathState(PathState.DRIVE_BALL2POS3_SHOOT3POS);
                    }
                }else {
                    setPathState(PathState.DRIVE_BALL2POS3_SHOOT3POS);
                }

                break;

            case DRIVE_BALL2POS3_SHOOT3POS:


                if(!follower.isBusy()){
                    follower.followPath(ball2PosShoot3Pos, true);
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
        shooter.startLimeLight();
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


