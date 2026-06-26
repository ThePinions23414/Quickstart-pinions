
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
public class BlueSmallTriangleCorner15 extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private FlywheelLogicNotSortedST shooter = new FlywheelLogicNotSortedST();
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
        DRIVE_PICK1POS_SHOOT2POS,
        SHOOT_PRELOAD2,
        DRIVE_SHOOT2POS_CORNERPOS1,
        DRIVE_CORNERPOS1_CORNERPOS2,
        DRIVE_CORNERPOS2_CORNERPOS3,
        DRIVE_CORNERPOS3_SHOOT3POS,
        SHOOT_PRELOAD3,
        DRIVE_SHOOT3POS_CORNER2POS1,
        DRIVE_CORNER2POS1_CORNER2POS2,
        DRIVE_CORNER2POS2_CORNER2POS3,
        DRIVE_CORNER2POS3_SHOOT4POS,
        SHOOT_PRELOAD4,
        DRIVE_SHOOT4POS_CORNER3POS1,
        DRIVE_CORNER3POS1_CORNER3POS2,
        DRIVE_CORNER3POS2_CORNER3POS3,
        DRIVE_CORNER3POS3_SHOOT5POS,
        SHOOT_PRELOAD5,
        DRIVE_SHOOT5POS_ENDPOS
    }

    PathState pathState;

    private final Pose startPose = new Pose(55.9975, 8.177777777777768, Math.toRadians(90));
    private final Pose shoot1Pose = new Pose(55.55555, 12.177777777777763, Math.toRadians(116));
    private final Pose ball1Pose = new Pose(47.86541, 35.227930535455855, Math.toRadians(180));
    private final Pose pick1Pose = new Pose(12.73661, 35.21635311143271, Math.toRadians(180));
    private final Pose shoot2Pose = new Pose(55.55555, 12.177777777777763, Math.toRadians(116));
    private final Pose cornerPose1 = new Pose(12, 13, Math.toRadians(205));
    private final Pose cornerPose2 = new Pose(24, 16.333333333333334, Math.toRadians(195));
    private final Pose cornerPose3 = new Pose(12, 25, Math.toRadians(195));
    private final Pose shoot3Pose = new Pose(55.55555, 12.177777777777763, Math.toRadians(117));
    private final Pose corner2Pose1 = new Pose(12, 13, Math.toRadians(205));
    private final Pose corner2Pose2 = new Pose(24, 16.333333333333334, Math.toRadians(195));
    private final Pose corner2Pose3 = new Pose(12, 25, Math.toRadians(195));
    private final Pose shoot4Pose = new Pose(55.55555, 12.177777777777763, Math.toRadians(117));
    private final Pose corner3Pose1 = new Pose(12, 13, Math.toRadians(205));
    private final Pose corner3Pose2 = new Pose(24, 16.333333333333334, Math.toRadians(195));
    private final Pose corner3Pose3 = new Pose(12, 25, Math.toRadians(195));
    private final Pose shoot5Pose = new Pose(55.55555, 12.177777777777763, Math.toRadians(117));
    private final Pose endPose = new Pose(45.4, 24.37153723529663, Math.toRadians(90));



    private PathChain driveStartPosShoot1Pos, driveShoot1PosBall1Pos, driveBall1PosPick1Pos, drivePick1PosShoot2Pos, driveShoot2PosCornerPos1, driveCornerPos1CornerPos2, driveCornerPos2CornerPos3, driveCornerPos3Shoot3Pos, driveShoot3PosCorner2Pos1, driveCorner2Pos1Corner2Pos2, driveCorner2Pos2Corner2Pos3, driveCorner2Pos3Shoot4Pos, driveShoot4PosCorner3Pos1, driveCorner3Pos1Corner3Pos2, driveCorner3Pos2Corner3Pos3, driveCorner3Pos3Shoot5Pos, driveShoot5PosEndPos;

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
        driveShoot2PosCornerPos1 = follower.pathBuilder()
                .addPath(new BezierLine(shoot2Pose, cornerPose1))
                .setLinearHeadingInterpolation(shoot2Pose.getHeading(), cornerPose1.getHeading())
                .build();
        driveCornerPos1CornerPos2 = follower.pathBuilder()
                .addPath(new BezierLine(cornerPose1, cornerPose2))
                .setLinearHeadingInterpolation(cornerPose1.getHeading(), cornerPose2.getHeading())
                .build();
        driveCornerPos2CornerPos3 = follower.pathBuilder()
                .addPath(new BezierLine(cornerPose2, cornerPose3))
                .setLinearHeadingInterpolation(cornerPose2.getHeading(), cornerPose3.getHeading())
                .build();
        driveCornerPos3Shoot3Pos = follower.pathBuilder()
                .addPath(new BezierLine(cornerPose3, shoot3Pose))
                .setLinearHeadingInterpolation(cornerPose3.getHeading(), shoot3Pose.getHeading())
                .build();
        driveShoot3PosCorner2Pos1 = follower.pathBuilder()
                .addPath(new BezierLine(shoot3Pose, corner2Pose1))
                .setLinearHeadingInterpolation(shoot3Pose.getHeading(), corner2Pose1.getHeading())
                .build();
        driveCorner2Pos1Corner2Pos2 = follower.pathBuilder()
                .addPath(new BezierLine(corner2Pose1, corner2Pose2))
                .setLinearHeadingInterpolation(corner2Pose1.getHeading(), corner2Pose2.getHeading())
                .build();
        driveCorner2Pos2Corner2Pos3 = follower.pathBuilder()
                .addPath(new BezierLine(corner2Pose2, corner2Pose3))
                .setLinearHeadingInterpolation(corner2Pose2.getHeading(), corner2Pose3.getHeading())
                .build();
        driveCorner2Pos3Shoot4Pos = follower.pathBuilder()
                .addPath(new BezierLine(corner2Pose3, shoot4Pose))
                .setLinearHeadingInterpolation(corner2Pose3.getHeading(), shoot4Pose.getHeading())
                .build();
        driveShoot4PosCorner3Pos1 = follower.pathBuilder()
                .addPath(new BezierLine(shoot4Pose, corner3Pose1))
                .setLinearHeadingInterpolation(shoot4Pose.getHeading(), corner3Pose1.getHeading())
                .build();
        driveCorner3Pos1Corner3Pos2 = follower.pathBuilder()
                .addPath(new BezierLine(corner3Pose1, corner3Pose2))
                .setLinearHeadingInterpolation(corner3Pose1.getHeading(), corner3Pose2.getHeading())
                .build();
        driveCorner3Pos2Corner3Pos3 = follower.pathBuilder()
                .addPath(new BezierLine(corner3Pose2, corner3Pose3))
                .setLinearHeadingInterpolation(corner3Pose2.getHeading(), corner3Pose3.getHeading())
                .build();
        driveCorner3Pos3Shoot5Pos = follower.pathBuilder()
                .addPath(new BezierLine(corner3Pose3, shoot5Pose))
                .setLinearHeadingInterpolation(corner3Pose3.getHeading(), shoot5Pose.getHeading())
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

                        setPathState(PathState.DRIVE_SHOOT2POS_CORNERPOS1);
                    }
                }
                break;
            case DRIVE_SHOOT2POS_CORNERPOS1:
                if(pathTimer.getElapsedTimeSeconds() < 1.5) {
                    intake.intakeBalls(3);

                    if (!follower.isBusy()) {
                        follower.followPath(driveShoot2PosCornerPos1, true);
                        setPathState(PathState.DRIVE_CORNERPOS1_CORNERPOS2);
                    }

                }else{
                    setPathState(PathState.DRIVE_CORNERPOS1_CORNERPOS2);
                }
                break;
            case DRIVE_CORNERPOS1_CORNERPOS2:

                if(pathTimer.getElapsedTimeSeconds() < 1){
                    if(!follower.isBusy()){
                        follower.followPath(driveCornerPos1CornerPos2, true);
                        setPathState(PathState.DRIVE_CORNERPOS2_CORNERPOS3);
                    }
                }else{
                    setPathState(PathState.DRIVE_CORNERPOS2_CORNERPOS3);
                }

                break;

            case DRIVE_CORNERPOS2_CORNERPOS3:

                if(pathTimer.getElapsedTimeSeconds() < 1){
                    if(!follower.isBusy()){
                        follower.followPath(driveCornerPos2CornerPos3, true);
                        setPathState(PathState.DRIVE_CORNERPOS3_SHOOT3POS);
                    }
                }else {
                    setPathState(PathState.DRIVE_CORNERPOS3_SHOOT3POS);
                }

                break;

            case DRIVE_CORNERPOS3_SHOOT3POS:


                if(!follower.isBusy()){
                    follower.followPath(driveCornerPos3Shoot3Pos, true);
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
                        setPathState(PathState.DRIVE_SHOOT3POS_CORNER2POS1);
                    }
                }
                break;

            case DRIVE_SHOOT3POS_CORNER2POS1:
                if(pathTimer.getElapsedTimeSeconds() < 1.5) {
                    intake.intakeBalls(3);

                    if (!follower.isBusy()) {
                        follower.followPath(driveShoot3PosCorner2Pos1, true);
                        setPathState(PathState.DRIVE_CORNER2POS1_CORNER2POS2);
                    }

                }else{
                    setPathState(PathState.DRIVE_CORNER2POS1_CORNER2POS2);
                }
                break;
            case DRIVE_CORNER2POS1_CORNER2POS2:

                if(pathTimer.getElapsedTimeSeconds() < 1){
                    if(!follower.isBusy()){
                        follower.followPath(driveCorner2Pos1Corner2Pos2, true);
                        setPathState(PathState.DRIVE_CORNER2POS2_CORNER2POS3);
                    }
                }else{
                    setPathState(PathState.DRIVE_CORNER2POS2_CORNER2POS3);
                }

                break;

            case DRIVE_CORNER2POS2_CORNER2POS3:

                if(pathTimer.getElapsedTimeSeconds() < 1){
                    if(!follower.isBusy()){
                        follower.followPath(driveCorner2Pos2Corner2Pos3, true);
                        setPathState(PathState.DRIVE_CORNER2POS3_SHOOT4POS);
                    }
                }else {
                    setPathState(PathState.DRIVE_CORNER2POS3_SHOOT4POS);
                }

                break;

            case DRIVE_CORNER2POS3_SHOOT4POS:


                if(!follower.isBusy()){
                    follower.followPath(driveCorner2Pos3Shoot4Pos, true);
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
                        setPathState(PathState.DRIVE_SHOOT4POS_CORNER3POS1);
                    }
                }
                break;
            case DRIVE_SHOOT4POS_CORNER3POS1:
                if(pathTimer.getElapsedTimeSeconds() < 1.5) {
                    intake.intakeBalls(3);

                    if (!follower.isBusy()) {
                        follower.followPath(driveShoot4PosCorner3Pos1, true);
                        setPathState(PathState.DRIVE_CORNER3POS1_CORNER3POS2);
                    }

                }else{
                    setPathState(PathState.DRIVE_CORNER3POS1_CORNER3POS2);
                }
                break;
            case DRIVE_CORNER3POS1_CORNER3POS2:

                if(pathTimer.getElapsedTimeSeconds() < 1){
                    if(!follower.isBusy()){
                        follower.followPath(driveCorner3Pos1Corner3Pos2, true);
                        setPathState(PathState.DRIVE_CORNER3POS2_CORNER3POS3);
                    }
                }else{
                    setPathState(PathState.DRIVE_CORNER3POS2_CORNER3POS3);
                }

                break;

            case DRIVE_CORNER3POS2_CORNER3POS3:

                if(pathTimer.getElapsedTimeSeconds() < 1){
                    if(!follower.isBusy()){
                        follower.followPath(driveCorner3Pos2Corner3Pos3, true);
                        setPathState(PathState.DRIVE_CORNER3POS3_SHOOT5POS);
                    }
                }else {
                    setPathState(PathState.DRIVE_CORNER3POS3_SHOOT5POS);
                }

                break;

            case DRIVE_CORNER3POS3_SHOOT5POS:


                if(!follower.isBusy()){
                    follower.followPath(driveCorner3Pos3Shoot5Pos, true);
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



