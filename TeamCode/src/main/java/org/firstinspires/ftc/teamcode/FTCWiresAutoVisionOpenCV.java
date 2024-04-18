/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.jetbrains.annotations.NotNull;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.lang.Math;
import java.util.List;

/**
 * FTC WIRES Autonomous Example for only vision detection using tensorflow and park
 */
@Autonomous(name = "FTC Wires Auto Open CV Vision", group = "00-Autonomous", preselectTeleOp = "FTC Wires TeleOp")
public class FTCWiresAutoVisionOpenCV extends LinearOpMode {

    public static String TEAM_NAME = "A.S.T.R.O. Vikings"; //TO DO: Enter team Name //done
    public static int TEAM_NUMBER = 23974; //TO DO: Enter team Number //done

    static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    //Vision parameters
    VisionOpenCV visionOpenCV;

    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    public static START_POSITION startPosition;

    public enum IDENTIFIED_SPIKE_MARK_LOCATION {
        LEFT,
        MIDDLE,
        RIGHT
    }
    public static IDENTIFIED_SPIKE_MARK_LOCATION identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;

    DcMotor leftExtend;
    DcMotor rightExtend;
    DcMotor leftRotate;
    DcMotor rightRotate;
    Servo wristServo;
    Servo leftFingerServo;
    Servo rightFingerServo;

    Servo droneServo;
    double extendPower = 0.4;
    double rotatePower = 0.4;

//    int upperExtendLimit = 7800;
//    int upperRotateLimit = 1850;
//    int lowerMotorLimit = -30;
    final int wristDown = 0;
    double wristUp = 0.8;
    final int leftFingerOpen = 0;
    final int leftFingerClosed = 1;
    final int rightFingerOpen = 1;
    final int rightFingerClosed = 0;
    final int armRotateTop = 1850;
    final int armRotateBottom = 0;

    int armExtensionPos = 0;

    //Method to set arms to extend to certain spot

    public void extendArmTo(int extensionPos) {
//        if (leftExtend.getCurrentPosition() > lowerMotorLimit && leftExtend.getCurrentPosition() < upperExtendLimit) {
            leftExtend.setTargetPosition(extensionPos);
            rightExtend.setTargetPosition(extensionPos);
            leftExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftExtend.setPower(0.5);
            rightExtend.setPower(0.5);
//        }
    }
    //Method to set arms to rotate to certain spot

    public void rotateArmTo(int rotationPos) {
//        if (leftRotate.getCurrentPosition() > lowerMotorLimit && leftRotate.getCurrentPosition() < upperRotateLimit) {
            leftRotate.setTargetPosition(rotationPos);
            rightRotate.setTargetPosition(rotationPos);
            leftRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRotate.setPower(0.5);
            rightRotate.setPower(0.5);
//        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        leftExtend = hardwareMap.get(DcMotor.class, "leftExtend");
        leftExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightExtend = hardwareMap.get(DcMotor.class, "rightExtend");
        rightExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRotate = hardwareMap.get(DcMotor.class, "leftRotate");
        leftRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRotate = hardwareMap.get(DcMotor.class, "rightRotate");
        rightRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        rightRotate.setDirection(DcMotorSimple.Direction.REVERSE);
        rightExtend.setDirection(DcMotorSimple.Direction.REVERSE);

        wristServo = hardwareMap.servo.get("wrist");
        leftFingerServo = hardwareMap.servo.get("leftFinger");
        rightFingerServo = hardwareMap.servo.get("rightFinger");

        droneServo = hardwareMap.servo.get("droneRelease");

        leftFingerServo.setPosition(leftFingerClosed);
        rightFingerServo.setPosition(rightFingerClosed);

        //Key Pay inputs to selecting Starting Position of robot
        selectStartingPosition();
        telemetry.addData("Selected Starting Position", startPosition);

        //Activate Camera Vision that uses Open CV Vision processor for Team Element detection
        initOpenCV();

        // Wait for the DS start button to be touched.
        telemetry.addLine("Open CV Vision for Red/Blue Team Element Detection");
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addLine("The starting point of the robot is assumed to be on the starting tile, " +
                "and along the edge farther from the truss legs. ");
        telemetry.addLine("You should also have a webcam connected and positioned in a way to see " +
                "the middle spike mark and the spike mark away from the truss (and ideally nothing else). " +
                "We assumed the camera to be in the center of the robot. ");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        //waitForStart();

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Selected Starting Position", startPosition);

            //Run Open CV Object Detection and keep watching for the Team Element on the spike marks.
            runOpenCVObjectDetection();
        }

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            //Build parking trajectory based on last detected target by vision
            runAutonoumousMode();
        }
    }   // end runOpMode()

    public void runAutonoumousMode() {

        droneServo.setPosition(0);

        //Initialize Pose2d as desired
        Pose2d initPose = new Pose2d(0, 0, 0); // Starting Pose
        Pose2d moveBeyondTrussPose = new Pose2d(0,0,0);
        Pose2d dropPurplePixelPose = new Pose2d(0, 0, 0);
        Pose2d reOrientAfterPurple = new Pose2d(0,0,0);
        Pose2d midwayPose1 = new Pose2d(0,0,0);
        Pose2d midwayPose1a = new Pose2d(0,0,0);
        Pose2d intakeStack = new Pose2d(0,0,0);
        Pose2d midwayPose2 = new Pose2d(0,0,0);
        Pose2d dropYellowPixelPose = new Pose2d(0, 0, 0);
        Pose2d safeParkAdjustment = new Pose2d(0,0,0);
        Pose2d parkPose = new Pose2d(0,0, 0);
        double waitSecondsBeforeDrop = 0;
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        initPose = new Pose2d(0, 0, Math.toRadians(0)); //Starting pose
        moveBeyondTrussPose = new Pose2d(15,0,0);

        switch (startPosition) {
            case BLUE_LEFT:
                drive = new MecanumDrive(hardwareMap, initPose);
                safeParkAdjustment = new Pose2d(5, 35, Math.toRadians(-90));
                reOrientAfterPurple = new Pose2d(15, 3, Math.toRadians(0));
                wristUp = 0.6;
                switch(identifiedSpikeMarkLocation){
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(26, 12, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(21, 38, Math.toRadians(-90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(30, 3, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(28, 38,  Math.toRadians(-90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(30, -6, Math.toRadians(-45));
                        dropYellowPixelPose = new Pose2d(35, 38, Math.toRadians(-90));
                        break;
                }
                midwayPose1 = new Pose2d(15, 24, Math.toRadians(-90));
                waitSecondsBeforeDrop = 0; //TODO: Adjust time to wait for alliance partner to move from board
                parkPose = new Pose2d(5, 48, Math.toRadians(-90));
                break;

            case RED_RIGHT:
                drive = new MecanumDrive(hardwareMap, initPose);
                safeParkAdjustment = new Pose2d(5, -35, Math.toRadians(90));
                reOrientAfterPurple = new Pose2d(15, -3, Math.toRadians(0));
                wristUp = 0.6;
                switch(identifiedSpikeMarkLocation){
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(28, 7, Math.toRadians(70));
                        dropYellowPixelPose = new Pose2d(39, -38, Math.toRadians(90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(30, 1, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(32, -38,  Math.toRadians(90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(26, -6, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(25, -38, Math.toRadians(90));
                        break;
                }
                midwayPose1 = new Pose2d(15, -24, Math.toRadians(90));
                waitSecondsBeforeDrop = 0; //TODO: Adjust time to wait for alliance partner to move from board
                parkPose = new Pose2d(5, -48, Math.toRadians(90));
                break;

            case BLUE_RIGHT:
                drive = new MecanumDrive(hardwareMap, initPose);
                safeParkAdjustment = new Pose2d(55, 86, Math.toRadians(-90));
                reOrientAfterPurple = new Pose2d(20, -3, Math.toRadians(0));
                wristUp = 0.6;
                switch(identifiedSpikeMarkLocation){
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(27, 11, Math.toRadians(45));
                        dropYellowPixelPose = new Pose2d(24, 90, Math.toRadians(-90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(30, 6, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(30, 90, Math.toRadians(-90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(26, -8, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(34, 90, Math.toRadians(-90));
                        break;
                }
                midwayPose1 = new Pose2d(8, -8, Math.toRadians(0));
                midwayPose1a = new Pose2d(18, -20, Math.toRadians(-90));
                intakeStack = new Pose2d(54, -20,Math.toRadians(-90));
                midwayPose2 = new Pose2d(54, 62, Math.toRadians(-90));
                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board
                parkPose = new Pose2d(55, 98, Math.toRadians(-90));
                break;

            case RED_LEFT:
                drive = new MecanumDrive(hardwareMap, initPose);
                safeParkAdjustment = new Pose2d(55, -86, Math.toRadians(90));
                reOrientAfterPurple = new Pose2d(20, -3, Math.toRadians(-45));
                wristUp = 0.6;
                switch(identifiedSpikeMarkLocation){
                    case LEFT:
                        reOrientAfterPurple = new Pose2d(20, -3, Math.toRadians(45));
                        dropPurplePixelPose = new Pose2d(20, 4, Math.toRadians(45));
                        dropYellowPixelPose = new Pose2d(39, -90, Math.toRadians(90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(30, -3, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(30, -90, Math.toRadians(90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(27, -7, Math.toRadians(-45));
                        dropYellowPixelPose = new Pose2d(27, -91, Math.toRadians(90));
                        break;
                }
                midwayPose1 = new Pose2d(8, 8, Math.toRadians(0));
                midwayPose1a = new Pose2d(18, 18, Math.toRadians(90));
                intakeStack = new Pose2d(54, 19,Math.toRadians(90));
                midwayPose2 = new Pose2d(54, -62, Math.toRadians(90));
                waitSecondsBeforeDrop = 0; //TODO: Adjust time to wait for alliance partner to move from board
                parkPose = new Pose2d(55, -98, Math.toRadians(90));
                break;
        }

        wristServo.setPosition(wristDown); // put wrist down

        //Move robot to dropPurplePixel based on identified Spike Mark Location
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(moveBeyondTrussPose.position, moveBeyondTrussPose.heading)
                        .strafeToLinearHeading(dropPurplePixelPose.position, dropPurplePixelPose.heading)
                        .build());

        rotateArmTo(armRotateBottom);
        rightFingerServo.setPosition(rightFingerOpen);
//        safeWaitSeconds(1);

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(reOrientAfterPurple.position, reOrientAfterPurple.heading)
                        .build());

        wristServo.setPosition(wristUp);

        //Move robot to midwayPose1
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .build());

        //For Blue Right and Red Left, intake pixel from stack
        if (startPosition == START_POSITION.BLUE_RIGHT ||
                startPosition == START_POSITION.RED_LEFT) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose1a.position, midwayPose1a.heading)
                            .strafeToLinearHeading(intakeStack.position, intakeStack.heading)
                            .build());

            //TODO : Code to intake pixel from stack
            safeWaitSeconds(1);

            //Move robot to midwayPose2 and to dropYellowPixelPose
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose2.position, midwayPose2.heading)
                            .build());
//            armExtensionPos = 500;
        }

        safeWaitSeconds(waitSecondsBeforeDrop);
        rotateArmTo(armRotateTop);
        extendArmTo(armExtensionPos);
        safeWaitSeconds(1);


        //Move robot to midwayPose2 and to dropYellowPixelPose
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(dropYellowPixelPose.position, dropYellowPixelPose.heading, new TranslationalVelConstraint(10))
                        .build());


        //TODO : Code to drop Pixel on Backdrop
        safeWaitSeconds(1.5);
        leftFingerServo.setPosition(leftFingerOpen);
        //Todo:  finish this, this is just test.
        safeWaitSeconds(0.2);
        extendArmTo(0);

//        Pose2d safeLowerArm = new Pose2d(28, 32,  Math.toRadians(-90));

        rotateArmTo(armRotateBottom);


        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(safeParkAdjustment.position, safeParkAdjustment.heading)
                        .build());

        //Move robot to park in Backstage
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(parkPose.position, parkPose.heading)
                        .build());
        telemetry.addData("endPose", drive.pose);
        telemetry.update();
    }



    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while(!isStopRequested()){
            telemetry.addData("Initializing FTC Wires (ftcwires.org) Autonomous adopted for Team:",
                    TEAM_NAME, " ", TEAM_NUMBER);
            telemetry.addData("---------------------------------------","");
            telemetry.addLine("This Auto program uses Open CV Vision Processor for Team Element detection");
            telemetry.addData("Select Starting Position using XYAB on Logitech (or ▢ΔOX on Playstayion) on gamepad 1:","");
            telemetry.addData("    Blue Left   ", "(X / ▢)");
            telemetry.addData("    Blue Right ", "(Y / Δ)");
            telemetry.addData("    Red Left    ", "(B / O)");
            telemetry.addData("    Red Right  ", "(A / X)");
            if(gamepad1.x){
                startPosition = START_POSITION.BLUE_LEFT;
                break;
            }
            if(gamepad1.y){
                startPosition = START_POSITION.BLUE_RIGHT;
                break;
            }
            if(gamepad1.b){
                startPosition = START_POSITION.RED_LEFT;
                break;
            }
            if(gamepad1.a){
                startPosition = START_POSITION.RED_RIGHT;
                break;
            }
            telemetry.update();
        }
        telemetry.clearAll();
    }

    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }

    /**
     * Initialize the Open CV Object Detection processor.
     */
    public Rect rectLeftOfCameraMid, rectRightOfCameraMid;
    private void initOpenCV() {

        if (startPosition == START_POSITION.RED_LEFT ||
                startPosition == START_POSITION.BLUE_LEFT) {
            visionOpenCV = new VisionOpenCV(hardwareMap, "Webcam 1");
            rectLeftOfCameraMid = new Rect(120, 90, 150, 240); // left skyscraper rect x + 190 y + 50
            rectRightOfCameraMid = new Rect(350, 40, 280, 160); // center long rect
        } else { //RED_RIGHT or BLUE_RIGHT
            visionOpenCV = new VisionOpenCV(hardwareMap, "Webcam 2");
            rectLeftOfCameraMid = new Rect(10, 170, 240, 160); // center long rect //fix x and y size on both
            rectRightOfCameraMid = new Rect(260, 170, 150, 240); // right skyscraper rect
        }
    }

    /**
     * Add telemetry about Object Detection recognitions.
     */
    private void runOpenCVObjectDetection() {
        visionOpenCV.getSelection();
        telemetry.addLine("Open CV based Vision Processor for Team Element Detection");
        telemetry.addData("Identified Parking Location", identifiedSpikeMarkLocation);
        telemetry.addData("SatLeftOfCameraMid", visionOpenCV.satRectLeftOfCameraMid);

        telemetry.addData("SatRightOfCameraMid", visionOpenCV.satRectRightOfCameraMid);
        telemetry.addData("SatRectNone", visionOpenCV.satRectNone);
        telemetry.update();
    }

    public class VisionOpenCV implements VisionProcessor {

        CameraSelectedAroundMid selectionAroundMid = CameraSelectedAroundMid.NONE;

        public VisionPortal visionPortal;

        Mat submat = new Mat();
        Mat hsvMat = new Mat();

        public double satRectLeftOfCameraMid, satRectRightOfCameraMid;
        public double satRectNone = 35.0; //TODO adjust for field conditions

        public VisionOpenCV(HardwareMap hardwareMap, String webcam){
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, webcam), this);
        }

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

            satRectLeftOfCameraMid = getAvgSaturation(hsvMat, rectLeftOfCameraMid);
            satRectRightOfCameraMid = getAvgSaturation(hsvMat, rectRightOfCameraMid);

            if ((satRectLeftOfCameraMid > satRectRightOfCameraMid) && (satRectLeftOfCameraMid > satRectNone)) {
                return CameraSelectedAroundMid.LEFT_OF_CAMERA_MID;
            } else if ((satRectRightOfCameraMid > satRectLeftOfCameraMid) && (satRectRightOfCameraMid > satRectNone)) {
                return CameraSelectedAroundMid.RIGHT_OF_CAMERA_MID;
            }
            return CameraSelectedAroundMid.NONE;
        }

        protected double getAvgSaturation(Mat input, Rect rect) {
            submat = input.submat(rect);
            Scalar color = Core.mean(submat);
            return color.val[1];
        }

        private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
            int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
            int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
            int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
            int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

            return new android.graphics.Rect(left, top, right, bottom);
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
            Paint selectedPaint = new Paint();
            selectedPaint.setColor(Color.RED);
            selectedPaint.setStyle(Paint.Style.STROKE);
            selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

            Paint nonSelectedPaint = new Paint(selectedPaint);
            nonSelectedPaint.setColor(Color.GREEN);

            android.graphics.Rect drawRectangleLeft = makeGraphicsRect(rectLeftOfCameraMid, scaleBmpPxToCanvasPx);
            android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(rectRightOfCameraMid, scaleBmpPxToCanvasPx);

            selectionAroundMid = (CameraSelectedAroundMid) userContext;
            switch (selectionAroundMid) {
                case LEFT_OF_CAMERA_MID:
                    canvas.drawRect(drawRectangleLeft, selectedPaint);
                    canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                    break;
                case RIGHT_OF_CAMERA_MID:
                    canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                    canvas.drawRect(drawRectangleMiddle, selectedPaint);
                    break;
                case NONE:
                    canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                    canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                    break;
            }
        }

        public void getSelection() {
            if (startPosition == START_POSITION.RED_LEFT ||
                    startPosition == START_POSITION.BLUE_LEFT) {
                switch (selectionAroundMid) {
                    case NONE:
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT;
                        break;
                    case LEFT_OF_CAMERA_MID:
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
                        break;
                    case RIGHT_OF_CAMERA_MID:
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE;
                        break;
                }
            } else { //RED_RIGHT or BLUE_RIGHT
                switch (selectionAroundMid) {
                    case NONE:
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
                        break;
                    case LEFT_OF_CAMERA_MID:
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE;
                        break;
                    case RIGHT_OF_CAMERA_MID:
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT;
                        break;
                }
            }
        }
    }

    public enum CameraSelectedAroundMid {
        NONE,
        LEFT_OF_CAMERA_MID,
        RIGHT_OF_CAMERA_MID
    }
}   // end class

//TODO funny blue line on the side :D