package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Timer;
import java.util.TimerTask;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

/**
 * FTC WIRES TeleOp Example
 *
 */



@TeleOp(name = "FTC Wires TeleOp", group = "00-Teleop")
public class FTCWiresTeleOpMode extends LinearOpMode {
    DcMotor leftExtend;
    DcMotor rightExtend;
    DcMotor leftRotate;
    DcMotor rightRotate;

    // these are coefficients for tuning the PID controllers for the arm extention and rotation
    double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;

    int upperExtendLimit = 7800;
    int upperRotateLimit = 1850;
    int lowerMotorLimit = -30;
    int upperHangExtend = 4600;
    int lowerHangExtend = 1700;
    int rotateToForClimb = 1370;
    boolean isHangingTime = false;
    boolean canMoveWrist = false;
    int rotateBottom = 0;
    int extendBottom = 0;

    //Method to set arms to extend to certain spot

    public void extendArmToPos(int extensionPos) {
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

    public void rotateArmToPos(int rotationPos) {
//        if (leftRotate.getCurrentPosition() > lowerMotorLimit && leftRotate.getCurrentPosition() < upperRotateLimit) {
            leftRotate.setTargetPosition(rotationPos);
            rightRotate.setTargetPosition(rotationPos);
            leftRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRotate.setPower(0.5);
            rightRotate.setPower(0.5);
//        }
    }
    //Method to rotate Arms

    public void rotateArm(double rotationAmount) {
        if (leftRotate.getCurrentPosition() > lowerMotorLimit && leftRotate.getCurrentPosition() < upperRotateLimit && !isHangingTime) {
            leftRotate.setPower(rotationAmount);
            rightRotate.setPower(rotationAmount);
        } else if (leftRotate.getCurrentPosition() < lowerMotorLimit) {
            leftRotate.setPower(0.5);
            rightRotate.setPower(0.5);
        } else if(leftRotate.getCurrentPosition() > upperRotateLimit){
            leftRotate.setPower(-0.5);
            rightRotate.setPower(-0.5);
        }
    }
//        leftRotate.setPower(rotationAmount);
//        rightRotate.setPower(rotationAmount);
//        if ((leftRotate.getCurrentPosition() >= -10) && (leftRotate.getCurrentPosition() < 1800)) {
//            leftRotate.setPower(rotationAmount);
//            rightRotate.setPower(rotationAmount);
//        } else {
//            leftRotate.setPower(-rotationAmount - 0.5);
//            rightRotate.setPower(-rotationAmount - 0.5);
//        }

//        }
//            leftRotate.setDirection(DcMotorSimple.Direction.FORWARD);
//            rightRotate.setDirection(DcMotorSimple.Direction.REVERSE);
//        } else {
//            leftRotate.setDirection(DcMotorSimple.Direction.REVERSE);
//            rightRotate.setDirection(DcMotorSimple.Direction.FORWARD);
//        }
//        leftRotate.setPower(rotationAmount);
//        rightRotate.setPower(rotationAmount);

    //Method to Extend Arms

    public void extendArm(double extensionAmount) {
        if (leftExtend.getCurrentPosition() > lowerMotorLimit && leftExtend.getCurrentPosition() < upperExtendLimit && !isHangingTime) {
            leftExtend.setPower(extensionAmount);
            rightExtend.setPower(extensionAmount);
        } else if (leftExtend.getCurrentPosition() < lowerMotorLimit) {
            leftExtend.setPower(0.25);
            rightExtend.setPower(0.25);
        } else if (leftExtend.getCurrentPosition() > upperExtendLimit){
            leftExtend.setPower(-0.25);
            rightExtend.setPower(-0.25);
        }
    }
//        if ((leftExtend.getCurrentPosition() >= -10) && (leftExtend.getCurrentPosition() < 7800)) {
//
//            leftExtend.setPower(extensionAmount);
//            rightExtend.setPower(extensionAmount);
//        } else {
//            leftExtend.setPower(-extensionAmount);
//            rightExtend.setPower(-extensionAmount);
//        }

//        }
//            leftExtend.setDirection(DcMotorSimple.Direction.FORWARD);
//            rightExtend.setDirection(DcMotorSimple.Direction.REVERSE);
//        } else {
//            leftExtend.setDirection(DcMotorSimple.Direction.REVERSE);
//            rightExtend.setDirection(DcMotorSimple.Direction.FORWARD);
//        }
//        leftExtend.setPower(extensionAmount);
//        rightExtend.setPower(extensionAmount);

//    DcMotor leftFrontDrive = hardwareMap.dcMotor.get("FLdrive");
//    DcMotor leftBackDrive = hardwareMap.dcMotor.get("BLdrive");
//    DcMotor rightFrontDrive = hardwareMap.dcMotor.get("FRdrive");
//    DcMotor rightBackDrive = hardwareMap.dcMotor.get("BRdrive");

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


        Servo wristServo = hardwareMap.servo.get("wrist");
        Servo leftFingerServo = hardwareMap.servo.get("leftFinger");
        Servo rightFingerServo = hardwareMap.servo.get("rightFinger");

        Servo droneServo = hardwareMap.servo.get("droneRelease");
        droneServo.setPosition(0);

//        double SLOW_DOWN_FACTOR = 0.5; //TODO Adjust to driver comfort // unnecessary for us
        telemetry.addData("Initializing FTC Wires (ftcwires.org) TeleOp adopted for Team:","TEAM NUMBER");
        telemetry.update();

        boolean leftGrab = false;
        boolean rightGrab = false;

        final boolean[] leftCanGrab = {true};
        final boolean[] rightCanGrab = {true};

        //TODO: testing variables remove
//        int extendTo = 0;
//        int rotateTo = 0;


        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            leftExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // all this needs to be replaced with a custom pid tuned motion method
            leftRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//            11192 extend limit
//            2035 rotate limit

            waitForStart();

            while (opModeIsActive()) {

                Timer timer = new Timer();
                long grabToggleDur = 500L;
                boolean plane = false;
                double slowMode = gamepad1.left_trigger;
                double slowCoeff = 0.3;

                rotateArm(-gamepad2.left_stick_y);
                extendArm(-gamepad2.right_stick_y);


                telemetry.addData("Running FTC Wires (ftcwires.org) TeleOp Mode adopted for Team:","TEAM NUMBER");

                if (slowMode > 0.2) { // check if we are trying to enter "slowmode" with the left trigger on driver gamepad
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    -gamepad1.left_stick_y * slowCoeff,
                                    -gamepad1.left_stick_x * slowCoeff
                            ),
                            -gamepad1.right_stick_x * slowCoeff
                    ));
                } else {
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x
                            ),
                            -gamepad1.right_stick_x
                    ));
                }

                //todo maybe add driver control back or maybe not
                if (gamepad2.left_trigger > 0.2 && canMoveWrist) { // rotate CCW
                    wristServo.setPosition(wristServo.getPosition() + 0.02);
                } else  if (gamepad2.right_trigger > 0.2 && canMoveWrist) { // rotate CW
                    wristServo.setPosition(wristServo.getPosition() - 0.02);
                }

                if (gamepad2.right_bumper && leftCanGrab[0]) { // swapped as per Max's suggestion
                    leftGrab = !leftGrab;
                    leftCanGrab[0] = false;
                    TimerTask toggleLeftGrab = new TimerTask() {
                        @Override
                        public void run() {
                            leftCanGrab[0] = true ;
                        }
                    };
                    timer.schedule(toggleLeftGrab,grabToggleDur);
                }

                if (gamepad2.left_bumper && rightCanGrab[0]) {
                    rightGrab = !rightGrab;
                    rightCanGrab[0] = false;
                    TimerTask toggleRightGrab = new TimerTask() {
                        @Override
                        public void run() {
                            rightCanGrab[0] = true ;
                        }
                    };
                    timer.schedule(toggleRightGrab,grabToggleDur);
                }

                if(gamepad2.dpad_up && gamepad2.start)
                {
                    isHangingTime = true;
                    rotateArmToPos(rotateToForClimb);
                    extendArmToPos(upperHangExtend);
                }
                if(gamepad2.dpad_down && gamepad2.start){
                    extendArmToPos(lowerHangExtend);
                }

                if(gamepad2.y){
                    leftExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    //this probably isn't good, extend and rotate should be two separate things, and drivers need to make sure the things are in the zero pos or it will mess up the limiting
                }

                if(leftRotate.getCurrentPosition() > 900){
                    wristServo.setPosition(0.6);
                }
                if(leftRotate.getCurrentPosition() < 100){
                    canMoveWrist = true;
                } else {
                    canMoveWrist = false;
                }

                if (leftGrab) {
                    leftFingerServo.setPosition(1);
                } else {
                    leftFingerServo.setPosition(0);
                }

                if (rightGrab) {
                    rightFingerServo.setPosition(0);
                } else {
                    rightFingerServo.setPosition(1);
                }

                if (gamepad1.y) { // plane lanuch code, this should work like the first time we programmed the claw fingers so the bumpers must be held to fully launch the drone
                    plane = true;
                }
                if(plane){
                    droneServo.setPosition(1);
                } else{
                    droneServo.setPosition(0);
                }


                telemetry.addData("extend encoder ", leftExtend.getCurrentPosition());
                telemetry.addData("rotate encoder ", leftRotate.getCurrentPosition());

                drive.updatePoseEstimate();

                telemetry.addLine("Current Pose");
                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading", Math.toDegrees(drive.pose.heading.log()));
                telemetry.update();
            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) { // This should NEVER run on our robot, so I commented it all out
//            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));
//
//            waitForStart();
//
//            while (opModeIsActive()) {
//                drive.setDrivePowers(new PoseVelocity2d(
//                        new Vector2d(
//                                -gamepad1.left_stick_y * SLOW_DOWN_FACTOR,
//                                0.0
//                        ),
//                        -gamepad1.right_stick_x * SLOW_DOWN_FACTOR
//                ));
//
//                drive.updatePoseEstimate();
//
//                telemetry.addData("x", drive.pose.position.x);
//                telemetry.addData("y", drive.pose.position.y);
//                telemetry.addData("heading", drive.pose.heading);
//                telemetry.update();
//            }
        } else {
            throw new AssertionError();
        }
    }
}