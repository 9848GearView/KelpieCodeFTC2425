/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.AutoServoConstants;
import org.firstinspires.ftc.teamcode.rr.MecanumDrive;

import java.util.Timer;
import java.util.TimerTask;


/*
 * This sample demonstrates a basic (but battle-tested and essentially
 * 100% accurate) method of detecting the TeamElement when lined up with
 * the sample regions over the first 3 stones.
 */
@Autonomous(name = "Last Half Left")
@Disabled
public class SirenAutoLastHalfLeft extends LinearOpMode
{
    enum DriveDirection {
        FORWARD,
        LEFT,
        RIGHT,
        BACKWARD
    }

    enum StartingPositionEnum {
        LEFT,
        RIGHT
    }

    enum SlidePackDirection {
        UP,
        DOWN
    }

    private ElapsedTime runtime = new ElapsedTime();
    private Timer timer = new Timer();
    private DcMotor FLMotor = null;
    private DcMotor FRMotor = null;
    private DcMotor BLMotor = null;
    private DcMotor BRMotor = null;
    private DcMotor LeftSlide = null;
    private DcMotor RightSlide = null;
    private CRServo IntakeServo = null;
    private Servo WristServo = null;
    private Servo LeftElbowServo = null;
    private Servo RightElbowServo = null;

    private ElapsedTime eTime = new ElapsedTime();

    private int index = 0;
    private int wristIndex;
    private double[] LEServoPositions = AutoServoConstants.LEServoPositions;
    private double[] REServoPositions = AutoServoConstants.REServoPositions;
    private double[] IServoPositions = AutoServoConstants.IServoPositions;
    private double[] WServoPositions = AutoServoConstants.WServoPositions;
    private int[] LSMotorPositions = AutoServoConstants.LSMotorPositions;
    private int[] RSMotorPositions = AutoServoConstants.RSMotorPositions;

    private final int DELAY_BETWEEN_MOVES = 100;

    class LowerArmToCertainServoPosition extends TimerTask {
        int i;
        public LowerArmToCertainServoPosition(int i) {
            this.i = i;
        }
        public void run() {
            LeftElbowServo.setPosition(LEServoPositions[i]);
            RightElbowServo.setPosition(REServoPositions[i]);

            telemetry.addData("index", i);
            telemetry.update();
//                sleep(1000);
            index = i;
        }
    }
    class MoveWristServoPosition extends TimerTask {
        int i;

        public MoveWristServoPosition(int i) {
            this.i = i;
        }

        public void run() {
            WristServo.setPosition(WServoPositions[i]);

            telemetry.addData("Wrist Index", i);
            telemetry.update();
            //   sleep(1000);
            wristIndex = i;
        }
    }

    class IntakeState extends TimerTask {
        int i;

        public IntakeState(int i) {
            this.i = i;
        }
        public void run() {
            IntakeServo.setPower(IServoPositions[i]);
        }
    }

    class SlidePosition extends TimerTask {
        int i;
        double power;

        public SlidePosition(int i, double power) {
            this.i = i;
            this.power = power;
        }
        public void run() {
            LeftSlide.setTargetPosition(LSMotorPositions[i]);
            LeftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LeftSlide.setPower(power);

            RightSlide.setTargetPosition(RSMotorPositions[i]);
            RightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightSlide.setPower(power);
        }
    }


    StartingPositionEnum sideOfFieldToStartOn = StartingPositionEnum.LEFT;

    @Override
    public void runOpMode()
    {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */


        telemetry.addData("Status", "sInitialized");
        telemetry.update();


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        FLMotor = hardwareMap.get(DcMotor.class, "FL");
        FRMotor = hardwareMap.get(DcMotor.class, "FR");
        BLMotor = hardwareMap.get(DcMotor.class, "BL");
        BRMotor = hardwareMap.get(DcMotor.class, "BR");
        LeftSlide = hardwareMap.get(DcMotor.class, "LS");
        RightSlide = hardwareMap.get(DcMotor.class, "RS");
        LeftElbowServo = hardwareMap.get(Servo.class, "LE");
        RightElbowServo = hardwareMap.get(Servo.class, "RE");
        WristServo = hardwareMap.get(Servo.class, "WS");
        IntakeServo = hardwareMap.get(CRServo.class, "IN");

        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        FLMotor.setDirection(DcMotor.Direction.REVERSE);
        FRMotor.setDirection(DcMotor.Direction.FORWARD);
        BLMotor.setDirection(DcMotor.Direction.REVERSE);
        BRMotor.setDirection(DcMotor.Direction.FORWARD);
        LeftSlide.setDirection(DcMotor.Direction.FORWARD);
        RightSlide.setDirection(DcMotor.Direction.REVERSE);
        LeftElbowServo.setDirection(Servo.Direction.FORWARD);
        RightElbowServo.setDirection(Servo.Direction.REVERSE);
        WristServo.setDirection(Servo.Direction.FORWARD);
        IntakeServo.setDirection(CRServo.Direction.FORWARD);

        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LeftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftSlide.setTargetPosition(LSMotorPositions[0]);
        LeftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightSlide.setTargetPosition(RSMotorPositions[0]);
        RightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LeftElbowServo.setPosition(LEServoPositions[4]);
        RightElbowServo.setPosition(REServoPositions[4]);
        WristServo.setPosition(WServoPositions[2]);
        // Wait for the game to start (driver presses PLAY)



        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
//        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-36, -67, Math.PI / 2));
//        timer.schedule(new PutGrabberToCertainPosition(0), 3000);


        waitForStart();
        while (opModeIsActive())
        {

            doActions(drive, sideOfFieldToStartOn);

            // Don't burn CPU cycles busy-looping in this sample
            sleep(15000);
            break;
        }

    }

    public class PlaceSampleIntoBucket implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            timer.schedule(new MoveWristServoPosition(0), 0);
            timer.schedule(new LowerArmToCertainServoPosition(3), 0 * DELAY_BETWEEN_MOVES);
            return false;
        }
    }

    public class IntakeSample implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            timer.schedule(new LowerArmToCertainServoPosition(1),  0 * DELAY_BETWEEN_MOVES);
            timer.schedule(new MoveWristServoPosition(0), 0 * DELAY_BETWEEN_MOVES);


            return false;
        }
    }

    public class Intake implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            IntakeServo.setPower(-0.8);
            timer.schedule(new IntakeState(1), 10 * DELAY_BETWEEN_MOVES);
            return false;
        }
    }
    public class Outtake implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            IntakeServo.setPower(0.5);
            timer.schedule(new IntakeState(1), 10 * DELAY_BETWEEN_MOVES);
            return false;
        }
    }

    public class Neutral implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            timer.schedule(new LowerArmToCertainServoPosition(4),  0 * DELAY_BETWEEN_MOVES);
            timer.schedule(new MoveWristServoPosition(0), 0 * DELAY_BETWEEN_MOVES);


            return false;
        }
    }

    public class Move implements Action {
        int pos;
        int posW;
        int delay;
        int delayW;
        public Move(int pos, int posW, int delay, int delayW) { this.pos = pos; this.posW = posW; this.delay = delay; this.delayW = delayW;}
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            timer.schedule(new LowerArmToCertainServoPosition(pos),  delay * DELAY_BETWEEN_MOVES);
            timer.schedule(new MoveWristServoPosition(posW), delayW * DELAY_BETWEEN_MOVES);

            return false;
        }
    }
    public class moveArm implements Action {
        int pos;
        int delay;
        public moveArm(int pos, int delay){ this.pos = pos; this.delay = delay; }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            timer.schedule(new SlidePosition(pos, 1),  delay * DELAY_BETWEEN_MOVES);

            return false;
        }
    }


    private void doActions(MecanumDrive drive, StartingPositionEnum position) {
//        smp = SpikeMarkPosition.TRES;
        boolean needInvert = (position != StartingPositionEnum.RIGHT);
        double multiplier = 1;
        if (needInvert) {
            multiplier = -1;
        }
        timer.schedule(new LowerArmToCertainServoPosition(4),  2 * DELAY_BETWEEN_MOVES);
        timer.schedule(new MoveWristServoPosition(0), 0 * DELAY_BETWEEN_MOVES);


        TrajectoryActionBuilder actionBuilder = drive.actionBuilder(drive.pose); //actually a genius

                /*.lineToY(multiplier * -33);
        actionBuilder = actionBuilder
                .lineToX(36)
                .strafeToConstantHeading(new Vector2d(44, multiplier * 1))
                .waitSeconds(5)
                .strafeToConstantHeading(new Vector2d(46, multiplier * 1))
                .turn(multiplier * 0.00001)
                .lineToX(60);*/
//                Actions.runBlocking(new ParallelAction(drive.actionBuilder(drive.pose).strafeToConstantHeading(new Vector2d(-40, -33)).build(), drive.actionBuilder(drive.pose).turn(13*(Math.PI/16)).build())); god dayum this is ugly
//                Actions.runBlocking(new ParallelAction(actionBuilder.build(), actionBuilder.build())); //figured it out :)



        //first sample
        Actions.runBlocking(new ParallelAction(actionBuilder.afterTime(1, new IntakeSample()).build(), actionBuilder.strafeToLinearHeading(new Vector2d(-28, -22), (Math.PI)).build()));
        Actions.runBlocking(actionBuilder.waitSeconds(0.5).build());
        actionBuilder = drive.actionBuilder(drive.pose);
        Actions.runBlocking(new ParallelAction(actionBuilder.strafeToConstantHeading(new Vector2d(-44, -22)).build(), actionBuilder.afterTime(0, new Intake()).build()));
        actionBuilder = drive.actionBuilder(drive.pose);
        Actions.runBlocking(new ParallelAction(actionBuilder.strafeToLinearHeading(new Vector2d(-50, -52), 21*(Math.PI/16)).build(), actionBuilder.afterTime(0, new moveArm(1, 0)).build(), actionBuilder.afterTime(0, new PlaceSampleIntoBucket()).build()));
        actionBuilder = drive.actionBuilder(drive.pose);

        Actions.runBlocking(actionBuilder.strafeToConstantHeading(new Vector2d(-57.5, -64.5)).build());
        Actions.runBlocking(actionBuilder.afterTime(0, new Outtake()).build());
        Actions.runBlocking(actionBuilder.waitSeconds(0.7).build());
        actionBuilder = drive.actionBuilder(drive.pose);

        //second sample
        Actions.runBlocking(new ParallelAction(actionBuilder.strafeToLinearHeading(new Vector2d(-40, -22), (Math.PI)).build(), actionBuilder.afterTime(1, new moveArm(0, 1)).build(), actionBuilder.afterTime(1, new IntakeSample()).build()));
        Actions.runBlocking(actionBuilder.waitSeconds(0.5).build());
        actionBuilder = drive.actionBuilder(drive.pose);
        Actions.runBlocking(new ParallelAction(actionBuilder.strafeToConstantHeading(new Vector2d(-49, -22)).build(), actionBuilder.afterTime(0, new Intake()).build()));
        actionBuilder = drive.actionBuilder(drive.pose);
        Actions.runBlocking(new ParallelAction(actionBuilder.strafeToLinearHeading(new Vector2d(-50, -52), 22*(Math.PI/16)).build(), actionBuilder.afterTime(0, new moveArm(1, 0)).build(), actionBuilder.afterTime(0, new PlaceSampleIntoBucket()).build()));
        actionBuilder = drive.actionBuilder(drive.pose);

        Actions.runBlocking(actionBuilder.strafeToConstantHeading(new Vector2d(-58, -64.5)).build());
        Actions.runBlocking(actionBuilder.afterTime(0, new Outtake()).build());
        Actions.runBlocking(actionBuilder.waitSeconds(0.7).build());
        actionBuilder = drive.actionBuilder(drive.pose);

        //third sample
        Actions.runBlocking(new ParallelAction(actionBuilder.strafeToLinearHeading(new Vector2d(-52, -23), (Math.PI)).build(), actionBuilder.afterTime(1, new moveArm(0, 1)).build(), actionBuilder.afterTime(1, new IntakeSample()).build()));
        Actions.runBlocking(actionBuilder.waitSeconds(0.5).build());
        actionBuilder = drive.actionBuilder(drive.pose);
        Actions.runBlocking(new ParallelAction(actionBuilder.strafeToConstantHeading(new Vector2d(-58, -25)).build(), actionBuilder.afterTime(0, new Intake()).build()));
        actionBuilder = drive.actionBuilder(drive.pose);
        Actions.runBlocking(new ParallelAction(actionBuilder.strafeToLinearHeading(new Vector2d(-50, -52), 22*(Math.PI/16)).build(), actionBuilder.afterTime(0, new moveArm(1, 0)).build(), actionBuilder.afterTime(0, new PlaceSampleIntoBucket()).build()));
        actionBuilder = drive.actionBuilder(drive.pose);

        Actions.runBlocking(actionBuilder.strafeToConstantHeading(new Vector2d(-53.5, -67.5)).build());
        Actions.runBlocking(actionBuilder.afterTime(0, new Outtake()).build());
        Actions.runBlocking(actionBuilder.waitSeconds(0.7).build());
        actionBuilder = drive.actionBuilder(drive.pose);
        Actions.runBlocking(new ParallelAction(actionBuilder.afterTime(0, new Move(0, 0, 0, 0)).build(), actionBuilder.afterTime(0, new moveArm(0, 0)).build(), actionBuilder.afterTime(0.5, new IntakeSample()).build(), actionBuilder.strafeToConstantHeading(new Vector2d(-50, -52)).build()));
        sleep(4000);
    }

    private DriveDirection getCorrectDirection(DriveDirection direction, boolean needInvert) {
        if (!needInvert)
            return direction;

        DriveDirection invertedDirection = direction;
        switch (direction) {
            case LEFT:
                invertedDirection = DriveDirection.RIGHT;
                break;
            case RIGHT:
                invertedDirection = DriveDirection.LEFT;
                break;
            case FORWARD:
                invertedDirection = DriveDirection.BACKWARD;
                break;
            case BACKWARD:
                invertedDirection = DriveDirection.FORWARD;
                break;
            default:
                break;
        }

        return invertedDirection;
    }
}