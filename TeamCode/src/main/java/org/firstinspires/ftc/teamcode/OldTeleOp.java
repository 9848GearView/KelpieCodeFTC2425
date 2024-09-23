/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.Math;

import java.util.TimerTask;
import java.util.Timer;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Y'know that one spongebob song thats the guitar and its sad and like, womp womp, womp womp womp, womp womp, womp WOMP^")

public class OldTeleOp extends LinearOpMode {

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
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
    private boolean oldCrossPressed = true;
    private boolean oldTrianglePressed = true;
    private boolean oldCirclePressed = true;
    private boolean oldSquarePressed = true;
    private boolean oldLBumper = true;
    private boolean oldRBumper = true;
    private boolean isArmMoving = false;

    private int index = 0;
    private int armIndex = 0;

    final double IntakeCollect    = -1.0;
    final double IntakeOff        =  0.0;
    final double IntakeDeposit    =  0.5;

    private final int DELAY_BETWEEN_MOVES = 100;

    final double[] IntakeServoPower = {-1.0, 0.5, 0.0};
    final double[] LEServoPositions = {0.0, 0.5, 0.8, 1.0};
    final double[] REServoPositions = {0.0, 0.5, 0.8, 1.0};
    final double[] WEServoPositions = {0.0, 0.5, 0.8, 1.0};


    @Override
    public void runOpMode() {

        class setIsArmMoving extends TimerTask {
            boolean val;
            public setIsArmMoving(boolean v) {
                this.val = v;
            }
            public void run() {
                isArmMoving = val;
            }
        }

        class LowerArmToCertainServoPosition extends TimerTask {
            int i;
            public LowerArmToCertainServoPosition(int i) {
                this.i = i;
            }
            public void run() {
                LeftElbowServo.setPosition(LEServoPositions[i]);
                RightElbowServo.setPosition(REServoPositions[i]);
                // WristServo.setPosition(WServoPositions[i]); don't use just yet

                telemetry.addData("index", i);
                telemetry.update();
                //                 sleep(1000);
            }
        }

        telemetry.addData("Status", "Initialized");
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
        Timer timer = new Timer();

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
        LeftSlide.setDirection(DcMotor.Direction.REVERSE);
        RightSlide.setDirection(DcMotor.Direction.FORWARD);
        LeftElbowServo.setDirection(Servo.Direction.FORWARD);
        RightElbowServo.setDirection(Servo.Direction.REVERSE);
        WristServo.setDirection(Servo.Direction.REVERSE);
        IntakeServo.setDirection(CRServo.Direction.FORWARD);

        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        int armIndex = 0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double FLPower = r * Math.cos(robotAngle) + rightX;
            final double FRPower = r * Math.sin(robotAngle) - rightX;
            final double BLPower = r * Math.sin(robotAngle) + rightX;
            final double BRPower = r * Math.cos(robotAngle) - rightX;

            // Send calculated power to wheels
            if (gamepad1.right_bumper || gamepad1.left_bumper) {
                //   if (gamepad1.right_stick_x == 0 && gamepad1.right_stick_y == 0) {
                //       if (gamepad1.right_bumper) {
                //           FLMotor.setPower(1);
                //           FRMotor.setPower(-1);
                //           BLMotor.setPower(-1);
                //           BRMotor.setPower(1);
                //       } else {
                //           FLMotor.setPower(-1);
                //           FRMotor.setPower(1);
                //           BLMotor.setPower(1);
                //           BRMotor.setPower(-1);
                //       }
                //   } else {
                if (-gamepad1.right_stick_y > 0) {
                    if (gamepad1.right_bumper) {
                        FLMotor.setPower(1);
                        FRMotor.setPower(-1 + Math.abs(gamepad1.right_stick_y));
                        BLMotor.setPower(-1 + Math.abs(gamepad1.right_stick_y));
                        BRMotor.setPower(1);
                    } else {
                        FLMotor.setPower(-1 + Math.abs(gamepad1.right_stick_y));
                        FRMotor.setPower(1);
                        BLMotor.setPower(1);
                        BRMotor.setPower(-1 + Math.abs(gamepad1.right_stick_y));
                    }
                } else {
                    if (gamepad1.right_bumper) {
                        FLMotor.setPower(1 - Math.abs(gamepad1.right_stick_y));
                        FRMotor.setPower(-1);
                        BLMotor.setPower(-1);
                        BRMotor.setPower(1 - Math.abs(gamepad1.right_stick_y));
                    } else {
                        FLMotor.setPower(-1);
                        FRMotor.setPower(1 - Math.abs(gamepad1.right_stick_y));
                        BLMotor.setPower(1 - Math.abs(gamepad1.right_stick_y));
                        BRMotor.setPower(-1);
                    }
                }
                //   }
            } else {
                FLMotor.setPower(FLPower);
                FRMotor.setPower(FRPower);
                BLMotor.setPower(BLPower);
                BRMotor.setPower(BRPower);
            }

            LeftSlide.setPower(gamepad2.left_stick_y);
            RightSlide.setPower(gamepad2.left_stick_y);

            boolean circlePressed = gamepad2.circle;
            boolean trianglePressed = gamepad2.triangle;
            boolean squarePressed = gamepad2.square;
            boolean crossPressed = gamepad2.cross;
            if (index == 0) {
                if (circlePressed && !oldCirclePressed && !isArmMoving) {
                    new setIsArmMoving(true).run();
                    timer.schedule(new LowerArmToCertainServoPosition(0), 8 * DELAY_BETWEEN_MOVES);

                    index = 2;
                }
            } else if (index == 2) {
                if (circlePressed && !oldCirclePressed && !isArmMoving) {
                    new setIsArmMoving(true).run();
                    timer.schedule(new LowerArmToCertainServoPosition(1), 8 * DELAY_BETWEEN_MOVES);
                    index = 0;
                } else if (trianglePressed && !oldTrianglePressed && !isArmMoving) {
                    new setIsArmMoving(true).run();
                    timer.schedule(new LowerArmToCertainServoPosition(2), 8 * DELAY_BETWEEN_MOVES);
                    index = 3;
                }
            } else if (index == 3) {
                if (circlePressed && !oldCirclePressed && !isArmMoving) {
                    new setIsArmMoving(true).run();
                    timer.schedule(new LowerArmToCertainServoPosition(3), 8 * DELAY_BETWEEN_MOVES);
                    index = 0;
                }
            }


            boolean dPadUpPressed = gamepad2.dpad_up;
            boolean dPadDownPressed = gamepad2.dpad_down;

            IntakeServo.setPower(gamepad2.dpad_up ? -1.0 : gamepad2.dpad_up ? 0.5 : 0);
            WristServo.setPosition(gamepad2.dpad_left ? 1.0 : gamepad2.dpad_right ? 0 : 0.5);



            // Show the elapsed game time and wheel power.
            //   telemetry.addData("Status", "Run Time: " + runtime.toString());
            //   telemetry.addData("INDEX", index % LEServoPositions.length);
//            telemetry.addData("", LEServoPositions[index]);
//            telemetry.addData("", REServoPositions[index]);
//            telemetry.addData("", RWServoPositions[index]);
            telemetry.update();
            oldCrossPressed = crossPressed;
            oldCirclePressed = circlePressed;
            oldSquarePressed = squarePressed;
            oldTrianglePressed = trianglePressed;

        }
    }
}
