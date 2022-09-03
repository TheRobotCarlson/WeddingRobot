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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This particular OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 * <p>
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Robot: Teleop POV", group = "Robot")
//@Disabled
public class RobotTeleopPOV_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor arm = null;
    public DcMotor fan = null;
    public Servo ringOpener = null;

    public Servo robotLR = null;
    public Servo robotTurn1 = null;
    public Servo robotTurn2 = null;
    public Servo robotClawTurn = null;
    public Servo robotClaw = null;

    // Debounce for buttons
    boolean leftDpadDebounce = false;
    boolean rightDpadDebounce = false;
    boolean downDpadDebounce = false;
    boolean upDpadDebounce = false;
    boolean yeet = false;

    double[] neutralArmPosition = {0.65, 0.55, 0.55};
    double[] boxArmPosition = {0.65, 0.7, 0.7};
    int stateIndex = 0;
    int armState = 0;
    int handState = 0;
    int boxState = 0;



    public static final int CYCLE_DIVIDE = 10;
    public static final double FAN_SPEED_START = 0.4;
    public static final double FAN_SPEED_IDLE = 0.0;

    public static final double RING_CLOSED = 1.0;
    public static final double RING_OPEN = 0.5;

    public static final double DRIVE_SPEED_START = 0.3;
    public static final double ARM_UP_POWER = 0.10;
    public static final double ARM_DOWN_POWER = -0.05;
    public static final int ARM_UPPER_LIMIT = 700;

    public static final double CLAW_OPEN = 0.1;
    public static final double CLAW_SHUT = 0.3;

    // {lrPos, turn1, turn2, clawTurn, claw, fan};
    //turn1, turn2 == 0.75 is in the box

    double boxBottom = 0.75;
    public static final double boxMiddle = 0.73;
    public static final double boxBack = 0.78;

    public static final double[] NEUTRAL_POSITION = {0.7, 0.5, 0.5, 0.00, CLAW_OPEN, FAN_SPEED_START};
    public static final double[] FAN_RELEASE = {0.6, 0.5, 0.5, 0.00, CLAW_OPEN, FAN_SPEED_START};
    public static final double[] FAN_POSITION = {0.6, 0.5, 0.5, 0.00, CLAW_SHUT, FAN_SPEED_START};

    public static final double[] BOX1 = {0.7, 0.75, boxMiddle, 0.00, CLAW_OPEN, FAN_SPEED_IDLE};
    public static final double[] BOX1_CLOSED = {0.7, 0.75, boxMiddle, 0.00, CLAW_SHUT, FAN_SPEED_START};

    public static final double[] BOX2 = {0.73, 0.7, boxBack, 0.00, CLAW_OPEN, FAN_SPEED_IDLE};
    public static final double[] BOX2_CLOSED = {0.73, 0.7, boxBack, 0.00, CLAW_SHUT, FAN_SPEED_START};

    public static final double[] BOX3 = {0.60, 0.7, boxBack, 0.00, CLAW_OPEN, FAN_SPEED_IDLE};
    public static final double[] BOX3_CLOSED = {0.60, 0.7, boxBack, 0.00, CLAW_SHUT, FAN_SPEED_START};
    public static final double[][] SEQUENCE = {
//            NEUTRAL_POSITION,
            BOX1,
            BOX1_CLOSED,
            FAN_POSITION,
            FAN_RELEASE,
//            NEUTRAL_POSITION,
            BOX2,
            BOX2_CLOSED,
            FAN_POSITION,
            FAN_RELEASE,
//            NEUTRAL_POSITION,
//            FAN_POSITION,
            BOX3,
            BOX3_CLOSED,
            FAN_POSITION,
            FAN_RELEASE
    };



    public void driveRobot(double drive, double turn, double speed) {
        // Combine drive and turn for blended motion.
        double left = (drive + turn) * speed;
        double right = (drive - turn) * speed;

        // Normalize the values so neither exceed +/- 1.0
        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0) {
            left /= max;
            right /= max;
        }

        // Output the safe vales to the motor drives.
        leftDrive.setPower(left);
        rightDrive.setPower(right);

        telemetry.addData("left", "%.2f", left);
        telemetry.addData("right", "%.2f", right);
    }

    public void liftPlatform(boolean up, boolean down, double armHoldPower) {
        // Use gamepad bumpers to move arm up (right) and down (left)
//            if (gamepad1.right_bumper && encoder + diffPos < armPeak) {
//                armPower = ARM_UP_POWER / ((double) (armPeak - encoder) / armPeak);
//            } else if (gamepad1.left_bumper && encoder + diffPos > startPos) {
//                armPower = ARM_DOWN_POWER / ((double) encoder / armPeak);
//            }

//        encoder = -1 * arm.getCurrentPosition();
//
//        int diffPos = encoder - lastPos;
//        lastPos = encoder;
        // if we're at the bottom, stop moving
//            if (encoder + diffPos <= startPos) {
//                armPower = 0.0;
//            }
//            // if we're overshooting, stop
//            if (armPeak <= encoder + diffPos) {
//                armPower = armHoldPower; // just enough to sustain
//            }

        if (up) {
            arm.setPower(ARM_UP_POWER);
        } else if (down) {
            arm.setPower(ARM_DOWN_POWER);
        }
//        else {
//            arm.setPower(armHoldPower);
//        }
    }

    public double tweakHoldPower(double higher, double lower, double armHoldPower) {
        if (higher > 0) {
            armHoldPower += 0.0001;
        } else if (lower > 0) {
            armHoldPower -= 0.0001;
        }
        return armHoldPower;
    }

    public double tweakSpeed(double higher, double lower, double speed) {
        if (higher > 0) {
            speed += 0.01;
        } else if (lower > 0) {
            speed -= 0.01;
        }
        return speed;
    }

    public double tweakFanSpeed(double higher, double lower, double speed) {
        if (higher > 0) {
            speed += 0.01;
        } else if (lower > 0) {
            speed -= 0.01;
        }
        return speed;
    }

    public void popRings(boolean pop) {
        if (pop) {
            ringOpener.setPosition(RING_OPEN);
        } else {
            ringOpener.setPosition(RING_CLOSED);
        }
    }

    public void moveArm() {
        // automatically transition through the states, but pace it since we run every 50 ms
        // {lrPos, turn1, turn2, clawTurn, claw, fan};
        int position = (stateIndex / CYCLE_DIVIDE) % SEQUENCE.length;
        telemetry.addData("position", "%d", position);
        double[] armPos = SEQUENCE[position];


        telemetry.addData("lrPos", "%.2f", armPos[0]);
        telemetry.addData("turn1", "%.2f", armPos[1]);
        telemetry.addData("turn2", "%.2f", armPos[2]);
        telemetry.addData("clawTurn", "%.2f", armPos[3]);
        telemetry.addData("claw", "%.2f", armPos[4]);
        telemetry.addData("fan", "%.2f", armPos[5]);

        robotLR.setPosition(armPos[0]);
        robotTurn1.setPosition(armPos[1]);
        robotTurn2.setPosition(armPos[2]);
        robotClawTurn.setPosition(armPos[3]);
        robotClaw.setPosition(armPos[4]);
        fan.setPower(armPos[5]);

//        return armPos;
    }
    public double[] moveArm2(boolean left, boolean right, boolean up, boolean down, double[] armPos) {
        // {lrPos, turn1, turn2, clawTurn, claw};
        if (left && armPos[0] < 1.0 && !leftDpadDebounce) {
            armPos[0] += 0.05;
            leftDpadDebounce = true;
        } else if (right && armPos[0] > 0.3 && !rightDpadDebounce) {
            armPos[0] -= 0.05;
        }
        if (down && armPos[1] < 1.0 && armPos[2] < 1.0 && !downDpadDebounce) {
            armPos[1] += 0.05;
            armPos[2] += 0.05;
        } else if (up && armPos[1] > 0.0 && armPos[2] > 0.0 && !upDpadDebounce) {
            armPos[1] -= 0.05;
            armPos[2] -= 0.05;
        }

        // Reset debounces
        leftDpadDebounce = left;
        rightDpadDebounce = right;
        upDpadDebounce = up;
        downDpadDebounce = down;

        if (gamepad1.a && armPos[3] < 1.0) {
            armPos[3] += 0.05;
        } else if (gamepad1.y && armPos[3] > 0.0) {
            armPos[3] -= 0.05;
        }
        telemetry.addData("lrPos", "%.2f", armPos[0]);
        telemetry.addData("turn1", "%.2f", armPos[1]);
        telemetry.addData("turn2", "%.2f", armPos[2]);
        telemetry.addData("clawTurn", "%.2f", armPos[3]);



        robotLR.setPosition(armPos[0]);
        robotTurn1.setPosition(armPos[1]);
        robotTurn2.setPosition(armPos[2]);
        robotClawTurn.setPosition(armPos[3]);
//        robotClaw.setPosition(armPos[4]);
//        fan.setPower(armPos[5]);

        return armPos;
    }

    private void alignState() {
        armState = ((stateIndex - 1) / 2) % 2;
        handState = (stateIndex / 2) % 2;
    }

    private void applyState() {
        // Apply arm state
        if (armState == 0) {
            robotLR.setPosition(neutralArmPosition[0]);
            robotTurn1.setPosition(neutralArmPosition[1]);
            robotTurn2.setPosition(neutralArmPosition[2]);
        }
        else {
            robotLR.setPosition(boxArmPosition[0]);
            robotTurn1.setPosition(boxArmPosition[1]);
            robotTurn2.setPosition(boxArmPosition[2]);
        }

        // Apply hand state
        if (handState == 0) {
            robotClaw.setPosition(CLAW_SHUT);
        }
        else {
            robotClaw.setPosition(CLAW_OPEN);
        }
    }

    public void yeetFlowers(boolean yeet, double fanSpeed) {
        if (yeet) {
            robotClaw.setPosition(CLAW_SHUT);
            fan.setPower(fanSpeed);
        } else {
            robotClaw.setPosition(CLAW_OPEN);
            fan.setPower(FAN_SPEED_IDLE);
        }
    }

    @Override
    public void runOpMode() {
        // Define drive motors
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        // platform arm
        arm = hardwareMap.get(DcMotor.class, "arm");
        // Flower fan
        fan = hardwareMap.get(DcMotor.class, "fan");
        // ring opener
        ringOpener = hardwareMap.get(Servo.class, "ring_opener");

        // Robot arm
        robotLR = hardwareMap.get(Servo.class, "robotLR");
        robotTurn1 = hardwareMap.get(Servo.class, "robotTurn1");
        robotTurn2 = hardwareMap.get(Servo.class, "robotTurn2");
        robotClawTurn = hardwareMap.get(Servo.class, "robotClawTurn");
        robotClaw = hardwareMap.get(Servo.class, "robotClaw");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.REVERSE);
        fan.setDirection(DcMotor.Direction.FORWARD);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
//        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Define and initialize ALL installed servos.
        ringOpener.setPosition(RING_CLOSED);
        double speed = DRIVE_SPEED_START;
        double fanSpeed = FAN_SPEED_START;

        double lrPos = 0.7;
        double turn1 = 0.5;
        double turn2 = 0.5;
        double clawTurn = 0.0;
        double claw = CLAW_OPEN;

        double[] armPos = {lrPos, turn1, turn2, clawTurn, claw, fanSpeed};

        double armPower = 0.0;
        double armHoldPower = 0.002;

//        int stateIndex = 0;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

//        double[] armPos1 = {0.65, 0.75, 0.75, 0.00, CLAW_OPEN};
//        double[] armPos2 = {0.65, 0.75, 0.75, 0.00, CLAW_SHUT};
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.

            speed = tweakSpeed(gamepad1.right_trigger, gamepad1.left_trigger, speed);
            driveRobot(-gamepad1.left_stick_y, gamepad1.right_stick_x, speed);
            liftPlatform(gamepad1.right_bumper, gamepad1.left_bumper, armHoldPower);

            // Use gamepad b button to pop the rings
            popRings(gamepad1.b);

            if(gamepad1.a){
                yeet = true;
            } else if(gamepad1.y){
                yeet = false;
            }

            if(yeet) {
                stateIndex++;
                moveArm();
            } else {
                if(!gamepad1.x && fanSpeed > FAN_SPEED_IDLE){
                    fanSpeed -= 0.001;
                }
                yeetFlowers(gamepad1.x, fanSpeed);
                armPos = moveArm2(gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.dpad_up, gamepad1.dpad_down, armPos);
            }

//            stateIndex = moveArm(gamepad1.dpad_left, gamepad1.dpad_right, )


            // Send telemetry message to signify robot running;

            telemetry.addData("arm power", "%.2f", armPower);
            telemetry.addData("arm hold power", "%.5f", armHoldPower);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }

//        while (opModeIsActive()) {
//            stateIndex++;
//            alignState();
//            applyState();
//
//            sleep(1000);
//        }
    }
}
