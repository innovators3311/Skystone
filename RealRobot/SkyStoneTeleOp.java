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

package org.firstinspires.ftc.teamcode.Skystone.RealRobot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="SkyStoneTeleOp", group="SkyStone")

public class SkyStoneTeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot           = new HardwarePushbot();   // Use a Pushbot's hardware
    double          clawOffset      = 0;                       // Servo mid position
    final double    CLAW_SPEED      = 0.02 ;                   // sets rate to move servo
    boolean left_trigger_down=false,right_trigger_down=false;
    double compass_ideal_heading=-1;
    public static final double CORRECTION_SCALE = 200.0;
    double left, right, drive, turn, max;

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double max;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, this,false,false);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("go", "ready");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Output the safe vales to the motor drives.
//            robot.leftDrive.setPower(left);
//            robot.rightDrive.setPower(right);
//
//            // Use gamepad left & right Bumpers to open and close the claw
//            if (gamepad1.right_bumper)
//                clawOffset += CLAW_SPEED;
//            else if (gamepad1.left_bumper)
//                clawOffset -= CLAW_SPEED;
//
//            // Move both servos to new position.  Assume servos are mirror image of each other.
//            clawOffset = Range.clip(clawOffset, -0.5, 0.5);
//            robot.leftClaw.setPosition(robot.MID_SERVO + clawOffset);
//            robot.rightClaw.setPosition(robot.MID_SERVO - clawOffset);
//
//            //Use gamepad buttons to move arm up (Y) and down (A)
           if (gamepad1.y)
                robot.rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            else if (gamepad1.a)
               robot.rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

            if(gamepad2.right_trigger>0) {
                telemetry.addData("Servo Position", "0");    //
                telemetry.update();
                robot.LeftServo.setPosition(1);
                robot.RightServo.setPosition(0);
            }
            else if(gamepad2.left_trigger>0) {
                telemetry.addData("Servo Position", "1");    //
                telemetry.update();
                robot.LeftServo.setPosition(0);
                robot.RightServo.setPosition(1);
            }
            else{
                telemetry.addData("front", "distance"+robot.sensorTimeOfFlightFront.getDistance(DistanceUnit.CM));    //
                telemetry.addData("right", "distance"+robot.sensorTimeOfFlightRightSide.getDistance(DistanceUnit.CM));    //
                telemetry.addData("left", "distance"+robot.sensorTimeOfFlightLeftSide.getDistance(DistanceUnit.CM));    //
            }

            if(gamepad2.left_stick_y>.1){
                robot.accessories.ArmUpDown.setPower(gamepad2.left_stick_y/2f);
            }
            else if(gamepad2.left_stick_y<-.1){
                robot.accessories.ArmUpDown.setPower(gamepad2.left_stick_y/2f);
            }
            else{
                robot.accessories.ArmUpDown.setPower(0);
            }
//            if (gamepad1.b)
//                robot.FlowAngleTime(90,.2f,4,this);
//            if (gamepad1.x)
//                robot.FlowAngleTime(90,-.2f,4,this);
//            if (gamepad1.right_bumper)
//                robot.FlowAngleStrafeTime(false,.5f,4,this);
//            if (gamepad1.left_bumper)
//                robot.FlowAngleStrafeTime(true,.5f,4,this);
//            else
//                robot.leftArm.setPower(0.0);

            // Send telemetry message to signify robot running;
//            telemetry.addData("angle",  "%.2f", robot.getAngle());
//            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.

            Driving();

//            robot.setMotors(left,left,right,right);
            telemetry.addData("armUpDown", "dd"+robot.accessories.ArmUpDown.getCurrentPosition());
            telemetry.addData("arm sensors", "dd"+robot.accessories.ArmUp.isPressed()+" down "+robot.accessories.ArmDowm.isPressed());    ////
            telemetry.update();
        }
    }
    private void Driving() {
        if(gamepad1.left_trigger>.5)//crab left
        {
            crabbing_right();
        }
        else if(gamepad1.right_trigger>.5)//crab right
        {
            crabbing_left();

        }
        else
        {
            normal_driving();
        }//done driving
    }
    private void crabbing_right() {
        if(right_trigger_down==false)
        {
            compass_ideal_heading=robot.getAngle();
        }
        //scale angle difference
        double angle_difference=(compass_ideal_heading-robot.getAngle())/ CORRECTION_SCALE;
        robot.leftBackDrive.setPower(.7-angle_difference);
        robot.leftFrontDrive.setPower(-.7-angle_difference);
        robot.rightBackDrive.setPower(-.7+angle_difference);
        robot.rightFrontDrive.setPower(.7+angle_difference);
        left_trigger_down=false;
        right_trigger_down=true;
        telemetry.addData("RIGHT !!!!! compass_ideal_heading", "right trigger hit",compass_ideal_heading,robot.getAngle());
        telemetry.update();
    }

    private void crabbing_left() {
        if(left_trigger_down==false)
        {
            compass_ideal_heading=robot.getAngle();
        }
        double angle_difference=(compass_ideal_heading-robot.getAngle())/CORRECTION_SCALE;
        robot.leftBackDrive.setPower(-.7-angle_difference);
        robot.leftFrontDrive.setPower(.7-angle_difference);
        robot.rightBackDrive.setPower(.7+angle_difference);
        robot.rightFrontDrive.setPower(-.7+angle_difference);
        left_trigger_down=true;
        right_trigger_down=false;
        telemetry.addData("LEFT !!!!  compass_ideal_heading", "%.2f",angle_difference);
        telemetry.update();
    }

    private void normal_driving() {
        //not crabbing - lets drive (no stick means stop)
        left_trigger_down=false;
        right_trigger_down=false;

        drive = gamepad1.left_stick_y*-1;
        turn  =  .5*gamepad1.right_stick_x;

        if(gamepad1.left_stick_y>0){
            turn*=-.25;
        }

//If driving backwards, we change the direction of the turns
        if(gamepad1.left_stick_y<0){
        }
        // Combine drive and turn for blended motion.
        left  = drive+turn;
        right = drive-turn;

        // Normalize the values so neither exceed +/- 1.0
        max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0)
        {
            left /= max;
            right /= max;
        }

        // Output the safe vales to the motor drives.
        robot.setMotors(left,left,right,right);
    }
}
