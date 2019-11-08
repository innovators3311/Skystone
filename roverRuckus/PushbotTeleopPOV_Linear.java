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

package org.firstinspires.ftc.teamcode.Skystone.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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
enum BeaterPickupPosition
{
    bottom, middle, top;
};

enum StringPullerPosition{
    in, out;
}
@TeleOp(name="TileRunnerPOV1", group="Test")
public class PushbotTeleopPOV_Linear extends LinearOpMode {

    public static final double CORRECTION_SCALE = 200.0;
    /* Declare OpMode members. */
    HardwarePushbot robot           = new HardwarePushbot();   // Use a Pushbot's hardware
                                                               // could also use HardwarePushbotMatrix class.
     boolean         crabing         = false;
    ElapsedTime timer = new ElapsedTime();
    Thread AllTheWayUpThread = null, AllTheWayDownThread = null;

   int gamepad2_y_down=0,gamepad2_a_down=0;
   int gamepad2_b_down=0,gamepad2_x_down=0;
    double left, right, drive, turn, max;
    double compass_ideal_heading=-1;
    boolean left_trigger_down=false,right_trigger_down=false;
    boolean left_bumper_down=false,right_bumper_down=false;
    BeaterPickupPosition bpp=BeaterPickupPosition.bottom;
    StringPullerPosition spp=StringPullerPosition.in;
    @Override
    public void runOpMode() {


        robot.init(hardwareMap, this);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.clear();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //particle pickup (Orange thing spins)/////////////////
            BeaterPower();
            bpp = BeaterPickupPosition(bpp);

//            spp=ArmExtendPosition(spp);
            //this triggers//////////////////////////////////////////////////////////////////
            LiftControl();

            PickupNudge();
            //car wash
            carWash();
            //bumpers//////////////////////////////////////////////////////////////////////////
            Bumpers();


            //driving or crabbing///////////////////////////////////////////////////////////////
            Driving();


            idle();
            telemetry.addData("2Y","pickup angle cur:"+robot.accesseries.BeaterPickup.getCurrentPosition()+" desored"+robot.accesseries.getDesiredPositionStored());
            telemetry.addData("gamepad","amepad2.right_stick_y"+ gamepad2.right_stick_y);
            telemetry.addData("aaaaaaarrrrrmmm","stringpuller"+robot.accesseries.stringpuller.getCurrentPosition());
            telemetry.addData("ArmAngle", "ArmAngleStatus"+robot.accesseries.ArmAngleStatus);
//            telemetry.addData("difference","stringpuller"+robot.accesseries.stringpuller.getCurrentPosition()+"   tgt:"+robot.accesseries.getArmInOutTargetPosition());

            telemetry.update();


        }//done in op mode
        robot.accesseries.stop();
        //stop everything!
        robot.accesseries.controlLift(0);

        if(robot.accesseries.MoveBeaterToPosistionThread!=null)
            robot.accesseries.MoveBeaterToPosistionThread.interrupt();

        if(AllTheWayUpThread!=null)
            AllTheWayUpThread.interrupt();
        if(AllTheWayDownThread!=null)
            AllTheWayDownThread.interrupt();
        // Output the safe vales to the motor drives.
        robot.setMotors(0, 0);

    }

    private void carWash() {
        if(gamepad2.x){
            robot.accesseries.new_beater.setPower(.9);
            robot.accesseries.slowBeater.setPower(-.9);
        }

        else if(gamepad2.b){
            robot.accesseries.new_beater.setPower(-.9);
            robot.accesseries.slowBeater.setPower(.9);
        }

        else{
            robot.accesseries.new_beater.setPower(0);
            robot.accesseries.slowBeater.setPower(0);
        }
    }

    private void PickupNudge() {
        if(gamepad2.right_stick_y>.5){
            float CurrentPosition=robot.accesseries.getDesiredPositionStored();
            robot.accesseries.setDesiredPositionStored(CurrentPosition+20f,this);
        }
        else if(gamepad2.right_stick_y<-.5){
            float CurrentPosition=robot.accesseries.getDesiredPositionStored();
            robot.accesseries.setDesiredPositionStored(CurrentPosition-20f,this);
        }
        else{

        }
    }

    private void Driving() {
        if(gamepad1.left_trigger>.5)//crab left
        {
            crabbing_left();
        }
        else if(gamepad1.right_trigger>.5)//crab right
        {
            crabbing_right();

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

        drive = gamepad1.left_stick_y;
        turn  =  gamepad1.right_stick_x;

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
        robot.setMotors(left, right);
    }

    private void Bumpers() {
        if(gamepad2.right_bumper)//lift all the way up
        {
            if(right_bumper_down==false)
            {
                //only one at a time
                if(AllTheWayDownThread!=null)
                    AllTheWayDownThread.interrupt();
                AllTheWayUpThread=new Thread(robot.accesseries.new AllTheWayDownRunnable(this));
                AllTheWayUpThread.start();
            }
            right_bumper_down=true;
            left_bumper_down=false;
        }

        else if(gamepad2.left_bumper)
        {
            if(left_bumper_down==false)
            {
                //only one at a time
                if(AllTheWayUpThread!=null)
                    AllTheWayUpThread.interrupt();
                AllTheWayDownThread=new Thread(robot.accesseries.new AllTheWayUpRunnable(this));
                AllTheWayDownThread.start();
            }
            left_bumper_down=true;
            right_bumper_down=false;
        }

        else
        {

            left_bumper_down=false;
            right_bumper_down=false;
        }
    }

    private void LiftControl() {
        if(gamepad2.right_trigger>.1)
        {//right is down and down is +
            if(AllTheWayDownThread!=null)
                AllTheWayDownThread.interrupt();
            if(AllTheWayUpThread!=null)
                AllTheWayUpThread.interrupt();
            robot.accesseries.controlLift(gamepad2.right_trigger);
        }
        else if(gamepad2.left_trigger>.1)
        {//left is up and up is -
            if(AllTheWayDownThread!=null)
                AllTheWayDownThread.interrupt();
            if(AllTheWayUpThread!=null)
                AllTheWayUpThread.interrupt();
            robot.accesseries.controlLift(gamepad2.left_trigger*-1);
        }
        else
        {
            boolean DontStop=false;
            if(AllTheWayDownThread!=null) {
                if (AllTheWayDownThread.isAlive())
                {
                    DontStop = true;
                }
            }
            if(AllTheWayUpThread!=null) {
                if (AllTheWayUpThread.isAlive())
                {
                    DontStop = true;
                }
            }

            if (DontStop==false){
                robot.accesseries.controlLift(0);

            }
        }
    }
//    private StringPullerPosition ArmExtendPosition(StringPullerPosition spp) {
//
//        if(gamepad2.dpad_up){
//            //move arm out
////            robot.accesseries.beater.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            robot.accesseries.stringpuller.setTargetPosition((int) HardwarePushbot.ARM_OUT);
////            robot.accesseries.stringpuller.setPower(Math.abs(-.9f));
//            robot.accesseries.setArmInOutTargetPosition( HardwarePushbot.ARM_OUT);
//        }
//
//        if(gamepad2.dpad_down){
//            //move arm in
//            robot.accesseries.setArmInOutTargetPosition( HardwarePushbot.ARM_IN);
////            robot.accesseries.beater.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            robot.accesseries.stringpuller.setTargetPosition((int) HardwarePushbot.ARM_IN);
////            robot.accesseries.stringpuller.setPower(Math.abs(.9f));
//        }
//        return spp;
//    }

    String PickupStateString="";
    private BeaterPickupPosition BeaterPickupPosition(BeaterPickupPosition bpp) {
        if(gamepad2.y){

            //going up
            if(gamepad2_y_down<0){

                if(bpp==BeaterPickupPosition.bottom){
                    //pulls extender arm out
//                    robot.accesseries.setArmInOutTargetPosition((int)HardwarePushbot.ARM_IN/4);

//                    robot.accesseries.beater.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    robot.accesseries.stringpuller.setTargetPosition((int) HardwarePushbot.ARM_IN/4);
//                    robot.accesseries.stringpuller.setPower(Math.abs(-.99f));
                    robot.accesseries.MoveBeaterToPosistion(HardwarePushbot.MIDDLE_POSITION,this);
                    PickupStateString="MIDDLE";
                    bpp=BeaterPickupPosition.middle;
                }
                else if(bpp==BeaterPickupPosition.middle){
                    //pulls the extender arm out

//                    robot.accesseries.beater.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    //move the bobbin so it gives the string a little slack so the string doesn't snap when the arm moves up
//                    robot.accesseries.stringpuller.setTargetPosition((int) HardwarePushbot.ARM_OUT);
//                    robot.accesseries.stringpuller.setPower(Math.abs(.99f));
                    robot.accesseries.MoveBeaterToPosistion(HardwarePushbot.TOP_POSITION,this);
//                    robot.accesseries.setArmInOutTargetPosition((int)HardwarePushbot.ARM_OUT);
                    PickupStateString="TOP";
                    bpp=BeaterPickupPosition.top;
                }
                else {
                    PickupStateString="MAX";
                }
                //do nothing if it's top

            }
            gamepad2_y_down=5;
        }
        else{
            if(gamepad2_y_down>-1){
                gamepad2_y_down-=1;
                PickupStateString="DEBOUNCE Y";
            }
          //  telemetry.addData("2Y","current state"+bpp.toString());

        }

        //arm adjustments (not in out, but up down)
//        if(gamepad2.dpad_up){
//            robot.accesseries.setDesiredPositionStored(robot.accesseries.getDesiredPositionStored()-4, this);
//        }
//
//        if(gamepad2.dpad_down){
//            robot.accesseries.setDesiredPositionStored(robot.accesseries.getDesiredPositionStored()+4, this);
//        }


        if(gamepad2.a){
            //going down
            if(gamepad2_a_down<0){
                if(bpp==BeaterPickupPosition.top){

//                    robot.accesseries.stringpuller.setTargetPosition((int) HardwarePushbot.ARM_IN/4);
//                    robot.accesseries.stringpuller.setPower(Math.abs(-.99f));
                    robot.accesseries.MoveBeaterToPosistion(HardwarePushbot.MIDDLE_POSITION,this);
//                    robot.accesseries.setArmInOutTargetPosition((int)HardwarePushbot.ARM_IN/4);

                    PickupStateString="MIDDLE";
                    bpp=BeaterPickupPosition.middle;
                }
                else if(bpp==BeaterPickupPosition.middle){

//                    robot.accesseries.stringpuller.setTargetPosition((int) HardwarePushbot.ARM_OUT/2);
//                    robot.accesseries.stringpuller.setPower(Math.abs(.99f));
                    //dumping position without arm extending - operator will do it manually
                    robot.accesseries.MoveBeaterToPosistion(HardwarePushbot.BOTTOM_POSITION,this);
//                    robot.accesseries.setArmInOutTargetPosition((int)HardwarePushbot.ARM_OUT);
                    PickupStateString="BOTTOM";
                    bpp=BeaterPickupPosition.bottom;
                }
                //do nothing if it's bottom
                else{
                    PickupStateString="MIN";
                }

            }
            gamepad2_a_down=5;
        }
        else{
            if(gamepad2_a_down>-1){
                gamepad2_a_down-=1;
                PickupStateString="Debounce A";
            }
        }
        return bpp;
    }

    private void BeaterPower() {
        if(gamepad2.left_stick_y>.8){
           int poop= robot.accesseries.stringpuller.getCurrentPosition()+280;
//           robot.accesseries.setArmInOutTargetPosition(poop);
        }
        else if(gamepad2.left_stick_y<-.8){
            int poop= robot.accesseries.stringpuller.getCurrentPosition()-280;
//            robot.accesseries.setArmInOutTargetPosition(poop);
        }
        else{
//            if(!robot.accesseries.beater.isBusy()){
//                robot.accesseries.beater.setPower(0);
//            }
        }

    }


}
