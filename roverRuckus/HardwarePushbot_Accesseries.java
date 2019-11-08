package org.firstinspires.ftc.teamcode.Skystone.roverRuckus;

import android.util.Log;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwarePushbot_Accesseries {

    RevBlinkinLedDriver blinkinLedDriver;
    public DcMotor liftmotor = null;
    private DcMotor beater = null;
    public DcMotor stringpuller=null;
    public DcMotor BeaterPickup = null;
    HardwareMap hwMap = null;
    private DigitalChannel TopSwitch;                // Device Object
    DigitalChannel BottomSwitch;                // Device Object
    DigitalChannel color;                    // Device Object
    private int ArmInOutTargetPosition=0;
    private final int ArmInOutBuffer=50;
    private final float ArmInOutPower = .9f;
    public String ArmAngleStatus = "";
    public CRServo new_beater;
    public CRServo slowBeater;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//    public void setArmInOutTargetPosition(int NewArmInOut_specificcountswhatever){
//        ArmInOutTargetPosition=NewArmInOut_specificcountswhatever;
//    }
//
//    public int getArmInOutTargetPosition(){
//        return ArmInOutTargetPosition;
//    }
//
//    public class ArmInOutRunnable implements Runnable {
//
//        ArmInOutRunnable(LinearOpMode parent) {
//            StoredParent = parent;
//        }
//
//        @Override
//        public void run() {
//            while(! StoredParent.isStopRequested()  && !Thread.interrupted()){
//                int ArmCurrentPosition = stringpuller.getCurrentPosition();
//                int Offset = ArmCurrentPosition-ArmInOutTargetPosition;
//
//                if(Offset<-1*ArmInOutBuffer   ){
//                    stringpuller.setPower(ArmInOutPower);
//                    ArmAngleStatus="Going Out";
//                }
//
//                else if(Offset>ArmInOutBuffer){
//                    stringpuller.setPower(-1*ArmInOutPower);
//                    ArmAngleStatus="Going In";
//                }
//
//                else{
//                    stringpuller.setPower(0);
//                    ArmAngleStatus="Stopped";
//                }
//            }
//            Log.d("problem","poop");
//        }
//    }

    public class AllTheWayUpRunnable implements Runnable {
        LinearOpMode parent;

        AllTheWayUpRunnable(LinearOpMode parent) {
            this.parent = parent;
        }

        @Override
        public void run() {
            boolean keep_running = true;

            while (!parent.isStopRequested() && keep_running && !Thread.interrupted()) {
                keep_running = controlLift(-.5);
            }

        }
    }

    public boolean IsAtTop() {
        return !this.TopSwitch.getState();//use ! for mag switch and nothing for limit switch
    }

    public class AllTheWayDownRunnable implements Runnable {
        LinearOpMode parent;

        AllTheWayDownRunnable(LinearOpMode parent) {
            this.parent = parent;
        }

        @Override
        public void run() {
            boolean keep_running = true;

            while (!parent.isStopRequested() && keep_running && !Thread.interrupted()) {
                keep_running = controlLift(.5);
            }
        }
    }

    public boolean controlLift(double Up) {
        if (liftmotor == null)
            return false;

        if (Up < 0 && IsAtTop()) {
            //trying to go higher but up
            liftmotor.setPower(0);
            return false;
        } else if (Up > 0 && BottomSwitch.getState()) {//stop going down
            liftmotor.setPower(0);
            return false;
        }

        liftmotor.setPower(Up);
        return true;
    }

    Thread MoveBeaterToPosistionThread = null;
    private float DesiredPositionStored;

    public class MoveBeaterToPosistionRunable implements Runnable {

        @Override
        public void run() {
            boolean keep_running = true;
            while (!StoredParent.isStopRequested() && keep_running && !Thread.interrupted()) {


                BeaterPickup.setTargetPosition((int) DesiredPositionStored);
                BeaterPickup.setPower(Math.abs(.1f));
                while (BeaterPickup.isBusy() && keep_running && !Thread.interrupted()) {
                    if (StoredParent != null)
                        StoredParent.idle();

                }//hold loop done

            }
            BeaterPickup.setPower(0);

        }
    }

    public void setDesiredPositionStored(float newDesiredPosition, LinearOpMode parent) {
        StoredParent = parent;
        DesiredPositionStored = newDesiredPosition;
        if (MoveBeaterToPosistionThread == null) {
            MoveBeaterToPosistionThread = new Thread(new MoveBeaterToPosistionRunable());
            MoveBeaterToPosistionThread.start();
        }

    }

    public float getDesiredPositionStored() {
        return DesiredPositionStored;
    }

    LinearOpMode StoredParent;

    public void MoveBeaterToPosistion(int DesiredPosition, LinearOpMode Parent) {
        StoredParent = Parent;
        DesiredPositionStored = DesiredPosition;


        if (MoveBeaterToPosistionThread != null)
            MoveBeaterToPosistionThread.interrupt();
        MoveBeaterToPosistionThread = new Thread(new MoveBeaterToPosistionRunable());
        MoveBeaterToPosistionThread.start();
    }
    Thread ArmInOutPositionsThread;

    public void stop(){

        if(MoveBeaterToPosistionThread !=null)
            MoveBeaterToPosistionThread.interrupt();

        ArmInOutPositionsThread.interrupt();
    }
    public void init(HardwareMap ahwMap, LinearOpMode parent) {
        // Save reference to Hardware map
        //try {
            hwMap = ahwMap;
            blinkinLedDriver = hwMap.get(RevBlinkinLedDriver.class, "lights");
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            liftmotor = hwMap.get(DcMotor.class, "liftmotor");
            liftmotor.setPower(0);
            liftmotor.setDirection(DcMotorSimple.Direction.FORWARD);
            liftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE.BRAKE);

//            new_beater = hwMap.get(DcMotor.class, "new_beater");
//            new_beater.setPower(0);
//            new_beater.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            new_beater.setDirection(DcMotorSimple.Direction.FORWARD);
//            new_beater.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//            slowBeater = hwMap.crservo.get("slowBeater");
//
//            new_beater = hwMap.crservo.get("new_beater");

            BeaterPickup = hwMap.get(DcMotor.class, "pickup");
            BeaterPickup.setPower(0);
            BeaterPickup.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BeaterPickup.setDirection(DcMotorSimple.Direction.FORWARD);
            BeaterPickup.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            BeaterPickup.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE.BRAKE);

            beater = hwMap.get(DcMotor.class, "beater");
            stringpuller=beater;
            stringpuller.setPower(0);
            stringpuller.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            stringpuller.setDirection(DcMotorSimple.Direction.REVERSE);
            stringpuller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            stringpuller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE.BRAKE);

            TopSwitch = hwMap.get(DigitalChannel.class, "liftmax");// port one
            color = hwMap.get(DigitalChannel.class, "alliance");
            if (color.getState()) {
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else {
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            }

            BottomSwitch = hwMap.get(DigitalChannel.class, "liftmin"); //port three
            TopSwitch.setMode(DigitalChannel.Mode.INPUT);          // Set the direction of each channel
            BottomSwitch.setMode(DigitalChannel.Mode.INPUT);          // Set the direction of each channel
//            ArmInOutPositionsThread=new Thread(new ArmInOutRunnable(parent));
            ArmInOutPositionsThread.start();
       // } catch (Exception e) {

        //}

    }

}