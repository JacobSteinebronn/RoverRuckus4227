package Lazerz;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by hhs-robotics on 9/20/2018.
 */
@Disabled
@TeleOp (name="Printouts",group="Things")
public class Printers extends OpMode {
    DistanceSensor sensor1;
    DistanceSensor sensor2;


    double[] s1values=new double[5];
    int s1i=0;

    @Override
    public void init() {
        sensor1=hardwareMap.get(DistanceSensor.class, "s1");
        sensor2=hardwareMap.get(DistanceSensor.class, "s2");

    }

    @Override
    public void loop() {
        if(sensor1.getDistance(DistanceUnit.CM)<400)s1values[s1i++]=sensor1.getDistance(DistanceUnit.CM);

        telemetry.addData("D1:",getAverage(s1values));
        //telemetry.addData("D2:",sensor2.getDistance(DistanceUnit.CM));
        s1i%=5;
    }

    public double getAverage(double[] arr){
        double sum=0;
        for(double d:arr){sum+=d;}
        return sum/arr.length;
    }


}
