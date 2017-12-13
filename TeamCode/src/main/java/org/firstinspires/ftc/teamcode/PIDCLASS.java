package org.firstinspires.ftc.teamcode;

/**
 * Created by Itai Shufaro on 12/12/2017.
 */
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.hardware.Gyroscope;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import java.util.Set;
import java.math.*;

public class PIDCLASS {
    /**
     * Every time you use the PIDCLASS ma bke sure to change PIDCLASS
     * public variables.
     * Includes ONLY Kd,Ki,Kp
     */
    double Kd,Ki,Kp;
    public void setKp(double n) {
        Kp = n;
    }
    public void setKi(double n) {
        Ki = n;
    }
    public void setKd(double n) {
        Kd = n;
    }
    double e,e2,rpm,dt;//e2 is the priveous e. rpm is Rounds Per Minute
    Set<Axis> de;
    public double angaleturn (double angle,DcMotor rightMotor, DcMotor leftMotor,Gyroscope gyroscope){//notice that angle is in radians
        de = gyroscope.getAngularVelocityAxes();
        return agle;
    }
    public double movewheel (double position) {
        double pos = 0.0;
        return pos;
    }
}
