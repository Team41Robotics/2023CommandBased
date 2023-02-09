package frc.robot;

public class Position {

    private String name;

    private double theta1; 
    private double theta2;
    private double armHeight;

    public Position(double aH, double t1, double t2, String s){

        this.theta1 = t1;
        this.theta2 = t2;
        this.armHeight = aH;
        this.name = s;

    }

    public double getArmHeight(){
        return armHeight;
    }
    
    public double getTheta1(){
        return theta1;
    }

    public double getTheta2(){
        return theta2;
    }

    public String getName(){
        return name;
    }

}
