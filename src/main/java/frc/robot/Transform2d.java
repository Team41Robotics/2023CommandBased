package frc.robot;

import org.ejml.data.DMatrix3x3;

public class Transform2d {
	public double theta, cos, sin, x, y;

	public Transform2d() {
		cos = 1;
		sin = 0;
		x = y = 0;
	}

	public Transform2d(double x, double y, double theta) {
		this.x = x;
		this.y = y;
		this.theta = theta;
		this.cos = Math.cos(theta);
		this.sin = Math.sin(theta);
	}

	public Transform2d(double x, double y, double cos, double sin) {
		this.x = x;
		this.y = y;
		this.theta = Math.atan2(sin, cos);
		this.cos = cos;
		this.sin = sin;
	}

	public Transform2d(double x, double y, double cos, double sin, double theta) {
		this.x = x;
		this.y = y;
		this.theta = theta;
		this.cos = cos;
		this.sin = sin;
	}

	public void print() {
		new DMatrix3x3(cos, -sin, x, sin, cos, y, 0, 0, 1).print();
	}

	public Transform2d mul(Transform2d o) {
		return new Transform2d(
				x + cos * o.x - sin * o.y,
				y + sin * o.x + cos * o.y,
				cos * o.cos - sin * o.sin,
				cos * o.sin + sin * o.cos,
				theta + o.theta);
	}

	public Transform2d inv() {
		return new Transform2d(-cos * x - sin * y, sin * x - cos * y, cos, -sin, -theta);
	}
}
