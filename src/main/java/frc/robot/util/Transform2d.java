package frc.robot.util;

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

	/**
	 * Print in matrix format
	 */
	public void printmat() {
		new DMatrix3x3(cos, -sin, x, sin, cos, y, 0, 0, 1).print();
	}

	/**
	 * Print transform
	 */
	public void print() {
		System.out.println(toString());
	}

	/**
	 * @param o - other matrix to multiply by
	 * @return product of matrix multiplication
	 */
	public Transform2d mul(Transform2d o) {
		return new Transform2d(
				x + cos * o.x - sin * o.y,
				y + sin * o.x + cos * o.y,
				cos * o.cos - sin * o.sin,
				cos * o.sin + sin * o.cos,
				theta + o.theta);
	}

	/**
	 * @return inverse of the transform
	 */
	public Transform2d inv() {
		return new Transform2d(-cos * x - sin * y, sin * x - cos * y, cos, -sin, -theta);
	}

	@Override
	public String toString() {
		return "Transform2d(x: " + x + ", y: " + y + ", theta: " + theta + ")";
	}
}
