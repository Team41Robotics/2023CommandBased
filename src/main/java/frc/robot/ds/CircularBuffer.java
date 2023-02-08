package frc.robot.ds;

public class CircularBuffer<T> {
	T[] buf;
	int sp = 0;
	int ep = 0;

	@SuppressWarnings("unchecked")
	public CircularBuffer(int size) {
		buf = (T[]) new Object[size]; // wtv
	}

	public int size() {
		return ep - sp;
	}

	public void add(T val) {
		if (ep == sp) sp++;
		buf[ep++] = val;
	}

	public void pop() {
                buf[sp] = null;
		sp++;
	}

	public T first() {
		return buf[ep - 1];
	}

	public T last() {
		return buf[sp];
	}

	public T get(int i) {
		return buf[(sp + i) % buf.length];
	}

	public void set(int i, T val) {
		buf[(sp + i) % buf.length] = val;
	}
}
