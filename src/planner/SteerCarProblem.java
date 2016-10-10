package planner;

import java.awt.Rectangle;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Random;

public class SteerCarProblem extends SearchProblem {
	// six possible movement, {forward/backward, angle}
	double[][] moves = { { 1, 0 }, { -1, 0 }, { 1, 1 }, { -1, -1 }, { 1, -1 }, { -1, 1 } };
	// the radius of the car
	protected double car_radius;
	// The starting state of the car, {x, y, theta}
	protected double[] start;
	// The goal state of the car
	protected double[] goal;
	// World
	protected World world;
	// Samples in the world
	HashSet<SteerCarNode> samples;
	// Adjacent list of the tree
	HashMap<SteerCarNode, HashSet<SteerCarNode>> rrt;
	// move_ratio, indicates the difference between samples
	double mr;

	public SteerCarProblem(World w, double[] s, double[] g, int density, long seed, double move_ratio, int speed) {
		start = new double[3];
		for (int i = 0; i < 3; i++)
			start[i] = s[i];
		goal = new double[3];
		for (int i = 0; i < 3; i++)
			goal[i] = g[i];
		world = w;
		car_radius = 10;
		startNode = new SteerCarNode(start, this, -1);
		samples = new HashSet<SteerCarNode>();
		rrt = new HashMap<SteerCarNode, HashSet<SteerCarNode>>();
		samples.add((SteerCarNode) startNode);
		rrt.put((SteerCarNode) startNode, new HashSet<SteerCarNode>());
		mr = move_ratio;
		setVol(speed);
		System.out.println("Tree Constructing...");
		growTree(seed, density, move_ratio);
	}

	public void setVol(int ratio) {
		for (int i = 0; i < 6; i++) {
			moves[i][0] *= ratio;
		}
	}

	public double getRadius() {
		return car_radius;
	}

	public SteerCarNode getStart() {
		return new SteerCarNode(start, this, -1);
	}

	public SteerCarNode getGoal() {
		return new SteerCarNode(goal, this, -1);
	}

	public HashMap<SteerCarNode, HashSet<SteerCarNode>> getRRT() {
		return rrt;
	}
	public HashSet<SteerCarNode> getSamples(){return samples;}
	// Construct the tree, two methods are used:
	private void growTree(long seed, int density, double move_ratio) {
		Random rd = new Random(seed);
		while (samples.size() < density + 2) {
			// Generate a random car node
			double[] rndConfig = new double[3];
			rndConfig[0] = rd.nextDouble() * world.getWidth();
			rndConfig[1] = rd.nextDouble() * world.getHeight();
			rndConfig[2] = rd.nextDouble() * Math.PI * 2;
			SteerCarNode rndCar = new SteerCarNode(rndConfig, this, 0);
			// Expand tree if random car node is legal
			if (!rndCar.carCollide(world)) {
				SteerCarNode nearest = nearestCar(rndCar);
				SteerCarNode expandNode = expandTree(nearest, rndCar, move_ratio);
				// if no such node that can be expand, exit
				if (expandNode == null) {
					System.out.println("Expand tree failed!");
					System.exit(0);
				}
				// Expand tree if expand node is a new node
				if (!samples.contains(expandNode)) {
					samples.add(expandNode);
					rrt.get(nearest).add(expandNode);
					rrt.put(expandNode, new HashSet<SteerCarNode>());
					// if new node is close enough to the goal, break
					if (expandNode.goalTest())
						break;
				}
			}
			System.out.println(Integer.toString(samples.size()) + "/" + Integer.toString(density + 2) + " Done!");
		}

		if (samples.size() >= density + 2) {
			System.out.println("Fail to find a close enough node, use the closest node as the goal, which is:");
			double dist = Double.MAX_VALUE;
			SteerCarNode goalNearest = null;
			for (SteerCarNode cur : samples) {
				if (cur.getDistance(getGoal()) < dist) {
					goalNearest = cur;
					dist = cur.getDistance(getGoal());
				}
			}
			System.out.println(goalNearest.toString());
			goal[0] = goalNearest.getState(0);
			goal[1] = goalNearest.getState(1);
			goal[2] = goalNearest.getState(2);
		}
	}

	// Decide the expanding node according to a random car node,
	// by trying to expand the tree in 6 directions. This method
	// us used by method growTree(long, int): void
	private SteerCarNode expandTree(SteerCarNode nearest, SteerCarNode rndCar, double move_ratio) {
		double minDis = Double.MAX_VALUE;
		SteerCarNode expandCar = null;
		for (int i = 0; i < 6; i++) {
			SteerCarNode cur = nearest.moveCar(i, move_ratio);
			if (!cur.carCollide(world) && !nearest.carPathCollide(world, i, move_ratio, 0.01)) {
				double dis = cur.getDistance(rndCar);
				if (minDis > dis) {
					minDis = dis;
					expandCar = cur;
				}
			}
		}
		return expandCar;
	}

	// Get the nearest car node given a car node, this method
	// is used by method growTree(long, int) : void
	private SteerCarNode nearestCar(SteerCarNode rndCar) {
		SteerCarNode nearest = null;
		double minDis = Double.MAX_VALUE;
		for (SteerCarNode car : samples) {
			double dis = rndCar.getDistance(car);
			if (minDis > dis) {
				minDis = dis;
				nearest = car;
			}
		}
		return nearest;
	}

	// Interpolate intermediate paths to make the animation smoother
	public List<SteerCarNode> smoothPath(List<SearchNode> path, double step_size) {
		List<SteerCarNode> res = new ArrayList<SteerCarNode>();
		res.add((SteerCarNode) (path.get(0)));
		for (int i = 0; i < path.size() - 1; i++) {
			SteerCarNode curNode = (SteerCarNode) (path.get(i));
			SteerCarNode nextNode = (SteerCarNode) (path.get(i + 1));
			List<SteerCarNode> locPath = curNode.getPath(nextNode.getControl(), mr, step_size);
			res.addAll(locPath);
		}
		return res;
	}

	public class SteerCarNode implements SearchNode {
		protected double[] state;
		protected double cost;
		protected SteerCarProblem carRobot;
		int control;

		public SteerCarNode(SteerCarProblem cr, int ctrl) {
			state = new double[3];
			state[0] = 0;
			state[1] = 0;
			state[2] = 0;
			cost = 0;
			carRobot = cr;
			control = ctrl;
		}

		public SteerCarNode(double[] config, SteerCarProblem cr, int ctrl) {
			state = new double[3];
			state[0] = config[0];
			state[1] = config[1];
			state[2] = config[2];
			cost = 0;
			carRobot = cr;
			control = ctrl;
		}

		public SteerCarNode(SteerCarNode cur, double c, SteerCarProblem cr, int ctrl) {
			state = new double[3];
			state[0] = cur.getState(0);
			state[1] = cur.getState(1);
			state[2] = cur.getState(2);
			cost = c;
			carRobot = cr;
			control = ctrl;
		}

		public double getState(int i) {
			return state[i];
		}

		public int getControl() {
			return control;
		}

		public SteerCarProblem getCar() {
			return carRobot;
		}

		// Get the state of the car after a movement by a certain
		// control after t time
		public SteerCarNode moveCar(int ctrl, double t) {
			double radius = Math.abs(moves[ctrl][0] / moves[ctrl][1]);
			double[] newConfig = new double[3];
			if (ctrl == 0 || ctrl == 1) { // move forward or backward
				newConfig[0] = state[0] + t * moves[ctrl][0] * Math.cos(state[2]);
				newConfig[1] = state[1] + t * moves[ctrl][0] * Math.sin(state[2]);
				newConfig[2] = state[2];
			} else if (ctrl == 2 || ctrl == 3) { // move left forward or left
													// backward
				double[] center = new double[2];
				center[0] = state[0] - radius * Math.sin(state[2]);
				center[1] = state[1] + radius * Math.cos(state[2]);
				newConfig[2] = state[2] + t * moves[ctrl][1];
				newConfig[0] = center[0] + radius * Math.cos(newConfig[2] - Math.PI / 2);
				newConfig[1] = center[1] + radius * Math.sin(newConfig[2] - Math.PI / 2);
			} else { // move right forward or left backward
				double[] center = new double[2];
				center[0] = state[0] + radius * Math.sin(state[2]);
				center[1] = state[1] - radius * Math.cos(state[2]);
				newConfig[2] = state[2] + t * moves[ctrl][1];
				newConfig[0] = center[0] + radius * Math.cos(newConfig[2] + Math.PI / 2);
				newConfig[1] = center[1] + radius * Math.sin(newConfig[2] + Math.PI / 2);
			}
			newConfig[2] = (newConfig[2] + Math.PI * 2) % (Math.PI * 2);
			return new SteerCarNode(newConfig, carRobot, ctrl);
		}

		// Get the path from current state according to the
		// control and the time and the step. The return is
		// a list of movement from the current by moving a
		// certain time following ctrl.
		public List<SteerCarNode> getPath(int ctrl, double time, double step) {
			List<SteerCarNode> res = new ArrayList<SteerCarNode>();
			double interval = time * step;
			double cur_time = interval;
			while (cur_time < time) {
				res.add(moveCar(ctrl, cur_time));
				cur_time += interval;
			}
			res.add(moveCar(ctrl, time));
			return res;
		}

		// Get the distance between this state and the other
		public double getDistance(SteerCarNode other) {
			double ang = Math.abs(state[2] - other.getState(2));
			if (ang >= Math.PI)
				ang = 2 * Math.PI - ang;
			return Math.pow(Math.pow(state[0] - other.getState(0), 2) + Math.pow(state[1] - other.getState(1), 2), 0.5);
			// Weight angle more
		}

		// Test if the car collide with any of the obstacles
		public boolean carCollide(World w) {
			if (state[0] < carRobot.car_radius || state[0] > w.getHeight() - carRobot.car_radius
					|| state[1] < carRobot.car_radius || state[1] > w.getWidth() - carRobot.car_radius)
				return true;
			List<Rectangle> obstacles = w.getObstacles();
			for (Rectangle ob : obstacles) {
				if (rectIntersect(ob))
					return true;
			}
			return false;
		}

		// Test if the path of the car collide with the world
		// given a direction and a time
		public boolean carPathCollide(World w, int ctrl, double move_ratio, double step_size) {
			List<SteerCarNode> locPath = this.getPath(ctrl, move_ratio, step_size);
			for (SteerCarNode inter : locPath)
				if (inter.carCollide(world))
					return true;
			return false;
		}

		// Test whether this car state intersect a rectangle
		private boolean rectIntersect(Rectangle rect) {
			double rect_x = rect.getCenterX();
			double rect_y = rect.getCenterY();
			if (Math.abs(state[0] - rect_x) < carRobot.car_radius + rect.getWidth() / 2
					&& Math.abs(state[1] - rect_y) < carRobot.car_radius + rect.getHeight() / 2)
				return true;
			return false;
		}

		@Override
		public boolean equals(Object o) {
			for (int i = 0; i < 3; i++)
				if (state[i] != ((SteerCarNode) o).getState(i))
					return false;
			return true;
		}

		@Override
		public int hashCode() {
			int code = 0;
			for (int i = 0; i < 3; i++)
				code += Math.pow(state[i], Math.pow(100, i));
			return code;
		}

		@Override
		public int compareTo(SearchNode arg0) {
			// TODO Auto-generated method stub
			return (int) Math.signum(getPriority() - arg0.getPriority());
		}

		@Override
		public ArrayList<SearchNode> getSuccessors() {
			// TODO Auto-generated method stub
			ArrayList<SearchNode> suc = new ArrayList<SearchNode>();
			HashMap<SteerCarNode, HashSet<SteerCarNode>> rrtMap = carRobot.getRRT();
			for (SteerCarNode cur : rrtMap.get(this)) {
				SteerCarNode newAdd = new SteerCarNode(cur, cur.getDistance(this), carRobot, cur.getControl());
				suc.add(newAdd);
			}
			return suc;
		}

		@Override
		public boolean goalTest() {
			// TODO Auto-generated method stub
			return getDistance(carRobot.getGoal()) < 5;
		}

		@Override
		public double getCost() {
			// TODO Auto-generated method stub
			return cost;
		}

		@Override
		public double getHeuristic() {
			// TODO Auto-generated method stub
			return getDistance(carRobot.getGoal());
		}

		@Override
		public double getPriority() {
			// TODO Auto-generated method stub
			return getHeuristic() + getCost();
		}

		public String toString() {
			StringBuilder str = new StringBuilder();
			str.append('[');
			str.append(Double.toString(state[0]) + ", ");
			str.append(Double.toString(state[1]) + ", ");
			str.append(Double.toString(state[2]) + "]");
			str.append(" Control: " + Integer.toString(control));
			return str.toString();
		}
	}

}