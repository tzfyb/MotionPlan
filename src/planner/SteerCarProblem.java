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
	HashMap<SteerCarNode, HashSet<SteerCarNode>> rrt = new HashMap<>();

	public SteerCarProblem(World w, double[] s, double[] g, int density, long seed) {
		start = s;
		goal = g;
		world = w;
		car_radius = 5;
		samples.add(new SteerCarNode(start));
		rrt.put(new SteerCarNode(start), new HashSet<SteerCarNode>());
		growTree(seed, density);
	}

	private void growTree(long seed, int density) {
		while (samples.size() < density + 2) {
			//Generate a random car node
			SteerCarNode rndCar = getRndCar(seed);
			//Expand tree if random car node is legal
			if(!rndCar.carCollide(world)){
				SteerCarNode nearest = nearestCar(rndCar);
				SteerCarNode expandNode = expandTree(nearest, rndCar);
				//Expand tree if expand node is a new node
				if(!samples.contains(expandNode)){
					samples.add(expandNode);
					rrt.get(nearest).add(expandNode);
					rrt.put(expandNode, new HashSet<SteerCarNode>());
					//if new node is close enough to the goal, break
					if(expandNode.goalTest()) break;
				}
			}
		}
	}

	//Get A random car node, this function is used by
	//function growTree(long, int) : void
	private SteerCarNode getRndCar(long seed) {
		// Get random steer car
		Random rd = new Random(seed);
		double[] rndConfig = new double[3];
		rndConfig[0] = rd.nextDouble() * world.getWidth();
		rndConfig[1] = rd.nextDouble() * world.getHeight();
		rndConfig[2] = rd.nextDouble() * Math.PI * 2;
		return new SteerCarNode(rndConfig);
	}
	
	//Expand the tree with one node according to a random car node,
	//by trying to expand the tree in 6 movements
	private SteerCarNode expandTree(SteerCarNode nearest, SteerCarNode rndCar){
		double minDis = Double.MAX_VALUE;
		SteerCarNode expandCar = null;
		for(int i = 0; i < 6; i++){
			SteerCarNode cur = nearest.moveCar(i, 1);
			if(!cur.carCollide(world)){
				double dis = cur.getDistance(rndCar);
				if(minDis > dis){
					minDis = dis;
					expandCar = cur;
				}
			}
		}
		return expandCar;
	}
	
	//Get the nearest car node given a car node, this method
	//is used by method growTree(long, int) : void
	private SteerCarNode nearestCar(SteerCarNode rndCar){
		SteerCarNode nearest = null;
		double minDis = Double.MAX_VALUE;
		for(SteerCarNode car : samples){
			double dis = rndCar.getDistance(car);
			if(minDis > dis){
				minDis = dis;
				nearest = car;
			}
		}
		return nearest;
	}

	public class SteerCarNode implements SearchNode {
		protected double[] state;
		protected double cost;

		public SteerCarNode() {
			state = new double[3];
			state[0] = 0;
			state[1] = 0;
			state[2] = 0;
			cost = 0;
		}

		public SteerCarNode(double[] config) {
			state = new double[3];
			state[0] = config[0];
			state[1] = config[1];
			state[2] = config[2];
			cost = 0;
		}
		
		public SteerCarNode(SteerCarNode cur, double c){
			double[] state = new double[3];
			state[0] = cur.getState(0);
			state[1] = cur.getState(1);
			state[2] = cur.getState(2);
			cost = c;
		}

		public double getState(int i) {
			return state[i];
		}
		
		// Get the state of the car after a movement by a certain
		// control after t time
		public SteerCarNode moveCar(int ctrl, double t) {
			double radius = Math.abs(moves[ctrl][0] / moves[ctrl][1]);
			double[] newConfig = new double[3];
			if (ctrl == 0 || ctrl == 1) {
				newConfig[0] = state[0] + t * moves[ctrl][0] * Math.cos(state[2]);
				newConfig[1] = state[1] + t * moves[ctrl][0] * Math.sin(state[1]);
				newConfig[2] = state[2];
			} else if (ctrl == 2 || ctrl == 3) {
				double[] center = new double[2];
				center[0] = state[0] - radius * Math.sin(state[2]);
				center[1] = state[1] + radius * Math.cos(state[2]);
				newConfig[2] = state[2] + t * moves[ctrl][1];
				newConfig[0] = center[0] + radius * Math.cos(newConfig[2] - Math.PI / 2);
				newConfig[1] = center[1] + radius * Math.sin(newConfig[2] - Math.PI / 2);
			} else {
				double[] center = new double[2];
				center[0] = state[0] + radius * Math.sin(state[2]);
				center[1] = state[1] - radius * Math.cos(state[2]);
				newConfig[2] = state[2] + t * moves[ctrl][1];
				newConfig[0] = center[0] + radius * Math.cos(newConfig[2] + Math.PI / 2);
				newConfig[1] = center[1] + radius * Math.sin(newConfig[2] + Math.PI / 2);
			}
			newConfig[2] = (newConfig[2] + Math.PI * 2) % (Math.PI * 2);
			return new SteerCarNode(newConfig);
		}

		// Get the path from current state according to the
		// control and the time and the step. The return is
		// a list of movement from the current by moving a
		// certain time following ctrl.
		public List<SteerCarNode> getPath(int ctrl, int time, double step) {
			List<SteerCarNode> res = new ArrayList<SteerCarNode>();
			double interval = time / step;
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
			return Math.pow(state[0] - other.getState(0), 2) 
					+ Math.pow(state[1] - other.getState(1), 2) 
					+ 100 * ang; //Weight angle more
		}
		
		//Test if the car collide with any of the obstacles
		public boolean carCollide(World w){
			List<Rectangle> obstacles = w.getObstacles();
			for(Rectangle ob : obstacles){
				if(rectIntersect(ob)) return true;
			}
			return false;
		}
		
		//Test whether this car state intersect a rectangle
		private boolean rectIntersect(Rectangle rect){
			double rect_x = rect.getCenterX();
			double rect_y = rect.getCenterY();
			if(Math.abs(state[0] - rect_x) < car_radius + rect.getWidth() &&
				Math.abs(state[1] - rect_y) < car_radius + rect.getHeight())
				return true;
			return false;
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
			for(SteerCarNode cur : rrt.get(this)){
				SteerCarNode newAdd = new SteerCarNode(cur, cur.getDistance(this));
				suc.add(newAdd);
			}
			return suc;
		}

		@Override
		public boolean goalTest() {
			// TODO Auto-generated method stub
			return getDistance(new SteerCarNode(goal)) < 10;
		}

		@Override
		public double getCost() {
			// TODO Auto-generated method stub
			return cost;
		}

		@Override
		public double getHeuristic() {
			// TODO Auto-generated method stub
			return getDistance(new SteerCarNode(goal));
		}

		@Override
		public double getPriority() {
			// TODO Auto-generated method stub
			return getHeuristic() + getCost();
		}

	}

}