package planner;

import java.awt.Polygon;
import java.awt.Rectangle;
import java.awt.geom.Area;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Random;

public class ArmProblem extends SearchProblem {
	// base x, y
	protected double[] base;
	// links' length
	protected double[] link_len;
	// number of links
	protected int link_num;
	// width of link
	protected double width;
	// links' angles
	protected double[] start_config;
	// Goal
	protected double[] goal_config;
	// step ratio, how accuracy the collision test will be
	protected double step_ratio;
	// World grid
	protected World world;
	protected int k_neigh; // number of neighbors
	protected int density; // number of sample points to be generated
	protected HashSet<ArmProblemNode> samples;
	protected HashMap<ArmProblemNode, HashSet<ArmProblemNode>> adjList;

	public ArmProblem(int num, double[] s, double[] g, double[] b, 
			World w, int k, int d, double sr, long seed) {
		base = new double[2];
		base[0] = b[0];
		base[1] = b[1];
		width = 10;
		link_num = num;
		// set start and goal configuration
		start_config = new double[link_num];
		for (int i = 0; i < link_num; i++)
			start_config[i] = s[i];
		goal_config = new double[link_num];
		for (int i = 0; i < link_num; i++)
			goal_config[i] = g[i];
		// set link length
		link_len = new double[link_num];
		for (int i = 0; i < link_num; i++)
			link_len[i] = 30;
		world = w;
		k_neigh = k;
		samples = new HashSet<ArmProblemNode>();
		adjList = new HashMap<>();
		density = d;
		startNode = new ArmProblemNode(start_config, 0, this);
		step_ratio = sr;

		sampling(seed);
		System.out.println("Sampling Done");
		System.out.println("Constructing Map...");
		constructMap();
	}

	// sampling the points in configuration space
	public void sampling(long seed) {
		Random rd = new Random(seed);
		samples.add(new ArmProblemNode(start_config, this));
		samples.add(new ArmProblemNode(goal_config, this));
		double[] randConfig = new double[link_num];
		while (samples.size() < density + 2) {
			for (int i = 0; i < link_num; i++)
				randConfig[i] = Math.PI * 2 * rd.nextDouble();
			ArmProblemNode cur = new ArmProblemNode(randConfig, this);
			if (!cur.armCollision(world) && !samples.contains(cur))
				samples.add(cur);
		}
	}

	// use the local planner to link the points in configuration space
	public void constructMap() {
		// To store the goal configuration
		double[] real_goal = new double[link_num];
		for (int i = 0; i < link_num; i++)
			real_goal[i] = goal_config[i];
		// Then I can use the compareTo function to get the k nearest points
		System.out.println("Total Nodes Construct: " + Integer.toString(samples.size()));
		int curNum = 0;
		for (ArmProblemNode arm1 : samples) {
			for (int i = 0; i < link_num; i++)
				goal_config[i] = arm1.getConfig(i);
			PriorityQueue<ArmProblemNode> candidtae_neighbours = new PriorityQueue<ArmProblemNode>();
			for (ArmProblemNode arm2 : samples) {
				if (!arm1.equals(arm2)) {
					if (!arm1.armPathCollision(arm2, world))
						candidtae_neighbours.add(arm2);
				}
			}

			HashSet<ArmProblemNode> neighbours = new HashSet<ArmProblemNode>();
			for (int i = 0; i < k_neigh; i++) {
				if (candidtae_neighbours.peek() != null)
					neighbours.add(candidtae_neighbours.poll());
				else
					break;
			}
			adjList.put(arm1, neighbours);
			curNum++;
			System.out.println(Integer.toString(curNum) + "/" + Integer.toString(samples.size()) + " Done");
		}
		// restore the goal_config
		for (int i = 0; i < link_num; i++)
			goal_config[i] = real_goal[i];
	}

	public List<SearchNode> smoothPath(List<SearchNode> path) {
		List<SearchNode> res = new ArrayList<SearchNode>();
		res.add((ArmProblemNode) (path.get(0)));
		for (int i = 0; i < path.size() - 1; i++) {
			ArmProblemNode cur = (ArmProblemNode) (path.get(i));
			ArmProblemNode next = (ArmProblemNode) (path.get(i + 1));
			res.addAll(cur.localPath(next));
		}
		res.add(path.get(path.size() - 1));
		return res;
	}

	public class ArmProblemNode implements SearchNode {
		private double[] config;
		private double cost;
		ArmProblem armRobot;

		public ArmProblemNode(double[] configuration, ArmProblem _armRobot) {
			config = new double[link_num];
			for (int i = 0; i < link_num; i++)
				config[i] = configuration[i];
			cost = 0;
			armRobot = _armRobot;
		}

		public ArmProblemNode(double[] configuration, double c, ArmProblem _armRobot) {
			config = new double[link_num];
			for (int i = 0; i < link_num; i++)
				config[i] = configuration[i];
			cost = c;
			armRobot = _armRobot;
		}

		public int getLinkNum() {
			return link_num;
		}

		public String toString() {
			StringBuilder res = new StringBuilder();
			res.append('[');
			for (int i = 0; i < link_num - 1; i++)
				res.append(Double.toString(config[i]) + ", ");
			res.append(Double.toString(config[link_num - 1]));
			res.append(']');
			return res.toString();

		}

		// get the Manhattan distance of two angles
		private double manhattan(double ang1, double ang2) {
			double res = Math.abs(ang1 - ang2);
			return res % (Math.PI * 2);
		}

		public double getConfig(int i) {
			return config[i];
		}

		public double[] getConfig() {
			double[] retConfig = new double[link_num];
			for (int i = 0; i < link_num; i++)
				retConfig[i] = config[i];
			return retConfig;
		}

		public void setConfig(int i, double ang) {
			config[i] = ang;
		}

		// Get the rectangle (coordinates from four vertices)
		// the flag 'transform' determine whether we need to
		// transform the coordinate system to a JPanel 2D
		// coordinate system
		public Polygon getRec(int i, boolean transform) {

			double x = base[0];
			double y = base[1];
			double ang = 0;
			// the first link point of the link
			for (int j = 0; j < i; j++) {
				ang = (ang + config[j]) % (2 * Math.PI);
				x = x + link_len[j] * Math.cos(ang);
				y = y + link_len[j] * Math.sin(ang);
			}
			// the second link point of the link
			ang = (ang + config[i]) % (2 * Math.PI);
			double x_next = x + link_len[i] * Math.cos(ang);
			double y_next = y + link_len[i] * Math.sin(ang);

			// Get the rectangles four vertices' coordinates, we can think of
			// the edge as another arm with length = width / 2. The order of the
			// vertices are counter-clockwise, which are, top-left, top-right,
			// bottom-right and bottom left of the rectangle.
			int[] xpoints = new int[4];
			int[] ypoints = new int[4];
			xpoints[0] = (int) (x_next + width / 2 * Math.cos(ang + Math.PI / 2));
			ypoints[0] = (int) (y_next + width / 2 * Math.sin(ang + Math.PI / 2));
			xpoints[1] = (int) (x_next + width / 2 * Math.cos(ang + Math.PI * 1.5));
			ypoints[1] = (int) (y_next + width / 2 * Math.sin(ang + Math.PI * 1.5));
			xpoints[2] = (int) (x + width / 2 * Math.cos(ang + Math.PI * 1.5));
			ypoints[2] = (int) (y + width / 2 * Math.sin(ang + Math.PI * 1.5));
			xpoints[3] = (int) (x + width / 2 * Math.cos(ang + Math.PI / 2));
			ypoints[3] = (int) (y + width / 2 * Math.sin(ang + Math.PI / 2));

			if (transform) {
				for (int j = 0; j < 4; j++)
					ypoints[j] = (int) base[1] - (ypoints[j] - (int) base[1]);
			}
			return new Polygon(xpoints, ypoints, 4);
		}

		// Get all the link polygons
		List<Polygon> getAllPoly(boolean transform) {
			List<Polygon> res = new ArrayList<Polygon>();
			for (int i = 0; i < link_num; i++) {
				res.add(getRec(i, transform));
			}
			return res;
		}

		// check if the arm is collide with any of the obstacle
		public boolean armCollision(World world) {
			ArrayList<Area> links = new ArrayList<Area>();
			for (int i = 0; i < link_num; i++) {
				Polygon cur = getRec(i, true);
				links.add(new Area(cur));
			}
			List<Rectangle> obstacles = world.getObstacles();
			for (Rectangle obstacle : obstacles) {
				for (Area link : links) {
					if (link.intersects(obstacle))
						return true;
				}
			}
			return false;
		}

		// check if the path between two configuration states is in collision
		// with any of the obstacle. The main idea is to divide the path
		// into small intermediate steps and check each step's collision
		public boolean armPathCollision(ArmProblemNode other, World world) {
			Double[] vol = armLocalPlanner(other);
			double time = (other.getConfig(0) - config[0]) / vol[0];
			ArmProblemNode test = new ArmProblemNode(config, armRobot);
			double curTime = 0;
			while (curTime < time) {
				if (test.armCollision(world))
					return true;
				for (int i = 0; i < link_num; i++)
					test.setConfig(i, test.getConfig(i) + vol[i]);
				curTime += armRobot.getStepRatio();
			}
			return false;
		}

		// The local planner takes the maximum movement among all the links
		// and return the velocity of each link with the maximum velocity
		// = 1 PI per step, where the step is determined by the step size. The
		// less
		// the step ratio is, the more accuracy the local planner will be
		private Double[] armLocalPlanner(ArmProblemNode other) {
			double max_move = 0;
			for (int i = 0; i < link_num; i++)
				max_move = Math.max(max_move, Math.abs(config[i] - other.getConfig(i)));
			Double[] v = new Double[link_num];
			for (int i = 0; i < link_num; i++)
				v[i] = (other.getConfig(i) - config[i]) / max_move * armRobot.getStepRatio();
			return v;
		}

		// Local Path planner to get the intermediate steps between
		// two configurations
		public List<ArmProblemNode> localPath(ArmProblemNode other) {
			Double[] vol = armLocalPlanner(other);
			List<ArmProblemNode> locPath = new ArrayList<ArmProblemNode>();
			double[] nextConfig = new double[armRobot.getLinkNum()];
			for (int i = 0; i < armRobot.getLinkNum(); i++)
				nextConfig[i] = this.getConfig(i);
			double time = (other.getConfig(0) - config[0]) / vol[0];
			for (int curTime = 1; curTime < time; curTime++) {
				for (int i = 0; i < armRobot.getLinkNum(); i++)
					nextConfig[i] += vol[i];
				locPath.add(new ArmProblemNode(nextConfig, armRobot));
			}
			locPath.add(other);
			return locPath;
		}

		@Override
		public int compareTo(SearchNode arg0) {
			// TODO Auto-generated method stub
			return (int) Math.signum(getPriority() - arg0.getPriority());
		}

		@Override
		public boolean equals(Object o) {
			for (int i = 0; i < link_num; i++)
				if (config[i] != ((ArmProblemNode) o).getConfig(i))
					return false;
			return true;
		}

		@Override
		public int hashCode() {
			int code = 0;
			for (int i = 0; i < link_num; i++)
				code += Math.pow(config[i], Math.pow(37, i));
			return code;
		}

		@Override
		public ArrayList<SearchNode> getSuccessors() {
			// TODO Auto-generated method stub
			ArrayList<SearchNode> suc = new ArrayList<SearchNode>();
			HashMap<ArmProblemNode, HashSet<ArmProblemNode>> tree = armRobot.getAdjList();
			HashSet<ArmProblemNode> adjNodes = tree.get(this);
			for (ArmProblemNode apn : adjNodes) {
				double suc_cost = cost;
				for (int i = 0; i < link_num; i++)
					suc_cost += manhattan(apn.getConfig(i), config[i]);
				ArmProblemNode cur = new ArmProblemNode(apn.getConfig(), suc_cost, armRobot);
				suc.add(cur);
			}
			return suc;
		}

		@Override
		public boolean goalTest() {
			// TODO Auto-generated method stub
			double res = 0;
			for (int i = 0; i < link_num; i++)
				res += manhattan(config[i], armRobot.getGoal().getConfig(i));
			return res < 0.1;
		}

		@Override
		public double getCost() {
			// TODO Auto-generated method stub
			return cost;
		}

		@Override
		public double getHeuristic() {
			// TODO Auto-generated method stub
			double res = 0;
			for (int i = 0; i < link_num; i++)
				res += manhattan(config[i], goal_config[i]);
			return res;
		}

		@Override
		public double getPriority() {
			// TODO Auto-generated method stub
			return getHeuristic() + getCost();
		}

	}

	public void setBase(double x, double y) {
		base[0] = x;
		base[1] = y;
	}

	public void setWidth(double w) {
		width = w;
	}

	public ArmProblemNode getStart() {
		return new ArmProblemNode(start_config, this);
	}

	public ArmProblemNode getGoal() {
		return new ArmProblemNode(goal_config, this);
	}

	public HashMap<ArmProblemNode, HashSet<ArmProblemNode>> getAdjList() {
		return adjList;
	}

	public double getStepRatio() {
		return step_ratio;
	}

	public int getLinkNum() {
		return link_num;
	}
}
