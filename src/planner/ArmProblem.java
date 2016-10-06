package planner;

import java.awt.Polygon;
import java.awt.Rectangle;
import java.awt.geom.Area;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Random;

import planner.ArmProblem.ArmProblemNode;

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
	// World grid
	protected World world;
	protected int k_neigh; // number of neighbors
	protected int density; // number of sample points to be generated
	protected HashSet<ArmProblemNode> samples;
	protected HashMap<ArmProblemNode, HashSet<ArmProblemNode>> adjList;

	public ArmProblem(int num, double[] s, double[] g, double[] link_length, World w, int k, int d) {
		base = new double[2];
		base[0] = 0;
		base[1] = 0;
		link_len = link_length;
		width = 10;
		start_config = s;
		goal_config = g;
		world = w;
		k_neigh = k;
		samples = new HashSet<ArmProblemNode>();
		adjList = new HashMap<>();
		density = d;
	}

	// sampling the points in configuration space
	public void sampling(long seed) {
		Random rd = new Random(seed);
		samples.add(new ArmProblemNode(start_config));
		samples.add(new ArmProblemNode(goal_config));
		while (samples.size() < density + 2) {
			double[] randConfig = new double[link_num];
			for (int i = 0; i < link_num; i++)
				randConfig[i] = Math.PI * rd.nextDouble();
			ArmProblemNode cur = new ArmProblemNode(randConfig);
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
		for (ArmProblemNode arm1 : samples) {
			for (int i = 0; i < link_num; i++)
				goal_config[i] = arm1.getConfig(i);
			PriorityQueue<ArmProblemNode> candidtae_neighbours = new PriorityQueue<ArmProblemNode>();
			for (ArmProblemNode arm2 : samples) {
				if (!arm1.equals(arm2)) {
					if (!arm1.armPathCollision(arm2, world, 0.5))
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
		}
		// restore the goal_config
		for (int i = 0; i < link_num; i++)
			goal_config[i] = real_goal[i];
	}

	public class ArmProblemNode implements SearchNode {
		private double[] config;
		private double cost;

		public ArmProblemNode(double[] configuration) {
			config = configuration;
			cost = 0;
		}

		public ArmProblemNode(double[] configuration, double c) {
			config = configuration;
			cost = c;
		}

		public int getLinkNum() {
			return link_num;
		}

		// get the Manhattan distance of two angles
		private double manhattan(double ang1, double ang2) {
			double res = Math.abs(ang1 - ang2);
			if (res > Math.PI)
				return 2 * Math.PI - res;
			else
				return res;
		}

		public double getConfig(int i) {
			return config[i];
		}

		public double[] getConfig() {
			return config;
		}

		public void setConfig(int i, double ang) {
			config[i] = ang;
		}

		// Get the rectangle (coordinates from four vertices)
		public double[][] getRec(int i) {
			double[][] rect = new double[4][2];

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
			rect[0][0] = x_next + width / 2 * Math.cos(ang + Math.PI / 2);
			rect[0][1] = y_next + width / 2 * Math.sin(ang + Math.PI / 2);
			rect[1][0] = x_next + width / 2 * Math.cos(ang + Math.PI * 1.5);
			rect[1][1] = x_next + width / 2 * Math.sin(ang + Math.PI * 1.5);
			rect[2][0] = x + width / 2 * Math.cos(ang + Math.PI * 1.5);
			rect[2][1] = x + width / 2 * Math.sin(ang + Math.PI * 1.5);
			rect[3][0] = x + width / 2 * Math.cos(ang + Math.PI / 2);
			rect[3][1] = y + width / 2 * Math.sin(ang + Math.PI / 2);

			return rect;
		}

		// check if the arm is collide with any of the obstacle
		public boolean armCollision(World world) {
			ArrayList<Area> links = new ArrayList<Area>();
			for (int i = 0; i < link_num; i++) {
				Polygon cur = new Polygon();
				double[][] rect = getRec(i);
				for (int j = 0; j < 4; j++)
					cur.addPoint((int) Math.round(rect[j][0]), (int) Math.round(rect[j][1]));		
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
		public boolean armPathCollision(ArmProblemNode other, World world, double step_ratio) {
			Double[] vol = armLocalPlanner(other, step_ratio);
			double time = (other.getConfig(0) - config[0]) / vol[0];
			ArmProblemNode test = new ArmProblemNode(config);
			double curTime = 0;
			while (curTime < time) {
				if (test.armCollision(world))
					return true;
				for (int i = 0; i < link_num; i++)
					test.setConfig(i, test.getConfig(i) + vol[i]);
				curTime += step_ratio;
			}
			return false;
		}

		// The local planner takes the maximum movement among all the links
		// and return the velocity of each link with the maximum velocity
		// = 1 PI per step, where the step is determined by the step size. The
		// less
		// the step ratio is, the more accuracy the local planner will be
		private Double[] armLocalPlanner(ArmProblemNode other, double step_ratio) {
			double max_move = 0;
			for (int i = 0; i < link_num; i++)
				max_move = Math.max(max_move, Math.abs(config[i] - other.getConfig(i)));
			Double[] v = new Double[link_num];
			for (int i = 0; i < link_num; i++)
				v[i] = (other.getConfig(i) - config[i]) / max_move * step_ratio;
			return v;
		}

		@Override
		public int compareTo(SearchNode arg0) {
			// TODO Auto-generated method stub
			return (int) Math.signum(getPriority() - arg0.getPriority());
		}

		@Override
		public boolean equals(Object o) {
			double res = 0;
			for (int i = 0; i < link_num; i++)
				res += manhattan(config[i], ((ArmProblemNode) o).getConfig(i));
			return res < 1;
		}

		@Override
		public int hashCode() {
			return config.hashCode();
		}

		@Override
		public ArrayList<SearchNode> getSuccessors() {
			// TODO Auto-generated method stub
			ArrayList<SearchNode> suc = new ArrayList<SearchNode>();
			for(ArmProblemNode apn : adjList.get(this)){
				double suc_cost = cost;
				for(int i = 0; i < link_num; i++)
					suc_cost += Math.abs(apn.getConfig(i) - config[i]);
				ArmProblemNode cur = new ArmProblemNode(apn.getConfig(), suc_cost + cost);
				suc.add(cur);
			}
			return suc;
		}

		@Override
		public boolean goalTest() {
			// TODO Auto-generated method stub
			double res = 0;
			for (int i = 0; i < link_num; i++)
				res += manhattan(config[i], goal_config[i]);
			return res < 1;
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
}
