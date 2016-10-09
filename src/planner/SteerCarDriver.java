package planner;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Rectangle;
import java.util.List;

import javax.swing.BorderFactory;
import javax.swing.BoxLayout;
import javax.swing.JFrame;
import javax.swing.JPanel;

import planner.SearchProblem.SearchNode;
import planner.SteerCarProblem.SteerCarNode;

public class SteerCarDriver extends JFrame implements Runnable {
	private static final long serialVersionUID = 5248652917537143562L;

	protected World world;
	protected SteerCarProblem carRobot;
	protected List<SteerCarNode> path;
	//protected List<SearchNode> path;
	protected int curStep;
	protected int totalSteps;

	public static void main(String[] args) {
		SteerCarDriver scd = new SteerCarDriver();
		scd.run();
	}
	
	public SteerCarDriver() {
		// Create the world
		world = new World(600, 600);
		world.addWall(60, 0, 540, 30);
		world.addWall(0, 275, 540, 30);
		world.addWall(60, 570, 540, 30);
		world.addObs(80, 80, 40);
		world.addObs(200, 80, 60);
		world.addObs(380, 80, 100);
		world.addObs(80, 400, 40);
		world.addWall(350, 400, 250, 40);

		// Create ArmRobot and get the path
		double[] start = {15, 15, 0};
		double[] goal = {30, 570, 0};
		carRobot = new SteerCarProblem(world, start, goal, 10000, 2016, 3, 10);
		//path = carRobot.astarSearch();
		
		List<SearchNode> oriPath = carRobot.astarSearch();
		if (oriPath == null) {
			System.out.println("Path does not exist");
			System.exit(0);
		}
		
		path = carRobot.smoothPath(oriPath, 0.01);
		
		
		//path = new ArrayList<SearchNode>();
		//path.add(carRobot.getStart());
		//path.add(carRobot.getStart().moveCar(3, 3));
		//path.addAll(carRobot.getStart().getPath(1, 6, 0.01));
		//path.add(carRobot.getGoal());

		curStep = -1;
		totalSteps = path.size();

		// Prepare the JPanel
		MyPanel panel = new MyPanel();
		this.add(panel);

		// Set the jframe size and title
		this.setTitle("Steer Car Motion Plan");
		this.getContentPane().setLayout(new BoxLayout(this.getContentPane(), BoxLayout.Y_AXIS));
		this.setSize(650, 650);
		this.setResizable(false);
		this.setLocationRelativeTo(null);
		this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		this.setVisible(true);
	}
	
	public class MyPanel extends JPanel {
		private static final long serialVersionUID = 3235482271056001085L;

		@Override
		public void paint(Graphics g) {
			super.paint(g);
			Dimension expectedDimension = new Dimension(world.width, world.height);
			this.setPreferredSize(expectedDimension);
			this.setMinimumSize(expectedDimension);
			this.setMaximumSize(expectedDimension);
			this.setBorder(BorderFactory.createLineBorder(Color.BLACK));
			this.setBackground(Color.LIGHT_GRAY);

			g.setColor(Color.BLACK);
			for (Rectangle rect : world.getObstacles()) {
				g.fillRect(rect.x, rect.y, rect.width, rect.height);
			}

			//List<Polygon> arms = ((ArmProblemNode)(path.get(curStep))).getAllPoly(true);
			
			if(((SteerCarNode)(path.get(curStep))).carCollide(world))
				g.setColor(Color.RED);
			else
				g.setColor(Color.CYAN);
			
			//Get cur car node
			SteerCarNode cur = (SteerCarNode)(path.get(curStep));
			//Get car center coordinate
			double carX = cur.getState(0);
			double carY = cur.getState(1);
			//Draw the car cycle
			double ovalX = carX - carRobot.getRadius();
			double ovalY = carY - carRobot.getRadius();
			double ovalWidth = carRobot.getRadius() * 2;
			g.fillOval((int)ovalX, (int)ovalY, (int)ovalWidth, (int)ovalWidth);
			
			//Draw the center point
			ovalX = cur.getState(0) - 3;
			ovalY = cur.getState(1) - 3;
			g.setColor(Color.RED);
			g.drawOval((int)(ovalX), (int)(ovalY), 6, 6);
			
			//Draw the direction indicator
			double dirX = carX + carRobot.car_radius * Math.cos(cur.getState(2));
			double dirY = carY + carRobot.car_radius * Math.sin(cur.getState(2));
			g.fillOval((int)dirX - 2, (int)dirY - 2, 4, 4);
		}

	}

	@Override
	public void run() {
		// TODO Auto-generated method stub
		while (true) {
			curStep = (curStep + 1) % totalSteps;
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			System.out.println("Step: " + Integer.toString(curStep) + "/" + Integer.toString(path.size()));
			if(((SteerCarNode)(path.get(curStep))).carCollide(world))
				System.out.println("Intersect!");
			System.out.println(((SteerCarNode)(path.get(curStep))).toString());

			repaint();
		}
	}

}
