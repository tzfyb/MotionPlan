package planner;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Polygon;
import java.awt.Rectangle;
import java.util.List;

import javax.swing.BorderFactory;
import javax.swing.BoxLayout;
import javax.swing.JFrame;
import javax.swing.JPanel;

import planner.ArmProblem.ArmProblemNode;
import planner.SearchProblem.SearchNode;

public class ArmProblemDriver extends JFrame implements Runnable {
	private static final long serialVersionUID = 1L;
	protected World world;
	protected ArmProblem armRobot;
	protected List<SearchNode> path;
	protected int curStep;
	protected int totalSteps;

	public static void main(String[] args) {
		ArmProblemDriver apd = new ArmProblemDriver();
		apd.run();
	}

	public ArmProblemDriver() {
		// Create the world
		world = new World(600, 600);
		world.addObs(130, 130, 100);
		world.addObs(370, 130, 100);
		world.addObs(130, 370, 100);
		world.addObs(370, 370, 100);

		// Create ArmRobot and get the path
		double[] start = { 0, 0, 0, 0 };
		double[] goal = { Math.PI, Math.PI / 12, Math.PI / 12, Math.PI / 4 };
		double[] base = { world.width / 2, world.height / 2 };
		armRobot = new ArmProblem(4, start, goal, base, world, 20, 100, 0.1, 2016);

		List<SearchNode> oriPath = armRobot.astarSearch();
		if (oriPath == null) {
			System.out.println("Path does not exist");
			System.exit(0);
		}

		path = armRobot.smoothPath(oriPath);

		// Prepare the animation step
		curStep = -1;
		totalSteps = path.size();

		// Prepare the JPanel
		MyPanel panel = new MyPanel();
		this.add(panel);

		// Set the JFrame size and title
		this.setTitle("Arm Motion Plan");
		this.getContentPane().setLayout(new BoxLayout(this.getContentPane(), BoxLayout.Y_AXIS));
		this.setSize(650, 650);
		this.setResizable(false);
		this.setLocationRelativeTo(null);
		this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		this.setVisible(true);

	}

	public class MyPanel extends JPanel {
		private static final long serialVersionUID = 1L;

		@Override
		public void paint(Graphics g) {
			super.paint(g);
			Dimension expectedDimension = new Dimension(world.width, world.height);
			this.setPreferredSize(expectedDimension);
			this.setMinimumSize(expectedDimension);
			this.setMaximumSize(expectedDimension);
			this.setBorder(BorderFactory.createLineBorder(Color.BLACK));
			this.setBackground(Color.gray);

			// Draw the world
			g.setColor(Color.BLACK);
			for (Rectangle rect : world.getObstacles()) {
				g.fillRect(rect.x, rect.y, rect.width, rect.height);
			}
			List<Polygon> arms = null;
			//Draw the start and the goal
			g.setColor(Color.BLUE);
			arms = armRobot.getStart().getAllPoly(true);
			for(Polygon arm : arms)
				g.drawPolygon(arm);
			g.setColor(Color.ORANGE);
			arms = armRobot.getGoal().getAllPoly(true);
			for(Polygon arm : arms)
				g.drawPolygon(arm);
			//Draw the path
			arms = ((ArmProblemNode) (path.get(curStep))).getAllPoly(true);

			if (((ArmProblemNode) (path.get(curStep))).armCollision(world))
				g.setColor(Color.RED);
			else
				g.setColor(Color.LIGHT_GRAY);
			
			for (Polygon arm : arms) {
				g.drawPolygon(arm);
			}
			g.setColor(Color.RED);
			g.drawOval(world.width / 2 - 3, world.height / 2 - 3, 6, 6);
		}

	}

	@Override
	public void run() {
		// TODO Auto-generated method stub
		while (true) {
			curStep = (curStep + 1) % totalSteps;
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			System.out.println("Step: " + Integer.toString(curStep) + "/" + Integer.toString(path.size()));
			if (((ArmProblemNode) (path.get(curStep))).armCollision(world))
				System.out.println("Intersect!");
			System.out.println(((ArmProblemNode) (path.get(curStep))).toString());

			repaint();
		}
	}
}
