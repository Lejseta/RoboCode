package sample;
import robocode.*;

import javax.xml.crypto.Data;

import static robocode.util.Utils.normalRelativeAngleDegrees;
import java.awt.Color;
import java.util.*;
import java.awt.geom.*;
import java.util.EventListener;
//import java.awt.Color;

/**
 * Experimental Robot - a robot by Mirella Glowinska
 */
public class ExperimentalRobot extends Robot
{

	int turnDirection = 1; // Clockwise or counterclockwise
	boolean shooting = false;
	boolean controlSide = true;
	boolean peek;
	double moveAmount; // How much to move
	double bearingFromGun = 0;
	double bearingFromRadar = 0;
	int curDist;

	//prediction
	double predictedY, predictedX;
	//The averaged velocity.
	double velocityToAimAt;
	// Enemy
	DataAnalyzer analyzer;
	int targetIndex;
	boolean focused; //that is, can it change the target. false - no. true- yes

	//Events
	public void run() {
		moveAmount = Math.max(getBattleFieldWidth(), getBattleFieldHeight());
		peek = false;
		analyzer = new DataAnalyzer(getOthers());

		// setColors(Color.red,Color.blue,Color.green); // body,gun,radar
		setBodyColor(new Color(0, 0, 0));
		setGunColor(new Color(183, 128, 224));
		setRadarColor(new Color(115, 117, 245));
		setBulletColor(new Color(247, 10, 46));
		setScanColor(new Color(255, 200, 200));

		scan();
		// Robot main loop
		while (true) {
			turnRadarRight(Double.POSITIVE_INFINITY);
		}
	}

	/**
	 * onScannedRobot: What to do when you see another robot
	 */
	public void onScannedRobot(ScannedRobotEvent e) {

		int robotIndex = analyzer.registerRobot(e, getTime());
		int closestIndex = analyzer.getClosestRobotIndex(getTime());
		out.println(analyzer.enemyFiredProbability(closestIndex));

		if(shooting)
		{
			if(analyzer.enemyFiredProbability(closestIndex) > 0.5d) {
				moveInLineRandomly(60);
			}
			shooting = false;
			shotNow(e);
		} else {
			double max = analyzer.getMaximumVelocity(closestIndex, -1);
			double avg = analyzer.getAverageVelocity(closestIndex, -1);
			double ang = analyzer.getMostCommonBearing(closestIndex, -1);

			out.println("Robot " + analyzer.getLog(closestIndex).getRobotName() + ": " + max + ", " + avg + "," + ang);
			targetEnemy(closestIndex);
			moveInLineRandomly(40);

			if (e.getBearing() > 0)
				myTurnRight(-(e.getBearing() - 90));
			else
				myTurnLeft(-(e.getBearing() + 90));
			shooting = true;
		}
	}

	public void onHitRobot(HitRobotEvent e) {
		rotateToward(e);
		fire(3);
		if(e.getEnergy()<getEnergy())
		{
			myTurnRight(e.getBearing());//turn body towards it
			myMoveForward(40);//ram at it
			fire(3);

			curDist=0;//we hit the robot so distance is 0
		} else {
			myTurnRight(normalRelativeAngleDegrees(e.getBearing()-90));//if it has more energy than us
			moveInLineRandomly(100);
		}
	}

	public void onBulletHit(BulletHitEvent event) {
		if(Math.random() > 0.5)
			moveInLineRandomly(25);
		//when enemy losses energy, he probably fired a bullet. / ==  ((4 * bullet power) + (max(bullet power - 1, 0) * 2)).
		double enemyLostInEnergy = robocode.Rules.getBulletDamage(event.getBullet().getPower());
		out.println("Enemy lost in energy: " + enemyLostInEnergy);
		analyzer.registerBulletHit(getTime(), event);
	}

	public void onBulletHitBullet(BulletHitBulletEvent event) {
		moveInLineRandomly(35);
	}

	/**
	 * onHitByBullet: What to do when you're hit by a bullet
	 */
	public void onHitByBullet(HitByBulletEvent e) {
		moveInLineRandomly(50);
		double enemyGainInEnergy = robocode.Rules.getBulletHitBonus(e.getBullet().getPower());
	}

	/**
	 * onHitWall: What to do when you hit a wall
	 */
	public void onHitWall(HitWallEvent e)
	{
		Directions dir = recognizeAngle(e.getBearing());
		switch (dir)
		{
			case top:
				out.println("top");
				controlSide = false;
				break;
			case right:
				out.println("right");
				myTurnLeft(-(e.getBearing() - 90)); // to change
				break;
			case left:
				out.println("left");
				myTurnRight(-(e.getBearing() + 90)); // to change
				break;
			case bottom:
				out.println("back");
				controlSide = true;
				break;
		}
	}

	public void onRobotDeath(RobotDeathEvent e) {
		analyzer.clearIndexData(analyzer.getRobotIndexByName(e.getName()));
	}

	public void onDeath(DeathEvent e) {
		analyzer.cleanAllData();
	}

	void targetEnemy(int i) {
		if (getGunHeat() != 0) return;
		if (analyzer.targetsData[i].size() == 0) return;
		targetIndex = i;
	}

	void shotTarget(ScannedRobotEvent robot) {
		rotateToward(robot);

		fire(calculateFirePower(robot.getDistance()));
	}

	void rotateToward(ScannedRobotEvent e) {
		double gunTurnAmt = normalRelativeAngleDegrees(e.getBearing() + (getHeading() - getGunHeading()));
		turnGunRight(gunTurnAmt);
	}

	void rotateToward(HitRobotEvent e) {
		double gunTurnAmt = normalRelativeAngleDegrees(e.getBearing() + (getHeading() - getGunHeading()));
		turnGunRight(gunTurnAmt);
	}

	void moveInLineRandomly(double units) {
		if(Math.random() > 0.5)
			myMoveForward(units);
		else
			myMoveBack(units);
	}

	void myMoveForward(double units) {
		if(controlSide)
			ahead(units);
		else
			back(units);
	}

	void myMoveBack(double units) {
		if(controlSide)
			back(units);
		else
			ahead(units);
	}

	void myTurnRight(double angle) {
		if(controlSide)
			turnRight(angle);
		else
			turnLeft(angle);
	}

	void myTurnLeft(double angle) {
		if(controlSide)
			turnLeft(angle);
		else
			turnRight(angle);
	}
	void shotNow(ScannedRobotEvent e)
	{
		int closestIndex = analyzer.getClosestRobotIndex(getTime());
		double ang = analyzer.getMostCommonBearing(closestIndex, -1);
		// calculate firepower based on distance
		double firePower = Math.min(500 / e.getDistance(), 3);
		// distance = rate * time, v = s/t -> t = s/t
		double time = (e.getDistance() / bulletVelocity(firePower));

		double enemyY = getY() + e.getDistance() * Math.cos(Math.toRadians(getHeading() + e.getBearing()));
		double enemyX = getX() + e.getDistance() * Math.sin(Math.toRadians(getHeading() + e.getBearing()));

		//prediction
		double futureX = prospectiveTimeX(time, e.getHeading(), enemyX, e.getVelocity());
		out.println("Current X: " + enemyX + ". Future X: " + futureX);
		double futureY = prospectiveTimeY(time, e.getHeading(), enemyY, e.getVelocity());
		out.println("Current Y: " + enemyY + ". Future Y: " + futureY);
		/*
		int firstEnemy =  cos;
		if(analyzer.predictEnemyMovement(index) == Movement.descending) {
			double futureX = prospectiveTimeX(time, e.getHeading(), enemyX, analyzer.getAverageVelocity(index));
			out.println("Current X: " + enemyX + ". Future X: " + futureX);
			double futureY = prospectiveTimeY(time, e.getHeading(), enemyY, analyzer.getAverageVelocity(index));
			out.println("Current Y: " + enemyY + ". Future Y: " + futureY);

			double absDeg = enemyBearing(getX(), getY(), futureX, futureY);
			out.println("Bearing: " + normalizeBearing(absDeg - getGunHeading()));
			// turn the gun to the predicted x,y location
			turnGunRight(normalizeBearing(absDeg - getGunHeading()));
			//out.println(absDeg + " - " + getGunHeading() + " = " + (absDeg - getGunHeading()) + ". Normalizacja: " + normalizeBearing(absDeg - getGunHeading()));

			double remainingAngle = e.getBearing() + (getHeading() - getGunHeading());
			if (getGunHeat() == 0 && remainingAngle < 10) {
				fire(calculateFirePower(e.getDistance()));
			}
		} else if (analyzer.predictEnemyMovement(index) == Movement.ascending) {
			double futureX = prospectiveTimeX(time, e.getHeading(), enemyX, e. getVelocity());
			out.println("Current X: " + enemyX + ". Future X: " + futureX);
			double futureY = prospectiveTimeY(time, e.getHeading(), enemyY, e. getVelocity());
			out.println("Current Y: " + enemyY + ". Future Y: " + futureY);

			double absDeg = enemyBearing(getX(), getY(), futureX, futureY);
			out.println("Bearing: " + normalizeBearing(absDeg - getGunHeading()));
			// turn the gun to the predicted x,y location
			turnGunRight(normalizeBearing(absDeg - getGunHeading()));
			//out.println(absDeg + " - " + getGunHeading() + " = " + (absDeg - getGunHeading()) + ". Normalizacja: " + normalizeBearing(absDeg - getGunHeading()));

			double remainingAngle = e.getBearing() + (getHeading() - getGunHeading());
			if (getGunHeat() == 0 && remainingAngle < 10) {
				fire(calculateFirePower(e.getDistance()));
			}
		} else {
			double futureX = prospectiveTimeX(time, e.getHeading(), enemyX, e.getVelocity()));
			out.println("Current X: " + enemyX + ". Future X: " + futureX);
			double futureY = prospectiveTimeY(time, e.getHeading(), enemyY, e.getVelocity());
			out.println("Current Y: " + enemyY + ". Future Y: " + futureY);

			double absDeg = enemyBearing(getX(), getY(), futureX, futureY);
			out.println("Bearing: " + normalizeBearing(absDeg - getGunHeading()));
			// turn the gun to the predicted x,y location
			turnGunRight(normalizeBearing(absDeg - getGunHeading()));
			//out.println(absDeg + " - " + getGunHeading() + " = " + (absDeg - getGunHeading()) + ". Normalizacja: " + normalizeBearing(absDeg - getGunHeading()));

			double remainingAngle = e.getBearing() + (getHeading() - getGunHeading());
			if (getGunHeat() == 0 && remainingAngle < 10) {
				fire(calculateFirePower(e.getDistance()));
			}
		}
 			*/
		double absDeg = enemyBearing(getX(), getY(), futureX, futureY);
		out.println("Bearing: " + normalizeBearing(absDeg - getGunHeading()));
		// turn the gun to the predicted x,y location
		turnGunRight(normalizeBearing(absDeg - getGunHeading()));
		//out.println(absDeg + " - " + getGunHeading() + " = " + (absDeg - getGunHeading()) + ". Normalizacja: " + normalizeBearing(absDeg - getGunHeading()));

		double remainingAngle = e.getBearing() + (getHeading() - getGunHeading());
		if (getGunHeat() == 0 && remainingAngle < 10){
			fire(calculateFirePower(e.getDistance()));
		}
	}
	//absolute enemy bearing
	double enemyBearing(double currentX, double currentY, double futureX, double futureY)
	{
		// X-axis distance and Y-axis distance
		double DifferenceX = futureX - currentX;
		double DifferenceY = futureY - currentY;
		//soh cah toa
		double hyp = Point2D.distance(currentX, currentY, futureX, futureY);
		//s= opp/hyp, We're calculating angle
		double arcAngle = Math.toDegrees(Math.asin(DifferenceX / hyp));
		// initializing bearing
		double bearing = 0;
		//Location and actual angle
		if (DifferenceX > 0 && DifferenceY > 0) { // both pos: lower-Left
			bearing = arcAngle;
		} else if (DifferenceX < 0 && DifferenceY > 0) { // x neg, y pos: lower-right
			bearing = 360 + arcAngle; // arcsin is negative here, actually 360 - ang
		} else if (DifferenceX > 0 && DifferenceY < 0) { // x pos, y neg: upper-left
			bearing = 180 - arcAngle;
		} else if (DifferenceX < 0 && DifferenceY < 0) { // both neg: upper-right
			bearing = 180 - arcAngle; // arcsin is negative here, actually 180 + ang
		}
		return bearing;
	}

	// normalizes a bearing to between +180 and -180
	static double normalizeBearing(double angle) {
		while (angle > 180)
			angle -= 360;
		while (angle < -180)
			angle += 360;
		return angle;
	}

	// Tools
	static double calculateFirePower(double distance) {
		return 400 / distance;
	}

	static double maxEscapeAngle(double bulletSpeed) {
		return Math.asin(8 / bulletSpeed);
	}
	// calculate speed of bullet
	static double bulletVelocity(double power) {
		return 20d - (3d * power);
	}

	static double limit(double val, double min, double max) {
		return Math.max(min, Math.min(max, val));
	}

	static Directions recognizeAngle(double angle) {
		if(angle > -45 && angle <= 45)
			return Directions.top;
		else if(angle > 45 && angle < 135)
			return Directions.right;
		else if(angle >= 135 && angle < 180 || angle <= -135 && angle > -180)
			return Directions.bottom;
		else //if(angle < -45 && angle > -135)
			return Directions.left;
	}
	//future distance Y
	public double prospectiveTimeY(double time, double heading, double originY, double velocity)
	{
		// cah, hyp = velocity * time,  v=s/t -> s = vt
		return Math.cos(heading) * velocity * time + originY;
	}
	//future distance X
	public double prospectiveTimeX (double time, double heading, double originX, double velocity)
	{
		//soh
		//out.println("Vel: " + velocity + ". Time: " + time + ". OrX: " + originX);
		return Math.sin(heading) * velocity * time + originX;
	}

	/**-------------
	 * Data Analyzer
	 *--------------
	 * */
	class DataAnalyzer {
		private ArrayList<BulletHitLog> hitsData;
		private ArrayList<RobotLog>[] targetsData;

		private int hitsLogsCapacity = 20;
		private int targetsLogsCapacity = 100;

		/**
		 * Constructor
		 * @param count
		 */
		public DataAnalyzer(int count) {
			hitsData = new ArrayList<BulletHitLog>();
			targetsData = new ArrayList[count];
			for (int x = 0; x < targetsData.length; x++)
				targetsData[x] = new ArrayList<RobotLog>();
		}
		/**
		 * Function which enables us to obtains newest log. Returns empty object, when logs were empty.
		 * @param robotIndex
		 * @return
		 */
		public RobotLog getLog(int robotIndex) {
			return getLog(robotIndex, 0);
		}
		/**
		 * Function which enables us to obtains specified log.
		 * @param robotIndex
		 * @param logIndex
		 * @return
		 */
		public RobotLog getLog(int robotIndex, int logIndex) {
			robotIndex = limit(logIndex, 0, targetsData.length - 1);
			if(targetsData[robotIndex].size() == 0)
				return new RobotLog();
			else {
				logIndex = limit(logIndex, 0, targetsData[robotIndex].size() - 1);
				return targetsData[robotIndex].get(logIndex);
			}
		}
		/**
		 *
		 * @param robotIndex
		 * @return
		 */
		public BulletHitLog getHitLog(int robotIndex) {
			for(int x = 0; x < hitsData.size(); x++) {
				if(!hitsData.get(x).isEmpty() && hitsData.get(x).getIndex() == robotIndex)
					return hitsData.get(x);
			}
			return new BulletHitLog();
		}

		/*public BulletHitLog[] getHitLogs(int robotIndex) {

		}*/

		/**
		 * Registers or updates found enemy
		 * @param robot
		 * @param time
		 * @return
		 */
		public int registerRobot(ScannedRobotEvent robot, long time) {
			for (int x = 0; x < targetsData.length; x++) {
				if (targetsData[x].size() == 0 || targetsData[x].get(0).getRobotName() == robot.getName()) {
					//Registration or update bots and its logs
					targetsData[x].add(0, new RobotLog(time, robot));
					if(targetsData[x].size() >= targetsLogsCapacity)
						targetsData[x].remove(targetsData[x].size() - 1);
					return x;
				}
			}
			return 0;
		}
		/**
		 *Registers bullet hit
		 * @param bullet
		 * @return
		 */
		public boolean registerBulletHit(long currentTime, BulletHitEvent event) {
			BulletHitLog bullet = new BulletHitLog(currentTime, event);
			int index = getRobotIndexByName(bullet.getRobotName());
			if(index == -1) return false;
			bullet.setIndex(index);
			hitsData.add(0, bullet);
			if(hitsData.size() >= hitsLogsCapacity)
				hitsData.remove(hitsData.size() - 1);
			return true;
		}
		/**
		 * From the registry searches for the closest last enemy
		 * @param currentTime
		 * @return
		 */
		public int getClosestRobotIndex(long currentTime) {
			int index = -1;
			double distance = -1;
			for (int x = 0; x < targetsData.length; x++) {
				if (robotIsDead(x)) continue;
				RobotLog log = getLog(x);
				if (log.isEmpty()) continue;
				if (distance > log.getRobotDistance() || index == -1) {
					if(currentTime != log.getTick()) continue;
					index = x;
					distance = log.getRobotDistance();
				}
			}
			//So no info is NOT fresh, we take the "freshest" one
			if(index == -1) {
				long min = currentTime;
				for (int x = 0; x < targetsData.length; x++) {
					if (robotIsDead(x)) continue;
					RobotLog log = getLog(x);
					if(log.isEmpty()) continue;
					long ticksAhead = currentTime - log.getTick();
					if(ticksAhead < min || index == -1) {
						min = ticksAhead;
						index = x;
						distance = log.getRobotDistance();
					}
					else if (ticksAhead == min && distance > log.getRobotDistance()) {
						index = x;
						distance = log.getRobotDistance();
					}
				}
			}
			return index;
		}
		/**
		 * Search enemy's name and returns index
		 * @param name
		 * @return
		 */
		public int getRobotIndexByName(String name) {
			for (int x = 0; x < targetsData.length; x++) {
				if (!robotIsDead(x) && targetsData[x].get(0).getRobotName() == name) {
					return x;
				}
			}
			return -1;
		}
		/**
		 *Iterate through the last logs to get the last distance from the target
		 * @param robotIndex
		 * @return
		 */
		public double getRobotLastDistance(int robotIndex) {
			if(robotIsDead(robotIndex)) return -1;

			double distance = -1;
			int i = 0;
			do {
				RobotLog log = getLog(robotIndex, i);
				if (log.isEmpty()) continue;
				distance = log.getRobotDistance();
			} while(distance == -1 && i < targetsData[robotIndex].size());
			return distance;
		}
		//when the robot is smashed, we exclude it from the analysis
		public void clearIndexData(int robotIndex) {
			if (robotIndex <= -1 || robotIndex >= targetsData.length || robotIsDead(robotIndex)) return;
			targetsData[robotIndex].clear();
			for(int x = 0; x < hitsData.size(); x++)
				if(hitsData.get(x).getIndex() == robotIndex)
					hitsData.remove(x--);
		}

		public void cleanAllData() {
			for(int x = 0; x < targetsData.length; x++)
				targetsData[x].clear();
			targetsData = null;
		}

		// Analysis
		/**
		 *
		 * @param robotIndex
		 * @return
		 */
		public double getAverageVelocity(int robotIndex) {
			return getAverageVelocity(robotIndex, -1);
		}
		/**
		 * Function that we can use when we want to get average velocity of specified robot index
		 * @param robotIndex
		 * @param probesCount
		 * @return
		 */
		public double getAverageVelocity(int robotIndex, int probesCount) {
			if(robotIsDead(robotIndex)) return -1;

			if(probesCount <= -1) //all probes
				probesCount = targetsData[robotIndex].size();

			double average = 0;
			int count = 0;
			for(int x = 0; x < probesCount && x < targetsData[robotIndex].size(); x++) {
				// We skip data when the vehicle is stationary or barely moving
				RobotLog log = getLog(robotIndex, x);
				if(log.isEmpty()) continue;
				if(log.getData().getVelocity() > 0.1d) {
					count++;
					average += log.getData().getVelocity();
				}
			}
			return average / count;
		}
		/**
		 * Function to get max velocity of specified robot
		 * @param robotIndex
		 * @param probesCount
		 * @return
		 */
		public double getMaximumVelocity(int robotIndex, int probesCount) {
			if(robotIsDead(robotIndex)) return -1;

			if(probesCount <= -1) // all probes
				probesCount = targetsData[robotIndex].size();

			double maximum = -1;
			for(int x = 0; x < probesCount && x < targetsData[robotIndex].size(); x++) {
				RobotLog log = getLog(robotIndex, x);
				if(log.isEmpty()) continue;
				if(log.getData().getVelocity() > maximum);
					maximum = log.getData().getVelocity();
			}
			return maximum;
		}
		/**
		 *
		 * @param robotIndex
		 * @return
		 */
		public double getAverageDistance(int robotIndex) {
			return getAverageDistance(robotIndex, -1);
		}
		/**
		 *
		 * @param robotIndex
		 * @param probesCount
		 * @return
		 */
		public double getAverageDistance(int robotIndex, int probesCount) {
			if(robotIsDead(robotIndex)) return -1;

			if(probesCount <= -1) //all probes
				probesCount = targetsData[robotIndex].size();

			double average = 0;
			int count = 0;
			for(int x = 0; x < probesCount && x < targetsData[robotIndex].size(); x++) {
				RobotLog log = getLog(robotIndex, x);
				if(log.isEmpty()) continue;
				count++;
				average += log.getRobotDistance();
			}
			return average / count;
		}
		/**
		 * Function to get most common bearing of specified robot
		 * @param robotIndex
		 * @param probesCount
		 * @return
		 */
		public double getMostCommonBearing(int robotIndex, int probesCount) {
			if(robotIsDead(robotIndex)) return -1;

			Map<Double, Integer> commonBearings = new HashMap<Double, Integer>();

			if(probesCount <= -1) // all probes
				probesCount = targetsData[robotIndex].size();

			for(int x = 0; x < probesCount && x < targetsData[robotIndex].size(); x++) {
				RobotLog log = getLog(robotIndex, x);
				if(log.isEmpty()) continue;
				double bearing = log.getData().getBearing();
				if(!commonBearings.containsKey(bearing))
					commonBearings.put(bearing, 1);
				else
					commonBearings.replace(bearing, commonBearings.get(bearing) + 1);
			}

			int count = 0;
			double commonBearing = 0;
			for(Map.Entry<Double, Integer> entry : commonBearings.entrySet()) {
				if(entry.getKey() != commonBearing && count < entry.getValue()) {
					commonBearing = entry.getKey();
					count = entry.getValue();
				}
			}
			return commonBearing;
		}
		/**
		 * Returns if robot is asceding, desceding or constant
		 * @param robotIndex
		 * @return
		 */
		public Movement predictEnemyMovement(int robotIndex) {
			if(robotIsDead(robotIndex)) return Movement.unrecognized;

			int probesCount = targetsData[robotIndex].size();
			int correction = 0;

			RobotLog log = getLog(robotIndex, correction);
			while(log.isEmpty()) {
				if(correction >= probesCount)
					return Movement.unrecognized;
				correction++;
				log = getLog(robotIndex, correction);
			}

			RobotLog nextLog = getLog(robotIndex, correction + 1);
			while(nextLog.isEmpty()) {
				if(correction + 1 >= probesCount)
					return Movement.unrecognized;
				correction++;
				nextLog = getLog(robotIndex, correction + 1);
			}

			if(log.getData().getVelocity() == nextLog.getData().getVelocity())
				return Movement.constant;
			else if(log.getData().getVelocity() > nextLog.getData().getVelocity())
				return Movement.descending;
			else
				return Movement.ascending;
		}
		/**
		 * Returns value from 0 to 1.
		 * @param robotIndex
		 * @return
		 */
		public double enemyFiredProbability(int robotIndex) {
			if (robotIsDead(robotIndex)) return 0;
			// Secure probing
			int probesCount = Math.min(5, targetsData[robotIndex].size());

			// Preparing variables
			BulletHitLog lastHit = getHitLog(robotIndex);
			RobotLog firstLog = getLog(robotIndex);
			if(firstLog.isUsed()) {
				int count = 0;
				do {
					count++;
					firstLog = getLog(robotIndex, count);
				} while(firstLog.isUsed() && count < probesCount);
				if(count == probesCount)
					return 0;
			}

			double lowestEnergy = firstLog.getData().getEnergy();
			double highestEnergy = firstLog.getData().getEnergy();
			double healedBy = 0;
			double energyDrop = 0;

			// Picking lowest and highest energy
			RobotLog prevLog = firstLog;
			prevLog.setUsed();
			for(int x = 1; x < probesCount; x++) {
				RobotLog log = getLog(robotIndex, x);
				if(log.isEmpty() && log.isUsed()) continue;
				log.isUsed();

				double prevEnergy = prevLog.getData().getEnergy();
				double energy = log.getData().getEnergy();
				if(prevEnergy < energy)
					healedBy += energy - prevEnergy;
				else if(prevEnergy > energy)
					energyDrop += prevEnergy - energy;

				if(log.getData().getEnergy() < lowestEnergy)
					lowestEnergy = log.getData().getEnergy();
				else if(log.getData().getEnergy() > highestEnergy)
					highestEnergy = log.getData().getEnergy();

				prevLog = log;
			}

			int division = 0;
			double calculation = 0;
			boolean possibleCollision = false;

			// Probabiltiy formula
			double distance = firstLog.getRobotDistance();
			if(distance > 80)
				distance = getAverageDistance(robotIndex, probesCount);
			if(distance <= 80)
				possibleCollision = true;

			if(healedBy > energyDrop)
				calculation += 1;

			if(lastHit.isEmpty()) {
				// Without hit information
				if(possibleCollision) {
					if(energyDrop <= 3) {
						division++;
					}
					else if(energyDrop >= 3.1d) {
						calculation += Math.min(energyDrop - 2, 1.5d);
					}
				}
				else {
					if(energyDrop == 0d)
						division++;
					else
						calculation += Math.min(energyDrop, 2d);
				}
			}
			else {
				// With hit information
				if(possibleCollision) {
					if (energyDrop <= lastHit.getPower() + 3d)
						division++;
					else
						calculation += Math.min(energyDrop - lastHit.getPower() - 3d, 1.5d);
				}
				else {
					if(energyDrop <= lastHit.getPower())
						division++;
					else
						calculation += Math.min(energyDrop - lastHit.getPower(), 2d);
				}
			}

			hitsData.remove(lastHit);
			return limit(calculation / (division == 0 ? 1 : division),0,1);
		}

		// Tools
		public double limit(double val, double min, double max) {
			return Math.max(min, Math.min(max, val));
		}
		public int limit(int val, int min, int max) {
			return Math.max(min, Math.min(max, val));
		}
		private boolean robotIsDead(int index) { return targetsData[index].size() == 0 || targetsData[index] == null; }

		//Data classes
		class BulletHitLog {
			private final long tick;
			private final double power;
			private final String robotName;
			private int index = -1;

			public BulletHitLog(long tick, BulletHitEvent e) {
				this.tick = tick;
				this.power = robocode.Rules.getBulletDamage(e.getBullet().getPower());
				this.robotName = e.getName();
			}

			public BulletHitLog(long tick, double power, String robotName) {
				this.tick = tick;
				this.power = robocode.Rules.getBulletDamage(power);
				this.robotName = robotName;
			}

			public BulletHitLog() {
				this.tick = -1;
				this.power = -1;
				this.robotName = null;
			}

			public boolean isEmpty() { return tick == -1 || power == -1 || robotName == null; }

			public void setIndex(int i) {
				if(index != -1)
					index = i;
			}

			public double getPower() { return power; }

			public long getTick() { return tick; }

			public String getRobotName() { return robotName; }

			public int getIndex() {	return index; }
		}

		class RobotLog {
			private final long tick;
			private final ScannedRobotEvent data;
			private boolean used = false;

			public RobotLog() {
				this.tick = -1;
				this.data = null;
			}

			public RobotLog(long tick, ScannedRobotEvent data) {
				this.tick = tick;
				this.data = data;
			}

			public void setUsed() { used = true; }

			public boolean isEmpty() { return this.data == null || this.tick == -1; }

			public boolean isUsed() { return used; }

			public String getRobotName() { return data.getName(); }

			public double getRobotDistance() { return data.getDistance(); }

			public ScannedRobotEvent getData() { return data; }

			public long getTick() { return tick; }
		}
	}
	enum Movement {
		unrecognized(0), ascending(1), descending(2), constant(3);

		final int index;
		Movement(int index) {
			this.index = index;
		}

		public static Movement byIndex(int i) {
			for(Movement m : Movement.values())
				if(m.index == i)
					return m;
			return ascending;
		}
	}
}

enum Directions
{
	top,
	bottom,
	right,
	left
}