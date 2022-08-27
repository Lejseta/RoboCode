package sample;
import robocode.*;
import static robocode.util.Utils.normalRelativeAngleDegrees;
import java.awt.Point;
import java.util.*;
import java.awt.geom.*;
import java.awt.Color;
import java.util.List;

/**
 * Experimental Robot - a robot by Mirella Glowinska
 */
public class ExperimentalRobot extends Robot {
	int turnDirection = 1; // Clockwise or counterclockwise
	boolean shooting = false;
	boolean controlSide = true;
	double bearingFromGun = 0;
	double bearingFromRadar = 0;

	//prediction
	double predictedY, predictedX;
	//The averaged velocity.
	double velocityToAimAt;
	// Enemy
	DataAnalyzer analyzer;
	int targetIndex = -1;
	boolean focused; //that is, can it change the target. false - no. true- yes
	int movesInDirection = 0;

	double xDimension, yDimension;
	double safeZoneRadius = 100;

	//Events
	public void run() {
		analyzer = new DataAnalyzer(getOthers());
		xDimension = getBattleFieldWidth();
		yDimension = getBattleFieldHeight();

		setAdjustRadarForGunTurn(true);
		setAdjustRadarForRobotTurn(true);

		// body,gun,radar
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

		if (targetIndex == -1) {
			targetEnemy(closestIndex);
			RobotLog log = analyzer.getLog(targetIndex);
			if(log.isEmpty())
				myMoveForward(50);
		}

		// Target Data
		RobotLog log = analyzer.getLog(targetIndex);
		if(log.isEmpty()) {
			targetIndex = closestIndex;
			log = analyzer.getLog(targetIndex);
		}

		// Debug and analysis
		int debugIndex = targetIndex;
		int times = 10;
		double max = analyzer.getMaximumVelocity(debugIndex, times);
		double avg = analyzer.getAverageVelocity(debugIndex, times);
		double ang = analyzer.getMostCommonBearing(debugIndex, times);
		double dis = analyzer.getAverageDistance(debugIndex);
		Movement mov = analyzer.predictEnemyMovement(debugIndex);
		double prob = analyzer.enemyFiredProbability(debugIndex);

		out.println("-------- " + analyzer.getLog(debugIndex).getRobotName() + " T: " + getTime() + " --------");
		out.println("MAX: " + max + ", AVG: " + avg + ", ANGLE: " + ang);
		out.println("DIS: " + dis + ", MOV: " + mov + ", PROBAB: " + prob);

		if (prob >= 0.9d) {
			if(log.getRobotDistance() < 100)
				shotTarget(log.getData());
			else if(Math.random() > 0.7d) {
				shooting = predictedShooting();
				if(shooting)
					shotTarget(log.getData());
			}
			safeMove(70);
		}
		else if (prob >= 0.5d) {
			shooting = !predictedShooting();
			safeMove(60);
		}
		else {
			// If we fail to predict shot, try again. In other case stop shooting
			shooting = !predictedShooting();
			if(shooting) {
				shotTarget(log.getData());
				shooting = false;
			}
		}
	}

	public void onHitRobot(HitRobotEvent e) {
		rotateGunToward(e);
		fire(3);
		if(e.getEnergy() < getEnergy())
		{
			myTurnRight(e.getBearing());//turn body towards it
			myMoveForward(40);//ram at it
			fire(3);
		} else {
			myTurnRight(normalRelativeAngleDegrees(e.getBearing()-90));//if it has more energy than us
			moveInLineRandomly(100);
		}
	}

	public void onBulletHit(BulletHitEvent event) {
		analyzer.registerBulletHit(getTime(), event);
	}

	public void onBulletHitBullet(BulletHitBulletEvent event) {
		RobotLog log = analyzer.getLog(analyzer.getRobotIndexByName(event.getHitBullet().getName()));
		if(log.isEmpty()) {
			myMoveForward(40);
			return;
		}
		myMoveForward(30);
	}

	/**
	 * onHitByBullet: What to do when you're hit by a bullet
	 */
	public void onHitByBullet(HitByBulletEvent e) {
		rotateToward(e.getBearing(), 20, 60);
		myMoveForward(60);
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
				myMoveForward(60);
				movesInDirection = 0;
				break;
			case right:
				out.println("right");
				safeMove(60);
				break;
			case left:
				out.println("left");
				safeMove(60);
				break;
			case bottom:
				out.println("back");
				controlSide = true;
				myMoveForward(60);
				movesInDirection = 0;
				break;
		}
	}

	public void onRobotDeath(RobotDeathEvent e) {
		analyzer.clearIndexData(analyzer.getRobotIndexByName(e.getName()));
	}

	public void onDeath(DeathEvent e) {
		analyzer.cleanAllData();
	}

	void safeMove(double distance) {
		safeMove(distance, distance);
	}

	void safeMove(double minDistance, double maxDistance) {
		RobotLog log = analyzer.getLog(targetIndex);
		if(log.isEmpty()) return;
		tryChangeDirection();
		double distance = Math.random() * (maxDistance - minDistance) + minDistance;
		rotateToward(log.getRobotBearing(), 50, distance);
		myMoveForward(distance);
	}

	List<PointLog> lastPoints;
	/**
	 * Calculate is bearing is safe.
	 * 0: No danger to stuck in.
	 * 1: Minor danger
	 * 2: Medium danger
	 * 3: High danger
	 * @param distance
	 * @return
	 */
	int calculateSafeBearing(double angle, double distance)
	{
		int counter = 0;
		if(lastPoints == null)
			lastPoints = new ArrayList<PointLog>();
		else {
			for(int x = 0; x < lastPoints.size(); x++) {
				if(lastPoints.get(x).ticksBehind(getTime()) > 50)
					lastPoints.remove(x--);
			}
		}

		lastPoints.add(0, new PointLog(getX(), getY(), getTime()));

		double futureX = prospectiveTimeX(1, angle, getX(), distance);
		double futureY = prospectiveTimeY(1, angle, getY(), distance);

		boolean xSafe = !(futureX < safeZoneRadius || futureX > xDimension - safeZoneRadius);
		boolean ySafe = !(futureY < safeZoneRadius || futureY > yDimension - safeZoneRadius);

		for(int x = 0; x < lastPoints.size(); x++) {
			PointLog point = lastPoints.get(x);
			if(calculateDistance(point.x(), point.y(), getX(), getY()) < distance) {
				counter++;
				break;
			}
		}
		
		RobotLog log = analyzer.getLog(targetIndex);
		if(!log.isEmpty()) {
			double enemyY = getY() + log.getRobotDistance() * Math.cos(Math.toRadians(getHeading() + log.getRobotBearing()));
			double enemyX = getX() + log.getRobotDistance() * Math.sin(Math.toRadians(getHeading() + log.getRobotBearing()));

			if(calculateDistance(futureX, futureY, enemyX, enemyX) < 100)
				return 3;
		}

		if(!ySafe && !xSafe)
			counter += 3;
		else {
			counter += xSafe ? 0 : 1;
			counter += ySafe ? 0 : 1;
		}
		return limit(counter, 0 , 3);
	}

	void rotateToward(double bearing, double deviationAngle, double distanceCheck) {
		out.println("Enemy bearing: " + bearing);

		// DangerLevel , Bearing
		Map<Integer, List<Double>> bearings = new HashMap<Integer, List<Double>>();
		for(int x = 0; x <= 3; x++)
			bearings.put(x, new ArrayList<Double>());

		int dangerLevel = -1;
		double selectedBearing = 360;

		double posBearing = normalizeBearing(bearing + 90);
		double negBearing = normalizeBearing(bearing - 90);

		for(double testBearing = -deviationAngle; testBearing <= deviationAngle; testBearing += 10) {
			double checkPosBearing = normalizeBearing(posBearing + testBearing);
			double checkNegBearing = normalizeBearing(negBearing + testBearing);

			int posDanger = calculateSafeBearing(checkPosBearing, distanceCheck);
			int negDanger = calculateSafeBearing(checkNegBearing, distanceCheck);

			bearings.get(posDanger).add(checkPosBearing);
			bearings.get(negDanger).add(checkNegBearing);
		}

		for(int x = 0; x <= 3; x++) {
			Double[] array = new Double[0];
			for(Double checkBearing : bearings.get(x).toArray(array)) {
				dangerLevel = x;
				if(selectedBearing > Math.abs(smallestAngle(getHeading(), checkBearing)))
					selectedBearing = Math.abs(smallestAngle(getHeading(), checkBearing));
			}
			if(dangerLevel != -1)
				break;
		}
		myTurnRight(selectedBearing);
	}

	void targetEnemy(int i) {
		if (analyzer.robotIsDead(i)) return;
		targetIndex = i;
	}

	void shotTarget(ScannedRobotEvent robot) {
		rotateGunToward(robot);
		fire(calculateFirePower(robot.getDistance()));
	}

	void rotateGunToward(ScannedRobotEvent e) {
		double gunTurnAmt = normalRelativeAngleDegrees(e.getBearing() + (getHeading() - getGunHeading()));
		turnGunRight(gunTurnAmt);
	}

	void rotateGunToward(HitRobotEvent e) {
		double gunTurnAmt = normalRelativeAngleDegrees(e.getBearing() + (getHeading() - getGunHeading()));
		turnGunRight(gunTurnAmt);
	}

	void tryChangeDirection() {
		if(movesInDirection > 2) {
			if(Math.random() > 0.5)
				changeDirection();
		}
	}

	void changeDirection() {
		movesInDirection = 0;
		controlSide = !controlSide;
	}

	void moveInLineRandomly(double units) {
		if(Math.random() > 0.5)
			myMoveForward(units);
		else
			myMoveBack(units);
	}

	void myMoveForward(double units) {
		movesInDirection++;
		if(controlSide)
			ahead(units);
		else
			back(units);
	}

	void myMoveBack(double units) {
		movesInDirection++;
		if(controlSide)
			back(units);
		else
			ahead(units);
	}

	void myTurnRight(double angle) {
		double newAngle = smallestAngle(getHeading(),angle);
		if(Math.abs(newAngle) <= 3 || Math.abs(newAngle) > 177)
			return;
		turnRight(newAngle);
	}

	void myTurnLeft(double angle) {
		double newAngle = smallestAngle(getHeading(), angle);
		turnRight(newAngle);
	}

	double smallestAngle(double heading, double bearing) {
		heading = normalizeBearing(heading);
		bearing = normalizeBearing(bearing);

		out.println("Heading: " + heading + ", Bearing: " + bearing);
		if(Math.abs(bearing - heading) > 90d) {
			out.println("Out:" + normalizeBearing(bearing + 180 - heading));
			return normalizeBearing(bearing + 180 - heading);
		}
		out.println("Out:" + bearing);
		return bearing;
	}

	boolean predictedShooting() {
		// We need the newest information about our target
		RobotLog lastLog = analyzer.getLog(targetIndex);
		if(lastLog.isEmpty() || targetIndex == -1) return false; // Probably enemy is dead
		if(lastLog.ticksBehind(getTime()) > 5) return false; // Data is too old

		// Calculating firepower based on distance
		double firePower = calculateFirePower(lastLog.getRobotDistance());
		// distance = rate * time, v = s/t -> t = s/v
		double time = (lastLog.getRobotDistance() / bulletVelocity(firePower));

		// Preparing enemy position
		double enemyY = getY() + lastLog.getRobotDistance() * Math.cos(Math.toRadians(getHeading() + lastLog.getRobotBearing()));
		double enemyX = getX() + lastLog.getRobotDistance() * Math.sin(Math.toRadians(getHeading() + lastLog.getRobotBearing()));

		// Selecting velocity prediction based on data analyzing
		double velocity;
        Movement mov = analyzer.predictEnemyMovement(targetIndex);
        if(mov == Movement.constant)
            velocity = lastLog.getData().getVelocity();
        else if (mov == Movement.descending)
            velocity = lastLog.getData().getVelocity();
        else if (mov == Movement.ascending)
            velocity = (analyzer.getMaximumVelocity(targetIndex, 25) + lastLog.getData().getVelocity()) / 2;
        else
            return false;

		// Prediction based on bullet speed, enemy heading, and velocity
		double futureX = prospectiveTimeX(time, lastLog.getData().getHeading(), enemyX, velocity);
		out.println("Current X: " + enemyX + ". Future X: " + futureX);
		double futureY = prospectiveTimeY(time, lastLog.getData().getHeading(), enemyY, velocity);
		out.println("Current Y: " + enemyY + ". Future Y: " + futureY);

		if(calculateDistance(enemyX, enemyY, futureX, futureY) > 150) {
			out.println("Shot canceled");
			return false; // Bad analysis and data, cancel shooting
		}
		// Calculating bearing we need to calculate shot angle
		double absDeg = enemyBearing(getX(), getY(), futureX, futureY);
		//out.println("Bearing: " + normalizeBearing(absDeg - getGunHeading()));

		// Turn the gun to the predicted x,y location
		turnGunRight(normalizeBearing(absDeg - getGunHeading()));

		if (getGunHeat() == 0) {
			stop();
			fire(calculateFirePower(lastLog.getRobotDistance()));
			resume();
			return true;
		}
		return false;
	}
	//absolute enemy bearing
	double enemyBearing(double currentX, double currentY, double futureX, double futureY)
	{
		// X-axis distance and Y-axis distance
		double differenceX = futureX - currentX;
		double differenceY = futureY - currentY;
		//soh cah toa
		double hyp = Point2D.distance(currentX, currentY, futureX, futureY);
		//s= opp/hyp, We're calculating angle
		double arcAngle = Math.toDegrees(Math.asin(differenceX / hyp));
		// initializing bearing
		double bearing = 0;
		//Location and actual angle
		if (differenceX > 0 && differenceY > 0) { // both pos: lower-Left
			bearing = arcAngle;
		} else if (differenceX < 0 && differenceY > 0) { // x neg, y pos: lower-right
			bearing = 360 + arcAngle; // arcsin is negative here, actually 360 - ang
		} else if (differenceX > 0 && differenceY < 0) { // x pos, y neg: upper-left
			bearing = 180 - arcAngle;
		} else if (differenceX < 0 && differenceY < 0) { // both neg: upper-right
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
		return limit(600 / distance, 0.1d, 3d);
	}

	static double maxEscapeAngle(double bulletSpeed) {
		return Math.asin(8 / bulletSpeed);
	}
	
	static double bulletVelocity(double power) {
		return 20d - (3d * power);
	}

	static double limit(double val, double min, double max) {
		return Math.max(min, Math.min(max, val));
	}

	static int limit(int val, int min, int max) {
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
	// Future distance X
	public double prospectiveTimeX (double time, double heading, double originX, double velocity)
	{
		//soh
		return Math.sin(heading) * velocity * time + originX;
	}
	// Future distance Y
	public double prospectiveTimeY(double time, double heading, double originY, double velocity)
	{
		// cah, hyp = velocity * time,  v=s/t -> s = vt
		return Math.cos(heading) * velocity * time + originY;
	}

	public double calculateDistance(double x1, double y1, double x2, double y2) {
		return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
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
		 * @param robotIndex
		 * @return
		 */
		public BulletHitLog getHitLog(int robotIndex) {
			return getHitLog(robotIndex, -1);
		}

		public BulletHitLog getHitLog(int robotIndex, long ticksBehind) {
			for(int x = 0; x < hitsData.size(); x++) {
				if(!hitsData.get(x).isEmpty() && hitsData.get(x).getIndex() == robotIndex && (ticksBehind >= hitsData.get(x).ticksBehind(ticksBehind) || ticksBehind == -1))
					return hitsData.get(x);
			}
			return new BulletHitLog();
		}

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
		 * //Search enemy's name and returns index
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
				RobotLog log = getLog(robotIndex, i++);
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
		public double getAverageVelocity(int robotIndex) {
			return getAverageVelocity(robotIndex, -1);
		}
		public double getAverageVelocity(int robotIndex, int probesCount) {
			if(robotIsDead(robotIndex)) return -1;

			if(probesCount <= -1) //all probes
				probesCount = targetsData[robotIndex].size();
			probesCount = Math.min(probesCount, targetsData[robotIndex].size());

			double average = 0;
			int count = 0;
			for(int x = 0; x < probesCount; x++) {
				// We skip data when the vehicle is stationary or barely moving
				RobotLog log = getLog(robotIndex, x);
				if(log.isEmpty()) continue;
				double velocity = Math.abs(log.getData().getVelocity());
				if(velocity != 0d) {
					count++;
					average += velocity;
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

			if(probesCount <= -1) //all probes
				probesCount = targetsData[robotIndex].size();
			probesCount = Math.min(probesCount, targetsData[robotIndex].size());

			double maximum = -1;
			for(int x = 0; x < probesCount; x++) {
				RobotLog log = getLog(robotIndex, x);
				if(log.isEmpty()) continue;
				if(Math.abs(log.getData().getVelocity()) > maximum);
				maximum = Math.abs(log.getData().getVelocity());
			}
			return maximum;
		}

		public double getAverageDistance(int robotIndex) {
			return getAverageDistance(robotIndex, -1);
		}
		public double getAverageDistance(int robotIndex, int probesCount) {
			if(robotIsDead(robotIndex)) return -1;

			if(probesCount <= -1) //all probes
				probesCount = targetsData[robotIndex].size();
			probesCount = Math.min(probesCount, targetsData[robotIndex].size());

			double average = 0;
			int count = 0;
			for(int x = 0; x < probesCount; x++) {
				RobotLog log = getLog(robotIndex, x);
				if(log.isEmpty()) continue;
				count++;
				average += log.getRobotDistance();
			}
			return average / count;
		}

		public double getMostCommonBearing(int robotIndex, int probesCount) {
			if(robotIsDead(robotIndex)) return -1;

			Map<Double, Integer> commonBearings = new HashMap<Double, Integer>();

			if(probesCount <= -1) //all probes
				probesCount = targetsData[robotIndex].size();
			probesCount = Math.min(probesCount, targetsData[robotIndex].size());

			for(int x = 0; x < probesCount; x++) {
				RobotLog log = getLog(robotIndex, x);
				if(log.isEmpty()) continue;
				double bearing = (double)Math.round(log.getData().getBearing());
				bearing -= (bearing % 5);
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
			int probesCount = Math.min(3, targetsData[robotIndex].size());

			// Preparing variables
			BulletHitLog lastHit = getHitLog(robotIndex, 10);
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

			int division = 1;
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
			return limit(calculation / division,0,1);
		}

		// Tools
		public double limit(double val, double min, double max) {
			return Math.max(min, Math.min(max, val));
		}
		public int limit(int val, int min, int max) {
			return Math.max(min, Math.min(max, val));
		}
		private boolean robotIsDead(int index) { return targetsData[index] == null || targetsData[index].size() == 0; }
	}

	//Data classes
	public class BulletHitLog {
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

		public long ticksBehind(long current) { return current - tick; }
	}

	public class PointLog {
		private final long tick;
		private final double xPos;
		private final double yPos;

		public PointLog() {
			tick = -1;
			xPos = 0;
			yPos = 0;
		}

		public PointLog(int xPos, int yPos, long tick) {
			this.xPos = (double)xPos;
			this.yPos = (double)yPos;
			this.tick = tick;
		}

		public PointLog(double xPos, double yPos, long tick) {
			this.tick = tick;
			this.xPos = xPos;
			this.yPos = yPos;
		}

		public double x() {
			return xPos;
		}

		public double y() {
			return yPos;
		}

		public boolean isEmpty() { return tick == -1; }

		public long getTick() { return tick; }

		public long ticksBehind(long current) { return current - tick; }
	}

	public class RobotLog {
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

		public double getRobotBearing() { return data.getBearing();}

		public ScannedRobotEvent getData() { return data; }

		public long getTick() { return tick; }

		public long ticksBehind(long current) { return current - tick; }
	}

	public enum Movement {
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

	enum Directions {
		top,
		bottom,
		right,
		left
	}
}