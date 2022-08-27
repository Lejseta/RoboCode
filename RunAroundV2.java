package sample;
import robocode.*;
import java.awt.Color;
/**
 * RunAround - a robot by Gábor Major, Mirella Glowinska, Daniel Vetrila
 * The robot that won in RoboCode 2022
 */
public class RunAroundV2 extends Robot
{
	// robot size - 36x36
	// our robot (tank) variables
	double tX;
	double tY;
	double tEnergy;
	double tGunHeat;
	
	double tVelocity;
	double tDistanceRemaining;
	double tHeading;
	double tTurnReamaining;
	
	double tGunHeading;
	double tGunTurnRemaining;
	double tRadarHeading;
	double tRadarTurnRemaining;
	
	double targetX;
	double targetY;

	// enemy robot variables
	boolean ePositionKnown = false;
	boolean eLocked = false;
	double eX = 400;
	double eY = 400;
	
	double eEnergy = 100;
	double eDirection;
	double eDistance;
	double eHeading;
	double eVelocity;
	
	// sentry variables
	double sX = 400;
	double sY = 400;

	// constant variables
	final double battleFieldWidth = 800;
	final double battleFieldHeight = 800;
	final double lowerCoor = 300;
	final double higherCoor = 500;
	final double safeArea = 100;

	// other variables
	boolean sentryPositionKnown = false;
	long globalTime;
    int moveCount = 0;

	public void run()
	{
		// sets robot colour
		setBodyColor(Color.black);
		setGunColor(Color.black);
		setRadarColor(Color.magenta);

		// sets gun independt of the body
		setAdjustGunForRobotTurn(true);
		scanTowardsCentre();
		
		while(true)
		{
			// picks random point and moves to it
			pickNewRandomPoint();
			turnAndMoveToPoint(targetX, targetY);
			
			// if moved more than twice and not saw enemy then scan for it
            if (moveCount >= 2)
            {
                scanTowardsCentre();
            }
		}
	}

	// picks a new random point to move to
	// point can be North, South, East or West of the robot position
	// randomly selected based on weighting of robot position
	// robot wants to move closer to the centre of the map
	// robot wants to move further away from the sentry
	// weighting ratio 2:1
	// random distance also chosen between 100 and 200
	private void pickNewRandomPoint()
	{
		double right_chance = 0.5;
		double left_chance = 0.5;
		double up_chance = 0.5;
		double down_chance = 0.5;
		
		double x_ratio = tX / battleFieldWidth;
		double y_ratio = tY / battleFieldHeight;
		
		right_chance = 1 - x_ratio;
		left_chance = x_ratio;
		up_chance = 1 - y_ratio;
		down_chance = y_ratio;
		
		if (sentryPositionKnown)
		{
			right_chance /= 3;
			left_chance /= 3;
			up_chance /= 3;
			down_chance /= 3;

			double s_x_distance = sX - tX;
			double s_x_ratio = s_x_distance / sX;
			
			if (s_x_distance < 0)
			{
				s_x_ratio = s_x_distance / (800 - sX);
			}
			
			if (s_x_ratio > 0)
			{
				right_chance += (s_x_ratio * 2) / 3;
				left_chance += ((1 - s_x_ratio) * 2) / 3;
			}
			else
			{
				right_chance += ((1 + s_x_ratio) * 2) / 3;
				left_chance += (-s_x_ratio * 2) / 3;
			}
			
			double s_y_distance = sY - tY;
			double s_y_ratio = s_y_distance / sY;
			
			if (s_y_distance < 0)
			{
				s_y_ratio = s_y_distance / (800 - sY);
			}
			
			if (s_y_ratio > 0)
			{
				up_chance += (s_y_ratio * 2) / 3;
				down_chance += ((1 - s_y_ratio) * 2) / 3;
			}
			else
			{
				up_chance += ((1 + s_y_ratio) * 2) / 3;
				down_chance += (-s_y_ratio * 2) / 3;
			}
		}
		
		double chosen_direction = Math.random();
		
		right_chance /= 2;
		left_chance /= 2;
		up_chance /= 2;
		down_chance /= 2;
		
		if (chosen_direction < right_chance)
		{
			targetX = tX + 100 + (100 * Math.random());
			targetY = tY;
		}
		else if (chosen_direction < (right_chance + left_chance))
		{
			targetX = tX - 100 - (100 * Math.random());
			targetY = tY;
		}
		else if (chosen_direction < (right_chance + left_chance + up_chance))
		{
			targetX = tX;
			targetY = tY + 100 + (100 * Math.random());
		}
		else
		{
			targetX = tX;
			targetY = tY - 100 - (100 * Math.random());
		}
		//out.println(targetX);
		//out.println(targetY);
	}

	// turns towards the point and moves
	private void turnAndMoveToPoint(double x_Coor, double y_Coor)
	{
		double degrees_difference = returnDegressDifference(x_Coor, y_Coor, tX, tY, tHeading);
        moveCount += 1;
		
		if (Math.abs(180 - degrees_difference) < 0.1)
		{
			ahead(-distanceToPoint(targetX, targetY));
		}
		else
		{
			turnLeft(degrees_difference);
			ahead(distanceToPoint(targetX, targetY));
		}
	}

	// scans for enemy
	// turns towards centre as more likely to be there
	private void scanTowardsCentre()
	{
		double degrees_difference = returnDegressDifference((battleFieldWidth / 2), (battleFieldHeight / 2), tX, tY, tRadarHeading);
		turnRadarLeft(360 * (degrees_difference / Math.abs(degrees_difference)));
	}


	public void onScannedRobot(ScannedRobotEvent e)
	{
		//out.println("scan");
		if (!e.isSentryRobot())
		{
			// collects enemy information
            moveCount = 0;
			eDirection = tHeading - e.getBearing();
			if (eDirection >= 360)
			{
				eDirection -= 360;
			}
			eEnergy = e.getEnergy();
			eDistance = e.getDistance();
			eHeading = convertToProperDegrees(e.getHeading());
			eVelocity = e.getVelocity();
			
			eX = tX + (Math.cos(Math.toRadians(eDirection)) * eDistance);
			eY = tY + (Math.sin(Math.toRadians(eDirection)) * eDistance);

			// turns gun towards enemy
			turnGunLeft(returnDegressDifference(eX, eY, tX, tY, tGunHeading));

			if (tGunHeat == 0)
			{
				fire(3);
   			}
		}
		else
		{
			// collects sentry robot information
			double s_direction = tHeading - e.getBearing();
			double s_distance = e.getDistance();
			
			sX = tX + (Math.cos(Math.toRadians(s_direction)) * s_distance);
			sY = tY + (Math.sin(Math.toRadians(s_direction)) * s_distance);
			
			sentryPositionKnown = true;
		}
	}

	// when colliding with the wall robot moves back towards the centre
	public void onHitWall(HitWallEvent e)
	{
		double x = battleFieldWidth / 2;
		double y = battleFieldHeight / 2;
        turnLeft(returnDegressDifference(x, y, tX, tY, tHeading));
		ahead(distanceToPoint(x, y) / 3);
	}

	// collects and updates information about the robot
	public void onStatus(StatusEvent e)
	{
		RobotStatus robotStatus = e.getStatus();
		
		tX = robotStatus.getX();
	    tY = robotStatus.getY();
	    tEnergy = robotStatus.getEnergy();
	    tGunHeat = robotStatus.getGunHeat();
	
	    tVelocity = robotStatus.getVelocity();
	    tDistanceRemaining = robotStatus.getDistanceRemaining();
	    tHeading = convertToProperDegrees(robotStatus.getHeading());
	    tTurnReamaining = robotStatus.getTurnRemaining();
	
        tGunHeading = convertToProperDegrees(robotStatus.getGunHeading());
	    tGunTurnRemaining = robotStatus.getGunTurnRemaining();
		tRadarHeading = convertToProperDegrees(robotStatus.getRadarHeading());
	    tRadarTurnRemaining = robotStatus.getRadarTurnRemaining();
	}
	
	// gets turn amount in degrees between two angles
	// returns between -180 and 180
	private double getTurnAmount(double end_Degree, double start_Degree)
	{
		double turn_amount = end_Degree - start_Degree;
		
		if (turn_amount > 180)
		{
			turn_amount -= 360;
		}
		else if (turn_amount < -180)
		{
			turn_amount += 360;
		}
		
		return turn_amount;
	}

	// gets degrees amount between two locations
    private double returnDegressDifference(double toX, double toY, double fromX, double fromY, double thingHeading)
    {
        double relative_x = toX - fromX;
		double relative_y = toY - fromY;
		double point_degrees = Math.toDegrees(Math.atan2(relative_y, relative_x));
		return getTurnAmount(point_degrees, thingHeading);
    }

	// converts degree to system that I know
	// East - 0
	// Norht - 90
	// West - 180
	// South - 270
	private double convertToProperDegrees(double silly_Degrees)
	{
		/*silly_Degrees == 0 => proper_degrees == 90
		silly_Degrees == 90 => proper_degrees == 0
		silly_Degrees == 270 => proper_degrees == 180
		silly_Degrees == 180 => proper_degrees == 270
		silly_Degrees == 45 => proper_degrees == 45
		silly_Degrees == 135 => proper_degrees == 315
		silly_Degrees == 225 => proper_degrees == 225
		silly_Degrees == 315 => proper_degrees == 135*/

		double proper_degrees = 90 - silly_Degrees;
		
		if (proper_degrees < 0)
		{
			proper_degrees += 360;
		}
		return proper_degrees;
	}
	
	// returns distance to point from robot
	private double distanceToPoint(double x_Coor, double y_Coor)
	{
		return Math.sqrt(Math.pow(x_Coor - tX, 2) + Math.pow(y_Coor - tY, 2));
	}
}

