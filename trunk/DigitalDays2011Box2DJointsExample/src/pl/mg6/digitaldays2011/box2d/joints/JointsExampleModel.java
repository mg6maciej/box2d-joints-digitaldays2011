package pl.mg6.digitaldays2011.box2d.joints;

import java.util.ArrayList;
import java.util.List;

import org.jbox2d.callbacks.QueryCallback;
import org.jbox2d.collision.AABB;
import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.joints.DistanceJointDef;
import org.jbox2d.dynamics.joints.GearJointDef;
import org.jbox2d.dynamics.joints.LineJointDef;
import org.jbox2d.dynamics.joints.MouseJoint;
import org.jbox2d.dynamics.joints.MouseJointDef;
import org.jbox2d.dynamics.joints.PrismaticJoint;
import org.jbox2d.dynamics.joints.PrismaticJointDef;
import org.jbox2d.dynamics.joints.PulleyJointDef;
import org.jbox2d.dynamics.joints.RevoluteJoint;
import org.jbox2d.dynamics.joints.RevoluteJointDef;

import android.util.Log;

public class JointsExampleModel {
	
	private static final String TAG = JointsExampleModel.class.getSimpleName();

	private World world;
	private Body groundBody;
	
	private long timeAccumulator;
	
	private static final long stepInMillis = 20;
	private static final float stepInSeconds = stepInMillis / 1000.0f;
	private static final int velocityIterations = 10;
	private static final int positionIterations = 5;
	
	private List<MouseJoint> userActions = new ArrayList<MouseJoint>();
	
	public JointsExampleModel() {
		init();
	}
	
	private void init() {
		Vec2 gravity = new Vec2(0.0f, -10.0f);
		boolean doSleep = true;
		world = new World(gravity, doSleep);
		
		BodyDef groundBodyDef = new BodyDef();
		groundBodyDef.position.set(10.0f, 0.0f);
		groundBody = world.createBody(groundBodyDef);
		PolygonShape polygonShape = new PolygonShape();
		polygonShape.setAsBox(11.0f, 1.0f, new Vec2(0.0f, -1.0f), 0.0f);
		groundBody.createFixture(polygonShape, 1.0f);
		polygonShape.setAsBox(1.0f, 17.0f, new Vec2(-11.0f, 16.0f), 0.0f);
		groundBody.createFixture(polygonShape, 1.0f);
		polygonShape.setAsBox(1.0f, 17.0f, new Vec2(11.0f, 16.0f), 0.0f);
		groundBody.createFixture(polygonShape, 1.0f);
		polygonShape.setAsBox(11.0f, 1.0f, new Vec2(0.0f, 33.0f), 0.0f);
		groundBody.createFixture(polygonShape, 1.0f);
		
		createRevoluteJoint();
		createDistanceJoint();
		createPulleyJoint();
		createPrismaticJoint();
		createGearJoint();
		createLineJoint();
	}
	
	private Body createCircleBody(float x, float y, float radius) {
		BodyDef def = new BodyDef();
		def.type = BodyType.DYNAMIC;
		def.position.set(x, y);
		Body body = world.createBody(def);
		CircleShape shape = new CircleShape();
		shape.m_radius = radius;
		body.createFixture(shape, 1.0f);
		return body;
	}
	
	private Body createRectBody(float x, float y, float w, float h) {
		BodyDef def = new BodyDef();
		def.type = BodyType.DYNAMIC;
		def.position.set(x, y);
		Body body = world.createBody(def);
		PolygonShape shape = new PolygonShape();
		shape.setAsBox(w / 2.0f, h / 2.0f);
		body.createFixture(shape, 1.0f);
		return body;
	}
	
	private void createRevoluteJoint() {
		RevoluteJointDef def = new RevoluteJointDef();
		def.collideConnected = true;
		float x = 4.0f;
		float y = 4.0f;
		Body leftHand = createRectBody(x - 1.4f, y - 0.8f, 0.5f, 1.6f);
		Body rightHand = createRectBody(x + 1.0f, y, 0.5f, 1.6f);
		Body leftLeg = createRectBody(x - 0.5f, y - 2.2f, 0.5f, 2.4f);
		Body rightLeg = createRectBody(x + 0.5f, y - 2.2f, 0.5f, 2.4f);
		Body belly = createCircleBody(x, y, 1.0f);
		//belly.setFixedRotation(true);
		def.initialize(leftLeg, belly, new Vec2(x - 0.5f, y - 1.0f));
		world.createJoint(def);
		def.initialize(rightLeg, belly, new Vec2(x + 0.5f, y - 1.0f));
		world.createJoint(def);
		
		// hands do not collide with belly
		def.collideConnected = false;
		def.initialize(leftHand, belly, new Vec2(x - 1.4f, y));
		world.createJoint(def);
		
		// right hand with motor:
		def.initialize(rightHand, belly, new Vec2(x + 1.0f, y + 0.8f));
		def.enableMotor = true;
		def.motorSpeed = 2.0f;
		def.maxMotorTorque = 100.0f;
		world.createJoint(def);
	}
	
	private void createDistanceJoint() {
		DistanceJointDef def = new DistanceJointDef();
		def.collideConnected = true;
		float x = 7.0f;
		float y = 4.0f;
		Body left = createRectBody(x - 1.0f, y, 1.0f, 1.0f);
		Body right = createRectBody(x + 1.0f, y, 1.0f, 1.0f);
		Body upper = createCircleBody(x, y + 2.0f, 0.666f);
		upper.getFixtureList().setDensity(3.0f);
		upper.resetMassData();
		
		def.initialize(left, right, left.getWorldCenter(), right.getWorldCenter());
		world.createJoint(def);

		def.frequencyHz = 10.0f;
		def.dampingRatio = 0.1f;
		def.initialize(left, upper, left.getWorldCenter(), upper.getWorldCenter());
		world.createJoint(def);
		def.initialize(upper, right, upper.getWorldCenter(), right.getWorldCenter());
		world.createJoint(def);
	}
	
	private void createPulleyJoint() {
		PulleyJointDef def = new PulleyJointDef();
		def.collideConnected = true;
		float x1 = 10.0f;
		float y1 = 3.0f;
		float x2 = 12.0f;
		float y2 = 4.0f;
		Body left = createCircleBody(x1, y1, 0.5f);
		Body right = createRectBody(x2, y2, 1.0f, 1.0f);
		
		def.initialize(left, right, new Vec2(x1, y1 + 10.0f), new Vec2(x2 + 0.3f, y2 + 5.0f), left.getWorldCenter(), right.getWorldCenter(), 1.0f);
		world.createJoint(def);
	}
	
	private void createPrismaticJoint() {
		PrismaticJointDef def = new PrismaticJointDef();
		def.collideConnected = true;
		float x = 15.0f;
		float y = 6.0f;
		Body bottomLeft = createCircleBody(x - 1.0f, y - 1.0f, 0.75f);
		Body bottomRight = createCircleBody(x + 1.0f, y - 1.0f, 0.75f);
		Body topLeft = createCircleBody(x - 1.0f, y + 1.0f, 0.75f);
		Body topRight = createCircleBody(x + 1.0f, y + 1.0f, 0.75f);
		
		def.initialize(bottomLeft, bottomRight, bottomLeft.getWorldCenter(), new Vec2(1.0f, 0.0f));
		world.createJoint(def);
		def.initialize(bottomLeft, topLeft, bottomLeft.getWorldCenter(), new Vec2(0.0f, 1.0f));
		world.createJoint(def);
		def.initialize(topLeft, topRight, topLeft.getWorldCenter(), new Vec2(1.0f, 0.0f));
		world.createJoint(def);
		def.enableMotor = true;
		def.motorSpeed = 1.0f;
		def.maxMotorForce = 37.0f;
		def.enableLimit = true;
		def.lowerTranslation = -1.0f;
		def.upperTranslation = 3.0f;
		def.initialize(bottomRight, topRight, bottomRight.getWorldCenter(), new Vec2(0.0f, 1.0f));
		world.createJoint(def);
	}
	
	private void createGearJoint() {
		GearJointDef def = new GearJointDef();
		
		Body upper = createRectBody(5.0f, 9.1f, 5.0f, 1.0f);
		Body center = createCircleBody(5.0f, 8.0f, 0.5f);
		Body lower = createRectBody(5.0f, 6.9f, 5.0f, 1.0f);
		
		PrismaticJointDef prismaticJointDef = new PrismaticJointDef();
		prismaticJointDef.enableLimit = true;
		prismaticJointDef.lowerTranslation = -2.0f;
		prismaticJointDef.upperTranslation = 2.0f;
		prismaticJointDef.initialize(groundBody, upper, upper.getWorldCenter(), new Vec2(1.0f, 0.0f));
		PrismaticJoint prismaticJoint1 = (PrismaticJoint) world.createJoint(prismaticJointDef);
//		prismaticJointDef.enableMotor = true;
//		prismaticJointDef.maxMotorForce = 20.0f;
//		prismaticJointDef.motorSpeed = 5.0f;
		prismaticJointDef.initialize(groundBody, lower, lower.getWorldCenter(), new Vec2(-1.0f, 0.0f));
		PrismaticJoint prismaticJoint2 = (PrismaticJoint) world.createJoint(prismaticJointDef);
		
		RevoluteJointDef revoluteJointDef = new RevoluteJointDef();
		revoluteJointDef.initialize(groundBody, center, center.getWorldCenter());
		RevoluteJoint revoluteJoint = (RevoluteJoint) world.createJoint(revoluteJointDef);
		def.ratio = 1.0f;
		def.bodyB = center;
		def.joint2 = revoluteJoint;
		def.bodyA = upper;
		def.joint1 = prismaticJoint1;
		world.createJoint(def);
		def.bodyA = lower;
		def.joint1 = prismaticJoint2;
		world.createJoint(def);
	}
	
	private void createLineJoint() { // aka WheelJoint
		LineJointDef def = new LineJointDef();
		
		Body body = createRectBody(15.0f, 1.0f, 4.0f, 0.25f);
		Body wheel1 = createCircleBody(13.0f, 0.75f, 0.5f);
		Body wheel2 = createCircleBody(17.0f, 0.75f, 0.5f);
		
		def.enableLimit = true;
		def.lowerTranslation = 0.0f;
		def.upperTranslation = 0.3f;
		def.enableMotor = true;
		def.motorSpeed = 5.0f;
		def.maxMotorForce = 100.0f;
		def.initialize(body, wheel1, wheel1.getWorldCenter(), new Vec2(0.0f, -1.0f));
		world.createJoint(def);
		def.initialize(body, wheel2, wheel2.getWorldCenter(), new Vec2(0.0f, -1.0f));
		world.createJoint(def);
	}
	
	public void update(long dt) {
		timeAccumulator += dt;
		//int stepsDuringUpdate = 0;
		while (timeAccumulator >= stepInMillis) {
			world.step(stepInSeconds, velocityIterations, positionIterations);
			timeAccumulator -= stepInMillis;
			//stepsDuringUpdate++;
		}
		//Log.i(TAG, "steps during update: " + stepsDuringUpdate);
	}

	public Body getBodyList() {
		return world.getBodyList();
	}

	public void userActionStart(int pointerId, final float x, final float y) {
		final List<Fixture> fixtures = new ArrayList<Fixture>();
		final Vec2 vec = new Vec2(x, y);
		world.queryAABB(new QueryCallback() {
			public boolean reportFixture(Fixture fixture) {
				Log.i(TAG, "reportFixture: " + fixture);
				//if (fixture.testPoint(vec)) {
					fixtures.add(fixture);
				//}
				return true;
			}
		}, new AABB(vec, vec));
		if (fixtures.size() > 0) {
			Fixture fixture = fixtures.get(0);
			Log.i(TAG, "creating mouse joint: " + fixture);
			Body body = fixture.getBody();
			
			MouseJointDef def = new MouseJointDef();
			def.bodyA = body;
			def.bodyB = body;
			def.maxForce = 1000.0f * body.getMass();
			def.target.set(x, y);
			
			MouseJoint joint = (MouseJoint) world.createJoint(def);
			joint.m_userData = pointerId;
			userActions.add(joint);
		}
	}

	public void userActionUpdate(int pointerId, float x, float y) {
		for (MouseJoint joint : userActions) {
			if (pointerId == (Integer) joint.m_userData) {
				joint.setTarget(new Vec2(x, y));
				break;
			}
		}
	}

	public void userActionEnd(int pointerId, float x, float y) {
		for (MouseJoint joint : userActions) {
			if (pointerId == (Integer) joint.m_userData) {
				world.destroyJoint(joint);
				userActions.remove(joint);
				break;
			}
		}
	}
}
