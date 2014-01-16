
import javax.media.opengl.GL;
import static javax.media.opengl.GL2.*;
import javax.media.opengl.glu.GLUquadric;
import robotrace.Base;
import robotrace.Vector;
import static java.lang.Math.*;


// Cássio Holanda Gonçalves 0877290 
// Eduardo Piñar Menoyo 0875997


/**
 * Handles all of the RobotRace graphics functionality,
 * which should be extended per the assignment.
 * 
 * OpenGL functionality:
 * - Basic commands are called via the gl object;
 * - Utility commands are called via the glu and
 *   glut objects;
 * 
 * GlobalState:
 * The gs object contains the GlobalState as described
 * in the assignment:
 * - The camera viewpoint angles, phi and theta, are
 *   changed interactively by holding the left mouse
 *   button and dragging;
 * - The camera view width, vWidth, is changed
 *   interactively by holding the right mouse button
 *   and dragging upwards or downwards;
 * - The center point can be moved up and down by
 *   pressing the 'q' and 'z' keys, forwards and
 *   backwards with the 'w' and 's' keys, and
 *   left and right with the 'a' and 'd' keys;
 * - Other settings are changed via the menus
 *   at the top of the screen.
 * 
 * Textures:
 * Place your "track.jpg", "brick.jpg", "head.jpg",
 * and "torso.jpg" files in the same folder as this
 * file. These will then be loaded as the texture
 * objects track, bricks, head, and torso respectively.
 * Be aware, these objects are already defined and
 * cannot be used for other purposes. The texture
 * objects can be used as follows:
 * 
 * gl.glColor3f(1f, 1f, 1f);
 * track.bind(gl);
 * gl.glBegin(GL_QUADS);
 * gl.glTexCoord2d(0, 0);
 * gl.glVertex3d(0, 0, 0);
 * gl.glTexCoord2d(1, 0);
 * gl.glVertex3d(1, 0, 0);
 * gl.glTexCoord2d(1, 1);
 * gl.glVertex3d(1, 1, 0);
 * gl.glTexCoord2d(0, 1);
 * gl.glVertex3d(0, 1, 0);
 * gl.glEnd(); 
 * 
 * Note that it is hard or impossible to texture
 * objects drawn with GLUT. Either define the
 * primitives of the object yourself (as seen
 * above) or add additional textured primitives
 * to the GLUT object.
 */
public class RobotRace extends Base {
    
    /** Array of the four robots. */
    private final Robot[] robots;
    
    /** Instance of the camera. */
    private final Camera camera;
    
    /** Instance of the race track. */
    private final RaceTrack raceTrack;
    
    /** Instance of the terrain. */
    private final Terrain terrain;
    
    /**
     * Constructs this robot race by initializing robots,
     * camera, track, and terrain.
     */
    public RobotRace() {
        
        // Create a new array of four robots
        robots = new Robot[4];
        
        // Initialize robot 0
   // Initialize robot 0
        robots[0] = new Robot(Material.GOLD, 8.5d, 0d, 1d, 0.8d, 0.4d, 1.8d, 0.3d, 100);
        
        // Initialize robot 1
        robots[1] = new Robot(Material.SILVER, 9.5d, 0d, 1d, 0.8d, 0.4d, 1.8d, 0.3d, 150);
        
        // Initialize robot 2
        robots[2] = new Robot(Material.WOOD, 10.5d, 0d, 1d, 0.8d, 0.4d, 1.8d, 0.3d, 200);

        // Initialize robot 3
        robots[3] = new Robot(Material.ORANGE, 11.5d, 0d, 1d, 0.8d, 0.4d, 1.8d, 0.3d, 250);
       
        // Initialize the camera
        camera = new Camera();
        
        // Initialize the race track
        raceTrack = new RaceTrack();
        
        // Initialize the terrain
        terrain = new Terrain();
        
        
          
    }
    
    /**
     * Called upon the start of the application.
     * Primarily used to configure OpenGL.
     */
    @Override
    public void initialize() {        
        // Enable blending.
        gl.glEnable(GL_BLEND);
        gl.glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        
        // Enable anti-aliasing.
        gl.glEnable(GL_LINE_SMOOTH);
        gl.glEnable(GL_POLYGON_SMOOTH);
        gl.glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
        gl.glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
        
        // Enable depth testing.
        gl.glEnable(GL_DEPTH_TEST);
        gl.glDepthFunc(GL_LESS);
        
        // Enable textures. 
        gl.glEnable(GL_TEXTURE_2D);
        gl.glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
        gl.glBindTexture(GL_TEXTURE_2D, 0);
        
        
    }
    
    /**
     * Configures the viewing transform.
     */
    @Override
    public void setView() {
        // Select part of window.
        gl.glViewport(0, 0, gs.w, gs.h);
        
        // Set projection matrix.
        gl.glMatrixMode(GL_PROJECTION);
        gl.glLoadIdentity();

        // Set the perspective.
        /*
        the angle is calculated using the distance between the eye point and the center point
        and the width given by the assigment
        */
        glu.gluPerspective(toDegrees(atan2(gs.vWidth/2,gs.vDist)) * 2,
                          (float)gs.w / (float)gs.h, 0.1*gs.vDist, 10.0*gs.vDist);
        
        // Set camera.
        gl.glMatrixMode(GL_MODELVIEW);
        gl.glLoadIdentity();
               
        // Update the view according to the camera mode
        camera.update(gs.camMode);
        glu.gluLookAt(camera.eye.x(),    camera.eye.y(),    camera.eye.z(),
                      camera.center.x(), camera.center.y(), camera.center.z(),
                      camera.up.x(),     camera.up.y(),     camera.up.z());
        
    }
    
    /**
     * Draws the entire scene.
     */
    @Override
    public void drawScene() {
        
        // Background color.
        gl.glClearColor(1f, 1f, 1f, 0f);
        
        // Clear background.
        gl.glClear(GL_COLOR_BUFFER_BIT);
        
        // Clear depth buffer.
        gl.glClear(GL_DEPTH_BUFFER_BIT);
        
        // Set color to black.
        gl.glColor3f(0f, 0f, 0f);
        
        gl.glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        
        // Draw the axis frame
        if (gs.showAxes) {
            drawAxisFrame();
        }
        
        // Draw the first robot
        robots[0].draw(false);
        robots[1].draw(false);
        robots[2].draw(false);
        robots[3].draw(false);
       
        // Draw race track
        raceTrack.draw(gs.trackNr);
        
        // Draw terrain
        terrain.draw();
        
        
        raceTrack.runRobots(robots, gs.trackNr);
    }
    
    
    /**
     * Draws the x-axis (red), y-axis (green), z-axis (blue),
     * and origin (yellow).
     */
    public void drawAxisFrame() {
    /**
     * We defined a method that draws 1 axis and then we apply a rotation
     * matrix before redrawing the other axis with this same function
     */
        
        //Axis
        gl.glPushMatrix();
           //X Axis
           drawOneAxis(1f, 0f, 0f); //we have creasted this function.

           //Y axis
           /*rotating it X around Z axis, towards Y axis */
           gl.glRotatef(90, 0f, 0f, 1f);
           drawOneAxis(0f, 1f, 0f);

           //Z axis
           /* now, rotating X around Y axis, towards Z axis
           (90 degrees negastively or 270 degrees positively)*/
           gl.glRotatef(90, 0f, -1f, 0f);
           drawOneAxis(0f, 0f, 1f);
           
        gl.glPopMatrix();
         
       
        // Sphere showing origin
        gl.glPushMatrix();
            gl.glColor3f(1f, 1f, 0f); //using yeallow color
            glut.glutSolidSphere(0.12f, 10, 10); //drawing a small sphere with a small number of slices
        gl.glPopMatrix();
    
        
    }
    
    private void drawOneAxis(float red, float green, float blue) {
    /**  
     *  draw one set of block and cone forming a arrow to represent an Axis
     *  having the color of the axis as argument. This axis will be draw pointing
     *  to X axis. The other 2 axis need a rotation preceding this draw.
     *  This method is used inside "drawAxisFrame()" method. 
     *
     *  Our blocks will be created with side's length of 1f but we will 
     *  scale one of the sides to 0.7f corresponding to its respective axis.
     *   
     *  Our cones will be created with height 0.3f so both cone and block will
     *  together have 1f in  length.  
     */
        
        //drawing X axis
        gl.glPushMatrix();

            /* applying tranlation of half the length of our future block */
            gl.glTranslatef(0.35f, 0f, 0f);

            gl.glPushMatrix(); //pushing a mastrix to apply scale
                // scaling the block on x axis to the length we desire and narrowing in the other axis
                gl.glScalef(0.7f, 0.05f, 0.05f); 
                gl.glColor3f(red, green, blue); // using red color
                glut.glutSolidCube(1f); // drawing the cube
            gl.glPopMatrix(); //getting back to the translationated only matrix

            /* rotating the future cone 90 degrees around Y axis 
            (poiting Z to the positive side of the original X axis) */
            gl.glRotatef(90f, 0f, 1f, 0f); 
            /* tranlating the cone to on Z axis, towards the end of the block */
            gl.glTranslatef(0f, 0.f, 0.35f);
            /* drawing the cone with height 0.3f so both objects together are 1f in lenght */
            glut.glutSolidCone(0.07f, 0.3f, 50, 2);
            
        gl.glPopMatrix();
    }
    
    /**
     * Materials that can be used for the robots.
     */
    public enum Material {
        
        /** 
         * Gold material properties.
         * Modify the default values to make it look like gold.
         */
        GOLD (
            new float[] {0.8f, 0.8f, 0.8f, 1.0f},
            new float[] {0.0f, 0.0f, 0.0f, 1.0f}),
        
        /**
         * Silver material properties.
         * Modify the default values to make it look like silver.
         */
        SILVER (
            new float[] {0.8f, 0.8f, 0.8f, 1.0f},
            new float[] {0.0f, 0.0f, 0.0f, 1.0f}),
        
        /** 
         * Wood material properties.
         * Modify the default values to make it look like wood.
         */
        WOOD (
            new float[] {0.8f, 0.8f, 0.8f, 1.0f},
            new float[] {0.0f, 0.0f, 0.0f, 1.0f}),
        
        /**
         * Orange material properties.
         * Modify the default values to make it look like orange.
         */
        ORANGE (
            new float[] {0.8f, 0.8f, 0.8f, 1.0f},
            new float[] {0.0f, 0.0f, 0.0f, 1.0f});
        
        /** The diffuse RGBA reflectance of the material. */
        float[] diffuse;
        
        /** The specular RGBA reflectance of the material. */
        float[] specular;
        
        /**
         * Constructs a new material with diffuse and specular properties.
         */
        private Material(float[] diffuse, float[] specular) {
            this.diffuse = diffuse;
            this.specular = specular;
        }
    }
    
    /**
     * Represents a Robot, to be implemented according to the Assignments.
     */
    private class Robot {
        
        /** The material from which this robot is built. */
        private final Material material;
        
        
        private Vector torsoA, torsoB, torsoC, torsoD; //top surface points of torso
        private Vector torsoE, torsoF, torsoG, torsoH; //bottom surface points of torso
        private Vector leftLeg, rightLeg, leftArm, rightArm, neckAndHead; // points where legs, arms and neck will join the toso
        private double height, //height of the robot
                shoulderHeight, //distance between floor and top surface of the torso
                waistHeight, // distance between floor and bottom surface of the torso
                torsoThickness, // how fat is the robot
                torsoHeight, // distance between top and bottom surfaces of the torso
                rotationAngle; //the angle the robot will need to rotate while running in the track
        Shapes shapeDrawer; //object that will draw some parts of the robot
        SolidShapes solid = new SolidShapes(); //implementation of the drawing of some parts of the robot as solid shapes
        WiredShapes wired = new WiredShapes(); //implementation of the drawing of some parts of the robot as wired shapes
        int robotSpeed;
        
        public Vector position; //position of the robot
        
        int minLimbAngle = -40; //the min angle the limb can rotate to
        int maxLimbAngle = 60; //the max angle a limb can rotate to
        int limbMovementAngle = maxLimbAngle - minLimbAngle; //the total arc of movement of the limb in degrees
        double limbAngle;
        
        
        /**
         * Constructs the robot with initial parameters.
         */
        public Robot(Material material, 
                double x, double y, double z, 
                double shoulderWidth, double waistWidth, double heightOfRobot, double thicknessOfTorso, int speed) {
            
            this.material = material;
            
            // initial position of robot
            this.position = new Vector(x,y,z);
            robotSpeed = speed;
            
            // width of the shoulder of the robot. shoulder cannot be too narrow
            shoulderWidth /= 2;
            if (shoulderWidth < 0.4d) shoulderWidth = 0.4d;
            
            // width of the waist of the robot. waist cannot be too narrow
            waistWidth /= 2;
            if (waistWidth < 0.2d) waistWidth = 0.2d;
            
            // height of the robot, it cannot be too short
            height = heightOfRobot;
            if (height < 1.5d) height = 1.5d;
            
            //calculation proportion between neck and head and the rest of the body
            shoulderHeight = height * 0.8;
            waistHeight = shoulderHeight/2;
            torsoHeight = shoulderHeight - waistHeight;
            
            // thickness of the torso of the robot. torso cannot be too thin
            torsoThickness = thicknessOfTorso/2;
            if (torsoThickness < 0.1d) torsoThickness = 0.1d;
            
            //points of the top surface of the torso
            torsoA = new Vector(-shoulderWidth, -torsoThickness, torsoHeight);
            torsoB = new Vector(-shoulderWidth, torsoThickness, torsoHeight);
            torsoC = new Vector(shoulderWidth, torsoThickness, torsoHeight);
            torsoD = new Vector(shoulderWidth, -torsoThickness, torsoHeight);
            
            //points of the bottom surface of the torso
            torsoE = new Vector(-waistWidth, -torsoThickness, 0);
            torsoF = new Vector(-waistWidth, torsoThickness, 0);
            torsoG = new Vector(waistWidth, torsoThickness, 0);
            torsoH = new Vector(waistWidth, -torsoThickness, 0);
            
            //points where the arms, legs and neck will be placed
            leftLeg = torsoH.subtract(torsoG).scale(0.5d).add(torsoG);
            rightLeg = torsoE.subtract(torsoF).scale(0.5d).add(torsoF);
            leftArm = torsoA.subtract(torsoB).scale(0.5d).add(torsoB);
            rightArm = torsoD.subtract(torsoC).scale(0.5d).add(torsoC);
            neckAndHead = torsoA.subtract(torsoC).scale(0.5d).add(torsoC);
        }
                
        /**
         * Draws this robot (as a {@code stickfigure} if specified).
         */
        public void draw(boolean stickFigure) {
            gl.glPushMatrix();
                //methos that will calculate the rotation of each limb
                limbAngle();
                //the point where the robot starts to be drawn
                gl.glTranslated(position.x(), position.y(), position.z() + torsoHeight + runningBouce());
                //direction the robot is facing, only rotating in the Z axis as the track is flat
                //rotationAngle is calculated in another method
                gl.glRotated(rotationAngle, 0f, 0f, 1f); 
                
                //change class of implementations depending on stickFigure boolean value
                if(stickFigure){
                    shapeDrawer = wired;
                } else {
                    shapeDrawer = solid;
                }
                
                //Torso
                //inclination of the torso depending on speed. the faster the robot the more the torso inclines forward
                gl.glRotated(robotSpeed/10, -1f, 0f, 0f);
                //the 
                shapeDrawer.drawDeformedCube(new Vector[]{torsoA, torsoB, torsoC, torsoD}, 
                                             new Vector[]{torsoE, torsoF, torsoG, torsoH});
                
                //Left Leg
                drawLeg(leftLeg, true);
                //Right Right
                drawLeg(rightLeg, false);
                //Left arm
                drawArm(leftArm, true);
                //Right arm
                drawArm(rightArm, false);
                //Head
                drawHead(neckAndHead);
                
            gl.glPopMatrix();
        }
       
        /**
         * Returns a double with a value representing the height about the ground the robot
         * should be drawn to show he is having little jumps while running. It simulate the
         * effect of having each leg hitting the floor with step.
         */
        private double runningBouce(){
            double halfLimbAngle;
            /**
             * bouncing height should meet its maximum value at half the arc value of total limb possible rotation
             * bouncing height value should reach its maximum value for the second time when limbAngle reach for the first time
             * it means the robot will bounce up and down twice for each movement of one leg. 
             * it makes the effect of each leg moving the robot up for an impulse.
             * So we need a variable to behave like described and we are using halfLimbAngle
             */
            if (limbAngle < limbMovementAngle / 2 ) 
                halfLimbAngle = limbAngle;
            else 
                halfLimbAngle = limbMovementAngle - limbAngle;
            
            /**
             * up to here, halfLimbAngle is in function of limbAngle and still means nothing.
             * We have decided that the maximum value for bounceHeight has to be 20% of
             * robot's height. So, 20% of the robot's height is related to the max value 
             * of bounceHeight, which is half limbMovementAngle, and the halfLimbAngle is related to the bouncing height
             */
            return (height * 0.2 / limbMovementAngle) * halfLimbAngle;
        }
        
        /**
         * Calculate the angle a limb of the robot should rotate given the time and speed
         * of the robot.
         */
        private void limbAngle() {
            /**
             * multiplying the gs.tAnim by the speed of the robot, give me an increasing 
             * number that is bigger, the bigger the robotSpeed is. Then, this number
             * can be used to define how fast the robot limbs rotate move. Because
             * the faster the robot, the faster its limbs should rotate.
             */
            double time = gs.tAnim * robotSpeed;
            limbAngle = time % limbMovementAngle;
            if( (((long)time)/limbMovementAngle)%2 == 1 ) 
                limbAngle = limbMovementAngle - limbAngle;
        }
                
        /**
         * calculates the rotation of a limb and set is rotated position
         * taking into consideration if that limb is moving backwards or forward
         * (boolean backwards mean if the limb is the right side or left side)
         */
        private void limbRotation(boolean backwards) {
            if(backwards) 
                gl.glRotated(maxLimbAngle - limbAngle, 1f, 0f, 0f);
            else 
                gl.glRotated(minLimbAngle + limbAngle, 1f, 0f, 0f);
        }
        
        /**
         * Draw one leg based on the point where the leg should be drawn
         * and rotate this leg every drawing to animate the walking.
         * parameter backwards specify weather it should start rotating backwards
         * or not. which means each leg have opposite rotations.
         */
        private void drawLeg(Vector point, boolean backwards) {
            gl.glPushMatrix();
                //position of the connection between torso and leg
                gl.glTranslated(point.x(), point.y(), point.z());
                
                limbRotation(backwards);
                
                gl.glColor3f(1f, 1f, 0f);
                shapeDrawer.drawSphere(torsoThickness, 15, 15);
                
                //position of leg relatated to connection
                gl.glTranslated(0f, 0f, -waistHeight/2);
                gl.glScaled(torsoThickness, torsoThickness, waistHeight);
                gl.glColor3f(1f, 0.6f, 0f);
                shapeDrawer.drawCube(1f);
            gl.glPopMatrix();
        }
        
       /**
         * Draw one arm based on the point where the arm should be drawn
         * and rotate this arm every drawing to animate the walking.
         * parameter backwards specify weather it should start rotating backwards
         * or not. which means each arm have opposite rotations.
         */
        private void drawArm(Vector point, boolean backwards) {
            //to keep proportion, the arm thickness is based on torso thickness value
            double armThickess = torsoThickness*2/3;
            
            gl.glPushMatrix();
                //postion of the connection between torso and arm
                gl.glTranslated(point.x(), point.y(), point.z() -armThickess);
                
                limbRotation(backwards);
                
                gl.glColor3f(1f, 1f, 0f);
                shapeDrawer.drawSphere(armThickess, 10, 10);
                
                //position of arm relatated to connection
                gl.glTranslated(0f, 0f, -(torsoHeight/2));
                gl.glScaled(armThickess, armThickess, torsoHeight);
                gl.glColor3f(1f, 0.6f, 0f);
                shapeDrawer.drawCube(1f);
            gl.glPopMatrix();
        }
        
        private void drawHead(Vector point) {
            double neckRadius = torsoThickness*4/5;
            double headHeight = height*0.4/3;
            
            gl.glPushMatrix();
                //postion of the neck, between torso and head
                gl.glTranslated(point.x(), point.y(), point.z());
                gl.glColor3f(1f, 1f, 0f);
                shapeDrawer.drawSphere(neckRadius, 15, 15);
                
                //position of head relatated to neck
                gl.glTranslated(0d, 0d, neckRadius);
                gl.glColor3f(1f, 0.6f, 0f);
                GLUquadric quadric = glu.gluNewQuadric();
                shapeDrawer.drawCylinder(glu.gluNewQuadric(), torsoThickness, torsoThickness, headHeight, 15, 15);
                
                //disk to close bottom of cylinder
                gl.glColor3f(1f, 0.3f, 0f);
                shapeDrawer.drawDisk(quadric, 0, torsoThickness, 15, 15);
                
                //disk to close top of cylinder
                gl.glTranslated(0d, 0d, headHeight);
                shapeDrawer.drawDisk(quadric, 0, torsoThickness, 15, 15);
                glu.gluDeleteQuadric(quadric);
            gl.glPopMatrix();
        }
        
        //exposed method to set robots position
        public void setRobotPosition(Vector point) {
            position = point;
        }
        
        //based on the tangent of the robot position in the track, rotates the direction the robot is facing
        public void setRobotRotation(double angle) {
            rotationAngle = angle;
        }
        
    }
    
    //interface that will be used to drawn volumes in wired or solid shapes
    interface Shapes {
        public void drawSphere(double a, int b, int c);
        public void drawCube(float a);
        public void drawCylinder(GLUquadric q, double a, double b, double c, int d, int e);
        public void drawDisk(GLUquadric q, double a, double b, int c, int d);
        public void drawDeformedCube(Vector[] topPoints, Vector[] bottomPoints);
    }
    
    //implementation of the solid shapes drawing
    class SolidShapes extends Drawer implements Shapes {
        SolidShapes() {
            typeOfLine = GL_QUADS;
        }
        
        @Override
        public void drawSphere(double a, int b, int c){
            glut.glutSolidSphere(a, b, c);
        }
        
        @Override
        public void drawCube(float a){
            glut.glutSolidCube(a);
        }
        
        @Override
        public void drawCylinder(GLUquadric q, double a, double b, double c, int d, int e) {
            glu.gluCylinder(q, a, b, c, d, e);
        }
        
        @Override
        public void drawDisk(GLUquadric q, double a, double b, int c, int d) {
            glu.gluDisk(q, a, b, c, d);
        }
        
        @Override
        public void drawDeformedCube(Vector[] topPoints, Vector[] bottomPoints){
            drawDeformedCubeImplementation(topPoints, bottomPoints);
        }
    }
    
    //implementation of the wired shapes drawing
    class WiredShapes extends Drawer implements Shapes {
        WiredShapes() {
            typeOfLine = GL_LINE_LOOP;
        }
        
        @Override
        public void drawSphere(double a, int b, int c){
            glut.glutWireSphere(a, b, c);
        }
        
        @Override
        public void drawCube(float a){
            glut.glutWireCube(a);
        }
        
        @Override
        public void drawCylinder(GLUquadric q, double a, double b, double c, int d, int e) {
            gl.glPushMatrix();
                glu.gluDisk(q, a+0.01d, a, d, d);
                gl.glTranslated(0d, 0d, c);
                glu.gluDisk(q, a+0.01d, b, d, d);
            gl.glPopMatrix();
        }
        
        @Override
        public void drawDisk(GLUquadric q, double a, double b, int c, int d) {
            glu.gluDisk(q, b+0.01d, b, c, d);
        }
        
        @Override
        public void drawDeformedCube(Vector[] topPoints, Vector[] bottomPoints){
            drawDeformedCubeImplementation(topPoints, bottomPoints);
        }
    }
    
    //implementing the drawing of the torso
    class Drawer {
        protected int typeOfLine;
        
        //the torso has 6 surfaces that should be drawn based on 8 vertexes given as parameters, top and bottom vertexes.
        protected void drawDeformedCubeImplementation(Vector[] topPoints, Vector[] bottomPoints){
            //checking if the size of the arrays are correct
            if(topPoints.length == 4 && bottomPoints.length == 4) { 
                //array of points that will form one quad surface to be drawn
                Vector[] points = new Vector[4]; 
                
                for(int i = 0; i < 6; i++){ //the 6 surfaces
                    //making each surface slight lighter than the other so we can clearly see each of them
                    gl.glColor3f(0.1f*(i+1), 0.1f*i, 0.1f*i);
                    if (i < 4) { // 4 sides surfaces
                        //all the side surfaces use two top vertexes and two bottom vertexes
                        //the vertexes in topPoints and bottomPoints array should be, and are, ordered and aligned with each array
                        //the first vertex on topPoints is right above the first vertex on bottomPoints, and so on.
                        points[0] = topPoints[i];
                        points[1] = bottomPoints[i];
                        points[2] = bottomPoints[(i+1)%4];
                        points[3] = topPoints[(i+1)%4];
                    } else if (i == 4) { // top surface
                        points = topPoints;
                    } else if (i == 5) { // bottom surfaces
                        points = bottomPoints;
                    }
                    //now that the vertexes are selected, the quad surface can be drawn
                    drawQuadSurface(points);
                }
            }
        }
        
        //drawing one quad surface
        private void drawQuadSurface(Vector[] points){
            if(points.length == 4) {
                gl.glBegin(typeOfLine); //type of line is specified by the class that extends this class
                    for(int j = 0; j < 4; j++)
                        gl.glVertex3d(points[j].x(), points[j].y(), points[j].z());
                gl.glEnd();
            }
        }
    }

    /**
     * Implementation of a camera with a position and orientation. 
     */
    private class Camera {
        
        /** The position of the camera. */
        public Vector eye;
        
        /** The point to which the camera is looking. */
        public Vector center;
        
        /** The up vector. */
        public Vector up;

        /**
         * Constructor
         */
        public Camera(){
            setDefaultMode();
            
//            float[] lightAmbient = {0.5f, 0.5f, 0.5f, 1.0f};
//            float[] lightDiffuse = {1.0f, 1.0f, 1.0f, 1.0f};
//            float[] lightPosition =  {0.0f, 0.0f, 2.0f, 1.0f}; //{(float)eye.x() - 1.0f, (float)eye.y() + 1.0f, (float)eye.z()};
//            
//            gl.glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient, 0);
//            gl.glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse, 0);
//            gl.glLightfv(GL_LIGHT0, GL_POSITION, lightPosition, 0);
//            
//            gl.glEnable(GL_LIGHT0);
//            gl.glDisable(GL_LIGHTING);
        }
        
        /**
         * Updates the camera viewpoint and direction based on the
         * selected camera mode.
         */
        public void update(int mode) {
            
            
            // Helicopter mode
            if (1 == mode) {  
                setHelicopterMode();
                
            // Motor cycle mode
            } else if (2 == mode) { 
                setMotorCycleMode();
                
            // First person mode
            } else if (3 == mode) { 
                setFirstPersonMode();
                
            // Auto mode
            } else if (4 == mode) { 
                setAutoMode();
                
            // Default mode
            } else {
                setDefaultMode();
            }
            
            
        }
        
        /**
         * Computes {@code eye}, {@code center}, and {@code up}, based
         * on the camera's default mode.
         */
        private void setDefaultMode() {
            
            /* Deriving position of the eye point (Xe, Ye, Ze) from parameters
            We want to set the values of the eye point: Xe, Ye and Ze.
            we have the center point (Xc, Yc, Zc), distance between eye point
            and center point and the values of the angles phi and theta (according
            to assignment paper)
            
            The difference between the Z value of eye point and center point is 
            the Z value of the eye point (Ze) minus the Z value of the center 
            point (Zc):
                Ze - Zc = height -> Ze = height + Zc
            this height can be calculated as:
                height = gs.cnt.z() + vdist * sin(phi)
            
            same for the difference in X and Y coordinates, but now using the
            projection of V on the plane XY;
            for X coordinate:
                Xe - Xc = xDif -> Xe = xDif + Xc
            calculated as:
                Xe = gs.cnt.x() + vdist * cos(phi) * cos(theta)
            
            for Y coordiante:
                Ye - Yc = yDif -> Ye = yDif + Yc
            calculated as:
                Ye = gs.cnt.y() + vdist * cos(phi) * sin(theta)
            */
            
            
            //calculating values of the angles phi and theta
            double phiCos,phiSin,thetaCos,thetaSin,m,x,y,z;
            phiCos = cos(gs.phi);
            phiSin = sin(gs.phi);
            thetaCos = cos(gs.theta);
            thetaSin = sin(gs.theta);
            m=phiCos*gs.vDist;
            x=thetaCos*m;
            y=thetaSin*m;
            z=phiSin*gs.vDist;
            //setting new vector for eye point
            this.eye = new Vector(x,y,z);
            this.center=gs.cnt;
            this.up=Vector.Z;
        }
        
        /**
         * Computes {@code eye}, {@code center}, and {@code up}, based
         * on the helicopter mode.
         */
        private void setHelicopterMode() {
            double x,y;
            double time = (gs.tAnim/10)%1;
            x=raceTrack.getPoint(time).x();
            y=raceTrack.getPoint(time).y();
            
            this.eye = new Vector(x,y,15);
            this.center=raceTrack.getPoint(time);
            this.up=raceTrack.getTangent(time);
            
        }
        
        /**
         * Computes {@code eye}, {@code center}, and {@code up}, based
         * on the motorcycle mode.
         */
        private void setMotorCycleMode() {
            double x,y,distX,distY;
            double time = (gs.tAnim/10)%1;
            x=raceTrack.getPoint(time).x();
            y=raceTrack.getPoint(time).y();
            distX=10*cos(2*PI*time);
            distY=10*sin(2*PI*time);
            
            this.eye = new Vector(x+distX,y+distY,2);
            this.center=raceTrack.getPoint(time);
            this.up=Vector.Z;
        }
        
        /**
         * Computes {@code eye}, {@code center}, and {@code up}, based
         * on the first person mode.
         */
        private void setFirstPersonMode() {
            double x,y,z;
            double time = (gs.tAnim/10)%1;
            x=raceTrack.getPoint(time).x();
            y=raceTrack.getPoint(time).y();
            z=robots[robots.length-1].height;
            
            this.eye = new Vector(x,y,z);
            this.center=raceTrack.getTangent(time);
            this.up=Vector.Z;
        }
        
        /**
         * Computes {@code eye}, {@code center}, and {@code up}, changing each 
         * short period of time to the different camera modes
         */
        int secondsToChangeMode = 5; //seconds before changing to next camera in auto mode
        float lastChangeTime = 0; // last gs.tAnim when a camera has been changed in auto mode
        int nextCamera = 0; // number of the next camera to change to in auto mode
        private void setAutoMode(){
            if (gs.tAnim - lastChangeTime > secondsToChangeMode) {
                lastChangeTime = gs.tAnim;
                nextCamera++;
                if (nextCamera == 4 ) 
                    nextCamera = 1;
            }
            update(nextCamera);
        }
        
    }
    
    /**
     * Implementation of a race track that is made from Bezier segments.
     */
    private class RaceTrack {
        
        /** Array with control points for the O-track. */
        private Vector[] controlPointsOTrack;
        
        /** Array with control points for the L-track. */
        private Vector[] controlPointsLTrack;
        
        /** Array with control points for the C-track. */
        private Vector[] controlPointsCTrack;
        
        /** Array with control points for the custom track. */
        private Vector[] controlPointsCustomTrack;
        
        /**
         * Constructs the race track, sets up display lists.
         */
        public RaceTrack() {
            /**
             * Fill the cotrolPointsOTrack Array with the control points (outside line of the track)
             * For O track there are 4 segments, so we need 3*4=12 control Points
             */
            controlPointsOTrack=new Vector[]{new Vector(-20,0,1),new Vector(-20,10,1),new Vector(-10,20,1),
                new Vector(0,20,1),new Vector(10,20,1),new Vector(20,10,1),new Vector(20,0,1),new Vector(20,-10,1),
                new Vector(10,-20,1),new Vector(0,-20,1),new Vector(-10,-20,1),new Vector(-20,-10,1),new Vector(-20,0,1)};
            
        }
        
        /**
         * Draws this track, based on the selected track number.
         */
        public void draw(int trackNr) {
            
            // The test track is selected
            if (0 == trackNr) {
                           
                    double n=50;    //number of points to draw the raceTrack
                    double dt=1/n; 
                    //This loop draws the top face of the Race Track
                    gl.glColor3f(1.0f,1.0f,1.0f);
                    track.bind(gl);
                    gl.glBegin(GL_QUADS);
                        for(int i = 0; i < n; i++){
                            Vector v = getPoint(dt*i); //Centerline
                            double x,y,z,distX,distY;
                            x=v.x();    y=v.y();    z=v.z();
                            distX=2*cos(2*PI*dt*i);
                            distY=2*sin(2*PI*dt*i);
                            gl.glTexCoord2d(0, 0);
                            gl.glVertex3d(x-distX, y-distY, z);     //back interior line
                            gl.glTexCoord2d(1, 0);
                            gl.glVertex3d(x+distX, y+distY, z);     //back exterior line
                            
                            int k = i + 1;
                            v = getPoint(dt*k);
                            x=v.x();    y=v.y();    z=v.z();
                            distX=2*cos(2*PI*dt*k);
                            distY=2*sin(2*PI*dt*k);
                            gl.glTexCoord2d(1, 1);
                            gl.glVertex3d(x+distX, y+distY, z);     //front exterior line
                            gl.glTexCoord2d(0, 1);
                            gl.glVertex3d(x-distX, y-distY, z);     //front interior line
                        }
                    gl.glEnd();
                    
                    //This loop draws the exterior side of the Race Track
                    gl.glColor3f(1f,1f,1f);
                    brick.bind(gl);
                    gl.glBegin(GL_QUADS);
                        for(int i = 0; i < n; i++){         
                            Vector v = getPoint(dt*i);
                            double x1,y1,z1,distXBack,distYBack;
                            x1 = v.x();    y1 = v.y();    z1 = v.z();
                            distXBack=2*cos(2*PI*dt*i);
                            distYBack=2*sin(2*PI*dt*i);
                            gl.glTexCoord2d(0, 1);
                            gl.glVertex3d(x1+distXBack, y1+distYBack, z1);       //back superior(exterior) line
                            gl.glTexCoord2d(0, 0);
                            gl.glVertex3d(x1+distXBack, y1+distYBack, -1.0);     //back inferior(exterior) line
                            
                            int k = i + 1;
                            v = getPoint(dt*k);
                            double x2, y2, z2, distXFront, distYFront;
                            x2 = v.x();    y2 = v.y();    z2 = v.z();
                            distXFront=2*cos(2*PI*dt*k);
                            distYFront=2*sin(2*PI*dt*k);
                            gl.glTexCoord2d(1, 0);
                            gl.glVertex3d(x2+distXFront, y2+distYFront, -1.0);     //front inferior(exterior) line
                            gl.glTexCoord2d(1, 1);
                            gl.glVertex3d(x2+distXFront, y2+distYFront, z2);       //front superior(exterior) line
                            
                            
                            gl.glTexCoord2d(1, 1);
                            gl.glVertex3d(x1-distXBack, y1-distYBack, z1);     //back superior(interior) line
                            gl.glTexCoord2d(1, 0);
                            gl.glVertex3d(x1-distXBack, y1-distYBack, -1.0);   //back inferior(interior) line
                            gl.glTexCoord2d(0, 0);
                            gl.glVertex3d(x2-distXFront, y2-distYFront, -1.0); //front inferior(interior) line
                            gl.glTexCoord2d(0, 1);
                            gl.glVertex3d(x2-distXFront, y2-distYFront, z2);   //front superior(interior) line
                        }    
                    gl.glEnd();
                    

            // The O-track is selected
            } else if (1 == trackNr) {
                double n = 30;
                double dt=1.0/n;
                gl.glBegin(GL_LINE_LOOP);
                gl.glColor3f(0f,0f,0f);
                for(int i=0;i<10;i+=3){
                    Vector p0=controlPointsOTrack[i];
                    Vector p1=controlPointsOTrack[i+1];
                    Vector p2=controlPointsOTrack[i+2];
                    Vector p3=controlPointsOTrack[i+3];
                    for(int j=0;j<n;j++){
                        Vector v;
                        v = getCubicBezierPnt(dt*j, p0, p1, p2, p3);
                        gl.glVertex3d(v.x(),v.y(),v.z());
                    }
                }
                gl.glEnd();
            // The L-track is selected
            } else if (2 == trackNr) {
                // code goes here ...
                
            // The C-track is selected
            } else if (3 == trackNr) {
                // code goes here ...
                
            // The custom track is selected
            } else if (4 == trackNr) {
                // code goes here ...
                
            }
        }
        
        /**
         * Returns the position of the curve at 0 <= {@code t} <= 1.
         */
        public Vector getPoint(double t) {
            double x,y;
            x=10*cos(2*PI*t);
            y=14*sin(2*PI*t);
            return new Vector(x,y,1); 
        }
        
        /**
         * Returns the tangent of the curve at 0 <= {@code t} <= 1.
         */
        public Vector getTangent(double t) {
            double x,y;
            x=-20*PI*sin(2*PI*t);
            y=28*PI*cos(2*PI*t);
            return new Vector(x,y,0);
        }
        
        /**
        * Function to evaluate a cubic Bézier segment for parameter value t. 
        */
       public Vector getCubicBezierPnt(double t, Vector P0, Vector P1,Vector P2, Vector P3){
           Vector v0,v1,v2,v3;
           double d0,d1,d2,d3;
           d0=pow((1-t),3);
           d1=3*t*pow((1-t),2);
           d2=3*pow(t,2)*(1-t);
           d3=pow(t,3);
           v0=P0.scale(d0);
           v1=P1.scale(d1);
           v2=P2.scale(d2);
           v3=P3.scale(d3);
           return v0.add(v1.add(v2.add(v3)));
       }


       /**
        * Function to evaluate the tangent of a cubic B´ezier segment for parameter value t. 
        */
       public Vector getCubicBezierTng(double t, Vector P0, Vector P1,Vector P2, Vector P3){
           Vector v0,v1,v2,v3;
           double d0,d11,d12,d21,d22,d3;
           d0=-3*pow((1-t),2);
           d11=3*pow((1-t),2);
           d12=-6*t*(1-t);
           d21=d12;
           d22=3*pow(t,2);
           d3=3*pow(t,2);
           v0=P0.scale(d0);
           v1=P1.scale(d11).subtract(P1.scale(d12));
           v2=P2.scale(d21).subtract(P2.scale(d22));
           v3=P3.scale(d3);
           return v0.add(v1.add(v2.add(v3)));
       }
        
        public void runRobots(Robot[] robots, int number){
            if (number == 0) {
                
                /**
                 * Drawing the robots.
                 * This loop will iterate on each robot and set each position of them in the race track.
                 */
                float distance = -1.5f;
                for(int i = 0; i < 4; i++) {
                    
                        double time = (gs.tAnim * robots[i].robotSpeed /1000)%1;
                        Vector central = raceTrack.getPoint(time);
                        double x,y,z,distX,distY;
                        x=central.x();   y=central.y();  z=central.z();
                        Vector pos;
                    
                        robots[i].draw(gs.showStick);
                        distX = distance * cos(2*PI*time);
                        distY = distance * sin(2*PI*time);
                        pos = new Vector(x+distX, y+distY, z);
                        robots[i].setRobotPosition(pos);
                        
                        Vector tangent = getTangent(time);
                        double xRot = tangent.x();
                        double rotationAngle = toDegrees(acos(xRot/tangent.length()));
                        if(robots[i].position.x() > 0) rotationAngle -= 90;
                        else rotationAngle = 270 - rotationAngle;
                        
                        robots[i].setRobotRotation(rotationAngle);
                        distance += 1.0f;
                }
            }
        }
        
    }
    
    /**
     * Implementation of the terrain.
     */
    private class Terrain {
        
        /**
         * Can be used to set up a display list.
         */
        public Terrain() {
//            int lengthSide = 5;
//            int slicePerMeter = 10;
//            Vector[][] ter = new Vector[lengthSide*slicePerMeter][lengthSide*slicePerMeter];
//            float initialX = -lengthSide/2.0f;
//            float initialY = -lengthSide/2.0f;
//            float finalX = lengthSide/2.0f;
//            float finalY = lengthSide/2.0f;
//            float z;
//            for(float x = initialX + 0.1f; x < finalX; x += 0.1f){
//                for(float y = initialY; y +0.1f < finalY; y += 0.1f) {
//                    float i = x - 0.1f;
//                    z = heightAt(i, y);
////                    if (z > 0.5f) gl.glColor3f(0f, 1.0f, 0f);
////                    else if (0f < z & z < 0.5f) gl.glColor3f(1.0f, 1.0f, 0f);
////                    else gl.glColor3f(0f, 0f, 1.0f);
//                    ter[x*slicePerMeter]
//                    gl.glVertex3f(i, y, z);
//
//                    z = heightAt(x, y);
//                    if (z > 0.5f) gl.glColor3f(0f, 1.0f, 0f);
//                    else if (0f < z & z < 0.5f) gl.glColor3f(1.0f, 1.0f, 0f);
//                    else gl.glColor3f(0f, 0f, 1.0f);
//                    gl.glVertex3f(x, y, z);
//
//                }
//            }
        }
        
        /**
         * Draws the terrain.
         */
        public void draw() {
            
            
        }
        
        
        /**
         * Computes the elevation of the terrain at ({@code x}, {@code y}).
         */
        public float heightAt(float x, float y) {
//            return 0.6f * (float)cos((0.3f*x) + (0.2f*y)) + 0.4f * (float)cos(x - (0.5f * y));
            return 0;
        }
    }
    
    /**
     * Main program execution body, delegates to an instance of
     * the RobotRace implementation.
     */
    public static void main(String args[]) {
        RobotRace robotRace = new RobotRace();
    }
    
    
    
    
}
