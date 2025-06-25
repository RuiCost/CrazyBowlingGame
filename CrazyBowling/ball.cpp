/*
	ball.cpp 
	Miguel Leitao, 2012, 2019
	Enhanced with scoring and sound effects
*/

#include <osgViewer/Viewer> 
#include <osg/Material>
#include <iostream>
#include <cmath>
#include <cstdlib>
#include "btosg.h"
#include "btosgHUD.h"
#include <iomanip> 

#define _DEBUG_ (0)

int ResetFlag=0;
double frame_time = 0.;

// Scoring system
bool firstGame = true; 
int knockedPins = 0;
int totalScore = 0;
bool pinsChecked[10] = {false}; // Track which pins have been knocked down
bool gameStarted = false;
bool gameEnded = false;
double gameStartTime = 0.0;
double bestTime = 0.0;

osgText::Text* scoreText = nullptr;
osgText::Text* wonText = nullptr;
osgText::Text* timerText = nullptr;
osgText::Text* bestimeText = nullptr;
// Sound effect function - plays specified MP3 file
void playCollisionSound() {
    system("/usr/bin/mpg123 -q collision.mp3 &> /dev/null &");
}
// Create World
btosgWorld myWorld;
btosgSphere *myBall;

// Forward declaration
class BowlingPin;
BowlingPin *myPin[10];

// class to handle events
class EventHandler : public osgGA::GUIEventHandler
{
	public:
		bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
		{
			osgViewer::Viewer* viewer = dynamic_cast<osgViewer::Viewer*>(&aa);
			if (!viewer) return false;
			switch(ea.getEventType())
			{ 
				case(osgGA::GUIEventAdapter::KEYDOWN):
					switch ( ea.getKey() ) {
						case osgGA::GUIEventAdapter::KEY_Down:
							if (!gameStarted) {
								gameStarted = true;
								gameStartTime = osg::Timer::instance()->time_s();
							}
							myBall->body->activate(true);
							myBall->body->applyCentralImpulse(btVector3(0.,-1.,0.));
							return true;
						case osgGA::GUIEventAdapter::KEY_Up:
							if (!gameStarted) {
								gameStarted = true;
								gameStartTime = osg::Timer::instance()->time_s();
							}
							myBall->body->activate(true);
							myBall->body->applyCentralImpulse(btVector3(0.,2.,0.));
							std::cout << "frente" << std::endl;
							return true;
						case osgGA::GUIEventAdapter::KEY_Left:
							if (!gameStarted) {
								gameStarted = true;
								gameStartTime = osg::Timer::instance()->time_s();
							}
							myBall->body->activate(true);
							myBall->body->applyCentralImpulse(btVector3(-1,0.,0.));
							std::cout << "esquerda" << std::endl;
							return true;
						case osgGA::GUIEventAdapter::KEY_Right:
							if (!gameStarted) {
								gameStarted = true;
								gameStartTime = osg::Timer::instance()->time_s();
							}
							myBall->body->activate(true);
							myBall->body->applyCentralImpulse(btVector3(1,0.,0.));
							return true;
						
						case osgGA::GUIEventAdapter::KEY_Q:
                                                      if (!gameStarted) {
                                                          gameStarted = true;
                                                          gameStartTime = osg::Timer::instance()->time_s();
                                                      }
                                                      myBall->body->activate(true);
                                                      // Aplica torque para girar no eixo Z (para frente/trás)
                                                      myBall->body->applyTorque(btVector3(0., 0., 10.));
                                                      return true;
						
						case 'r':
							ResetFlag = 1;
							break;
					}	
				default:
				return false;
			}
		}
};

class BowlingPin : public btosgObject {
	private:
		float height;
		float rbody;
		float rhead = 0.04;
		float rbase = 0.048;
		float rneck = 0.035;
		float zbody = -0.05;
		
	public:
		int pinId;
		bool isKnocked;
		btVector3 originalPosition;
		
		BowlingPin(int id = 0) {
			pinId = id;
			isKnocked = false;
			
			mass = 1.55;
			height = 0.4; // Altura do pino, em metros
			rbody = 0.0655; // Raio maximo da barriga
			
			btCompoundShape* cShape = new btCompoundShape();
			if ( !cShape ) fprintf(stderr,"Error creating btCompoundShape\n");
			// Neck
			cShape->addChildShape(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)),
			new btCylinderShapeZ(btVector3(rneck, rneck, height/2)) );
			// Body
			cShape->addChildShape(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,zbody)), new btSphereShape(rbody) );
			// Base
			cShape->addChildShape(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,-height/4.)), new btCylinderShapeZ(btVector3(rbase, rbase, height/4.)) );
			// Head
			cShape->addChildShape(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,height/2-rhead)), new btSphereShape(rhead) );
			shape = cShape;

			// Model
			loadObjectModel("pino.obj");
			model->setNodeMask(CastsShadowTraversalMask);
			
			if ( !cShape ) fprintf(stderr,"Error creating btCompoundShape\n");
			// Colision Shape
			shape = new btCylinderShapeZ( btVector3(rbody, rbody, height/2.) );
			shape->setMargin( 0.0002 ) ;
			btVector3 inertia(0,0,0);
			shape->calculateLocalInertia(mass, inertia);
			// Rigid Body
			btDefaultMotionState* mState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0.,0.,0.)));
			btRigidBody::btRigidBodyConstructionInfo cInfo(mass,mState,shape,inertia);
			cInfo.m_restitution = 0.75f;
			cInfo.m_friction = 0.3f;
			body = new btRigidBody(cInfo);
			if ( !body ) fprintf(stderr, "Error creating btBody for BowlingPin\n");
			else body->setDamping(0., 0.2);
		};
		
		void setOriginalPosition(btVector3 pos) {
			originalPosition = pos;
		}
		
		bool checkIfKnocked() {
			if (isKnocked) return true;
			
			// Don't check for knocked pins until game has started and physics settled
			if (!gameStarted) return false;
			
			// Add a delay after game start to let physics settle
			double currentTime = osg::Timer::instance()->time_s();
			if (currentTime - gameStartTime < 2.0) return false; // Wait 2 seconds after game start
			
			btTransform trans;
			body->getMotionState()->getWorldTransform(trans);
			btVector3 currentPos = trans.getOrigin();
			
			// Get rotation to check if pin has tilted significantly
			btQuaternion rotation = trans.getRotation();
			btVector3 upVector = btVector3(0, 0, 1);
			btVector3 currentUp = btMatrix3x3(rotation) * upVector;
			
			// Check angle with vertical (upward) direction
			float dotProduct = currentUp.dot(upVector);
			float angle = acos(fabs(dotProduct)); // Angle from vertical
			
			// Check if pin has fallen (angle > 45 degrees or significant horizontal movement)
			float heightDifference = currentPos.getZ() - originalPosition.getZ();
			float horizontalDistance = sqrt(pow(currentPos.getX() - originalPosition.getX(), 2) + 
											pow(currentPos.getY() - originalPosition.getY(), 2));
			
			// Pin is considered knocked if:
			// 1. It has tilted more than 60 degrees from vertical (more strict), OR
			// 2. It has moved significantly horizontally (>0.3m, more strict), OR  
			// 3. It has fallen down significantly (height difference < -0.2m, more strict)
			if (angle > 1.047 || horizontalDistance > 0.3 || heightDifference < -0.2) {
				if (!isKnocked) {
					isKnocked = true;
					return true;
				}
			}
			return false;
		}
};

// Collision detection callback
bool lastCollisionTime = 0;
// Collision detection callback
void collisionCallback(btDynamicsWorld *world, btScalar timeStep) {
    static bool soundPlayed = false; // Track if sound has been played in this collision frame
    soundPlayed = false; // Reset for each physics step
    
    int numManifolds = world->getDispatcher()->getNumManifolds();
    
    for (int i = 0; i < numManifolds && !soundPlayed; i++) {
        btPersistentManifold* contactManifold = world->getDispatcher()->getManifoldByIndexInternal(i);
        const btCollisionObject* obA = contactManifold->getBody0();
        const btCollisionObject* obB = contactManifold->getBody1();
        
        btTransform ballTrans;
        myBall->body->getMotionState()->getWorldTransform(ballTrans);
        float ballZ = ballTrans.getOrigin().z();
 
            if (wonText && !gameEnded && ballZ < -10.0) {  // Só mostra LOST se a bola estiver abaixo de -0.5
                wonText->setText("YOU LOST!");
                wonText->setColor(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
                gameEnded = true;
            }
        
            
        int numContacts = contactManifold->getNumContacts();
        if (numContacts > 0) {
            // Check if collision involves the ball and a pin
            bool ballInvolved = (obA == myBall->body || obB == myBall->body);
            bool pinInvolved = false;
            
            for (int p = 0; p < 10; p++) {
                if (obA == myPin[p]->body || obB == myPin[p]->body) {
                    pinInvolved = true;
                    break;
                }
            }
            
            if (ballInvolved && pinInvolved) {
                float impulse = contactManifold->getContactPoint(0).getAppliedImpulse();
                
                // Only play sound for first significant impact in this frame
                if (impulse > 5.0f && !soundPlayed) { // Increased threshold to 5.0 for more significant impacts
                    playCollisionSound();
                    soundPlayed = true; // Mark sound as played for this frame
                    if (_DEBUG_) std::cout << "Significant collision sound played (Force: " << impulse << ")" << std::endl;
                }
            }
        }
    }
}
// Function to check and update score
void updateScore() {
    // Don't update score until game has started or if game has already ended
    if (!gameStarted || gameEnded) return;
    
    // Add a delay after game start to let physics settle
    double currentTime = osg::Timer::instance()->time_s();
    if (currentTime - gameStartTime < 2.0) return; // Wait 2 seconds after game start
    
    int currentKnockedPins = 0;
    
    for (int i = 0; i < 10; i++) {
        if (myPin[i]->checkIfKnocked()) {
            if (!pinsChecked[i]) {
                pinsChecked[i] = true;
                knockedPins++;
                if (knockedPins == 10) {
                    if (wonText) {
                        wonText->setText("You won!!!!!!!");
                        wonText->setColor(osg::Vec4(1.0f, 0.843f, 0.0f, 1.0f));
                    }
                    gameEnded = true; // Mark game as ended
                    
                            double elapsedTime = currentTime - gameStartTime;
    
                        // For first game or if current time is better than best time
                        if (firstGame || elapsedTime < bestTime) {
                            bestTime = elapsedTime;
                            firstGame = false;
                            
                            // Update best time display
                            std::ostringstream bestStream;
                            bestStream << "Best Time: " << std::fixed << std::setprecision(1) << bestTime << "s";
                            bestimeText->setText(bestStream.str());
                            }
                }
            }
        }
    }
    
    if (scoreText) {
        scoreText->setText("Pinos derrubados: " + std::to_string(knockedPins));
    }
}

// Function to reset scoring
void resetScoring() {
    knockedPins = 0;
    totalScore = 0;
    gameStarted = false;
    gameEnded = false;
    gameStartTime = 0.0;
    
    // Reset all pins
    for (int i = 0; i < 10; i++) {
        pinsChecked[i] = false;
        myPin[i]->isKnocked = false;
    }
    
    // Reset HUD elements
    if (scoreText) {
        scoreText->setText("Pinos derrubados: 0");
    }
    if (wonText) {
        wonText->setText("");
    }
    if (timerText) {
        timerText->setText("0.0");
    }

    // Update best time display (in case it changed)
    if (bestimeText) {
        if (firstGame) {
            bestimeText->setText("Best Time: --");
        } else {
            std::ostringstream bestStream;
            bestStream << "Best Time: " << std::fixed << std::setprecision(1) << bestTime << "s";
            bestimeText->setText(bestStream.str());
        }
    }
}

int main()
{
    btosgVec3 up(0., 0., 1.);
    btosgVec3 gravity = up*-9.8;
    myWorld.dynamic->setGravity(gravity);

    //  Ball
    myBall = new btosgSphere(0.217);
    myBall->setMass(7.);
    myBall->setTexture("ball.png");
    myBall->setPosition(0.,-4.,0.2);
    myWorld.addObject( myBall );

    // Material for planes
    osg::ref_ptr<osg::Material> matRamp = new osg::Material;
    matRamp->setAmbient (osg::Material::FRONT_AND_BACK, osg::Vec4(0., 0., 0., 1.0));
    matRamp->setDiffuse (osg::Material::FRONT_AND_BACK, osg::Vec4(0.7, 0.8, 0.0, 1.0));
    matRamp->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 1.0));
    matRamp->setShininess(osg::Material::FRONT_AND_BACK, 64);



    // Plane 1
  // Plane 1 (invisível mas com colisão ativa)
  btosgPlane *myRampL = new btosgPlane(20., 20., 0.);
  myRampL->setPosition(0., 0., -11.0);
  myRampL->setName("Ramp1");
  myRampL->model->setNodeMask(0x0);  // <<--- Torna invisível aqui!
  myWorld.addObject(myRampL);
  
  

  btosgBox *myRamp = new btosgBox(3., 13., 0.1); // Largura, comprimento, espessura
  myRamp->setPosition(0.,0,0); 
  myRamp->setMass(0.); 
  myRamp->body->setFriction(0.5); 
  myRamp->setTexture("piso_512.png");
  myWorld.addObject(myRamp);
// Rampas reais

// (largura, Comprimento, altura)
//Lacunas
	btosgBox *myBox;
        //dois planos na direita
	myBox = new btosgBox(0.19,13,0.01);
	myBox->setPosition(1.59,0.,-0);
	myBox->setMass(0.);
	myBox->setRotation(osg::Quat(osg::DegreesToRadians(20.0), osg::Vec3(0.0f, 1.0f, 0.0f)));
	myWorld.addObject( myBox );
	myBox = new btosgBox(0.19,13,0.01);
	myBox->setPosition(1.75,0.,-0);
	myBox->setMass(0.);
	myBox->setRotation(osg::Quat(osg::DegreesToRadians(160.0), osg::Vec3(0.0f, 1.0f, 0.0f)));
        myWorld.addObject( myBox );
        //dois planos na esquerda
	myBox = new btosgBox(0.19,13,0.01);
	myBox->setPosition(-1.75,0.,-0);
	myBox->setMass(0.);
	myBox->setRotation(osg::Quat(osg::DegreesToRadians(20.0), osg::Vec3(0.0f, 1.0f, 0.0f)));
	myWorld.addObject( myBox );
	myBox = new btosgBox(0.19,13,0.01);
	myBox->setPosition(-1.59,0.,-0);
	myBox->setMass(0.);
	myBox->setRotation(osg::Quat(osg::DegreesToRadians(160.0), osg::Vec3(0.0f, 1.0f, 0.0f)));
        myWorld.addObject( myBox );
        // Rampa adrenalina
        myBox = new btosgBox(1,2,1);
	myBox->setPosition(0,0.,-0.05);
	myBox->setMass(0.);
	myBox->setRotation(osg::Quat(osg::DegreesToRadians(30.0), osg::Vec3(1.0f, 0.0f, 0.0f)));
        myWorld.addObject( myBox );

	

	//Lombas
	btosgCone *myCone;
	myCone = new btosgCone(.10,.4,1.25);
	myCone->setPosition(-0.5, 2.5 ,0.);
	myCone->setRotation(osg::Quat(-osg::PI/2.,osg::Vec3(0.,1.,0.)));
	myCone->setMass(0.);
	myWorld.addObject( myCone );
	
	btosgCone *myCone1;
	myCone1 = new btosgCone(.10,.4,1.25);
	myCone1->setPosition(-.5, 2.5 ,0.);
	myCone1->setRotation(osg::Quat(-osg::PI/2.,osg::Vec3(0.,-1.,0.)));
	myCone1->setMass(0.);
	myWorld.addObject( myCone1 );
	
	btosgCone *myCone2;
	myCone2 = new btosgCone(.10,.4,1.25);
	myCone2->setPosition(0.5, 2.5 ,0.);
	myCone2->setRotation(osg::Quat(-osg::PI/2.,osg::Vec3(0.,1.,0.)));
	myCone2->setMass(0.);
	myWorld.addObject( myCone2 );
	
	btosgCone *myCone3;
	myCone3 = new btosgCone(.10,.4,1.25);
	myCone3->setPosition(0.5, 2.5 ,0.);
	myCone3->setRotation(osg::Quat(-osg::PI/2.,osg::Vec3(0.,-1.,0.)));
	myCone3->setMass(0.);
	myWorld.addObject( myCone3 );
	
	btosgHUD* myHUD = new btosgHUD();
	// Add the HUD as a child of the root node.
	myWorld.scene->addChild(myHUD);
	myHUD->setBackground();

	// Add Light Source
	osg::LightSource *ls = new osg::LightSource;
	ls->getLight()->setPosition(osg::Vec4(0.5, -0.7, 1., 0.));
	ls->getLight()->setAmbient(osg::Vec4(0.2, 0.2, 0.2, 1.));
	myWorld.scene->addChild( ls );

	// 10 pinos
	int x, y, p = 0;
	float space = 12 * 0.0254; // 12 inches
	for( y=0 ; y<4 ; y++ ) {
		for( x=0 ; x<=y ; x++ ) {
			myPin[p] = new BowlingPin(p);
			myPin[p]->setName("pin");
			btVector3 pinPos(0.+(x-y/2.)*space, 3.5+(y)*space, 0.20);
			myPin[p]->setPosition(pinPos.getX(), pinPos.getY(), pinPos.getZ());
			myPin[p]->setOriginalPosition(pinPos);
			myWorld.addObject( myPin[p] );
			p += 1;
		}
	}
	
	// Set up collision callback
	myWorld.dynamic->setInternalTickCallback(collisionCallback);
	

	
    // Creating the viewer
    osgViewer::Viewer viewer ;

    // Setup camera
    osg::Matrix matrix;
    matrix.makeLookAt( osg::Vec3(0.,8.,5.), osg::Vec3(0.,0.,0.), up );
    viewer.getCamera()->setViewMatrix(matrix);

   /* // Light
    osg::ref_ptr<osg::LightSource> ls = new osg::LightSource;
    ls->getLight()->setPosition(osg::Vec4(2.5,-10+30*up[1],-10+30.*up[2],1.)); 
    ls->getLight()->setAmbient(osg::Vec4(0.1, 0.1, 0.1, 1.0));
    ls->getLight()->setDiffuse(osg::Vec4(1.0, 1.0, 1.0, 1.0));
    ls->getLight()->setSpecular(osg::Vec4(0.2, 0.2, 0.2, 1.0));
    myWorld.scene->addChild(ls.get());
*/
    viewer.setSceneData( myWorld.scene );

    viewer.getCamera()->setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR ); 
        
    // Manipulator
    osg::ref_ptr<osgGA::TrackballManipulator> manipulator = new osgGA::TrackballManipulator;
    viewer.setCameraManipulator( manipulator );
    // Set the desired home coordinates for the manipulator
    osg::Vec3d eye(osg::Vec3(0., -15., -5.)+up*20.);
    osg::Vec3d center(0., 0., 0.);
    // Make sure that OSG is not overriding our home position
    manipulator->setAutoComputeHomePosition(false);
    // Set the desired home position of the Trackball Manipulator
    manipulator->setHomePosition(eye, center, up);
    // Force the camera to move to the home position
    manipulator->home(0.0);
        
    // record the timer tick at the start of rendering.
    osg::Timer myTimer;
    double timenow = myTimer.time_s();
    double last_time = timenow;
    frame_time = 0.;
    
    
    
      // Text instance to show up in the HUD:
   osgText::Text* textOne = new osgText::Text();
   textOne->setCharacterSize(25);
   textOne->setFont("arial.ttf");
   textOne->setText("INTMU Bowling");
   textOne->setAxisAlignment(osgText::Text::SCREEN);
   textOne->setPosition( osg::Vec3(400., 100., -1) );
   textOne->setColor(osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
   // Add the text to the HUD:
   myHUD->addDrawable( textOne );


  scoreText = new osgText::Text();
  scoreText->setCharacterSize(25);
  scoreText->setFont("arial.ttf");
  scoreText->setAxisAlignment(osgText::Text::SCREEN);
  scoreText->setPosition(osg::Vec3(400., 50., -1));
  scoreText->setColor(osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
  scoreText->setText("Pinos derrubados: 0");

  myHUD->addDrawable(scoreText);

  wonText = new osgText::Text();
  wonText->setCharacterSize(50);
  wonText->setFont("arial.ttf");
  wonText->setAxisAlignment(osgText::Text::SCREEN);
  wonText->setPosition(osg::Vec3(400., 450., -1));
  wonText->setColor(osg::Vec4(1.0f, 0.843f, 0.0f, 1.0f));
  wonText->setText("");

  myHUD->addDrawable(wonText);


// Para timer HUD:


   osgText::Text* textTwo = new osgText::Text();
   textTwo->setCharacterSize(25);
   textTwo->setFont("arial.ttf");
   textTwo->setText("Timer:");
   textTwo->setAxisAlignment(osgText::Text::SCREEN);
   textTwo->setPosition( osg::Vec3(800., 75., -1) );
   textTwo->setColor(osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
   // Add the text to the HUD:
   myHUD->addDrawable( textTwo );

  timerText = new osgText::Text();
  timerText->setCharacterSize(25);
  timerText->setFont("arial.ttf");
  timerText->setAxisAlignment(osgText::Text::SCREEN);
  timerText->setPosition(osg::Vec3(900., 75., -1));
  timerText->setColor(osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
  timerText->setText("0.0");

  myHUD->addDrawable(timerText);

//bestimeText
  bestimeText = new osgText::Text();
  bestimeText->setCharacterSize(25);
  bestimeText->setFont("arial.ttf");
  bestimeText->setAxisAlignment(osgText::Text::SCREEN);
  bestimeText->setPosition(osg::Vec3(90., 75., -1));
  bestimeText->setColor(osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
  bestimeText->setText("Best Time: --");

   myHUD->addDrawable(bestimeText);
   
   
   
    viewer.addEventHandler(new EventHandler());	
    while( !viewer.done() )
    {
    		// add the Event handler
		
	 	myWorld.stepSimulation(frame_time,10);
	 	
	 	// Update score system
	 	updateScore();
	 	
             // In your main loop, replace the timer update section with:
            if (gameStarted && !gameEnded) {
                double currentTime = osg::Timer::instance()->time_s();
                double elapsedTime = currentTime - gameStartTime;
                std::ostringstream stream;
                stream << std::fixed << std::setprecision(1) << elapsedTime << "s";
                timerText->setText(stream.str());
                
                // Check for game completion
                if (knockedPins >= 10) {
                    gameEnded = true;
                    
                    // Update best time if this is better
                    if (firstGame || elapsedTime < bestTime) {
                        bestTime = elapsedTime;
                        firstGame = false;
                        
                        // Update best time display
                        std::ostringstream bestStream;
                        bestStream << "Best Time: " << std::fixed << std::setprecision(1) << bestTime << "s";
                        bestimeText->setText(bestStream.str());
                    }
                }
            }
                              
	  	viewer.frame();
	  	timenow = myTimer.time_s();
	  	frame_time = timenow - last_time;
	  	last_time = timenow;
        	
		if (ResetFlag>0) {
                    myWorld.reset();
                    resetScoring();
		    ResetFlag = 0;
		    
		}
		
    }
}

