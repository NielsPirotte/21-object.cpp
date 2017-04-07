//------------------------------------------------------------------------------
#include "chai3d.h"
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
#include <cmath>
#include <ist/InnerSphereTree.h>
#include <collisions/Voxelizer.h>
#include <ist/SaveIST.h>
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

//update boundarybox
bool wilupdaten = false;

//colors of the collisiontrees for rendering
cColorf colorOnderkaak;
cColorf colorBovenkaak;

//------------------------------------------------------------------------------
// STATES
//------------------------------------------------------------------------------
enum MouseState
{
	MOUSE_IDLE,
	MOUSE_MOVE_CAMERA,
	MOUSE_SELECTION_TRANSLATE,
	MOUSE_SELECTION_ROTATE
};

//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cDirectionalLight *light;

// a virtual object
cMesh* onderkaak;
cMesh* bovenkaak;

// materials for the object
cMaterial* matOnderkaak;
cMaterial* matBovenkaak;

//selected object by the mouse
cGenericObject* selectedObject;

// offset between the position of the mmouse click on the object and the object reference frame location.
cVector3d selectedObjectOffset;

// position of mouse click.
cVector3d selectedPoint;


//Bolleke om te zien waar de kaak raakt
cMesh* bolleke;

cVector3d* positie = new cVector3d(0.0,0.0,0.0);

// a colored background
cBackground* background;

// a font for rendering text
cFontPtr font;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;

// een label om te tonen of de broad phase zegt of de kaken raken.
cLabel* labelRaakt;

// a flag that indicates if the haptic simulation is currently running
bool simulationRunning = false;

// a flag that indicates if the haptic simulation has terminated
bool simulationFinished = true;

// display options
bool showTriangles = true;

// display level for collision tree
int collisionTreeDisplayLevelOnderkaak = 0;
int collisionTreeDisplayLevelBovenkaak = 0;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread* hapticsThread;

// a handle to window display context
GLFWwindow* window = NULL;

// current width of window
int width  = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;

// root resource path
string resourceRoot;

// mouse state
MouseState mouseState = MOUSE_IDLE;

// last mouse position
double mouseX, mouseY;

vector<cCollisionAABBBox> leafnodes1;
vector<cCollisionAABBBox> leafnodes2;

// Als de objecten ver van elkaar staan,
// kijk pas als de afstand tussen de 2 overschreden is.
cVector3d* gelopenAfstand = new cVector3d(0.0,0.0,0.0);
float minimumTeLopen = 0;

InnerSphereTree* istBovenkaak;
InnerSphereTree* istOnderkaak;


//------------------------------------------------------------------------------
// DECLARED MACROS
//------------------------------------------------------------------------------

// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// callback to handle mouse click
void mouseButtonCallback(GLFWwindow* a_window, int a_button, int a_action, int a_mods);

// callback to handle mouse motion
void mouseMotionCallback(GLFWwindow* a_window, double a_posX, double a_posY);

// callback to handle mouse scroll
void mouseScrollCallback(GLFWwindow* a_window, double a_offsetX, double a_offsetY);

// callback to render graphic scene
void updateGraphics(void);

// this function renders the scene
void updateGraphics(void);

// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function closes the application
void close(void);

// Hulpfucntie om alle boxen van de collisie detectie te doorlopen.
//void doorloopBoxes(void);

int main(int argc, char* argv[])
{
    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "Bachelorproef" << endl;
    cout << "Casper Vranken | Niels Pirotte" << endl;
    cout << "Uhasselt | KULeuven" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[2] - Wireframe (ON/OFF)" << endl;
    cout << "[3] - Collision tree (ON/OFF)" << endl;
    cout << "[4] - Increase collision tree display depth" << endl;
    cout << "[5] - Decrease collision tree display depth" << endl;
    cout << "[t] - Enable/Disable display of triangles" << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[q] - Exit application" << endl;
    cout << endl << endl;

    // parse first arg to try and locate resources
    resourceRoot = string(argv[0]).substr(0,string(argv[0]).find_last_of("/\\")+1);


    //--------------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // initialize GLFW library
    if (!glfwInit())
    {
        cout << "failed initialization" << endl;
        cSleepMs(1000);
        return 1;
    }

    // set error callback
    glfwSetErrorCallback(errorCallback);

    // compute desired size of window
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    int w = 0.8 * mode->height;
    int h = 0.5 * mode->height;
    int x = 0.5 * (mode->width - w);
    int y = 0.5 * (mode->height - h);

    // set OpenGL version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    // set active stereo mode
    if (stereoMode == C_STEREO_ACTIVE)
    {
        glfwWindowHint(GLFW_STEREO, GL_TRUE);
    }
    else
    {
        glfwWindowHint(GLFW_STEREO, GL_FALSE);
    }

    // create display context
    window = glfwCreateWindow(w, h, "Bachelorproef - Casper Vranken | Niels Pirotte", NULL, NULL);
    if (!window)
    {
        cout << "failed to create window" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    // get width and height of window
    glfwGetWindowSize(window, &width, &height);

    // set position of window
    glfwSetWindowPos(window, x, y);

    // set key callback
    glfwSetKeyCallback(window, keyCallback);

	// set mouse position callback
	glfwSetCursorPosCallback(window, mouseMotionCallback);

	// set mouse button callback
	glfwSetMouseButtonCallback(window, mouseButtonCallback);

	// set mouse scroll callback
	glfwSetScrollCallback(window, mouseScrollCallback);

    // set resize callback
    glfwSetWindowSizeCallback(window, windowSizeCallback);

    // set current display context
    glfwMakeContextCurrent(window);

    // sets the swap interval for the current display context
    glfwSwapInterval(swapInterval);

#ifdef GLEW_VERSION
    // initialize GLEW library
    if (glewInit() != GLEW_OK)
    {
        cout << "failed to initialize GLEW library" << endl;
        glfwTerminate();
        return 1;
    }
#endif


    //--------------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //--------------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    world->m_backgroundColor.setBlack();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

	// define a basis in spherical coordinates for the camera
	camera->setSphericalReferences(cVector3d(0, 0, 0),    // origin
		cVector3d(0, 0, 1),    // zenith direction
		cVector3d(1, 0, 0));   // azimuth direction

	camera->setSphericalDeg(100,    // spherical coordinate radius
		60,     // spherical coordinate polar angle
		10);    // spherical coordinate azimuth angle

    // set the near and far clipping planes of the camera
    // anything in front or behind these clipping planes will not be rendered
    camera->setClippingPlanes(1, 1000);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.03);
    camera->setStereoFocalLength(1.5);

    // enable multi-pass rendering to handle transparent objects
    camera->setUseMultipassTransparency(true);

    // create a light source
    light = new cDirectionalLight(world);

    // attach light to camera
    camera->addChild(light);

    // enable light source
    light->setEnabled(true);

    // define the direction of the light beam
    light->setDir(-3.0,-0.5, 0.0);

    // set lighting conditions
    light->m_ambient.set(0.4f, 0.4f, 0.4f);
    light->m_diffuse.set(0.8f, 0.8f, 0.8f);
    light->m_specular.set(1.0f, 1.0f, 1.0f);

    //--------------------------------------------------------------------------
    // CREATE OBJECT
    //--------------------------------------------------------------------------

    // create a virtual mesh
    onderkaak = new cMesh();
	bovenkaak = new cMesh();
	bolleke = new cMesh();

    // add object to world
    world->addChild(onderkaak);
	world->addChild(bovenkaak);
	world->addChild(bolleke);

	// set materials for the objects
	matOnderkaak = new cMaterial();
	matOnderkaak->setWhite();
	onderkaak->setMaterial(*matOnderkaak);

	matBovenkaak = new cMaterial();
	matBovenkaak->setWhite();
	bovenkaak->setMaterial(*matBovenkaak);

    // load an object file
    bool fileload;
    fileload = onderkaak->loadFromFile2(RESOURCE_PATH("../resources/models/kaken/mandibulary_export_Brecht Beckers.stl"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = onderkaak->loadFromFile2("../../../bin/resources/models/kaken/mandibulary_export_Brecht Beckers.stl");
        #endif
    }
    if (!fileload)
    {
        cout << "Error - Onderkaak kon niet geladen worden." << endl;
        close();
        return (-1);
    }

	fileload = bovenkaak->loadFromFile2(RESOURCE_PATH("../resources/models/kaken/maxillary_export_Brecht Beckers.stl"));
	if (!fileload)
	{
		#if defined(_MSVC)
		fileload = bovenkaak->loadFromFile2("../../../bin/resources/models/kaken/maxillary_export_Brecht Beckers.stl");
		#endif
	}
	if (!fileload)
	{
		cout << "Error - Bovenkaak kon niet geladen worden." << endl;
		close();
		return (-1);
	}

	fileload = bolleke->loadFromFile2(RESOURCE_PATH("../resources/models/bol.stl"));
	if (!fileload)
	{
	#if defined(_MSVC)
		fileload = bolleke->loadFromFile2("../../../bin/resources/models/bol.stl");
	#endif
	}
	if (!fileload)
	{
		cout << "Error - Bolleke kon niet geladen worden." << endl;
		close();
		return (-1);
	}

	bolleke->setWireMode(true);

    // disable culling so that faces are rendered on both sides
	onderkaak->setUseCulling(true);
	bovenkaak->setUseCulling(true);

    // get dimensions of object
	onderkaak->computeBoundaryBox(true);
    double size = cSub(onderkaak->getBoundaryMax(), onderkaak->getBoundaryMin()).length();

    // resize object to screen
    if (size > 0.001)
    {
		//onderkaak->scale(1.0 / size);
    }

	bovenkaak->computeBoundaryBox(true);
	size = cSub(bovenkaak->getBoundaryMax(), bovenkaak->getBoundaryMin()).length();

	if (size > 0.001) {
		//bovenkaak->scale(1.0 / size);
	}

	bolleke->computeBoundaryBox(true);
	size = cSub(bolleke->getBoundaryMax(), bolleke->getBoundaryMin()).length();

	if (size > 0.001) {
		//bolleke->scale(0.1 / size);
	}
	
	cColorf color;
	color.setOrangeTomato();

	for (unsigned int i = 0; i < bolleke->getNumVertices(); i++) {
		bolleke->m_vertices->setColor(i, color);
	}

    // show/hide boundary box
	onderkaak->setShowBoundaryBox(false);
	bovenkaak->setShowBoundaryBox(false);

    // compute collision detection algorithm
	onderkaak->createAABBCollisionDetector(0.0001);
	bovenkaak->createAABBCollisionDetector(0.0001);

    // enable display list for faster graphic rendering
	onderkaak->setUseDisplayList(true);
	bovenkaak->setUseDisplayList(true);

    // center object in scene
	/*onderkaak->setLocalPos(bovenkaak->getBoundaryCenter());
	bovenkaak->setLocalPos(onderkaak->getBoundaryCenter());*/

	onderkaak->setLocalPos(cVector3d(0, 0, 0));

    // rotate object in scene
	//onderkaak->rotateExtrinsicEulerAnglesDeg(-90, 0, -90, C_EULER_ORDER_XYZ);
	//bovenkaak->rotateExtrinsicEulerAnglesDeg(-90, 0, -90, C_EULER_ORDER_XYZ);
	onderkaak->rotateExtrinsicEulerAnglesDeg(0, 0, 0, C_EULER_ORDER_XYZ);
	bovenkaak->rotateExtrinsicEulerAnglesDeg(0, 0, 0, C_EULER_ORDER_XYZ);

    // display options
	onderkaak->setShowTriangles(true);
	bovenkaak->setShowTriangles(true);

	onderkaak->createAABBCollisionDetector(0.01);
	// Bouw de innerspheretree op van de onderkaak.
	//Voxelizer* voxelizerOnderkaak = new Voxelizer();
	//cCollisionAABB* colliderOnderkaak = dynamic_cast<cCollisionAABB*>(onderkaak->getCollisionDetector());
	//cout << "test: " << *(test->getTriangles()[0].p1) << endl;
	//voxelizerOnderkaak->setObject(colliderOnderkaak);
	//voxelizerOnderkaak->setPositie(cVector3d(0,0,0));
	//voxelizerOnderkaak->setAccuraatheid(10);
	//voxelizerOnderkaak->initialize();

	//istOnderkaak = voxelizerOnderkaak->buildInnerTree(6, onderkaak->getLocalPos(), (onderkaak->getBoundaryMax()-onderkaak->getBoundaryMin()).length());
	//delete voxelizerOnderkaak;

	//istOnderkaak->printAABBCollisionTree(5);
	//saveIST(istOnderkaak, "Onderkaak75_6");

	//istOnderkaak = loadIST("Onderkaak");

	//onderkaak->setCollisionDetector(istOnderkaak);
	// Bouw van innerspheretree van de onderkaak is klaar.

	// Bouw de innerspheretree van de bovenkaak
	//Voxelizer* voxelizerBovenkaak = new Voxelizer();
	//cCollisionAABB* colliderBovenkaak = dynamic_cast<cCollisionAABB*>(bovenkaak->getCollisionDetector());
	////cout << "test: " << *(test->getTriangles()[0].p1) << endl;
	//voxelizerBovenkaak->setObject(colliderBovenkaak);
	//voxelizerBovenkaak->setPositie(cVector3d(0,0,0));
	//voxelizerBovenkaak->setAccuraatheid(10);
	//voxelizerBovenkaak->initialize();

	//istBovenkaak = voxelizerBovenkaak->buildInnerTree(6, bovenkaak->getLocalPos(), (bovenkaak->getBoundaryMax() - bovenkaak->getBoundaryMin()).length());
	//delete voxelizerBovenkaak;

	////istBovenkaak->printAABBCollisionTree(5);
	//saveIST(istBovenkaak, "Bovenkaak75_6");

	////istBovenkaak = loadIST("Bovenkaak");
	////istBovenkaak->printAABBCollisionTree(5);

	//istBovenkaak = loadIST("Bovenkaak");
	//bovenkaak->setCollisionDetector(istBovenkaak);

	//set kleur collision detectors for rendering
	colorOnderkaak.setGreenForest();
	onderkaak->setCollisionDetectorProperties(0, colorOnderkaak, true);

	colorBovenkaak.setOrangeTomato();
	bovenkaak->setCollisionDetectorProperties(0, colorBovenkaak, true);
	// Bouw van de innerspheretree van de bovenkaak is klaar.


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    font = NEW_CFONTCALIBRI32();
    
    // create a label to display the haptic and graphic rate of the simulation en of de broadphase raakt.
    labelRates = new cLabel(font);
	labelRaakt = new cLabel(font);

	labelRates->m_fontColor.setRedSalmon();
	labelRaakt->m_fontColor.setRedSalmon();

	camera->m_frontLayer->addChild(labelRaakt);
    camera->m_frontLayer->addChild(labelRates);

	labelRaakt->setText("Raakt");

    //--------------------------------------------------------------------------
    // START SIMULATION
    //--------------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);


    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP
    //--------------------------------------------------------------------------

    // call window size callback at initialization
    windowSizeCallback(window, width, height);

    // main graphic loop
    while (!glfwWindowShouldClose(window))
    {
        // get width and height of window
        glfwGetWindowSize(window, &width, &height);

        // render graphics
        updateGraphics();

        // swap buffers
        glfwSwapBuffers(window);

        // process events
        glfwPollEvents();

        // signal frequency counter
        freqCounterGraphics.signal(1);
    }

    // close window
    glfwDestroyWindow(window);

    // terminate GLFW library
    glfwTerminate();

    // exit
    return 0;
}

//------------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width  = a_width;
    height = a_height;
}

//------------------------------------------------------------------------------

void errorCallback(int a_error, const char* a_description)
{
    cout << "Error: " << a_description << endl;
}

//------------------------------------------------------------------------------

void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
    // filter calls that only include a key press
    if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
    {
        return;
    }

    // option - exit
    else if ((a_key == GLFW_KEY_Q))
    {
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
    }

	else if ((a_key == GLFW_KEY_ESCAPE)) {
		// toggle state variable
		fullscreen = !fullscreen;

		// get handle to monitor
		GLFWmonitor* monitor = glfwGetPrimaryMonitor();

		// get information about monitor
		const GLFWvidmode* mode = glfwGetVideoMode(monitor);

		// set fullscreen or window mode
		if (fullscreen)
		{
			glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
			glfwSwapInterval(swapInterval);
		}
		else
		{
			int w = 0.8 * mode->height;
			int h = 0.5 * mode->height;
			int x = 0.5 * (mode->width - w);
			int y = 0.5 * (mode->height - h);
			glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
			glfwSwapInterval(swapInterval);
		}
	}

    // option - enable/disable wire mode
    else if (a_key == GLFW_KEY_2)
    {
        bool useWireMode = onderkaak->getWireMode();
		onderkaak->setWireMode(!useWireMode, true);
		bovenkaak->setWireMode(!useWireMode, true);
    }

    // option - show/hide collision detection tree
    else if (a_key == GLFW_KEY_3)
    {
		//onderkaak->setCollisionDetectorProperties(collisionTreeDisplayLevelOnderkaak, color, true);
		//bovenkaak->setCollisionDetectorProperties(collisionTreeDisplayLevelBovenkaak, color, true);
        bool show = onderkaak->getShowCollisionDetector();
		onderkaak->setShowCollisionDetector(!show, true);
		bovenkaak->setShowCollisionDetector(!show, true);
		//doorloopBoxes();
    }

    // option - decrease depth level of collision tree
    else if (a_key == GLFW_KEY_4)
    {
        collisionTreeDisplayLevelOnderkaak--;
		collisionTreeDisplayLevelBovenkaak--;
        if (collisionTreeDisplayLevelOnderkaak < 0) { collisionTreeDisplayLevelOnderkaak = 0; }
		if (collisionTreeDisplayLevelBovenkaak < 0) { collisionTreeDisplayLevelBovenkaak = 0; }
		onderkaak->setCollisionDetectorProperties(collisionTreeDisplayLevelOnderkaak, colorOnderkaak, true);
		onderkaak->setShowCollisionDetector(true, true);

		bovenkaak->setCollisionDetectorProperties(collisionTreeDisplayLevelBovenkaak, colorBovenkaak, true);
		bovenkaak->setShowCollisionDetector(true, true);
    }

    // option - increase depth level of collision tree
    else if (a_key == GLFW_KEY_5)
    {
        collisionTreeDisplayLevelOnderkaak++;
		collisionTreeDisplayLevelBovenkaak++;
		onderkaak->setCollisionDetectorProperties(collisionTreeDisplayLevelOnderkaak, colorOnderkaak, true);
		onderkaak->setShowCollisionDetector(true, true);

		bovenkaak->setCollisionDetectorProperties(collisionTreeDisplayLevelBovenkaak, colorBovenkaak, true);
		bovenkaak->setShowCollisionDetector(true, true);
    }

    // option - show/hide triangles
    else if (a_key == GLFW_KEY_T)
    {
        showTriangles = !showTriangles;
		onderkaak->setShowTriangles(showTriangles);
		bovenkaak->setShowTriangles(showTriangles);
    }

    // option - toggle fullscreen
    else if (a_key == GLFW_KEY_F)
    {
        // toggle state variable
        fullscreen = !fullscreen;

        // get handle to monitor
        GLFWmonitor* monitor = glfwGetPrimaryMonitor();

        // get information about monitor
        const GLFWvidmode* mode = glfwGetVideoMode(monitor);

        // set fullscreen or window mode
        if (fullscreen)
        {
            glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
        else
        {
            int w = 0.8 * mode->height;
            int h = 0.5 * mode->height;
            int x = 0.5 * (mode->width - w);
            int y = 0.5 * (mode->height - h);
            glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
    }

	// Beweeg bovenkaak
	else if (a_key == GLFW_KEY_RIGHT) {
		bovenkaak->setLocalPos(cVector3d(bovenkaak->getLocalPos().x(),
			bovenkaak->getLocalPos().y() + 1,
			
			bovenkaak->getLocalPos().z())
		);

		gelopenAfstand->add(0, 1, 0);
	}
	else if (a_key == GLFW_KEY_LEFT) {
		bovenkaak->setLocalPos(cVector3d(bovenkaak->getLocalPos().x(),
			bovenkaak->getLocalPos().y() - 1,
			bovenkaak->getLocalPos().z())
		);
		gelopenAfstand->sub(0, 1, 0);
	}
	else if (a_key == GLFW_KEY_UP) {
		bovenkaak->setLocalPos(cVector3d(bovenkaak->getLocalPos().x(),
			bovenkaak->getLocalPos().y(),
			bovenkaak->getLocalPos().z()+1)
		);
		gelopenAfstand->add(0, 0, 1);
	}
	else if (a_key == GLFW_KEY_DOWN) {
		bovenkaak->setLocalPos(cVector3d(bovenkaak->getLocalPos().x(),
			bovenkaak->getLocalPos().y(),
			bovenkaak->getLocalPos().z() - 1)
		);
		gelopenAfstand->sub(0, 0, 1);
	}

	if(istBovenkaak != nullptr) istBovenkaak->setPosition(-(bovenkaak->getLocalPos()));
	wilupdaten = true;
}

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------

void mouseMotionCallback(GLFWwindow* a_window, double a_posX, double a_posY)
{
	if (mouseState == MOUSE_MOVE_CAMERA)
	{
		// compute mouse motion
		int dx = a_posX - mouseX;
		int dy = a_posY - mouseY;
		mouseX = a_posX;
		mouseY = a_posY;

		// compute new camera angles
		double azimuthDeg = camera->getSphericalAzimuthDeg() - 0.5 * dx;
		double polarDeg = camera->getSphericalPolarDeg() - 0.5 * dy;

		// assign new angles
		camera->setSphericalAzimuthDeg(azimuthDeg);
		camera->setSphericalPolarDeg(polarDeg);
	}
	else if ((selectedObject != NULL) && (mouseState == MOUSE_SELECTION_TRANSLATE))
	{
		// get the vector that goes from the camera to the selected point (mouse click)
		cVector3d vCameraObject = selectedPoint - camera->getLocalPos();

		// get the vector that point in the direction of the camera. ("where the camera is looking at")
		cVector3d vCameraLookAt = camera->getLookVector();

		// compute the angle between both vectors
		double angle = cAngle(vCameraObject, vCameraLookAt);

		// compute the distance between the camera and the plane that intersects the object and 
		// which is parallel to the camera plane
		double distanceToObjectPlane = vCameraObject.length() * cos(angle);

		// convert the pixel in mouse space into a relative position in the world
		double factor = (distanceToObjectPlane * tan(0.5 * camera->getFieldViewAngleRad())) / (0.5 * height);
		double posRelX = factor * (a_posX - (0.5 * width));
		double posRelY = factor * ((height - a_posY) - (0.5 * height));

		// compute the new position in world coordinates
		cVector3d pos = camera->getLocalPos() +
			distanceToObjectPlane * camera->getLookVector() +
			posRelX * camera->getRightVector() +
			posRelY * camera->getUpVector();

		// compute position of object by taking in account offset
		cVector3d posObject = pos - selectedObjectOffset;

		// apply new position to object
		selectedObject->setLocalPos(posObject);
	}
	else if ((selectedObject != NULL) && (mouseState == MOUSE_SELECTION_ROTATE))
	{
		// get the vector that goes from the camera to the selected point (mouse click)
		cVector3d vCameraObject = selectedPoint - camera->getLocalPos();

		// get the vector that point in the direction of the camera. ("where the camera is looking at")
		cVector3d vCameraLookAt = camera->getLookVector();

		// compute the angle between both vectors
		double angle = cAngle(vCameraObject, vCameraLookAt);

		// compute the distance between the camera and the plane that intersects the object and 
		// which is parallel to the camera plane
		double distanceToObjectPlane = vCameraObject.length() * cos(angle);

		// convert the pixel in mouse space into a relative position in the world
		double factor = (distanceToObjectPlane * tan(0.5 * camera->getFieldViewAngleRad())) / (0.5 * height);
		double posRelX = factor * (a_posX - (0.5 * width));
		double posRelY = factor * ((height - a_posY) - (0.5 * height));

		// compute the new position in world coordinates
		cVector3d pos = camera->getLocalPos() +
			distanceToObjectPlane * camera->getLookVector() +
			posRelX * camera->getRightVector() +
			posRelY * camera->getUpVector();

		cVector3d rotAxis = cVector3d(-1.0/pos.x(),-1.0/pos.y(),-1.0/pos.z());

		// compute position of object by taking in account offset
		cVector3d posObject = pos - selectedObjectOffset;

		// apply new position to object
		cMatrix3d rot = selectedObject->getLocalRot();
		rot.setAxisAngleRotationDeg(rotAxis,1);
		selectedObject->setLocalRot(rot);
	}
}

//------------------------------------------------------------------------------

void mouseScrollCallback(GLFWwindow* a_window, double a_offsetX, double a_offsetY)
{
	double r = camera->getSphericalRadius();
	r = cClamp(r - 1 * a_offsetY, 30.0, 100.0);
	camera->setSphericalRadius(r);
}

void mouseButtonCallback(GLFWwindow* a_window, int a_button, int a_action, int a_mods)
{
	if (a_button == GLFW_MOUSE_BUTTON_RIGHT && a_action == GLFW_PRESS)
	{
		// store mouse position
		glfwGetCursorPos(window, &mouseX, &mouseY);

		// update mouse state
		mouseState = MOUSE_MOVE_CAMERA;
	}
	else if ((a_button == GLFW_MOUSE_BUTTON_LEFT || a_button == GLFW_MOUSE_BUTTON_MIDDLE) && a_action == GLFW_PRESS)
	{
		// store mouse position
		glfwGetCursorPos(window, &mouseX, &mouseY);

		// variable for storing collision information
		cCollisionRecorder recorder;
		cCollisionSettings settings;
		
		// detect for any collision between mouse and world
		bool hit = camera->selectWorld(mouseX, (height - mouseY), width, height, recorder, settings);
		if (hit)
		{
			selectedPoint = recorder.m_nearestCollision.m_globalPos;
			selectedObject = recorder.m_nearestCollision.m_object;
			selectedObjectOffset = recorder.m_nearestCollision.m_globalPos - selectedObject->getLocalPos();
			if (selectedObject != NULL) {
				if(a_button == GLFW_MOUSE_BUTTON_LEFT) mouseState = MOUSE_SELECTION_TRANSLATE;
				else  mouseState = MOUSE_SELECTION_ROTATE;
			}

			if (selectedObject == onderkaak) {
				onderkaak->m_material->setBlue();
			}

			if (selectedObject == bovenkaak) {
				bovenkaak->m_material->setBlue();
			}
		}
	}
	else
	{
		onderkaak->m_material->setWhite();
		bovenkaak->m_material->setWhite();
		mouseState = MOUSE_IDLE;
	}
}

//------------------------------------------------------------------------------

//void mouseMotionCallback(GLFWwindow* a_window, double a_posX, double a_posY)
//{
//	if ((selectedObject != NULL) && (mouseState == MOUSE_SELECTION))
//	{
//		// get the vector that goes from the camera to the selected point (mouse click)
//		cVector3d vCameraObject = selectedPoint - camera->getLocalPos();
//
//		// get the vector that point in the direction of the camera. ("where the camera is looking at")
//		cVector3d vCameraLookAt = camera->getLookVector();
//
//		// compute the angle between both vectors
//		double angle = cAngle(vCameraObject, vCameraLookAt);
//
//		// compute the distance between the camera and the plane that intersects the object and 
//		// which is parallel to the camera plane
//		double distanceToObjectPlane = vCameraObject.length() * cos(angle);
//
//		// convert the pixel in mouse space into a relative position in the world
//		double factor = (distanceToObjectPlane * tan(0.5 * camera->getFieldViewAngleRad())) / (0.5 * height);
//		double posRelX = factor * (a_posX - (0.5 * width));
//		double posRelY = factor * ((height - a_posY) - (0.5 * height));
//
//		// compute the new position in world coordinates
//		cVector3d pos = camera->getLocalPos() +
//			distanceToObjectPlane * camera->getLookVector() +
//			posRelX * camera->getRightVector() +
//			posRelY * camera->getUpVector();
//
//		// compute position of object by taking in account offset
//		cVector3d posObject = pos - selectedObjectOffset;
//
//		// apply new position to object
//		selectedObject->setLocalPos(posObject);
//
//		// place cursor at the position of the mouse click
//		sphereSelect->setLocalPos(pos);
//	}
//}

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // delete resources
    delete hapticsThread;
    delete world;
}

//------------------------------------------------------------------------------

void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update haptic and graphic rate data
    labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
        cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

    // update position of label
    labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);


    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // render world
    camera->renderView(width, height);

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;
}

//------------------------------------------------------------------------------

enum cMode
{
    IDLE,
    SELECTION
};

void updateHaptics(void)
{
    cMode state = IDLE;
    cTransform tool_T_object;

    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    // main haptic simulation loop
	bool hulp = true;
	while (simulationRunning)
	{
		/////////////////////////////////////////////////////////////////////////
		// HAPTIC RENDERING
		/////////////////////////////////////////////////////////////////////////

		// signal frequency counter
		freqCounterHaptics.signal(1);

		// compute global reference frames for each object
		//world->computeGlobalPositions(true); 

		///////////////////////////////////////////////////////////////////////////
		// BROAD PHASE OP BASIS VAN BOUNDARY BOX
		///////////////////////////////////////////////////////////////////////////

		/*cVector3d bovenkaakBoundaryMax = bovenkaak->getBoundaryMax() + bovenkaak->getLocalPos();
		cVector3d onderkaakBoundaryMax = onderkaak->getBoundaryMax() + onderkaak->getLocalPos();
		cVector3d bovenkaakBoundaryMin = bovenkaak->getBoundaryMin() + bovenkaak->getLocalPos();
		cVector3d onderkaakBoundaryMin = onderkaak->getBoundaryMin() + onderkaak->getLocalPos();*/

		/*if ((onderkaakBoundaryMax.y() > bovenkaakBoundaryMin.y())
			&&(onderkaakBoundaryMin.y() < bovenkaakBoundaryMax.y())) {

			if ((onderkaakBoundaryMax.x() > bovenkaakBoundaryMin.x())
				&& (onderkaakBoundaryMin.x() < bovenkaakBoundaryMax.x())) {

				if ((onderkaakBoundaryMax.z() > bovenkaakBoundaryMin.z())
					&& (onderkaakBoundaryMin.z() < bovenkaakBoundaryMax.z())) {

					hit = true;
				}
			}
		}*/

		bool accuraatRaakt = false;
		bool kijken = false;
		if ((abs(gelopenAfstand->length()) >= minimumTeLopen)) kijken = true;

		if (kijken) {
		double dist = 0;
		accuraatRaakt = world->computeCollision(onderkaak, bovenkaak, traversalSetting::DISTANCE, dist, 50, *positie);
		bolleke->setLocalPos((*positie));

		minimumTeLopen = (float)dist;
		gelopenAfstand->zero();
		hulp = false;
		}

		if (!accuraatRaakt) labelRaakt->m_fontColor.setA(0);
		else labelRaakt->m_fontColor.setA(1);

		//int diepte = cMultiMesh::checkRaakt(onderkaak, bovenkaak, 1);
		
		//cout << diepte << endl;

		/*cout << "DATA" << endl;
		cout << "IST1: " << istOnderkaak->getRootSphere()->getPosition() << " : " << istOnderkaak->getRootSphere()->getRadius() << endl;
		cout << "IST1: " << istBovenkaak->getRootSphere()->getPosition() << " : " << istBovenkaak->getRootSphere()->getRadius() << endl;

		cout << istOnderkaak->getRootSphere()->distance(istBovenkaak->getRootSphere(), cVector3d(0,0,0), cVector3d(0,0,0)) << " afstand tussen roots" << endl << endl;*/

		if (wilupdaten) {
			//bovenkaak->getCollisionDetector()->update();
			wilupdaten = false;
		}

    }
    
    // exit haptics thread
    simulationFinished = true;
}

//////////////////////////////////////////////////////////////////////////