#include "stdafx.h"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/io.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

#include <GL/glew.h>

#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <future>
#include <chrono>
#include <unordered_set>
#include <unordered_map>

#include "leidaSep.hpp"
#include "Shader.hpp"
#include "Camera.hpp"
#include "VeryNaiveSphere.hpp"
#include "cyl_test.hpp"
#include "box3D.hpp"
#include "hashMap_shiftC.hpp"
//#include "NatNetClientPCL.hpp"
#include "Server.h"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr obj_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr s_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

//NatNetClientPCL natnet_client("192.168.1.29", "192.168.1.33");
Server * myServer;

float min_x{ 0 }, max_x{ 0 }, min_y{ 0 }, max_y{ 0 }, min_z{ 0 }, max_z{ 0 };
float double_diagonal_length{ 0 };
float received_pitch{ 0.f }, received_yaw{ 0.f };

// Function prototypes
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void mouse_press_callback(GLFWwindow* window, int button, int action, int mods);
void Do_Camera_movement(pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> &octree);
void Do_Sphere_movement(VeryNaiveSphere &mySphere, pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> &octree);
void drawCrosshair(GLuint MatrixID_model, glm::mat4 Model, glm::vec3 pos);
void drawNormals(GLuint MatrixID_model, glm::mat4 Model, glm::vec3 pos);
void deleteMarkedPoints();
void optimize_octree_resolution();
void floodFill(glm::vec3);
void reverseLastSelected();
void unmarkPrevLastSelected();
void CaptureScreen();
void PCLseg();

// Window dimensions
constexpr GLuint WIDTH = 1300, HEIGHT = 720;

// my matrices for visualization on the screen
glm::mat4 ModelView = glm::mat4(1.0f);
glm::mat4 Proj = glm::mat4(1.0f);

float cam_near{ 0.1f }, cam_far{ 90000.f }; // define clipping plane

											// For camera, and key statements
Camera *camera;
bool keys[1024]; // to store if a key is actually pressed
GLfloat lastX = WIDTH / 2.f, lastY = HEIGHT / 2.f;
bool firstMouse = true;
bool isValidSearchPoint{ true };
bool rb_move{ false };
bool rb_orientation{ false };

float movementSensitivity{ 0.f };

float seedAvgZ{ 0.f };
float sPointZ{ 0.f };
double threshold{ 0.1 };


GLfloat deltaTime = 0.0f;
GLfloat lastFrame = 0.0f;

std::vector<GLfloat> vertices;
std::vector<GLfloat> vertices_sep;
std::vector<GLfloat> vertices_cross(6, 0.f);
std::vector<GLfloat> vertices_line;
std::vector<std::vector<int>> radius_vector;

size_t kNN_num{ 1000 };
float search_radius{ 0.09f };
int pic_num{ 0 };

float resolution = 1.f; // for the leaf level of the octree
float divider = 2.f;

float orientationValues[3];
float movementForwardOrBackward[1];

GLuint VBO, VAO, VBO_cross, VAO_cross, VBO_line, VAO_line;
pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> *octree;
pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> *obj_octree;

// The MAIN function, from here we start the application and run the main loop
int main()
{
	//PCLseg();
	std::cout << "Float: " << sizeof(float) << "\n";
	/*
	bool bResult = false;

	Server mylink(65510, 65511, &bResult);
	if (!bResult)
	{
	printf("Failed to create Server object!\n");
	return 0;
	}

	printf("Server, waiting for connection...\n");
	fflush(NULL);
	mylink.Connect();

	printf("Server, got a connection...\n");
	fflush(NULL);
	*/
	octree = new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(resolution);
	///*************************************************************************************///
	///******************************** PCL LOADING PART ***********************************///
	///*************************************************************************************///

	//PCLseg();

	// (This part will be separated later!)
	std::string inFile = "example.pcd";
	std::string inFile2 = "example.pcd";
	std::cout << "Filename: ";
	std::cin >> inFile;
	std::cout << "Filename2: ";
	std::cin >> inFile2;

	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(inFile, *cloud) == -1) // load the file
	{
		PCL_ERROR("Couldn't read the file: \n");
		return (-1);
	}
	std::cout << "Loaded " << cloud->width * cloud->height << " data points from test_pcd.pcd with the following fields: " << std::endl;


	std::cout << cloud->width << std::endl;
	std::cout << cloud->height << std::endl;

	// Set the initial minimum value of each coordinate:
	min_x = cloud->points[0].x;
	min_y = cloud->points[0].y;
	min_z = cloud->points[0].z;

	// Calculate the minimum value of each coordinate:
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		if (cloud->points[i].x < min_x) min_x = cloud->points[i].x;
		if (cloud->points[i].y < min_y) min_y = cloud->points[i].y;
		if (cloud->points[i].z < min_z) min_z = cloud->points[i].z;

		//if(i % 4 == 0) obj_cloud->push_back(cloud->points[i]);
	}
	/*
	cloud->is_dense = false;
	pcl::io::savePCDFileBinary(inFile2, *obj_cloud);

	return 0;
	*/
	/*
	// Transform the cloud to the origin of its coordinate system, for easier handling of the cloud data.
	// This part of the code should be removed later. (Just helped me at the beginning)
	for(size_t i = 0; i < cloud->points.size (); ++i)
	{
	cloud->points[i].x -= min_x;
	cloud->points[i].y -= min_y;
	cloud->points[i].z -= min_z;
	}
	*/
	// Set the initial minimum value of each coordinate:
	min_x = cloud->points[0].x;
	min_y = cloud->points[0].y;
	min_z = cloud->points[0].z;
	max_x = cloud->points[0].x;
	max_y = cloud->points[0].y;
	max_z = cloud->points[0].z;

	// Calculate the minimum value of each coordinate:
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		if (cloud->points[i].x < min_x) min_x = cloud->points[i].x;
		if (cloud->points[i].y < min_y) min_y = cloud->points[i].y;
		if (cloud->points[i].z < min_z) min_z = cloud->points[i].z;
		if (cloud->points[i].x > max_x) max_x = cloud->points[i].x;
		if (cloud->points[i].y > max_y) max_y = cloud->points[i].y;
		if (cloud->points[i].z > max_z) max_z = cloud->points[i].z;
	}

	std::cout << "MIN_X: " << min_x << '\n';
	std::cout << "MIN_Y: " << min_y << '\n';
	std::cout << "MIN_Z: " << min_z << '\n';
	std::cout << "MAX_X: " << max_x << '\n';
	std::cout << "MAX_Y: " << max_y << '\n';
	std::cout << "MAX_Z: " << max_z << '\n';

	// Compute diag length
	double_diagonal_length = sqrt(((max_x - min_x) * (max_x - min_x)) + ((max_y - min_y) * (max_y - min_y)) + ((max_z - min_z) * (max_z - min_z))) * 2.f;
	std::cout << "DIAG_LENGTH: " << double_diagonal_length << '\n';

	movementSensitivity = double_diagonal_length / 119.713f;

	camera = new Camera(movementSensitivity, glm::vec3(min_x, min_y, min_z));

	///*************************************************************************************///
	///******************************** GLFW INIT PART ***********************************///
	///*************************************************************************************///

	// Init GLFW
	glfwInit();

	// Set all the required options for GLFW
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

	// Create a GLFWwindow object that we can use for GLFW's functions
	GLFWwindow* window = glfwCreateWindow(WIDTH, HEIGHT, "PCL with OpenGL", nullptr, nullptr);
	glfwMakeContextCurrent(window);

	std::cout << "hello\n";
	// Set the required callback functions
	glfwSetKeyCallback(window, key_callback);
	std::cout << "hello\n";
	glfwSetCursorPosCallback(window, mouse_callback);
	glfwSetScrollCallback(window, scroll_callback);
	glfwSetMouseButtonCallback(window, mouse_press_callback);

	// Options
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	// This is how GLEW knows that it should use a modern approach for retrieving function pointers and extensions
	glewExperimental = GL_TRUE;

	// Initialize GLEW to setup the OpenGL Function pointers
	glewInit();

	// Define the viewport dimensions
	glViewport(0, 0, WIDTH, HEIGHT);

	glEnable(GL_DEPTH_TEST);
	glPointSize(1.3f);

	///*************************************************************************************///
	///******************************** VBO LOADING PART ***********************************///
	///*************************************************************************************///

	Shader ourShader("shader.vs", "shader.frag"); // load the shaders

	std::size_t v_size = cloud->points.size() * 6; // size of my points
												   // (all of them contains 6 member)

												   // I will fill up this vector with all the data from cloud->points
	vertices.resize(v_size);

	auto time_start = std::chrono::steady_clock::now();

	auto numOfCores = std::thread::hardware_concurrency();
	auto points_part = cloud->points.size() / numOfCores;

	std::vector<std::future<void>> read_threads;

	for (size_t j{ 0 }; j < numOfCores; j++)
	{
		read_threads.emplace_back(std::async([j, numOfCores, points_part]()
		{
			for (size_t i = j * points_part; i < (j + 1) * points_part; i++)
			{
				size_t num = (i * 6);

				vertices[num + 0] = cloud->points[i].x;
				vertices[num + 1] = cloud->points[i].y;
				vertices[num + 2] = cloud->points[i].z;

				vertices[num + 3] = (float)cloud->points[i].r / 256.f;
				vertices[num + 4] = (float)cloud->points[i].g / 256.f;
				vertices[num + 5] = (float)cloud->points[i].b / 256.f;


				//std::cout << "COLOR: " << vertices[num+3] << " || " << vertices[num+4] << " || " << vertices[num+5] << '\n';
			}
		}));
	}

	for (auto &thread : read_threads) thread.get();

	/*
	// another half of my points
	for(size_t i = 0; i < cloud->points.size(); i++)
	{
	size_t num = (i * 6);

	vertices[num + 0] = cloud->points[i].x;
	vertices[num + 1] = cloud->points[i].y;
	vertices[num + 2] = cloud->points[i].z;

	vertices[num + 3] = (float)cloud->points[i].r / 256.f;
	vertices[num + 4] = (float)cloud->points[i].g / 256.f;
	vertices[num + 5] = (float)cloud->points[i].b / 256.f;
	}

	*/
	auto time_end = std::chrono::steady_clock::now();
	std::cout << "TIME: " << std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count() << " ms\n";
	// Now, I've filled up my vector!

	std::cout << "*****!!!DONE! (read)!!!*****" << std::endl;


	/*
	std::future<void>(std::async([j, numOfCores, points_part]()
	{

	}));
	*/

	/// Create the VBO from the data:
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);

	// Bind the Vertex Array Object first, then bind and set the vertex buffer(s) and the attribute pointer(s).
	glBindVertexArray(VAO);

	std::cout << "KERESZT_SZINBEALLITASA!\n";

	vertices_cross[3] = 254.f;
	vertices_cross[4] = 254.f;
	vertices_cross[5] = 0.f;

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * vertices.size(), vertices.data(), GL_STATIC_DRAW);


	// Position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)0);
	glEnableVertexAttribArray(0);

	// Color attribute
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)(3 * sizeof(GLfloat)));
	glEnableVertexAttribArray(1);

	glBindVertexArray(0); // Unbind VAO

	std::cout << "*****!!!DONE! (VBO load)!!!*****" << std::endl;

	glGenVertexArrays(1, &VAO_cross);
	glGenBuffers(1, &VBO_cross);

	//cross vertices

	// Bind the Vertex Array Object first, then bind and set the vertex buffer(s) and the attribute pointer(s).
	glBindVertexArray(VAO_cross);

	glBindBuffer(GL_ARRAY_BUFFER, VBO_cross);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * vertices_cross.size(), vertices_cross.data(), GL_STATIC_DRAW);

	// Position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)0);
	glEnableVertexAttribArray(0);

	// Color attribute
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)(3 * sizeof(GLfloat)));
	glEnableVertexAttribArray(1);

	glBindVertexArray(0); // Unbind

	glGenVertexArrays(1, &VAO_line);
	glGenBuffers(1, &VBO_line);

	//cross vertices

	// Bind the Vertex Array Object first, then bind and set the vertex buffer(s) and the attribute pointer(s).
	glBindVertexArray(VAO_line);

	glBindBuffer(GL_ARRAY_BUFFER, VBO_line);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * vertices_line.size(), vertices_line.data(), GL_STATIC_DRAW);

	// Position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)0);
	glEnableVertexAttribArray(0);

	// Color attribute
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)(3 * sizeof(GLfloat)));
	glEnableVertexAttribArray(1);

	glBindVertexArray(0); // Unbind

	ourShader.Use();

	/// Load transformation matrices
	GLuint MatrixID_modelview = glGetUniformLocation(ourShader.Program, "modelview");
	GLuint MatrixID_proj = glGetUniformLocation(ourShader.Program, "projection");

	// Send our transformations to the currently bound shader
	glUniformMatrix4fv(MatrixID_modelview, 1, GL_FALSE, &ModelView[0][0]);
	glUniformMatrix4fv(MatrixID_proj, 1, GL_FALSE, &Proj[0][0]);

	//vertices.clear();
	//cloud.reset();

	std::cout << "Sphere init...";

	VeryNaiveSphere mySphere(500, 500, glm::vec3(10000.f, 10000.f, 10000.f));
	mySphere.init_sphere();
	std::cout << "done!\n";
	/*
	std::cout << "SEPARATION PART: !\n";

	s_cloud = separation(cloud);
	std::cout << "Sep done!\n";


	Approximation(cloud, s_cloud);

	std::cout << "Approx done!\n";

	obj_cloud = redundancy(s_cloud);
	std::cout << "Redundancy done!\n";
	*/
	
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(inFile2, *obj_cloud) == -1) // load the file
	{
		PCL_ERROR("Couldn't read the file: \n");
		return (-1);
	}
	std::cout << "Loaded " << obj_cloud->width * obj_cloud->height << " data points from test_pcd.pcd with the following fields: " << std::endl;

	obj_cloud->width = obj_cloud->points.size();
	obj_cloud->height = 1;
	obj_cloud->is_dense = false;
	
	

	vertices_sep.resize(obj_cloud->points.size() * 6);
	points_part = obj_cloud->points.size() / numOfCores;

	read_threads.clear();

	for (size_t j{ 0 }; j < numOfCores; j++)
	{
		read_threads.emplace_back(std::async([j, numOfCores, points_part]()
		{
			for (size_t i = j * points_part; i < (j + 1) * points_part; i++)
			{
				size_t num = (i * 6);

				vertices_sep[num + 0] = obj_cloud->points[i].x;
				vertices_sep[num + 1] = obj_cloud->points[i].y;
				vertices_sep[num + 2] = obj_cloud->points[i].z;

				vertices_sep[num + 3] = (float)obj_cloud->points[i].r / 256.f;
				vertices_sep[num + 4] = (float)obj_cloud->points[i].g / 256.f;
				vertices_sep[num + 5] = (float)obj_cloud->points[i].b / 256.f;


				//std::cout << "COLOR: " << vertices[num+3] << " || " << vertices[num+4] << " || " << vertices[num+5] << '\n';
			}
		}));
	}

	std::cout << "Make_Vertices_sep done!\n";

	for (auto &thread : read_threads) thread.get();

	std::cout << "Make_Vertices_sep get done!\n";

	// Fill up my tree:


	octree->setInputCloud(cloud);
	octree->addPointsFromInputCloud();
	std::cout << "Octree init done!\n";

	std::vector<float> pointRadiusSquaredDistance; // distance of the indexed points from the reference point

	pcl::PointXYZRGB sPoint;

	int num_of_empty_vectors{ 0 };
	/*
	radius_vector.resize(cloud->points.size());

	time_start = std::chrono::steady_clock::now();
	for(size_t i{0}; i < vertices.size(); i += 6)
	{
	sPoint.x = vertices[i];
	sPoint.y = vertices[i+1];
	sPoint.z = vertices[i+2];

	std::vector<int> pointIdxRadiusSearch; // index of the cloud points within the radius
	pointRadiusSquaredDistance.clear();

	octree->radiusSearch(sPoint, 0.1, pointIdxRadiusSearch, pointRadiusSquaredDistance);

	radius_vector[i/6] = pointIdxRadiusSearch;

	if(radius_vector[i/6].size() == 0) num_of_empty_vectors++;
	}
	time_end = std::chrono::steady_clock::now();
	std::cout << "MERET_2: " << num_of_empty_vectors << '\n';
	std::cout << "TIME: " << std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count() / 1000.f << " sec\n";
	*/
	///*************************************************************************************///
	///******************************** MAIN LOOP PART ***********************************///
	///*************************************************************************************///

	std::cout << "*****!!!LOOP!!!*****" << std::endl;
	// Main loop

	//SetConsoleTitle("PCL_with_OpenGL_console");

	while (!glfwWindowShouldClose(window))
	{
		// Set frame time
		GLfloat currentFrame = glfwGetTime();
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;


		//unsigned char *pdata = new unsigned char[WIDTH*HEIGHT*3];
		//glReadPixels(0, 0, WIDTH, HEIGHT, GL_RGB, GL_UNSIGNED_BYTE, pdata);

		//std::cout << "MERET: " << sizeof(unsigned char) * (WIDTH*HEIGHT*3) / (1024 * 1024) << '\n';
		/*
		FILE *outfile;
		if ((outfile = fopen("sample.jpeg", "wb")) == NULL) {
		printf("can't open %s");
		exit(1);
		}

		struct jpeg_compress_struct cinfo;
		struct jpeg_error_mgr       jerr;

		cinfo.err = jpeg_std_error(&jerr);
		jpeg_create_compress(&cinfo);
		jpeg_stdio_dest(&cinfo, outfile);

		cinfo.image_width      = width;
		cinfo.image_height     = height;
		cinfo.input_components = 3;
		cinfo.in_color_space   = JCS_RGB;

		jpeg_set_defaults(&cinfo);
		set the quality [0..100]
		jpeg_set_quality (&cinfo, 100, true);
		jpeg_start_compress(&cinfo, true);

		JSAMPROW row_pointer;
		int row_stride = width * 3;

		while (cinfo.next_scanline < cinfo.image_height) {
		row_pointer = (JSAMPROW) &pdata[cinfo.next_scanline*row_stride];
		jpeg_write_scanlines(&cinfo, &row_pointer, 1);
		}

		jpeg_finish_compress(&cinfo);

		fclose(outfile);
		*/
		// Check if any events have been activiated (key pressed, mouse moved etc.) and call corresponding response functions

		//mylink.RecvFloats(movementForwardOrBackward, 1);
		//mylink.RecvFloats(orientationValues, 3);

		glfwPollEvents();
		//natnet_client.getFrameData();
		/*
		if (rb_orientation)
		{
		camera->Front = natnet_client.getRigidBodyFrontVector();
		camera->Right = natnet_client.getRigidBodyRightVector();
		camera->Up = natnet_client.getRigidBodyUpVector();

		camera->Yaw = orientationValues[0];
		camera->Pitch = -orientationValues[1];

		camera->updateCameraVectors();

		}
		*/
		//if(rb_move) camera->ProcessRigidBodyMovement(natnet_client.getXoffset(), natnet_client.getYoffset(), natnet_client.getZoffset());
		Do_Camera_movement(*octree);
		//Do_Sphere_movement(mySphere, *octree);
		deleteMarkedPoints();

		// Render
		// Clear the colorbuffer and the depthbuffer
		glClearColor(0.f, 0.f, 0.f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Draw
		ourShader.Use();
		Proj = glm::perspective(camera->Zoom, (float)WIDTH / (float)HEIGHT, cam_near, cam_far);

		ModelView = camera->GetViewMatrix();

		glUniformMatrix4fv(MatrixID_proj, 1, GL_FALSE, &Proj[0][0]);
		glUniformMatrix4fv(MatrixID_modelview, 1, GL_FALSE, &ModelView[0][0]);

		glBindVertexArray(VAO);
		glDrawArrays(GL_POINTS, 0, vertices.size() / divider);
		glBindVertexArray(0);


		drawNormals(MatrixID_modelview, ModelView, camera->Position + camera->Front);

		drawCrosshair(MatrixID_modelview, ModelView, camera->Position + camera->Front);

		//mySphere.draw(MatrixID_modelview, ModelView);

		// Swap the screen buffers
		glfwSwapBuffers(window);
	}
	// Properly de-allocate all resources once they've outlived their purpose
	glDeleteVertexArrays(1, &VAO);
	glDeleteBuffers(1, &VBO);

	delete camera;

	// Terminate GLFW, clearing any resources allocated by GLFW.
	glfwTerminate();

	char toSaveChar = 'n';

	std::cout << "Do u wanna save the file?(y/n)\n";
	std::cin >> toSaveChar;

	if (toSaveChar == 'y' || toSaveChar == 'Y')
	{
		std::string outFile = "example_out.pcd";
		std::cout << "Filename: ";
		std::cin >> outFile;

		cloud->is_dense = false;
		pcl::io::savePCDFileBinary(outFile, *cloud);
	}

	return 0;
}

///*************************************************************************************///
///******************************** FUNC IMPLEMETATION ***********************************///
///*************************************************************************************///

// Moves/alters the camera positions based on user input
void Do_Camera_movement(pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> &octree)
{
	// Camera controls
	if (keys[GLFW_KEY_W] || (movementForwardOrBackward[0] > 1.f))
		camera->ProcessKeyboard(FORWARD, deltaTime, cloud, octree);
	if (keys[GLFW_KEY_S] || (movementForwardOrBackward[0] < -1.f))
		camera->ProcessKeyboard(BACKWARD, deltaTime, cloud, octree);
	if (keys[GLFW_KEY_A])
		camera->ProcessKeyboard(LEFT, deltaTime, cloud, octree);
	if (keys[GLFW_KEY_D])
		camera->ProcessKeyboard(RIGHT, deltaTime, cloud, octree);
	if (keys[GLFW_KEY_SPACE])
		camera->ProcessKeyboard(UP, deltaTime, cloud, octree);
}

void Do_Sphere_movement(VeryNaiveSphere &mySphere, pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> &octree)
{
	// Sphere conrols
	// Set pre-defined directions:
	if (keys[GLFW_KEY_T])
		mySphere.changeDirToX_neg();
	if (keys[GLFW_KEY_Z])
		mySphere.changeDirToX_pos();
	if (keys[GLFW_KEY_G])
		mySphere.changeDirToY_neg();
	if (keys[GLFW_KEY_H])
		mySphere.changeDirToY_pos();
	if (keys[GLFW_KEY_B])
		mySphere.changeDirToZ_neg();
	if (keys[GLFW_KEY_N])
		mySphere.changeDirToZ_pos();

	// Move the sphere:
	if (keys[GLFW_KEY_M])
		mySphere.move(deltaTime, cloud, octree);
}

void deleteMarkedPoints()
{
	if (keys[GLFW_KEY_E])
	{
		// if(indicesToDelete.size() == 0) return;

		auto originalSize = vertices.size();

		std::vector<GLfloat> new_vertices;
		cloud->points.clear();

		size_t idx{ 0 };

		for (size_t i{ 0 }; i < vertices.size(); i += 6)
		{
			if (vertices[i + 5] > 0.f)
			{
				new_vertices.push_back(vertices[i]);
				new_vertices.push_back(vertices[i + 1]);
				new_vertices.push_back(vertices[i + 2]);
				new_vertices.push_back(vertices[i + 3]);
				new_vertices.push_back(vertices[i + 4]);
				new_vertices.push_back(vertices[i + 5]);

				pcl::PointXYZRGB pt;
				pt.x = vertices[i];
				pt.y = vertices[i + 1];
				pt.z = vertices[i + 2];
				pt.r = vertices[i + 3] * 256.f;
				pt.g = vertices[i + 4] * 256.f;
				pt.b = vertices[i + 5] * 256.f;

				cloud->points.push_back(pt);

				idx++;
			}
		}

		std::cout << "IDX: " << idx << '\n';

		cloud->width = cloud->points.size();

		// Fill up my tree:
		pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> *removable_octree;
		removable_octree = octree;

		octree = new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(resolution);
		octree->setInputCloud(cloud);
		octree->addPointsFromInputCloud();

		delete removable_octree;

		vertices = new_vertices;
		//        indicesToDelete.clear();

		glBindVertexArray(VAO);

		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * vertices.size(), vertices.data(), GL_DYNAMIC_DRAW);

		glBindVertexArray(0);
	}
}

// This is called whenever a key is pressed/released via GLFW
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode)
{
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GL_TRUE);

	if (key == GLFW_KEY_C && action == GLFW_PRESS)
	{
		glBindVertexArray(VAO);

		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * vertices.size(), vertices.data(), GL_STATIC_DRAW);

		glBindVertexArray(0);
	}

	if (key == GLFW_KEY_V && action == GLFW_PRESS)
	{
		glBindVertexArray(VAO);

		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * vertices_sep.size(), vertices_sep.data(), GL_STATIC_DRAW);

		glBindVertexArray(0);
	}

	if (key == GLFW_KEY_R && action == GLFW_PRESS)
	{
		reverseLastSelected();
	}

	if (key == GLFW_KEY_O && action == GLFW_PRESS)
	{
		optimize_octree_resolution();
	}

	if (key == GLFW_KEY_L && action == GLFW_PRESS)
	{
		//natnet_client.showRBtext(false);
	}

	if (key == GLFW_KEY_K && action == GLFW_PRESS)
	{
		rb_orientation = !rb_orientation;
	}

	if (key == GLFW_KEY_J && action == GLFW_PRESS)
	{
		rb_move = !rb_move;
	}

	if (key == GLFW_KEY_KP_ADD && action == GLFW_PRESS)
	{
		threshold += 0.01;
		std::cout << "Threshold: " << threshold * 100 << "%\n";
	}

	if (key == GLFW_KEY_KP_SUBTRACT && action == GLFW_PRESS)
	{
		threshold -= 0.01;
		std::cout << "Threshold: " << threshold * 100 << "%\n";
	}

	if (key >= 0 && key < 1024)
	{
		if (action == GLFW_PRESS)
			keys[key] = true;
		else if (action == GLFW_RELEASE)
			keys[key] = false;
	}

	if (key == GLFW_KEY_P && action == GLFW_PRESS)
	{
		CaptureScreen();
	}
}

void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
	if (firstMouse)
	{
		lastX = xpos;
		lastY = ypos;
		firstMouse = false;
	}

	GLfloat xoffset = xpos - lastX;
	GLfloat yoffset = lastY - ypos;  // Reversed, since y-coordinates go from bottom-left corner

	lastX = xpos;
	lastY = ypos;

	camera->ProcessMouseMovement(xoffset, yoffset);
}

void mouse_press_callback(GLFWwindow* window, int button, int action, int mods)
{
	if (action == GLFW_PRESS &&  button == GLFW_MOUSE_BUTTON_1)
	{
		unmarkPrevLastSelected();

		glm::vec3 normalDirectedDiagonal(camera->Front * (double_diagonal_length / 2.f));

		Point P0{ camera->Position.x - normalDirectedDiagonal.x, camera->Position.y - normalDirectedDiagonal.y, camera->Position.z - normalDirectedDiagonal.z };
		Point P1{ camera->Position.x + normalDirectedDiagonal.x, camera->Position.y + normalDirectedDiagonal.y, camera->Position.z + normalDirectedDiagonal.z };

		Line L0{ P0, P1 };

		std::vector<std::pair<int, float>> indicesToDist;

		for (size_t i{ 0 }; i < vertices.size(); i += 6)
		{
			if (camera->Front.x < 0.f && vertices[i] > camera->Position.x) continue;
			if (camera->Front.x > 0.f && vertices[i] < camera->Position.x) continue;
			if (camera->Front.y < 0.f && vertices[i + 1] > camera->Position.y) continue;
			if (camera->Front.y > 0.f && vertices[i + 1] < camera->Position.y) continue;
			if (camera->Front.z < 0.f && vertices[i + 2] > camera->Position.z) continue;
			if (camera->Front.z > 0.f && vertices[i + 2] < camera->Position.z) continue;

			Point P_test{ vertices[i], vertices[i + 1], vertices[i + 2] };

			if (dist_Point_to_Line(P_test, L0) < 0.1f)
			{
				glm::vec3 cameraToPointVec{ vertices[i] - camera->Position.x, vertices[i + 1] - camera->Position.y, vertices[i + 2] - camera->Position.z };

				indicesToDist.push_back(std::make_pair(i / 6, glm::length(cameraToPointVec)));
			}
		}

		size_t minIndex{ 0 };
		float minDist{ std::numeric_limits<float>::max() };

		for (size_t i{ 0 }; i < indicesToDist.size(); i++)
		{
			if (indicesToDist[i].second < minDist)
			{
				minDist = indicesToDist[i].second;
				minIndex = indicesToDist[i].first;
			}
		}

		if (indicesToDist.size() > 0)
		{
			pcl::PointXYZRGB searchPoint;

			// This is the point, where I want to search the nearest points inside the radius.
			// This should be the center of the sphere.
			searchPoint.x = vertices[(minIndex * 6)];
			searchPoint.y = vertices[(minIndex * 6) + 1];
			searchPoint.z = vertices[(minIndex * 6) + 2];

			std::vector<GLfloat> obj_vertices(obj_cloud->points.size() * 6);

			std::cout << "Obj_Vector beallitasa...";

			for (size_t i = 0; i < obj_cloud->points.size(); i++)
			{
				size_t num = (i * 6);

				obj_vertices[num + 0] = obj_cloud->points[i].x;
				obj_vertices[num + 1] = obj_cloud->points[i].y;
				obj_vertices[num + 2] = obj_cloud->points[i].z;

				obj_vertices[num + 3] = (float)obj_cloud->points[i].r / 256.f;
				obj_vertices[num + 4] = (float)obj_cloud->points[i].g / 256.f;
				obj_vertices[num + 5] = (float)obj_cloud->points[i].b / 256.f;
			}

			std::cout << "KESZ!\n";

			Box3D myBox(glm::vec3(searchPoint.x, searchPoint.y, searchPoint.z), 0.3);

			bool isFindNewPoint{ false };
			int last_size{ 0 };
			int new_size{ 0 };
			int first_size{ 0 };

			std::cout << "Box beallitasa...\n";

			for (size_t i{ 0 }; i < obj_vertices.size(); i += 6)
			{
				if (myBox.isPointInsideBox(glm::vec3(obj_vertices[i], obj_vertices[i + 1], obj_vertices[i + 2]))) { new_size++; }
			}

			if (new_size > last_size)
			{
				last_size = new_size;
				first_size = new_size;
				isFindNewPoint = true;
			}

			std::cout << "Ciklusba lepes...\n";

			bool isValidBox{ true };

			while (isValidBox)
			{
				last_size = 0;
				int counter{ 0 };
				isFindNewPoint = true;

				while (isFindNewPoint)
				{
					counter++;
					isFindNewPoint = false;

					// MAX_X
					while (true)
					{
						new_size = 0;
						myBox.lengthenMax_X();
						for (size_t i{ 0 }; i < obj_vertices.size(); i += 6)
						{
							if (myBox.isPointInsideBox(glm::vec3(obj_vertices[i], obj_vertices[i + 1], obj_vertices[i + 2]))) { new_size++; }
						}

						if (new_size > last_size)
						{
							last_size = new_size;
							isFindNewPoint = true;
						}
						else
						{
							myBox.shortenMax_X();
							break;
						}
					}

					// MAX_Y
					while (true)
					{
						new_size = 0;
						myBox.lengthenMax_Y();
						for (size_t i{ 0 }; i < obj_vertices.size(); i += 6)
						{
							if (myBox.isPointInsideBox(glm::vec3(obj_vertices[i], obj_vertices[i + 1], obj_vertices[i + 2]))) { new_size++; }
						}

						if (new_size > last_size)
						{
							last_size = new_size;
							isFindNewPoint = true;
						}
						else
						{
							myBox.shortenMax_Y();
							break;
						}
					}

					// MIN_X
					while (true)
					{
						new_size = 0;
						myBox.lengthenMin_X();
						for (size_t i{ 0 }; i < obj_vertices.size(); i += 6)
						{
							if (myBox.isPointInsideBox(glm::vec3(obj_vertices[i], obj_vertices[i + 1], obj_vertices[i + 2]))) { new_size++; }
						}

						if (new_size > last_size)
						{
							last_size = new_size;
							isFindNewPoint = true;
						}
						else
						{
							myBox.shortenMin_X();
							break;
						}
					}

					// MIN_Y
					while (true)
					{
						new_size = 0;
						myBox.lengthenMin_Y();
						for (size_t i{ 0 }; i < obj_vertices.size(); i += 6)
						{
							if (myBox.isPointInsideBox(glm::vec3(obj_vertices[i], obj_vertices[i + 1], obj_vertices[i + 2]))) { new_size++; }
						}

						if (new_size > last_size)
						{
							last_size = new_size;
							isFindNewPoint = true;
						}
						else
						{
							myBox.shortenMin_Y();
							break;
						}
					}

				}


				size_t idx{ 0 };

				isValidBox = false;

				for (size_t i{ 0 }; i < vertices.size(); i += 6)
				{
					if (myBox.isPointInsideBox(glm::vec3(vertices[i], vertices[i + 1], vertices[i + 2])))
					{
						// if(vertices[i+5] < 0.f) continue;

						isValidBox = true;

						vertices[i + 3] = 254.f;
						vertices[i + 4] = 0.f;
						vertices[i + 5] = -1.f;
						/*
						cloud->points[i/6].r = vertices[i+3] * 256.f;
						cloud->points[i/6].g = vertices[i+4] * 256.f;
						cloud->points[i/6].b = vertices[i+5] * 256.f;
						*/
						idx++;
					}
				}

				myBox.lengthenMax_Z();
				myBox.shortenMin_Z();

			}

			Box3D myBox2(glm::vec3(searchPoint.x, searchPoint.y, searchPoint.z), 0.08);
			isValidBox = true;


			last_size = 0;
			isFindNewPoint = true;

			while (isFindNewPoint)
			{
				isFindNewPoint = false;

				// MAX_X
				while (true)
				{
					new_size = 0;
					myBox2.lengthenMax_X();
					for (size_t i{ 0 }; i < obj_vertices.size(); i += 6)
					{
						if (myBox2.isPointInsideBox(glm::vec3(obj_vertices[i], obj_vertices[i + 1], obj_vertices[i + 2]))) { new_size++; }
					}

					if (new_size > last_size)
					{
						last_size = new_size;
						isFindNewPoint = true;
					}
					else
					{
						myBox2.shortenMax_X();
						break;
					}
				}

				// MAX_Y
				while (true)
				{
					new_size = 0;
					myBox2.lengthenMax_Y();
					for (size_t i{ 0 }; i < obj_vertices.size(); i += 6)
					{
						if (myBox2.isPointInsideBox(glm::vec3(obj_vertices[i], obj_vertices[i + 1], obj_vertices[i + 2]))) { new_size++; }
					}

					if (new_size > last_size)
					{
						last_size = new_size;
						isFindNewPoint = true;
					}
					else
					{
						myBox2.shortenMax_Y();
						break;
					}
				}

				// MIN_X
				while (true)
				{
					new_size = 0;
					myBox2.lengthenMin_X();
					for (size_t i{ 0 }; i < obj_vertices.size(); i += 6)
					{
						if (myBox2.isPointInsideBox(glm::vec3(obj_vertices[i], obj_vertices[i + 1], obj_vertices[i + 2]))) { new_size++; }
					}

					if (new_size > last_size)
					{
						last_size = new_size;
						isFindNewPoint = true;
					}
					else
					{
						myBox2.shortenMin_X();
						break;
					}
				}

				// MIN_Y
				while (true)
				{
					new_size = 0;
					myBox2.lengthenMin_Y();
					for (size_t i{ 0 }; i < obj_vertices.size(); i += 6)
					{
						if (myBox2.isPointInsideBox(glm::vec3(obj_vertices[i], obj_vertices[i + 1], obj_vertices[i + 2]))) { new_size++; }
					}

					if (new_size > last_size)
					{
						last_size = new_size;
						isFindNewPoint = true;
					}
					else
					{
						myBox2.shortenMin_Y();
						break;
					}
				}

				// MIN_Z
				while (true)
				{
					new_size = 0;
					myBox2.lengthenMin_Z();
					for (size_t i{ 0 }; i < obj_vertices.size(); i += 6)
					{
						if (myBox2.isPointInsideBox(glm::vec3(obj_vertices[i], obj_vertices[i + 1], obj_vertices[i + 2]))) { new_size++; }
					}

					if (new_size > last_size)
					{
						last_size = new_size;
						isFindNewPoint = true;
					}
					else
					{
						myBox2.shortenMin_Z();
						break;
					}
				}

			}

			size_t idx{ 0 };

			for (size_t i{ 0 }; i < vertices.size(); i += 6)
			{
				if (myBox2.isPointInsideBox(glm::vec3(vertices[i], vertices[i + 1], vertices[i + 2])))
				{
					vertices[i + 3] = 254.f;
					vertices[i + 4] = 0.f;
					vertices[i + 5] = -1.f;
					/*
					cloud->points[i/6].r = vertices[i+3] * 256.f;
					cloud->points[i/6].g = vertices[i+4] * 256.f;
					cloud->points[i/6].b = vertices[i+5] * 256.f;
					*/
					idx++;
				}
			}



			std::cout << "Box beallitas vege!\n";
			/*            std::cout << "Resz pontfelho keszites eredertibol...\n";

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_part(new pcl::PointCloud<pcl::PointXYZRGB>());

			for(auto point : obj_cloud->points)
			{
			if(myBox.isPointInsideBox(glm::vec3(point.x, point.y, point.z)))
			{
			cloud_part->points.push_back(point);
			}
			}

			cloud_part->width = cloud_part->points.size();
			cloud_part->height = 1;
			cloud_part->is_dense = false;

			std::cout << "Szeparacio a resz felhore...\n";

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr sep_cloud_part(new pcl::PointCloud<pcl::PointXYZRGB>());
			sep_cloud_part = separation(cloud_part);
			sep_cloud_part->width = sep_cloud_part->points.size();
			sep_cloud_part->height = 1;
			sep_cloud_part->is_dense = false;

			std::cout << "Approximacio a resz felhore...\n";

			Approximation(cloud_part, sep_cloud_part);

			std::cout << "Redundancia kiiktatasa...\n";
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr sep_cloud_part_final(new pcl::PointCloud<pcl::PointXYZRGB>());

			sep_cloud_part_final = redundancy(sep_cloud_part);
			sep_cloud_part_final->width = sep_cloud_part_final->points.size();
			sep_cloud_part_final->height = 1;
			sep_cloud_part_final->is_dense = false;

			for(auto pt : cloud_part->points)
			{
			new_vertices.push_back(pt.x);
			new_vertices.push_back(pt.y);
			new_vertices.push_back(pt.z);

			pt.r = 254.f * 256.f;
			pt.g = -256.f;
			pt.b = -256.f;

			new_vertices.push_back((float)pt.r / 256.f);
			new_vertices.push_back((float)pt.g / 256.f);
			new_vertices.push_back((float)pt.b / 256.f);

			cloud->points.push_back(pt);
			}

			std::cout << "Vektorok beoltese az ujak beletetelevel KESZ\n";

			// cloud->width = cloud->points.size();

			// Fill up my tree:
			pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> *removable_octree;
			removable_octree = octree;

			octree = new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(resolution);
			octree->setInputCloud (cloud);
			octree->addPointsFromInputCloud();

			delete removable_octree;

			vertices = new_vertices;

			std::cout << "Uj felho szeparalasa\n";
			obj_cloud = separation(cloud);
			std::cout << "****DONE***\n";
			*/
			std::cout << "****Teglalap adatok:****\n";
			std::cout << "Min_X: " << myBox.getMin_X() << " || Max_X: " << myBox.getMax_X() << '\n';
			std::cout << "Min_Y: " << myBox.getMin_Y() << " || Max_Y: " << myBox.getMax_Y() << '\n';
			std::cout << "Min_Z: " << myBox.getMin_Z() << " || Max_Z: " << myBox.getMax_Z() << '\n';
		}

		glBindVertexArray(VAO);

		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(GLfloat) * vertices.size(), vertices.data());

		glBindVertexArray(0);
	}



	if (action == GLFW_PRESS &&  button == GLFW_MOUSE_BUTTON_2 && isValidSearchPoint)
	{
		std::cout << "\n ez itt a 2-es gomb!!!!!\n";
		unmarkPrevLastSelected();

		glm::vec3 normalDirectedDiagonal(camera->Front * (double_diagonal_length / 2.f));

		Point P0{ camera->Position.x - normalDirectedDiagonal.x, camera->Position.y - normalDirectedDiagonal.y, camera->Position.z - normalDirectedDiagonal.z };
		Point P1{ camera->Position.x + normalDirectedDiagonal.x, camera->Position.y + normalDirectedDiagonal.y, camera->Position.z + normalDirectedDiagonal.z };

		Line L0{ P0, P1 };

		std::vector<std::pair<int, float>> indicesToDist;

		for (size_t i{ 0 }; i < vertices.size(); i += 6)
		{
			if (camera->Front.x < 0.f && vertices[i] > camera->Position.x) continue;
			if (camera->Front.x > 0.f && vertices[i] < camera->Position.x) continue;
			if (camera->Front.y < 0.f && vertices[i + 1] > camera->Position.y) continue;
			if (camera->Front.y > 0.f && vertices[i + 1] < camera->Position.y) continue;
			if (camera->Front.z < 0.f && vertices[i + 2] > camera->Position.z) continue;
			if (camera->Front.z > 0.f && vertices[i + 2] < camera->Position.z) continue;

			Point P_test{ vertices[i], vertices[i + 1], vertices[i + 2] };

			if (dist_Point_to_Line(P_test, L0) < 0.01f)
			{
				glm::vec3 cameraToPointVec{ vertices[i] - camera->Position.x, vertices[i + 1] - camera->Position.y, vertices[i + 2] - camera->Position.z };

				indicesToDist.push_back(std::make_pair(i / 6, glm::length(cameraToPointVec)));
			}
		}

		size_t minIndex{ 0 };
		float minDist{ std::numeric_limits<float>::max() };

		for (size_t i{ 0 }; i < indicesToDist.size(); i++)
		{
			if (indicesToDist[i].second < minDist)
			{
				minDist = indicesToDist[i].second;
				minIndex = indicesToDist[i].first;
			}
		}

		if (indicesToDist.size() > 0)
		{
			pcl::PointXYZRGB searchPoint;

			// This is the point, where I want to search the nearest points inside the radius.
			// This should be the center of the sphere.
			searchPoint.x = vertices[(minIndex * 6)];
			searchPoint.y = vertices[(minIndex * 6) + 1];
			searchPoint.z = vertices[(minIndex * 6) + 2];

			std::vector<GLfloat> obj_vertices(obj_cloud->points.size() * 6);

			std::cout << "Obj_Vector beallitasa...";
			/*
			for(size_t i = 0; i < obj_cloud->points.size(); i++)
			{
			size_t num = (i * 6);

			obj_vertices[num + 0] = obj_cloud->points[i].x;
			obj_vertices[num + 1] = obj_cloud->points[i].y;
			obj_vertices[num + 2] = obj_cloud->points[i].z;

			obj_vertices[num + 3] = (float)obj_cloud->points[i].r / 256.f;
			obj_vertices[num + 4] = (float)obj_cloud->points[i].g / 256.f;
			obj_vertices[num + 5] = (float)obj_cloud->points[i].b / 256.f;
			}

			std::cout << "KESZ!\n";

			Box3D myBox(glm::vec3(searchPoint.x, searchPoint.y, searchPoint.z), 0.2);

			bool isFindNewPoint{false};
			int last_size{0};
			int new_size{0};
			int first_size{0};

			std::cout << "Box beallitasa...\n";

			for(size_t i{0}; i < obj_vertices.size(); i+=6)
			{
			if(myBox.isPointInsideBox(glm::vec3(obj_vertices[i], obj_vertices[i+1], obj_vertices[i+2]))) { new_size++; }
			}

			if(new_size > last_size)
			{
			last_size = new_size;
			first_size = new_size;
			isFindNewPoint = true;
			}

			std::cout << "Ciklusba lepes...\n";

			bool isValidBox{true};

			std::queue<size_t> indicesQueue;
			indicesQueue.push(minIndex);

			float minSpZ{searchPoint.z};

			std::vector<int> pointIdxRadiusSearch; // index of the cloud points within the radius
			std::vector<float> pointRadiusSquaredDistance; // distance of the indexed points from the reference point

			bool findNewPoints{true};

			std::cout << "region_growing_start\n";

			auto time_start = std::chrono::steady_clock::now();

			//floodFill(glm::vec3(searchPoint.x, searchPoint.y, searchPoint.z));

			//region growing algorithm

			Box3D myBox2(glm::vec3(searchPoint.x, searchPoint.y, searchPoint.z), 0.1);

			isValidBox = true;

			last_size = 0;
			isFindNewPoint = true;

			while(isFindNewPoint)
			{
			isFindNewPoint = false;

			// MAX_X
			while(true)
			{
			new_size = 0;
			myBox2.lengthenMax_X();
			for(size_t i{0}; i < obj_vertices.size(); i+=6)
			{
			if(myBox2.isPointInsideBox(glm::vec3(obj_vertices[i], obj_vertices[i+1], obj_vertices[i+2]))) { new_size++; }
			}

			if(new_size > last_size)
			{
			last_size = new_size;
			isFindNewPoint = true;
			}
			else
			{
			myBox2.shortenMax_X();
			break;
			}
			}

			// MAX_Y
			while(true)
			{
			new_size = 0;
			myBox2.lengthenMax_Y();
			for(size_t i{0}; i < obj_vertices.size(); i+=6)
			{
			if(myBox2.isPointInsideBox(glm::vec3(obj_vertices[i], obj_vertices[i+1], obj_vertices[i+2]))) { new_size++; }
			}

			if(new_size > last_size)
			{
			last_size = new_size;
			isFindNewPoint = true;
			}
			else
			{
			myBox2.shortenMax_Y();
			break;
			}
			}

			// MIN_X
			while(true)
			{
			new_size = 0;
			myBox2.lengthenMin_X();
			for(size_t i{0}; i < obj_vertices.size(); i+=6)
			{
			if(myBox2.isPointInsideBox(glm::vec3(obj_vertices[i], obj_vertices[i+1], obj_vertices[i+2]))) { new_size++; }
			}

			if(new_size > last_size)
			{
			last_size = new_size;
			isFindNewPoint = true;
			}
			else
			{
			myBox2.shortenMin_X();
			break;
			}
			}

			// MIN_Y
			while(true)
			{
			new_size = 0;
			myBox2.lengthenMin_Y();
			for(size_t i{0}; i < obj_vertices.size(); i+=6)
			{
			if(myBox2.isPointInsideBox(glm::vec3(obj_vertices[i], obj_vertices[i+1], obj_vertices[i+2]))) { new_size++; }
			}

			if(new_size > last_size)
			{
			last_size = new_size;
			isFindNewPoint = true;
			}
			else
			{
			myBox2.shortenMin_Y();
			break;
			}
			}

			// MIN_Z
			while(true)
			{
			new_size = 0;
			myBox2.lengthenMin_Z();
			for(size_t i{0}; i < obj_vertices.size(); i+=6)
			{
			if(myBox2.isPointInsideBox(glm::vec3(obj_vertices[i], obj_vertices[i+1], obj_vertices[i+2]))) { new_size++; }
			}

			if(new_size > last_size)
			{
			last_size = new_size;
			isFindNewPoint = true;
			}
			else
			{
			myBox2.shortenMin_Z();
			break;
			}
			}

			}

			minSpZ = myBox2.getMin_Z();
			*/

			std::vector<int> pointIdxRadiusSearch; // index of the cloud points within the radius
			std::vector<float> pointRadiusSquaredDistance; // distance of the indexed points from the reference point

			bool findNewPoints{ true };

			std::queue<size_t> indicesQueue;
			indicesQueue.push(minIndex);

			octree->nearestKSearch(cloud->points[minIndex], 10, pointIdxRadiusSearch, pointRadiusSquaredDistance);

			Eigen::Vector4f vec4seed;
			//float curvatureSeed{0.f};

			// And finally, I compute the normal:
			//pcl::computePointNormal(*cloud, pointIdxRadiusSearch, vec4seed, curvatureSeed);

			//glm::vec3 seedNormal(vec4seed[0], vec4seed[1], vec4seed[2]);

			size_t num_of_pointsInRadius{ 0 };

			float actual_search_radius{ search_radius };

			float minAvgOfDistances = std::accumulate(pointRadiusSquaredDistance.begin(), pointRadiusSquaredDistance.end(), 0.0) / 100.f;

			while (!indicesQueue.empty())
			{
				auto index = indicesQueue.front();
				indicesQueue.pop();

				vertices[(index * 6) + 3] = 254.f;
				vertices[(index * 6) + 4] = 0.f;
				vertices[(index * 6) + 5] = -1.f;

				searchPoint = cloud->points[index];
				pointIdxRadiusSearch.clear();
				pointRadiusSquaredDistance.clear();

				octree->nearestKSearch(searchPoint, 10, pointIdxRadiusSearch, pointRadiusSquaredDistance);
				//std::cout << pointIdxRadiusSearch.size() << '\n';

				float avgOfDistances = std::accumulate(pointRadiusSquaredDistance.begin(), pointRadiusSquaredDistance.end(), 0.0) / 100.f;

				// Eigen::Vector4f vec4;
				//float curvature{0.f};

				// And finally, I compute the normal:
				// pcl::computePointNormal(*cloud, pointIdxRadiusSearchNE, vec4, curvature);
				/*
				glm::vec3 normal(vec4[0], vec4[1], vec4[2]);

				float dotOfNormals = glm::dot(seedNormal, normal);
				float seedLength = glm::length(seedNormal);
				float actualLength = glm::length(normal);

				float angle = std::abs(glm::degrees(glm::acos(dotOfNormals / (seedLength * actualLength))));

				std::cout << "dot: " << dotOfNormals << '\n';
				std::cout << "seedl: " << seedLength << '\n';
				std::cout << "actl: " << actualLength << '\n';
				std::cout << "szog: " << angle << '\n';

				*/


				for (auto idx : pointIdxRadiusSearch)
				{
					/*
					std::vector<int> pointIdxRadiusSearchNE; // index of the cloud points within the radius
					std::vector<float> pointRadiusSquaredDistanceNE; // distance of the indexed points from the reference point

					octree->nearestKSearch(cloud->points[idx], 1, pointIdxRadiusSearchNE, pointRadiusSquaredDistanceNE);

					float nearestDistance = pointRadiusSquaredDistanceNE[1];
					std::cout << "nearest " << nearestDistance << '\n';
					std::cout << "min " << minDistance << '\n';
					*/
					if (vertices[(idx * 6) + 5] >= 0.f)
					{
						/*
						std::cout << "salala" << '\n';
						std::cout << minAvgOfDistances << '\n';
						std::cout << avgOfDistances << '\n';
						*/
						if (avgOfDistances < (1.f + threshold) * minAvgOfDistances && avgOfDistances >(1.f - threshold) * minAvgOfDistances) indicesQueue.push(idx);
						vertices[(idx * 6) + 3] = 254.f;
						vertices[(idx * 6) + 4] = 0.f;
						vertices[(idx * 6) + 5] = -1.f;
					}
				}

			}

			//      auto time_end = std::chrono::steady_clock::now();
			//   std::cout << "REGION GROWING TIME: " << std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count() / 1000.f << " sec\n";
			std::cout << "region_growing_done\n";

			//     std::cout << "Box beallitas vege!\n";
			/*            std::cout << "Resz pontfelho keszites eredertibol...\n";

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_part(new pcl::PointCloud<pcl::PointXYZRGB>());

			for(auto point : obj_cloud->points)
			{
			if(myBox.isPointInsideBox(glm::vec3(point.x, point.y, point.z)))
			{
			cloud_part->points.push_back(point);
			}
			}

			cloud_part->width = cloud_part->points.size();
			cloud_part->height = 1;
			cloud_part->is_dense = false;

			std::cout << "Szeparacio a resz felhore...\n";

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr sep_cloud_part(new pcl::PointCloud<pcl::PointXYZRGB>());
			sep_cloud_part = separation(cloud_part);
			sep_cloud_part->width = sep_cloud_part->points.size();
			sep_cloud_part->height = 1;
			sep_cloud_part->is_dense = false;

			std::cout << "Approximacio a resz felhore...\n";

			Approximation(cloud_part, sep_cloud_part);

			std::cout << "Redundancia kiiktatasa...\n";
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr sep_cloud_part_final(new pcl::PointCloud<pcl::PointXYZRGB>());

			sep_cloud_part_final = redundancy(sep_cloud_part);
			sep_cloud_part_final->width = sep_cloud_part_final->points.size();
			sep_cloud_part_final->height = 1;
			sep_cloud_part_final->is_dense = false;

			for(auto pt : cloud_part->points)
			{
			new_vertices.push_back(pt.x);
			new_vertices.push_back(pt.y);
			new_vertices.push_back(pt.z);

			pt.r = 254.f * 256.f;
			pt.g = -256.f;
			pt.b = -256.f;

			new_vertices.push_back((float)pt.r / 256.f);
			new_vertices.push_back((float)pt.g / 256.f);
			new_vertices.push_back((float)pt.b / 256.f);

			cloud->points.push_back(pt);
			}

			std::cout << "Vektorok beoltese az ujak beletetelevel KESZ\n";

			// cloud->width = cloud->points.size();

			// Fill up my tree:
			pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> *removable_octree;
			removable_octree = octree;

			octree = new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(resolution);
			octree->setInputCloud (cloud);
			octree->addPointsFromInputCloud();

			delete removable_octree;

			vertices = new_vertices;

			std::cout << "Uj felho szeparalasa\n";
			obj_cloud = separation(cloud);
			std::cout << "****DONE***\n";

			std::cout << "****Teglalap adatok:****\n";
			std::cout << "Min_X: " << myBox.getMin_X() << " || Max_X: " << myBox.getMax_X() << '\n';
			std::cout << "Min_Y: " << myBox.getMin_Y() << " || Max_Y: " << myBox.getMax_Y() << '\n';
			std::cout << "Min_Z: " << myBox.getMin_Z() << " || Max_Z: " << myBox.getMax_Z() << '\n';
			*/
		}

		//Normalis kirajzoltatas
		/*
		glm::vec3 normalDirectedDiagonal(camera->Front * (double_diagonal_length / 2.f));

		Point P0{camera->Position.x - normalDirectedDiagonal.x, camera->Position.y - normalDirectedDiagonal.y, camera->Position.z - normalDirectedDiagonal.z};
		Point P1{camera->Position.x + normalDirectedDiagonal.x, camera->Position.y + normalDirectedDiagonal.y, camera->Position.z + normalDirectedDiagonal.z};

		vertices_line.push_back(P0.x);
		vertices_line.push_back(P0.y);
		vertices_line.push_back(P0.z);
		vertices_line.push_back(P1.x);
		vertices_line.push_back(P1.y);
		vertices_line.push_back(P1.z);
		vertices_line.push_back(254.f);
		vertices_line.push_back(0.f);
		vertices_line.push_back(0.f);

		glBindVertexArray(VAO_line);
		glBindBuffer(GL_ARRAY_BUFFER, VBO_line);
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * vertices_line.size(), vertices_line.data(), GL_DYNAMIC_DRAW);
		glBindVertexArray(0);

		Line L0{P0, P1};

		std::vector<std::pair<int, float>> indicesToDist;

		for(size_t i{0}; i < vertices.size(); i += 6)
		{
		if(camera->Front.x < 0.f && vertices[i] > camera->Position.x) continue;
		if(camera->Front.x > 0.f && vertices[i] < camera->Position.x) continue;
		if(camera->Front.y < 0.f && vertices[i+1] > camera->Position.y) continue;
		if(camera->Front.y > 0.f && vertices[i+1] < camera->Position.y) continue;
		if(camera->Front.z < 0.f && vertices[i+2] > camera->Position.z) continue;
		if(camera->Front.z > 0.f && vertices[i+2] < camera->Position.z) continue;

		Point P_test{vertices[i], vertices[i+1], vertices[i+2]};

		if(dist_Point_to_Line(P_test, L0) < 0.1f)
		{
		glm::vec3 cameraToPointVec{vertices[i] - camera->Position.x, vertices[i+1] - camera->Position.y, vertices[i+2] - camera->Position.z};

		indicesToDist.push_back(std::make_pair(i/6, glm::length(cameraToPointVec)));
		}
		}

		size_t minIndex{0};
		float minDist{std::numeric_limits<float>::max()};

		for(size_t i{0}; i < indicesToDist.size(); i++)
		{
		if(indicesToDist[i].second < minDist)
		{
		minDist = indicesToDist[i].second;
		minIndex = indicesToDist[i].first;
		}
		}

		float maxOfZ{vertices[(minIndex * 6) + 2]};

		for(size_t i{0}; i < vertices.size(); i += 6)
		{
		if(vertices[i+5] < 0.f)
		{
		if(vertices[i+2] <= maxOfZ)
		{
		vertices[i+3] = (float)cloud->points[i/6].r / 256.f;
		vertices[i+4] = (float)cloud->points[i/6].g / 256.f;
		vertices[i+5] = (float)cloud->points[i/6].b / 256.f;
		}
		}
		}

		*/

		glBindVertexArray(VAO);

		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(GLfloat) * vertices.size(), vertices.data());

		glBindVertexArray(0);
	}

	if (action == GLFW_PRESS &&  button == GLFW_MOUSE_BUTTON_3 && isValidSearchPoint)
	{
		unmarkPrevLastSelected();

		glm::vec3 normalDirectedDiagonal(camera->Front * (double_diagonal_length / 2.f));

		Point P0{ camera->Position.x - normalDirectedDiagonal.x, camera->Position.y - normalDirectedDiagonal.y, camera->Position.z - normalDirectedDiagonal.z };
		Point P1{ camera->Position.x + normalDirectedDiagonal.x, camera->Position.y + normalDirectedDiagonal.y, camera->Position.z + normalDirectedDiagonal.z };

		Line L0{ P0, P1 };

		std::vector<std::pair<int, float>> indicesToDist;

		for (size_t i{ 0 }; i < vertices.size(); i += 6)
		{
			if (camera->Front.x < 0.f && vertices[i] > camera->Position.x) continue;
			if (camera->Front.x > 0.f && vertices[i] < camera->Position.x) continue;
			if (camera->Front.y < 0.f && vertices[i + 1] > camera->Position.y) continue;
			if (camera->Front.y > 0.f && vertices[i + 1] < camera->Position.y) continue;
			if (camera->Front.z < 0.f && vertices[i + 2] > camera->Position.z) continue;
			if (camera->Front.z > 0.f && vertices[i + 2] < camera->Position.z) continue;

			Point P_test{ vertices[i], vertices[i + 1], vertices[i + 2] };

			if (dist_Point_to_Line(P_test, L0) < 0.1f)
			{
				glm::vec3 cameraToPointVec{ vertices[i] - camera->Position.x, vertices[i + 1] - camera->Position.y, vertices[i + 2] - camera->Position.z };

				indicesToDist.push_back(std::make_pair(i / 6, glm::length(cameraToPointVec)));
			}
		}

		size_t minIndex{ 0 };
		float minDist{ std::numeric_limits<float>::max() };

		for (size_t i{ 0 }; i < indicesToDist.size(); i++)
		{
			if (indicesToDist[i].second < minDist)
			{
				minDist = indicesToDist[i].second;
				minIndex = indicesToDist[i].first;
			}
		}

		if (indicesToDist.size() > 0)
		{
			pcl::PointXYZRGB searchPoint;

			// This is the point, where I want to search the nearest points inside the radius.
			// This should be the center of the sphere.
			searchPoint.x = vertices[(minIndex * 6)];
			searchPoint.y = vertices[(minIndex * 6) + 1];
			searchPoint.z = vertices[(minIndex * 6) + 2];

			std::vector<GLfloat> obj_vertices(obj_cloud->points.size() * 6);

			std::cout << "Obj_Vector beallitasa...";

			for (size_t i = 0; i < obj_cloud->points.size(); i++)
			{
				size_t num = (i * 6);

				obj_vertices[num + 0] = obj_cloud->points[i].x;
				obj_vertices[num + 1] = obj_cloud->points[i].y;
				obj_vertices[num + 2] = obj_cloud->points[i].z;

				obj_vertices[num + 3] = (float)obj_cloud->points[i].r / 256.f;
				obj_vertices[num + 4] = (float)obj_cloud->points[i].g / 256.f;
				obj_vertices[num + 5] = (float)obj_cloud->points[i].b / 256.f;
			}

			std::cout << "KESZ!\n";

			Box3D myBox(glm::vec3(searchPoint.x, searchPoint.y, searchPoint.z), 0.3);

			bool isFindNewPoint{ false };
			int last_size{ 0 };
			int new_size{ 0 };
			int first_size{ 0 };

			std::cout << "Box beallitasa...\n";

			for (size_t i{ 0 }; i < obj_vertices.size(); i += 6)
			{
				if (myBox.isPointInsideBox(glm::vec3(obj_vertices[i], obj_vertices[i + 1], obj_vertices[i + 2]))) { new_size++; }
			}

			if (new_size > last_size)
			{
				last_size = new_size;
				first_size = new_size;
				isFindNewPoint = true;
			}

			std::cout << "Ciklusba lepes...\n";

			bool isValidBox{ true };

			std::queue<size_t> indicesQueue;
			indicesQueue.push(minIndex);

			float minSpZ{ searchPoint.z };

			std::vector<int> pointIdxRadiusSearch; // index of the cloud points within the radius
			std::vector<float> pointRadiusSquaredDistance; // distance of the indexed points from the reference point

			bool findNewPoints{ true };

			std::cout << "region_growing_start\n";

			auto time_start = std::chrono::steady_clock::now();

			//floodFill(glm::vec3(searchPoint.x, searchPoint.y, searchPoint.z));

			//region growing algorithm

			Box3D myBox2(glm::vec3(searchPoint.x, searchPoint.y, searchPoint.z), 0.1);

			isValidBox = true;

			last_size = 0;
			isFindNewPoint = true;

			while (isFindNewPoint)
			{
				isFindNewPoint = false;

				// MAX_X
				while (true)
				{
					new_size = 0;
					myBox2.lengthenMax_X();
					for (size_t i{ 0 }; i < obj_vertices.size(); i += 6)
					{
						if (myBox2.isPointInsideBox(glm::vec3(obj_vertices[i], obj_vertices[i + 1], obj_vertices[i + 2]))) { new_size++; }
					}

					if (new_size > last_size)
					{
						last_size = new_size;
						isFindNewPoint = true;
					}
					else
					{
						myBox2.shortenMax_X();
						break;
					}
				}

				// MAX_Y
				while (true)
				{
					new_size = 0;
					myBox2.lengthenMax_Y();
					for (size_t i{ 0 }; i < obj_vertices.size(); i += 6)
					{
						if (myBox2.isPointInsideBox(glm::vec3(obj_vertices[i], obj_vertices[i + 1], obj_vertices[i + 2]))) { new_size++; }
					}

					if (new_size > last_size)
					{
						last_size = new_size;
						isFindNewPoint = true;
					}
					else
					{
						myBox2.shortenMax_Y();
						break;
					}
				}

				// MIN_X
				while (true)
				{
					new_size = 0;
					myBox2.lengthenMin_X();
					for (size_t i{ 0 }; i < obj_vertices.size(); i += 6)
					{
						if (myBox2.isPointInsideBox(glm::vec3(obj_vertices[i], obj_vertices[i + 1], obj_vertices[i + 2]))) { new_size++; }
					}

					if (new_size > last_size)
					{
						last_size = new_size;
						isFindNewPoint = true;
					}
					else
					{
						myBox2.shortenMin_X();
						break;
					}
				}

				// MIN_Y
				while (true)
				{
					new_size = 0;
					myBox2.lengthenMin_Y();
					for (size_t i{ 0 }; i < obj_vertices.size(); i += 6)
					{
						if (myBox2.isPointInsideBox(glm::vec3(obj_vertices[i], obj_vertices[i + 1], obj_vertices[i + 2]))) { new_size++; }
					}

					if (new_size > last_size)
					{
						last_size = new_size;
						isFindNewPoint = true;
					}
					else
					{
						myBox2.shortenMin_Y();
						break;
					}
				}

				// MIN_Z
				while (true)
				{
					new_size = 0;
					myBox2.lengthenMin_Z();
					for (size_t i{ 0 }; i < obj_vertices.size(); i += 6)
					{
						if (myBox2.isPointInsideBox(glm::vec3(obj_vertices[i], obj_vertices[i + 1], obj_vertices[i + 2]))) { new_size++; }
					}

					if (new_size > last_size)
					{
						last_size = new_size;
						isFindNewPoint = true;
					}
					else
					{
						myBox2.shortenMin_Z();
						break;
					}
				}

			}

			minSpZ = myBox2.getMin_Z();

			size_t num_of_pointsInRadius{ 0 };

			float actual_search_radius{ search_radius };

			while (!indicesQueue.empty())
			{
				auto index = indicesQueue.front();
				indicesQueue.pop();

				searchPoint = cloud->points[index];
				if (searchPoint.z < minSpZ) continue;
				pointIdxRadiusSearch.clear();
				pointRadiusSquaredDistance.clear();

				octree->radiusSearch(searchPoint, actual_search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

				for (auto idx : pointIdxRadiusSearch)
				{
					if (vertices[(idx * 6) + 5] >= 0.f)
					{
						indicesQueue.push(idx);

						vertices[(idx * 6) + 3] = 254.f;
						vertices[(idx * 6) + 4] = 0.f;
						vertices[(idx * 6) + 5] = -1.f;
					}
				}
			}

			auto time_end = std::chrono::steady_clock::now();
			std::cout << "REGION GROWING TIME: " << std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count() / 1000.f << " sec\n";
			std::cout << "region_growing_done\n";

			//     std::cout << "Box beallitas vege!\n";
			/*            std::cout << "Resz pontfelho keszites eredertibol...\n";

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_part(new pcl::PointCloud<pcl::PointXYZRGB>());

			for(auto point : obj_cloud->points)
			{
			if(myBox.isPointInsideBox(glm::vec3(point.x, point.y, point.z)))
			{
			cloud_part->points.push_back(point);
			}
			}

			cloud_part->width = cloud_part->points.size();
			cloud_part->height = 1;
			cloud_part->is_dense = false;

			std::cout << "Szeparacio a resz felhore...\n";

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr sep_cloud_part(new pcl::PointCloud<pcl::PointXYZRGB>());
			sep_cloud_part = separation(cloud_part);
			sep_cloud_part->width = sep_cloud_part->points.size();
			sep_cloud_part->height = 1;
			sep_cloud_part->is_dense = false;

			std::cout << "Approximacio a resz felhore...\n";

			Approximation(cloud_part, sep_cloud_part);

			std::cout << "Redundancia kiiktatasa...\n";
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr sep_cloud_part_final(new pcl::PointCloud<pcl::PointXYZRGB>());

			sep_cloud_part_final = redundancy(sep_cloud_part);
			sep_cloud_part_final->width = sep_cloud_part_final->points.size();
			sep_cloud_part_final->height = 1;
			sep_cloud_part_final->is_dense = false;

			for(auto pt : cloud_part->points)
			{
			new_vertices.push_back(pt.x);
			new_vertices.push_back(pt.y);
			new_vertices.push_back(pt.z);

			pt.r = 254.f * 256.f;
			pt.g = -256.f;
			pt.b = -256.f;

			new_vertices.push_back((float)pt.r / 256.f);
			new_vertices.push_back((float)pt.g / 256.f);
			new_vertices.push_back((float)pt.b / 256.f);

			cloud->points.push_back(pt);
			}

			std::cout << "Vektorok beoltese az ujak beletetelevel KESZ\n";

			// cloud->width = cloud->points.size();

			// Fill up my tree:
			pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> *removable_octree;
			removable_octree = octree;

			octree = new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(resolution);
			octree->setInputCloud (cloud);
			octree->addPointsFromInputCloud();

			delete removable_octree;

			vertices = new_vertices;

			std::cout << "Uj felho szeparalasa\n";
			obj_cloud = separation(cloud);
			std::cout << "****DONE***\n";
			*/
			std::cout << "****Teglalap adatok:****\n";
			std::cout << "Min_X: " << myBox.getMin_X() << " || Max_X: " << myBox.getMax_X() << '\n';
			std::cout << "Min_Y: " << myBox.getMin_Y() << " || Max_Y: " << myBox.getMax_Y() << '\n';
			std::cout << "Min_Z: " << myBox.getMin_Z() << " || Max_Z: " << myBox.getMax_Z() << '\n';
		}

		//Normalis kirajzoltatas
		/*
		glm::vec3 normalDirectedDiagonal(camera->Front * (double_diagonal_length / 2.f));

		Point P0{camera->Position.x - normalDirectedDiagonal.x, camera->Position.y - normalDirectedDiagonal.y, camera->Position.z - normalDirectedDiagonal.z};
		Point P1{camera->Position.x + normalDirectedDiagonal.x, camera->Position.y + normalDirectedDiagonal.y, camera->Position.z + normalDirectedDiagonal.z};

		vertices_line.push_back(P0.x);
		vertices_line.push_back(P0.y);
		vertices_line.push_back(P0.z);
		vertices_line.push_back(P1.x);
		vertices_line.push_back(P1.y);
		vertices_line.push_back(P1.z);
		vertices_line.push_back(254.f);
		vertices_line.push_back(0.f);
		vertices_line.push_back(0.f);

		glBindVertexArray(VAO_line);
		glBindBuffer(GL_ARRAY_BUFFER, VBO_line);
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * vertices_line.size(), vertices_line.data(), GL_DYNAMIC_DRAW);
		glBindVertexArray(0);

		Line L0{P0, P1};

		std::vector<std::pair<int, float>> indicesToDist;

		for(size_t i{0}; i < vertices.size(); i += 6)
		{
		if(camera->Front.x < 0.f && vertices[i] > camera->Position.x) continue;
		if(camera->Front.x > 0.f && vertices[i] < camera->Position.x) continue;
		if(camera->Front.y < 0.f && vertices[i+1] > camera->Position.y) continue;
		if(camera->Front.y > 0.f && vertices[i+1] < camera->Position.y) continue;
		if(camera->Front.z < 0.f && vertices[i+2] > camera->Position.z) continue;
		if(camera->Front.z > 0.f && vertices[i+2] < camera->Position.z) continue;

		Point P_test{vertices[i], vertices[i+1], vertices[i+2]};

		if(dist_Point_to_Line(P_test, L0) < 0.1f)
		{
		glm::vec3 cameraToPointVec{vertices[i] - camera->Position.x, vertices[i+1] - camera->Position.y, vertices[i+2] - camera->Position.z};

		indicesToDist.push_back(std::make_pair(i/6, glm::length(cameraToPointVec)));
		}
		}

		size_t minIndex{0};
		float minDist{std::numeric_limits<float>::max()};

		for(size_t i{0}; i < indicesToDist.size(); i++)
		{
		if(indicesToDist[i].second < minDist)
		{
		minDist = indicesToDist[i].second;
		minIndex = indicesToDist[i].first;
		}
		}

		float maxOfZ{vertices[(minIndex * 6) + 2]};

		for(size_t i{0}; i < vertices.size(); i += 6)
		{
		if(vertices[i+5] < 0.f)
		{
		if(vertices[i+2] <= maxOfZ)
		{
		vertices[i+3] = (float)cloud->points[i/6].r / 256.f;
		vertices[i+4] = (float)cloud->points[i/6].g / 256.f;
		vertices[i+5] = (float)cloud->points[i/6].b / 256.f;
		}
		}
		}

		*/

		glBindVertexArray(VAO);

		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(GLfloat) * vertices.size(), vertices.data());

		glBindVertexArray(0);
	}
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
	//camera->ProcessMouseScroll(yoffset);
	if (yoffset < 0.f) search_radius -= 0.01;
	else search_radius += 0.01;

	std::cout << "SEARCH_RADIUS: " << search_radius << '\n';

	if (search_radius > 2 * resolution || search_radius < resolution / 2)
	{
		std::cout << "You should optimize octree resolution! Press 'o'!\n";
	}
	/*
	//camera->ProcessMouseScroll(yoffset);
	if (yoffset < 0.f) divider -= 1.f;
	else divider += 1.f;

	if(divider < 1.f) divider = 1.f;

	std::cout << "DIVIDER: " << divider << '\n';
	*/
}

void optimize_octree_resolution()
{
	std::cout << "Optimize octree resolution... ";

	delete octree;

	resolution = search_radius;

	octree = new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(resolution);
	octree->setInputCloud(cloud);
	octree->addPointsFromInputCloud();

	std::cout << "Done!\n";
}

void drawCrosshair(GLuint MatrixID_model, glm::mat4 Model, glm::vec3 pos)
{
	glPointSize(9.3f);
	//glColor3f(255.f, 255.f, 0.f);

	Model = glm::translate(Model, pos);

	glUniformMatrix4fv(MatrixID_model, 1, GL_FALSE, &Model[0][0]);

	glBindVertexArray(VAO_cross);
	glDrawArrays(GL_POINTS, 0, vertices_cross.size());
	glBindVertexArray(0);

	glPointSize(1.4f);
}

void drawNormals(GLuint MatrixID_model, glm::mat4 Model, glm::vec3 pos)
{
	glLineWidth(3.f);
	/*
	Model = glm::translate(Model, pos);

	glUniformMatrix4fv(MatrixID_model, 1, GL_FALSE, &Model[0][0]);
	*/
	glBindVertexArray(VAO_line);
	glDrawArrays(GL_LINES, 0, vertices_line.size());
	glBindVertexArray(0);
}

void floodFill(glm::vec3 pos)
{
	std::queue<Box3D> box_queue;
	std::vector<Box3D> actual_neighbors;
	shc::hash_table<std::string, bool> box_set;

	Box3D init_box(pos, 0.1);

	box_queue.push(init_box);
	box_set.set(init_box.getStrPos(), true);

	bool isFindPoints{ false };

	while (!box_queue.empty())
	{
		isFindPoints = false;

		if (box_queue.front().getZ() < init_box.getZ())
		{
			box_queue.pop();
			continue;
		}

		actual_neighbors.clear();

		for (size_t i{ 0 }; i < vertices.size(); i += 6)
		{
			if (box_queue.front().isPointInsideBox(glm::vec3(vertices[i], vertices[i + 1], vertices[i + 2])))
			{
				isFindPoints = true;

				vertices[i + 3] = 254.f;
				vertices[i + 4] = 0.f;
				vertices[i + 5] = -1.f;

				/// TODO: set cloud->points colors!
			}
		}

		if (isFindPoints)
		{
			actual_neighbors = box_queue.front().getAllNeighbors(true);
		}
		else
		{
			if (box_queue.front().isFirstEmptyChild())
			{
				actual_neighbors = box_queue.front().getAllNeighbors(false);
			}
		}

		bool findRedPoint{ false };

		for (auto box : actual_neighbors)
		{

			for (size_t i{ 0 }; i < vertices.size(); i += 6)
			{
				if (box.isPointInsideBox(glm::vec3(vertices[i], vertices[i + 1], vertices[i + 2])))
				{
					if (vertices[i + 5] < 0.f) findRedPoint = true;
				}

				if (findRedPoint) break;
			}

			if (!findRedPoint)
			{
				box_queue.push(box);
			}
		}

		box_queue.pop();
	}
}

void reverseLastSelected()
{
	for (size_t i{ 0 }; i < vertices.size(); i += 6)
	{
		if (vertices[i + 5] < 0.f)
		{
			if (vertices[i + 4] >= 0.f)
			{
				vertices[i + 3] = (float)cloud->points[i / 6].r / 256.f;
				vertices[i + 4] = (float)cloud->points[i / 6].g / 256.f;
				vertices[i + 5] = (float)cloud->points[i / 6].b / 256.f;
			}
		}
	}

	glBindVertexArray(VAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(GLfloat) * vertices.size(), vertices.data());

	glBindVertexArray(0);
}

void unmarkPrevLastSelected()
{
	for (size_t i{ 0 }; i < vertices.size(); i += 6)
	{
		if (vertices[i + 5] < 0.f) vertices[i + 4] = -1.f;
	}
}


void CaptureScreen()
{
	BITMAPFILEHEADER bf;
	BITMAPINFOHEADER bi;

	pic_num++;

	unsigned char *image = (unsigned char*)malloc(sizeof(unsigned char)*WIDTH*HEIGHT * 3);

	std::string filename_img = "capture" + std::to_string(pic_num) + ".bmp";

	FILE *file = fopen(filename_img.c_str(), "wb");

	if (image != NULL)
	{
		if (file != NULL)
		{
			glReadPixels(0, 0, WIDTH, HEIGHT, GL_BGR_EXT, GL_UNSIGNED_BYTE, image);

			memset(&bf, 0, sizeof(bf));
			memset(&bi, 0, sizeof(bi));

			bf.bfType = 'MB';
			bf.bfSize = sizeof(bf) + sizeof(bi) + WIDTH*HEIGHT * 3;
			bf.bfOffBits = sizeof(bf) + sizeof(bi);
			bi.biSize = sizeof(bi);
			bi.biWidth = WIDTH;
			bi.biHeight = HEIGHT;
			bi.biPlanes = 1;
			bi.biBitCount = 24;
			bi.biSizeImage = WIDTH*HEIGHT * 3;

			fwrite(&bf, sizeof(bf), 1, file);
			fwrite(&bi, sizeof(bi), 1, file);
			fwrite(image, sizeof(unsigned char), HEIGHT*WIDTH * 3, file);

			fclose(file);
		}
		free(image);
	}
}

void PCLseg()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointIndicesPtr ground (new pcl::PointIndices);

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read<pcl::PointXYZ> ("samp11-utm.pcd", *cloud);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  std::cout << "seg1\n";

  // Create the filtering object
  pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
  std::cout << "seg2\n";
  pmf.setInputCloud (cloud);
  std::cout << "seg3\n";
  pmf.setMaxWindowSize (20);
  std::cout << "seg4\n";
  pmf.setSlope (1.0f);
  std::cout << "seg5\n";
  pmf.setInitialDistance (0.5f);
  std::cout << "seg6\n";
  pmf.setMaxDistance (3.0f);
  std::cout << "seg7\n";
  pmf.extract (ground->indices);
  std::cout << "seg8\n";

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  std::cout << "seg9\n";
  extract.setInputCloud (cloud);
  std::cout << "seg10\n";
  extract.setIndices (ground);
  std::cout << "seg11\n";
  extract.filter (*cloud_filtered);
  std::cout << "seg12\n";
  std::cerr << "Ground cloud after filtering: " << std::endl;
  std::cout << "seg13\n";
  std::cerr << *cloud_filtered << std::endl;
  std::cout << "seg14\n";
  pcl::PCDWriter writer;
  std::cout << "seg15\n";
  writer.write<pcl::PointXYZ> ("autok_ground.pcd", *cloud_filtered, false);
  std::cout << "seg16\n";
  // Extract non-ground returns
  extract.setNegative (true);
  std::cout << "seg17\n";
  extract.filter (*cloud_filtered);
  std::cout << "seg18\n";
  std::cerr << "Object cloud after filtering: " << std::endl;
  std::cout << "seg19\n";
  std::cerr << *cloud_filtered << std::endl;
  std::cout << "seg20\n";
  writer.write<pcl::PointXYZ> ("autok_object.pcd", *cloud_filtered, false);
}


