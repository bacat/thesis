#ifndef CAMERA_HPP_INCLUDED
#define CAMERA_HPP_INCLUDED

#include <vector>

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "VeryNaiveSphere.hpp"



// Defines several possible options for camera movement. Used as abstraction to stay away from window-system specific input methods
enum Camera_Movement {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    UP,
    DOWN
};

// Default camera internal values. I want to get these in compile time.
constexpr GLfloat YAW        =  0.0f;
constexpr GLfloat PITCH      =  0.0f;
constexpr GLfloat SPEED      =  5.f;
constexpr GLfloat SENSITIVTY =  0.3f;
constexpr GLfloat ZOOM       =  45.0f;


// An abstract camera class that processes input and calculates the corresponding Eular Angles, Vectors and Matrices for use in OpenGL
class Camera
{
public:
    // Camera Attributes
    glm::vec3 Position;
    glm::vec3 Front;
    glm::vec3 Up;
    glm::vec3 Right;
    glm::vec3 WorldUp;

    // Eular Angles
    float Yaw;
    float Pitch;

    // Camera options
    GLfloat MovementSpeed;
    GLfloat MouseSensitivity;
	float movementSensitivity;
    GLfloat Zoom;

    VeryNaiveSphere* mySphere;

    // Constructor with vectors
	Camera(float initMovementSensitivity, glm::vec3 position = glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3 up = glm::vec3(0.0f, 0.0f, 1.0f), GLfloat yaw = YAW, GLfloat pitch = PITCH) : Front(glm::vec3(0.0f, 0.0f, -1.0f)), MovementSpeed(SPEED * initMovementSensitivity), MouseSensitivity(SENSITIVTY), Zoom(ZOOM)
    {
        this->Position = position;
        this->WorldUp = up;
        this->Yaw = yaw;
        this->Pitch = pitch;
		this->movementSensitivity = initMovementSensitivity * 0.01f;
        this->updateCameraVectors();
        mySphere = new VeryNaiveSphere(0.0006, 0, this->Position);
    }
    // Constructor with scalar values
    Camera(GLfloat posX, GLfloat posY, GLfloat posZ, GLfloat upX, GLfloat upY, GLfloat upZ, GLfloat yaw, GLfloat pitch) : Front(glm::vec3(0.0f, 0.0f, -1.0f)), MovementSpeed(SPEED), MouseSensitivity(SENSITIVTY), Zoom(ZOOM)
    {
        this->Position = glm::vec3(posX, posY, posZ);
        this->WorldUp = glm::vec3(upX, upY, upZ);
        this->Yaw = yaw;
        this->Pitch = pitch;
        this->updateCameraVectors();
        mySphere = new VeryNaiveSphere(0.0006, 0, this->Position);
    }

    // Returns the view matrix calculated by using Eular Angles and the LookAt Matrix
    glm::mat4 GetViewMatrix()
    {
        return glm::lookAt(this->Position, this->Position + this->Front, this->Up);
    }

    // Only, if I want to use legacy OpenGL (I don't want to use it.)
    void legacy_GetViewMatix()
    {
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        gluLookAt(this->Position[0], this->Position[1], this->Position[0],
                this->Position[0] + this->Front[0], this->Position[1] + this->Front[1], this->Position[2] + this->Front[2],
                this->Up[0], this->Up[1], this->Up[2]);
    }

    // Processes input received from any keyboard-like input system. Accepts input parameter in the form of camera defined ENUM (to abstract it from windowing systems)
    void ProcessKeyboard(Camera_Movement direction, GLfloat deltaTime,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> &octree)
    {
        glm::vec3 FrontToMove(this->Front[0], this->Front[1], 0.f);
        glm::vec3 UpFrontToMove(0.f, 0.f, 1.f);
        glm::vec3 DownFrontToMove(0.f, 0.f, -1.f);

        //Gömbön belül keresem a pontokat, a gömböm közepében pedig mindig a kamera van(gömböt igazítom a kamerához):
        mySphere->setPosition(this->Position);

        //adott sugaron belüli összes pontot keresem(ezzel/ezekkel történt az ütközés)
        std::vector<glm::vec3> insidePoints = mySphere->getInsidePoints(cloud, octree);
        std::vector<glm::vec3> normalizedInsideVectors;
        glm::vec3 maxZPoint(0.f, 0.f, 0.f);

        //a kamerából egy vektorral rámutatok ezekre és "átlagolom" az iráyukat, vagyis összeadom és a hosszal leosztom
        //(egységvektort kapok)
        for(auto point : insidePoints)
        {
            glm::vec3 vectorToPoint(point - this->Position);
            glm::vec3 normalizedVectorToPoint(glm::normalize(vectorToPoint));

            if(point.z > maxZPoint.z) maxZPoint = point; //közben keresme hogy melyik pont van a legmagasabban az összes közül(Z koord a legnagyobb)

            normalizedInsideVectors.push_back(normalizedVectorToPoint);
        }

        GLfloat velocity = this->MovementSpeed * deltaTime;

        if(normalizedInsideVectors.size() > 0)
        {
            //Adott irányú sebességvektorok:
            glm::vec3 frontSpeedVector(this->Front * velocity);
            glm::vec3 rightSpeedVector(this->Right * velocity);
            glm::vec3 upSpeedVector(UpFrontToMove * velocity);

            glm::vec3 sumOfNormalizedVectors(0.f, 0.f, 0.f);

            //Itt történik az átlagolás második része:
            for(auto normalizedVectorToPoint : normalizedInsideVectors)
            {
                sumOfNormalizedVectors += normalizedVectorToPoint;
            }

            glm::vec3 normalizedAvgVector = glm::normalize(sumOfNormalizedVectors);

            //Ütközés irányú sebességkomponens (előre definiált sebességvektorok mellett)
            glm::vec3 frontComponent(normalizedAvgVector * normalizedAvgVector * frontSpeedVector);
            glm::vec3 rightComponent(normalizedAvgVector * normalizedAvgVector * rightSpeedVector);
            glm::vec3 upComponent(normalizedAvgVector * normalizedAvgVector * frontSpeedVector);

            //HA az ütközés irányú sebességkomponens és az ütközés irányú vektor bezárt szöge kisebb, mint 90 fok, akkor
            //az eredetiből kivonom ezt a komponenst
            //Ellenkező esetben nem közeledtem a ponthoz, hanem távolodtam tőle, ezért nem kell kivonni.
            if(glm::acos((frontComponent * normalizedAvgVector) / glm::length(frontComponent) * glm::length(normalizedAvgVector))[0] < 90.f)
                frontSpeedVector -= frontComponent;

            if(glm::acos((rightComponent * normalizedAvgVector) / glm::length(rightComponent) * glm::length(normalizedAvgVector))[0] < 90.f)
                rightSpeedVector -= rightComponent;

            if(glm::acos((upComponent * normalizedAvgVector) / glm::length(upComponent) * glm::length(normalizedAvgVector))[0] < 90.f)
                upSpeedVector -= upComponent;

            //Végül a billentyűzeről kapott irányítási információ szerinti mozgatás már a megfelelő vektorral:
            if (direction == FORWARD)
                this->Position += frontSpeedVector;
            if (direction == BACKWARD)
                this->Position -= frontSpeedVector;
            if (direction == LEFT)
                this->Position -= rightSpeedVector;
            if (direction == RIGHT)
                this->Position += rightSpeedVector;
            if (direction == UP)
                this->Position += upSpeedVector;

            //magasság beállítása (600 itt most a gömb sugara, 350 az a távolsági tényező ami alapján ütközésnek, vagy megmászható objektumnak tekintjük
            // az adott pontot tartalmazó "objektumot", ezt lehet finomítani/testre szabni)
            if(std::abs(glm::length(maxZPoint - this->Position)) < 0.0003) this->Position.z = maxZPoint.z + 0.0006;
        }
        else
        {
            //Ha semmilyen ponttal nem érintkezek, akkor csak sima mozgatás:
            if (direction == FORWARD)
                this->Position += this->Front * velocity;
            if (direction == BACKWARD)
                this->Position -= this->Front * velocity;
            if (direction == LEFT)
                this->Position -= this->Right * velocity;
            if (direction == RIGHT)
                this->Position += this->Right * velocity;
            if (direction == UP)
                this->Position += UpFrontToMove * velocity;
        }

    }

    void setPositon(glm::vec3 pos) { this->Position = pos; }

	// Processes input received from a mouse input system. Expects the offset value in both the x and y direction.
	void ProcessRigidBodyOrientation(GLfloat xoffset, GLfloat yoffset, GLboolean constrainPitch = true)
	{
		this->Yaw = xoffset;
		std::cout << "Camera YAW: " << this->Yaw << '\n';
		//this->Pitch = yoffset;

		// Makes sure that when the pitch is out of bounds, screen doesn't get flipped
	/*	if (constrainPitch)
		{
			if (this->Pitch > 89.0f)
				this->Pitch = 89.0f;
			if (this->Pitch < -89.0f)
				this->Pitch = -89.0f;
		}*/

		// Update Front, Right and Up Vectors using the updated Eular angles
		this->updateCameraVectors();
	}

	void ProcessRigidBodyMovement(float x_offset, float y_offset, float z_offset)
	{
		this->Position.x += x_offset * this->movementSensitivity;
		this->Position.y -= z_offset * this->movementSensitivity;
		this->Position.z += y_offset * this->movementSensitivity;
	}

	void IncreaseMovementSensitivity() { this->movementSensitivity += 0.01f; }
	void DecreaseMovementSensitivity() { this->movementSensitivity -= 0.01f; }

    // Processes input received from a mouse input system. Expects the offset value in both the x and y direction.
    void ProcessMouseMovement(GLfloat xoffset, GLfloat yoffset, GLboolean constrainPitch = true)
    {
        xoffset *= this->MouseSensitivity;
        yoffset *= this->MouseSensitivity;

        this->Yaw   += xoffset;
        this->Pitch += yoffset;

        // Makes sure that when the pitch is out of bounds, screen doesn't get flipped
        if (constrainPitch)
        {
            if (this->Pitch > 89.0f)
                this->Pitch = 89.0f;
            if (this->Pitch < -89.0f)
                this->Pitch = -89.0f;
        }

        // Update Front, Right and Up Vectors using the updated Eular angles
        this->updateCameraVectors();
    }

    // Processes input received from a mouse scroll-wheel event. Only requires input on the vertical wheel-axis
    void ProcessMouseScroll(GLfloat yoffset)
    {
        if (this->Zoom >= 1.0f && this->Zoom <= 45.0f)
            this->Zoom -= yoffset;
        if (this->Zoom <= 1.0f)
            this->Zoom = 1.0f;
        if (this->Zoom >= 45.0f)
            this->Zoom = 45.0f;
    }


    // Calculates the front vector from the Camera's (updated) Eular Angles
    void updateCameraVectors()
    {
        // Calculate the new Front vector
        glm::vec3 front;

        // Spherical coordinate system again
        // Maybe calculus II. @ PPKE was not useless? :D
        front.x = sin(glm::radians(this->Yaw)) * cos(glm::radians(this->Pitch));
        front.y = cos(glm::radians(this->Yaw)) * cos(glm::radians(this->Pitch));
        front.z = sin(glm::radians(this->Pitch));
        this->Front = glm::normalize(front);

        // Also re-calculate the Right and Up vector
        this->Right = glm::normalize(glm::cross(this->Front, this->WorldUp));  // Normalize the vectors, because their length gets closer to 0 than we want, when looking up or down.
        this->Up    = glm::normalize(glm::cross(this->Right, this->Front));
    }
};

#endif // CAMERA_HPP_INCLUDED
