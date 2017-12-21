#ifndef BOX3D_HPP_INCLUDED
#define BOX3D_HPP_INCLUDED

struct Pos3D{ float x, y, z; };

class Box3D
{
    float x, y, z;
    float min_x, min_y, min_z;
    float max_x, max_y, max_z;
    float edge_increase_size, edge_size;
    bool isFirstEmpty;

public:
    Box3D(glm::vec3 point, float initial_edge_size, bool isFirstEmpty_init = true)
    {
        x = point.x;
        y = point.y;
        z = point.z;

        edge_size = initial_edge_size;
        isFirstEmpty = isFirstEmpty_init;

        min_x = point.x - initial_edge_size / 2.f;
        min_y = point.y - initial_edge_size / 2.f;
        min_z = point.z - initial_edge_size / 2.f;

        max_x = point.x + initial_edge_size / 2.f;
        max_y = point.y + initial_edge_size / 2.f;
        max_z = point.z + initial_edge_size / 2.f  ;

        edge_increase_size = initial_edge_size / 2.f;
    }

    bool operator==(const Box3D& that) const
    {
        if(this->getX() == that.getX() && this->getY() == that.getY() && this->getZ() == that.getZ()
            && this->getEdgeSize() == that.getEdgeSize()) return true;

        return false;
    }

    bool operator!=(const Box3D& that) const
    {
        return !(*this == that);
    }

    std::vector<Box3D> getAllNeighbors(bool isFirstEmpty_init) const
    {
        std::vector<Box3D> neighbors;

        neighbors.emplace_back(glm::vec3(x + edge_size, y, z), edge_size, isFirstEmpty_init);
        neighbors.emplace_back(glm::vec3(x - edge_size, y, z), edge_size, isFirstEmpty_init);
        neighbors.emplace_back(glm::vec3(x, y + edge_size, z), edge_size, isFirstEmpty_init);
        neighbors.emplace_back(glm::vec3(x, y - edge_size, z), edge_size, isFirstEmpty_init);
        neighbors.emplace_back(glm::vec3(x, y, z + edge_size), edge_size, isFirstEmpty_init);
        neighbors.emplace_back(glm::vec3(x, y, z - edge_size), edge_size, isFirstEmpty_init);

        return neighbors;
    }

    void setEdgeIncreaseSize(float num)
    {
        edge_increase_size = num;
    }

    bool isFirstEmptyChild() const { return isFirstEmpty; }

    void lengthenMax_X() { max_x += edge_increase_size; }
    void shortenMax_X() { max_x -= edge_increase_size; }
    void lengthenMax_Y() { max_y += edge_increase_size; }
    void shortenMax_Y() { max_y -= edge_increase_size; }
    void lengthenMax_Z() { max_z += edge_increase_size; }
    void shortenMax_Z() { max_z -= edge_increase_size; }

    void lengthenMin_X() { min_x -= edge_increase_size; }
    void shortenMin_X() { min_x += edge_increase_size; }
    void lengthenMin_Y() { min_y -= edge_increase_size; }
    void shortenMin_Y() { min_y += edge_increase_size; }
    void lengthenMin_Z() { min_z -= edge_increase_size; }
    void shortenMin_Z() { min_z += edge_increase_size; }

    bool isPointInsideBox(glm::vec3 test_point)
    {
        if (test_point.x < min_x || test_point.y < min_y || test_point.z < min_z
         || test_point.x > max_x || test_point.y > max_y || test_point.z > max_z)
            return false; //is not within cube

        return true; //is within cube
    }

    float getX() const { return x; }
    float getY() const { return y; }
    float getZ() const { return z; }

    float getEdgeSize() const { return edge_size; }

    float getMax_X() const { return max_x; }
    float getMax_Y() const { return max_y; }
    float getMax_Z() const { return max_z; }
    float getMin_X() const { return min_x; }
    float getMin_Y() const { return min_y; }
    float getMin_Z() const { return min_z; }

    std::string getStrPos() const
    {
        return (std::to_string(x) + std::to_string(y) + std::to_string(z));
    }

};

#endif // BOX3D_HPP_INCLUDED
