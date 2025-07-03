#include <iostream>
#include <cmath>
#include <iomanip>
#include <vector>
#include <memory>
#include <fstream>
#include <random>
#include <sstream>

const float angle_scale = 0.7;

class Matrix4x4 {
public:
    float m[4][4];

    // Constructor: initializes identity matrix
    Matrix4x4() {
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                m[i][j] = (i == j) ? 1.0 : 0.0;
    }

    // Matrix multiplication
    Matrix4x4 operator*(const Matrix4x4& other) const {
        Matrix4x4 result;

        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j) {
                result.m[i][j] = 0.0;
                for (int k = 0; k < 4; ++k)
                    result.m[i][j] += m[i][k] * other.m[k][j];
            }

        return result;
    }

    // Static method to generate transformation matrix from DH parameters
    static Matrix4x4 transform_frame(float d, float theta, float a, float alpha) {
        Matrix4x4 T;

        T.m[0][0] = cos(theta);
        T.m[0][1] = -sin(theta) * cos(alpha);
        T.m[0][2] = sin(theta) * sin(alpha);
        T.m[0][3] = a * cos(theta);

        T.m[1][0] = sin(theta);
        T.m[1][1] = cos(theta) * cos(alpha);
        T.m[1][2] = -cos(theta) * sin(alpha);
        T.m[1][3] = a * sin(theta);

        T.m[2][0] = 0.0;
        T.m[2][1] = sin(alpha);
        T.m[2][2] = cos(alpha);
        T.m[2][3] = d;

        T.m[3][0] = 0.0;
        T.m[3][1] = 0.0;
        T.m[3][2] = 0.0;
        T.m[3][3] = 1.0;

        return T;
    }

    // Print matrix
    void print() const {
        std::cout << std::fixed << std::setprecision(4);
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j)
                std::cout << std::right << std::setw(8) << m[i][j] << " ";
            std::cout << "\n";
        }
    }

    // get matrix form string
    std::string matrix_string() const {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(4);

        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j)
                oss << std::right << std::setw(8) << m[i][j] << " ";
            oss << "\n";
        }

        return "```txt\n" + oss.str() + "```\n";
    }

    // Data stream
    std::string data_string() const {
        std::string stream;
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 4; ++col)
                stream += std::to_string(m[row][col]) + ", ";
        }
        // Remove the last ", "
        return stream.erase(stream.length() - 2);
    }

    std::string headers_list() const {
        return "n_x, n_y, n_z, o_x, o_y, o_z, a_x, a_y, a_z, x, y, z";
    }
};

class DH_parameters {
public:
    float d, theta, r, alpha;

    DH_parameters(float d, float theta, float r, float alpha)
    : d(d), theta(theta), r(r), alpha(alpha) {
        d = std::max(0.0f, d);
        theta = fmod(theta, 2 * M_PI);
        r = std::max(0.0f, r);
        alpha = fmod(alpha, 2 * M_PI);
    }
};

class Joint {
public:
    virtual void update(float delta_time) = 0;
    virtual Matrix4x4 transform() const = 0;
    virtual ~Joint() = default;
};

class RotationalJoint : public Joint {
public:
    DH_parameters params;
    float angular_velocity;
    float angular_position_limit;

    RotationalJoint(DH_parameters params, float angular_velocity, float angular_position_limit) 
    : params(params), angular_velocity(angular_velocity), angular_position_limit(angular_position_limit) {
        angular_position_limit = fmod(angular_position_limit, 2 * M_PI);
    }

    void update(float delta) {
        params.theta += angular_velocity * delta;
        
        // Bounce back mechanism
        if (params.theta < 0.0) {
            // params.theta = 0.0;
            angular_velocity *= -angle_scale;
            return;
        }
        if (params.theta > angular_position_limit) {
            // params.theta = angular_position_limit;
            angular_velocity *= -angle_scale;
            return;
        }
    }

    void setAngularVelocity(float angular_velocity) {
        this->angular_velocity = angular_velocity;
    }

    Matrix4x4 transform() const {
        return Matrix4x4::transform_frame(params.d, params.theta, params.r, params.alpha);
    }

    std::string getConfiguration() const {
        std::string config = "Rotational joint configuration:\n\n";
        config += "- d: " + std::to_string(params.d) + "\n";
        config += "- theta: " + std::to_string(params.theta) + "\n";
        config += "- r: " + std::to_string(params.r) + "\n";
        config += "- alpha: " + std::to_string(params.alpha) + "\n";
        config += "- angular velocity: " + std::to_string(angular_velocity) + "\n";
        config += "- angular position limit: " + std::to_string(angular_position_limit) + "\n\n";
        return config;
    }
};

class PrismaticJoint : public Joint {
public:
    DH_parameters params;
    float linear_velocity;
    float upper_limit = 10.0;

    PrismaticJoint(DH_parameters params, float linear_velocity, float upper_limit) 
    : params(params), linear_velocity(linear_velocity), upper_limit(upper_limit) {}

    void update(float delta) {
        params.d += linear_velocity * delta;

        // Bounce back mechanism
        if (params.d < 0.0) {
            // params.d = 0.0;
            linear_velocity *= -angle_scale;
            return;
        }
        if (params.d > upper_limit) {
            // params.d = upper_limit;
            linear_velocity *= -angle_scale;
            return;
        }
    }

    void setLinearVelocity(float linear_velocity) {
        this->linear_velocity = linear_velocity;
    }

    Matrix4x4 transform() const {
        return Matrix4x4::transform_frame(params.d, params.theta, params.r, params.alpha);
    }

    std::string getConfiguration() const {
        std::string config = "Prismatic joint configuration:\n\n";
        config += "- d: " + std::to_string(params.d) + "\n";
        config += "- theta: " + std::to_string(params.theta) + "\n";
        config += "- r: " + std::to_string(params.r) + "\n";
        config += "- alpha: " + std::to_string(params.alpha) + "\n";
        config += "- linear velocity: " + std::to_string(linear_velocity) + "\n";
        config += "- upper limit: " + std::to_string(upper_limit) + "\n\n";
        return config;
    }
};

class Arm {
public:
    std::vector<std::shared_ptr<Joint>> joints;

    void addJoint(std::shared_ptr<Joint> joint) {
        joints.push_back(joint);
    }

    void update(float delta_time) {
        for (auto& joint : joints) {
            joint->update(delta_time);
        }
    }

    Matrix4x4 forwardKinematics() const {
        Matrix4x4 result;
        for (const auto& joint : joints) {
            result = joint->transform() * result;
        }
        return result;
    }

    Matrix4x4 getTransformMatrix() const {
        Matrix4x4 transformation_matrix = joints[0]->transform();
        for (size_t i = 1; i < joints.size(); ++i) {
            transformation_matrix = joints[i]->transform() * transformation_matrix;
        }
        return transformation_matrix;
    }

    std::string getConfiguration() const {
        std::string config = "Arm configuration:\n\n";
        for (size_t i = 0; i < joints.size(); ++i) {
            config += "Joint " + std::to_string(i) + ":\n";
            if (auto rj = std::dynamic_pointer_cast<RotationalJoint>(joints[i])) {
                config += rj->getConfiguration();
            } else if (auto pj = std::dynamic_pointer_cast<PrismaticJoint>(joints[i])) {
                config += pj->getConfiguration();
            } else {
                config += "Unknown joint type\n";
            }
        }
        return config;
    }
};

class Randomizer{
public:
    float random(float min, float max){
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(min, max);
        return dis(gen);
    }

    PrismaticJoint randomPrismaticJoint(){
        float d = random(0.0, 10.0);
        float theta = random(0.0, 2 * M_PI);
        float r = random(0.0, 10.0);
        float alpha = random(0.0, 2 * M_PI);
        float linear_velocity = random(-10.0, 10.0);
        float upper_limit = random(0.5, 10.0);
        return PrismaticJoint(DH_parameters(d, theta, r, alpha), linear_velocity, upper_limit);
    }

    RotationalJoint randomRotationalJoint(){
        float d = random(0.0, 10.0);
        float theta = random(0.0, 2 * M_PI);
        float r = random(0.0, 10.0);
        float alpha = random(0.0, 2 * M_PI);
        float angular_velocity = random(-2*M_PI, 2*M_PI);
        float angular_position_limit = random(0.5, 2 * M_PI);
        return RotationalJoint(DH_parameters(d, theta, r, alpha), angular_velocity, angular_position_limit);
    }
};

int main(int argc, char* argv[]) {
    std::string sequence;
    // Unpack arguments
    for (int i = 1; i < argc; i++) {
        sequence += argv[i];
    }
    Arm arm;

    Randomizer randomizer;
    for (char c: sequence) {
        if (c == 'P') {
            arm.addJoint(std::make_shared<PrismaticJoint>(randomizer.randomPrismaticJoint()));
        } else if (c == 'R') {
            arm.addJoint(std::make_shared<RotationalJoint>(randomizer.randomRotationalJoint()));
        }
    }

    // destructor Randomizer will be called
    randomizer.~Randomizer();
    
    std::ofstream report("package/"+sequence+"/report.md", std::ios_base::app);
    report << arm.getConfiguration();
    report << arm.getTransformMatrix().matrix_string();
    report.close();

    std::ofstream out("trajectory.csv");
    out << Matrix4x4().headers_list() << "\n";

    for (int t = 0; t < 100; ++t) {
        arm.update(0.01);
        Matrix4x4 pose = arm.forwardKinematics();
        out << pose.data_string() << "\n";
    }

    out.close();

    return 0;
}
